#include <bee/cooperative_grasp/wrench_allocator.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Cholesky>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace bee::grasp
{
// Equal vertical load per contact, with upward tangent support. This is a
// preload capacity check; the online QP still checks the complete 6D wrench.
double requiredPreload(const Vec3& normal, double vertical_load, double mu,
                              double normal_error, double tangent_error)
{
  const double up_z = normal.head<2>().norm();
  const double authority = mu*up_z-normal.z();
  if (authority <= 1e-6) throw std::invalid_argument("contact cannot support the preload design load");
  return std::max(normal_error+tangent_error/mu,
                  (vertical_load+up_z*(mu*normal_error+tangent_error))/authority);
}
bool preloadCapacityReady(const ContactState& contact, double command,
                                 double vertical_load, double mu, const ControlConfig& p)
{
  const double tangent_error = p.allocation_tangent_up_error+p.allocation_tangent_side_error;
  const double capacity = std::min(mu*(command-p.allocation_normal_error)-tangent_error,
      mu*(contact.normal_force-p.allocation_observer_normal_error)-tangent_error);
  return std::isfinite(capacity) && capacity >= 0 &&
      -contact.normal.z()*command+contact.normal.head<2>().norm()*capacity >= vertical_load;
}
// Use exactly the planned tangent vector, in the allocator's four-sided inner
// friction pyramid. Command and observer uncertainty are independent bounds.
double plannedContactCapacitySlack(const ContactState& contact, const Vec3& force,
                                          const ControlConfig& p)
{
  if (!force.allFinite() || !std::isfinite(contact.normal_force)) return -INFINITY;
  const Vec3 n = contact.normal;
  Vec3 up = Vec3::UnitZ()-n.z()*n;
  if (up.norm()<1e-6) up=n.unitOrthogonal(); else up.normalize();
  const Vec3 side=up.cross(n);
  const double normal = -n.dot(force);
  const double tangent = std::abs(up.dot(force))+std::abs(side.dot(force));
  const double error=p.allocation_tangent_up_error+p.allocation_tangent_side_error;
  const double capacity=std::min(p.allocation_mu*(normal-p.allocation_normal_error),
      p.allocation_mu*(contact.normal_force-p.allocation_observer_normal_error))-error;
  return capacity-tangent;
}
bool plannedContactCapacityReady(const ContactState& contact, const Vec3& force,
                                       const ControlConfig& p)
{
  const double normal=-contact.normal.dot(force);
  return normal>=0 && normal<=p.max_normal_force-p.allocation_normal_error &&
      plannedContactCapacitySlack(contact,force,p)>=-1e-6;
}

bool referenceMotionValid(const Vec3& velocity, const Vec3& acceleration, const ControlConfig& p)
{
  return velocity.allFinite() && acceleration.allFinite() &&
      velocity.norm() <= p.reference_velocity_limit + 1e-6 &&
      acceleration.norm() <= p.reference_acceleration_limit + 1e-6;
}

const std::vector<Vec3>& accelerationDirections()
{
  static const std::vector<Vec3> directions=[] {
    std::vector<Vec3> result;
    for(int x : {-1,0,1}) for(int y : {-1,0,1}) for(int z : {-1,0,1})
      if(x||y||z) result.push_back(Vec3(x,y,z).normalized());
    return result;
  }();
  return directions;
}
// 26 planes approximate the receiver's Euclidean ball from INSIDE. The largest
// vertex radius for unit plane bounds is 1/0.886451886..., so 0.88645 is safe.
// Eight L1 planes rejected feasible CoG motions during preload handover.
double allocationAccelerationLimit(const ControlConfig& p)
{
  return .88645*std::max(0.0, p.reference_acceleration_limit - 5e-5);
}

// Project ONE rigid object's motion, not each aircraft's acceleration. Offsets
// include centripetal terms and contact-to-CoG kinematics at the measured state.
bool limitObjectMotion(const std::vector<Vec3>& arms, const std::vector<Vec3>& offsets,
                              const ControlConfig& p, Vec3& acceleration, Vec3& alpha)
{
  const int count = arms.size();
  const auto& directions=accelerationDirections();
  const int rows=directions.size()*count;
  Eigen::MatrixXd A(rows, 6);
  Eigen::VectorXd lo = Eigen::VectorXd::Constant(rows, -1e6), hi(rows);
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6,6);
  H.bottomRightCorner<3,3>() *= .1;
  Eigen::VectorXd desired(6); desired << acceleration, alpha;
  int row = 0;
  for (int i=0;i<count;++i)
    for (const auto& sign : directions)
    {
      A.block<1,3>(row,0) = sign.transpose();
      A.block<1,3>(row,3) = arms[i].cross(sign).transpose();
      hi[row++] = std::max(0.0,allocationAccelerationLimit(p)-2e-5)-sign.dot(offsets[i]);
    }
  if ((A*desired-hi).maxCoeff() <= 0) return true;
  const auto result = solveContactQp(H,H*desired,A,lo,hi,Eigen::VectorXd::Zero(6));
  if (!result.converged) return false;
  acceleration = result.x.head<3>(); alpha = result.x.tail<3>();
  return true;
}

Eigen::Matrix3d crossMatrix(const Vec3& r)
{
  Eigen::Matrix3d m;
  m << 0, -r.z(), r.y(), r.z(), 0, -r.x(), -r.y(), r.x(), 0;
  return m;
}

AllocationResult allocateWrench(const std::vector<ContactState>& contacts, const Vec3& origin,
    const Wrench& desired, const std::vector<Vec3>& inertia_force,
    const Eigen::VectorXd& previous, double preload, double dt, const ControlConfig& p,
    const AllocationOptions& options)
{
  const int n = contacts.size(), size = 3*n;
  const int rows = (20+(options.constrain_acceleration ? accelerationDirections().size() : 0))*n+
                  (options.planning ? 6 : 0);
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(6, size), A = Eigen::MatrixXd::Zero(rows, size);
  Eigen::VectorXd lo = Eigen::VectorXd::Constant(rows, -1e6), hi = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd nominal = Eigen::VectorXd::Zero(size), w(6);
  w << desired.force, 10*desired.moment;
  const double normal_error = options.contact_margin_scale*p.allocation_normal_error;
  const double tangent_error = options.contact_margin_scale*
      (p.allocation_tangent_up_error+p.allocation_tangent_side_error);
  const double normal_upper = std::min(p.max_normal_force-p.allocation_normal_error,
                                       options.normal_command_limit);
  int row = 0;
  for (int i=0; i<n; ++i)
  {
    const Vec3 normal = contacts[i].normal;
    Vec3 up = Vec3::UnitZ() - normal.z()*normal;
    if (up.norm() < 1e-6) up = normal.unitOrthogonal(); else up.normalize();
    const Vec3 side = up.cross(normal);
    G.block<3,3>(0,3*i).setIdentity();
    G.block<3,3>(3,3*i) = 10*crossMatrix(contacts[i].position-origin);
    const double minimum = options.normal_minimum.size() == n ? options.normal_minimum[i] : preload+normal_error;
    nominal.segment<3>(3*i) = -minimum*normal;
    A.block<1,3>(row,3*i) = -normal.transpose();
    lo[row] = options.planning ? minimum : std::min(minimum,
        std::max(0.0, -normal.dot(previous.segment<3>(3*i))) + .5*p.allocation_force_rate*dt);
    hi[row++] = normal_upper;
    for (int a : {-1,1}) for (int b : {-1,1})
    {
      const Vec3 tangent = a*up+b*side;
      A.block<1,3>(row,3*i) = (tangent+p.allocation_mu*normal).transpose();
      hi[row++] = -p.allocation_mu*normal_error-tangent_error;
      if(options.planning)
      {
        A.block<1,3>(row,3*i)=(tangent+p.allocation_mu*options.planning_force_ratio*normal).transpose();
        hi[row++]=-p.allocation_mu*p.allocation_observer_normal_error-tangent_error;
      }
      else
      {
        A.block<1,3>(row,3*i) = tangent.transpose();
        hi[row++] = p.allocation_mu*std::max(0.0,
            contacts[i].normal_force-p.allocation_observer_normal_error)-tangent_error;
      }
    }
    Eigen::MatrixXd thrust_map = Eigen::MatrixXd::Zero(3, size);
    thrust_map.block<3,3>(0,3*i).setIdentity();
    Vec3 thrust_offset = inertia_force[i];
    if (options.moving_object_mass > 0)
    {
      const double mass_ratio = p.robot_mass/options.moving_object_mass;
      const Eigen::Matrix3d moment_map = -p.robot_mass*
          crossMatrix(contacts[i].position-origin)*options.object_inverse_inertia;
      for (int j=0;j<n;++j)
        thrust_map.block<3,3>(0,3*j) += mass_ratio*Eigen::Matrix3d::Identity()+
            moment_map*crossMatrix(contacts[j].position-origin);
      thrust_offset -= mass_ratio*desired.force+moment_map*desired.moment;
    }
    for (int a : {-1,1}) for (int b : {-1,1}) for (int c : {-1,1})
    {
      const Vec3 sign(a,b,c);
      A.row(row) = sign.transpose()*thrust_map;
      hi[row++] = p.allocation_thrust_limit-p.allocation_thrust_reserve-sign.dot(thrust_offset);
    }
    if (options.constrain_acceleration)
    {
      Eigen::MatrixXd acceleration_map = thrust_map;
      acceleration_map.block<3,3>(0,3*i) -= Eigen::Matrix3d::Identity();
      acceleration_map /= p.robot_mass;
      const Vec3 offset = thrust_offset/p.robot_mass+Vec3(0,0,-p.gravity);
      for (const auto& sign : accelerationDirections())
      {
        A.row(row) = sign.transpose()*acceleration_map;
        hi[row++] = allocationAccelerationLimit(p)-sign.dot(offset);
      }
    }
    for (int j=0; j<3; ++j)
    {
      A(row,3*i+j) = 1;
      lo[row] = options.planning ? -1e6 : previous[3*i+j]-p.allocation_force_rate*dt;
      hi[row++] = options.planning ? 1e6 : previous[3*i+j]+p.allocation_force_rate*dt;
    }
  }
  if(options.planning)
  {
    // A holding plan must actually balance the specified payload wrench. A
    // small soft moment residual otherwise trades support capacity for squeeze.
    A.bottomRows(6)=G;lo.tail(6)=w;hi.tail(6)=w;
  }
  Eigen::MatrixXd H;
  Eigen::VectorXd rhs;
  if(options.planning)
  {
    // The wrench is already an equality. Avoid adding a redundant, badly
    // conditioned wrench penalty to the strictly convex force objective.
    H=Eigen::MatrixXd::Identity(size,size);
    rhs=(nominal+2*previous)/3;
  }
  else if(options.releasing)
  {
    // The payload is already supported by the destination. Remove contact
    // forces, rather than preserving a cancelling internal squeeze. Bounds,
    // friction, thrust and slew constraints still apply during this ramp.
    H=Eigen::MatrixXd::Identity(size,size);
    rhs=Eigen::VectorXd::Zero(size);
  }
  else
  {
    H=G.transpose()*G;
    H.diagonal().array() += .0003;
    rhs=G.transpose()*w+.0001*nominal+.0002*previous;
  }
  const auto solved = solveContactQp(H, rhs, A, lo, hi, previous);
  AllocationResult result;
  result.converged = solved.converged;
  result.violation = solved.violation; result.primal = solved.primal;
  result.stationarity = solved.stationarity; result.complementarity = solved.complementarity;
  result.iterations = solved.iterations; result.solver_status = solved.status; result.seconds = solved.seconds;
  result.restarts = solved.restarts;
  result.force = solved.x;
  const Eigen::VectorXd achieved = G*result.force;
  result.achieved.force = achieved.head<3>(); result.achieved.moment = achieved.tail<3>()/10;
  return result;
}

QpResult solveContactQp(const Eigen::MatrixXd& H, const Eigen::VectorXd& rhs,
                       const Eigen::MatrixXd& A, const Eigen::VectorXd& lo,
                       const Eigen::VectorXd& hi, const Eigen::VectorXd& previous)
{
  QpResult result;
  result.x=previous;
  if(rhs.size()==0 || lo.size()==0 || H.rows()!=rhs.size() || H.cols()!=rhs.size() ||
     A.cols()!=rhs.size() || A.rows()!=lo.size() || lo.size()!=hi.size() ||
     previous.size()!=rhs.size() || !H.allFinite() || !rhs.allFinite() ||
     !A.allFinite() || !lo.allFinite() || !hi.allFinite() || !previous.allFinite() ||
     (lo.array()>hi.array()).any()) return result;
  const auto started=std::chrono::steady_clock::now();
  const auto elapsed=[&] { return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(); };
  // Motion projection, holding plans and release have a diagonal objective.
  // Keep their well-conditioned coordinates; row normalization can instead
  // slow an almost active holding equality/inequality boundary.
  const bool diagonal_objective=H.isDiagonal(1e-12);
  int solves=0;
  for(int attempt=diagonal_objective ? 0 : -1;attempt<(diagonal_objective ? 1 : 2);++attempt)
  {
    // The final attempt is the same full optimization problem in better scaled
    // coordinates, x=T*y. A force QP's squeeze directions are weakly penalized;
    // a rho-only restart can retain poor conditioning at contact boundaries.
    Eigen::MatrixXd T=Eigen::MatrixXd::Identity(rhs.size(),rhs.size()), inverse_T=T;
    Eigen::VectorXd row_scale=Eigen::VectorXd::Ones(lo.size());
    Eigen::MatrixXd solve_H=H,solve_A=A;
    Eigen::VectorXd gradient=-rhs,lower=lo,upper=hi,warm=previous;
    std::vector<int> near_rows;
    if(attempt < 0)
    {
      const Eigen::VectorXd ax=A*previous;
      for(int i=0;i<A.rows();++i)
        if(std::min(ax[i]-lo[i],hi[i]-ax[i]) <= .1) near_rows.push_back(i);
      if(near_rows.empty() || near_rows.size() >= A.rows()/2) continue;
      // A reduced candidate is accepted only after checking every original row.
      solve_A.resize(near_rows.size(),A.cols());
      lower.resize(near_rows.size()); upper.resize(near_rows.size());
      for(int j=0;j<near_rows.size();++j)
      {
        const int i=near_rows[j];
        solve_A.row(j)=A.row(i); lower[j]=lo[i]; upper[j]=hi[i];
      }
    }
    if(attempt > 0)
    {
      Eigen::LLT<Eigen::MatrixXd> factor(H);
      if(factor.info()!=Eigen::Success) break;
      inverse_T=factor.matrixU();
      T=inverse_T.triangularView<Eigen::Upper>().solve(T);
      solve_H=T.transpose()*H*T;
      solve_A=A*T;
      gradient=-T.transpose()*rhs;
      warm=inverse_T*previous;
      for(int i=0;i<A.rows();++i)
      {
        row_scale[i]=std::max(1e-3,solve_A.row(i).norm());
        solve_A.row(i)/=row_scale[i];lower[i]/=row_scale[i];upper[i]/=row_scale[i];
      }
    }
    const double remaining=.01-elapsed();
    if(remaining<=0 || result.iterations>=4000) break;
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setRho(diagonal_objective || attempt > 0 ? .1 : .001);
    solver.settings()->setScaling(10);
    solver.settings()->setAdaptiveRho(true);
    solver.settings()->setAdaptiveRhoInterval(25);
    // A normalized residual is multiplied by row_scale in the physical units.
    // Keep that residual below half the independent 2e-5 acceptance bound.
    solver.settings()->setAbsoluteTolerance(std::min(1e-6,1e-5/row_scale.maxCoeff()));
    solver.settings()->setRelativeTolerance(0);
    solver.settings()->setMaxIteraction(diagonal_objective || attempt > 0 ? 4000-result.iterations : 250);
    solver.settings()->setPolish(true);
    solver.settings()->setPolishRefineIter(5);
    solver.settings()->setTimeLimit(remaining);
    Eigen::SparseMatrix<double> hessian=solve_H.sparseView(),constraints=solve_A.sparseView();
    solver.data()->setNumberOfVariables(rhs.size());
    solver.data()->setNumberOfConstraints(lower.size());
    if(!solver.data()->setHessianMatrix(hessian) || !solver.data()->setGradient(gradient) ||
       !solver.data()->setLinearConstraintsMatrix(constraints) ||
       !solver.data()->setLowerBound(lower) || !solver.data()->setUpperBound(upper) ||
       !solver.initSolver() || !solver.setPrimalVariable(warm)) break;
    auto* work=solver.workspace().get();
    const double solve_budget=.01-elapsed();
    if(solve_budget<=0) break;
    // OSQP 0.6 counts setup_time again in its first-run timer. elapsed()
    // already includes that setup, so add it back only in OSQP's time base.
    osqp_update_time_limit(work,work->info->setup_time+solve_budget);
    result.restarts=solves++;
    const auto code=osqp_solve(work);
    const auto& info=*work->info;
    result.status=info.status_val;
    result.iterations+=info.iter;
    result.primal=info.pri_res;
    result.stationarity=info.dua_res;
    if(code!=0) break;
    if(result.status==OSQP_SOLVED || result.status==OSQP_SOLVED_INACCURATE)
    {
      const Eigen::VectorXd x=T*solver.getSolution();
      Eigen::VectorXd dual=Eigen::VectorXd::Zero(lo.size());
      const auto& solved_dual=solver.getDualSolution();
      if(near_rows.empty()) dual=solved_dual.cwiseQuotient(row_scale);
      else for(int j=0;j<near_rows.size();++j) dual[near_rows[j]]=solved_dual[j];
      if(!x.allFinite() || !dual.allFinite()) break;
      const Eigen::VectorXd ax=A*x;
      result.violation=std::max(0.0,std::max((lo-ax).maxCoeff(),(ax-hi).maxCoeff()));
      // Undo BOTH OSQP's scaling and our row scaling for the primal residual.
      // All acceptance checks remain in the original force/acceleration units.
      result.primal=result.violation;
      for(int j=0;j<lower.size();++j)
      {
        const int i=near_rows.empty() ? j : near_rows[j];
        const double z=row_scale[i]*work->z[j]*(work->settings->scaling ? work->scaling->Einv[j] : 1);
        result.primal=std::max(result.primal,std::abs(ax[i]-z));
      }
      result.stationarity=(H*x-rhs+A.transpose()*dual).lpNorm<Eigen::Infinity>();
      result.complementarity=0;
      for(int i=0;i<dual.size();++i)
        result.complementarity=std::max(result.complementarity,
            std::abs(dual[i]*(ax[i]-(dual[i]>=0 ? hi[i] : lo[i]))));
      result.converged=result.violation<2e-5 && result.primal<2e-5 &&
          result.stationarity<1e-4 && result.complementarity<1e-4;
      if(result.converged)
      {
        result.x=x;result.seconds=elapsed();return result;
      }
    }
    else if(near_rows.empty() && result.status!=OSQP_MAX_ITER_REACHED)
      break; // Never retry an infeasibility certificate or exceed the time limit.
  }
  result.seconds=elapsed();
  // Opt-in, failure-only simulation diagnostics. The invalid force is never sent.
  if(const char* path=std::getenv("BEE_GRASP_QP_FAILURE_FILE"))
  {
    std::ofstream out(path,std::ios::app);
    out<<std::setprecision(17)<<rhs.size()<<" "<<lo.size()<<" "<<result.status<<"\n";
    out<<H<<"\n"<<rhs.transpose()<<"\n"<<A<<"\n"<<lo.transpose()<<"\n"<<hi.transpose()<<"\n"<<previous.transpose()<<"\n";
  }
  return result;
}
}
