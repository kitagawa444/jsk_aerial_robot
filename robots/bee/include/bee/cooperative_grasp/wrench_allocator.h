#pragma once
#include <bee/cooperative_grasp/grasp_state_estimator.h>
#include <limits>

namespace bee::grasp
{
struct QpResult
{
  Eigen::VectorXd x;
  double violation = std::numeric_limits<double>::infinity();
  double primal = std::numeric_limits<double>::infinity();
  double stationarity = std::numeric_limits<double>::infinity();
  double complementarity = std::numeric_limits<double>::infinity();
  double seconds = 0;
  int iterations = 0, status = 0, restarts = 0;
  bool converged = false;
};
// min .5*x'Hx - rhs'x, lo <= A*x <= hi. Accepted-solution residuals use the
// original units; failed OSQP iterates may report residuals in solver coordinates.
QpResult solveContactQp(const Eigen::MatrixXd& H, const Eigen::VectorXd& rhs,
                       const Eigen::MatrixXd& A, const Eigen::VectorXd& lo,
                       const Eigen::VectorXd& hi, const Eigen::VectorXd& previous);

bool referenceMotionValid(const Vec3& velocity, const Vec3& acceleration, const ControlConfig& p);
const std::vector<Vec3>& accelerationDirections();
double allocationAccelerationLimit(const ControlConfig& p);
bool limitObjectMotion(const std::vector<Vec3>& arms, const std::vector<Vec3>& offsets,
                              const ControlConfig& p, Vec3& acceleration, Vec3& alpha);

double requiredPreload(const Vec3& normal, double vertical_load, double mu,
                              double normal_error, double tangent_error);
bool preloadCapacityReady(const ContactState& contact, double command,
                                 double vertical_load, double mu, const ControlConfig& p);
double plannedContactCapacitySlack(const ContactState& contact, const Vec3& force,
                                          const ControlConfig& p);
bool plannedContactCapacityReady(const ContactState& contact, const Vec3& force,
                                       const ControlConfig& p);

Eigen::Matrix3d crossMatrix(const Vec3& r);
struct AllocationResult
{
  Eigen::VectorXd force;
  Wrench achieved;
  double violation = 0, primal = 0, stationarity = 0;
  double seconds = 0, complementarity = 0;
  int iterations = 0, solver_status = 0, restarts = 0;
  bool converged = false;
};
struct AllocationOptions
{
  // Planning does not assume that contact force already exists. Execution still
  // enforces the observer capacity and slew constraints below.
  bool planning = false, constrain_acceleration = false, releasing = false;
  double planning_force_ratio = 1;
  Eigen::VectorXd normal_minimum;
  // Full uncertainty while supporting; ramp down only after touchdown/unload.
  double contact_margin_scale = 1;
  double normal_command_limit = std::numeric_limits<double>::infinity();
  // Positive only for the airborne, fully supported free object. Its achieved
  // wrench changes the commanded Bee acceleration, hence the thrust constraints.
  double moving_object_mass = 0;
  Eigen::Matrix3d object_inverse_inertia = Eigen::Matrix3d::Zero();
};
// Small convex QP, OSQP with explicit unscaled feasibility and KKT checks.
// min 1/2 ||W(Gx-w)||² + .0001/2 ||x-preload||² + .0002/2 ||x-previous||²
// s.t. l <= Ax <= u. Four-sided inner friction pyramid, thrust octahedron,
// measured normal-force capacity, normal bounds, and force slew limits.
AllocationResult allocateWrench(const std::vector<ContactState>& contacts, const Vec3& origin,
    const Wrench& desired, const std::vector<Vec3>& inertia_force,
    const Eigen::VectorXd& previous, double preload, double dt, const ControlConfig& p,
    const AllocationOptions& options = {});
}  // namespace bee::grasp
