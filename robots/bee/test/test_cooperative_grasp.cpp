#include <bee/cooperative_grasp/grasp_node.h>
#include <bee/cooperative_grasp/wrench_allocator.h>
#include <bee/control/bee_controller.h>
#include <bee/cooperative_grasp/grasp_controller.h>
#include <bee/cooperative_grasp/phase_manager.h>
#include <bee/cooperative_grasp/grasp_state_estimator.h>
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

using namespace bee::grasp;
namespace
{
using ForceTrackingState = bee::control::ForceTrackingState;
// Force error is Bee-on-object minus estimated Bee-on-object. All state vectors
// have acceleration units. Filter only force PI: FF and velocity damping bypass
// this filter. Reset on contact-mode exit/re-entry; freeze I on output saturation.
Vec3 forceTrackingAcceleration(const Vec3& n, const Vec3& own_acceleration,
    const Vec3& contact_velocity_error, const Vec3& target, const Vec3& external,
    ForceTrackingState& state, double dt, const ControlConfig& p, bool integrate = true)
{
  bee::control::ForceConfig generic;
  generic.mass = p.robot_mass; generic.gravity = p.gravity;
  generic.thrust_limit = p.allocation_thrust_limit; generic.correction_reserve = p.allocation_thrust_reserve;
  generic.kp = Vec3(p.normal_force_kp, p.contact_force_kp, p.contact_force_kp);
  generic.ki = p.contact_force_ki; generic.integral_limit = p.contact_force_i_limit;
  generic.damping = p.contact_damping; generic.filter_tau = p.contact_force_filter_tau;
  Vec3 up = tangentProjection(n, Vec3::UnitZ());
  if (up.norm() < 1e-6) up = n.unitOrthogonal(); else up.normalize();
  Eigen::Matrix3d axes;
  axes.col(0) = n; axes.col(1) = up.cross(n); axes.col(2) = up;
  const Vec3 upper(p.press_limit, p.approach_limit, p.common_z_limit + p.relative_z_limit + p.tilt_limit);
  return bee::control::forceTrackingAcceleration(own_acceleration, contact_velocity_error, target, external,
      axes, -upper, upper, true, state, dt, generic, integrate);
}
}

TEST(ContactFrame, TiltedNormalKeepsSupportOrthogonal)
{
  const Vec3 n = Vec3(1, 2, -1).normalized();
  const Vec3 a = contactAcceleration(n, Vec3::Zero(), Vec3::Zero(), -2,
      Vec3(0, 0, 4), Vec3::Zero(), true, 0, 1, 2, 3, 3, 20, 20, 20);
  EXPECT_NEAR(a.dot(n), -2, 1e-12);
  EXPECT_NEAR((tangentProjection(n, a) - 2 * tangentProjection(n, Vec3::UnitZ())).norm(), 0, 1e-12);
}
TEST(ContactFrame, VerticalNormalIsFinite)
{
  const Vec3 a = contactAcceleration(Vec3::UnitZ(), Vec3::Zero(), Vec3(1, 2, 3), -2,
      Vec3::Zero(), Vec3::Zero(), true, 1, 1, 2, 3, 3, 20, 20, 20);
  EXPECT_TRUE(a.allFinite());
  EXPECT_NEAR(a.z(), -2, 1e-12);
}
TEST(ContactFrame, HorizontalNormalMatchesLegacyLoadedAxes)
{
  const Vec3 a = contactAcceleration(Vec3::UnitX(), Vec3(.1, .2, .3), Vec3(.1, .2, .3), -2,
      Vec3(0, 0, 4), Vec3(9, 0, -4), true, 1, 1, 2, 3, 3, 20, 20, 20);
  EXPECT_NEAR((a - Vec3(-1.9, .8, 3.2)).norm(), 0, 1e-12);
}
namespace
{
SceneConfig scene(int count = 3)
{
  SceneConfig s;
  s.has_destination = true;
  for (int i = 0; i < count; ++i)
  {
    s.robot_names.push_back("bee" + std::to_string(i + 1));
    s.face_angles.push_back(kPi / count + 2 * kPi * i / count);
    s.face_radii.push_back(.2);
  }
  return s;
}
InputState input(const SceneConfig& s, double clearance = .104)
{
  InputState in;
  in.object.position = s.object_position;
  in.object_stamp = 1;
  for (std::size_t i = 0; i < s.robot_names.size(); ++i)
  {
    VehicleState v;
    v.flight_state = 5;
    const double angle = s.face_angles[i];
    v.base.orientation = Eigen::AngleAxisd(angle + kPi / 2, Vec3::UnitZ()) * Eigen::AngleAxisd(kPi / 2, Vec3::UnitX());
    v.base.position = s.object_position + (s.face_radii[i] + clearance) * Vec3(std::cos(angle), std::sin(angle), 0);
    v.cog = v.base;
    v.observer_cog.force = v.cog.orientation.conjugate() *
                           Vec3(4 * std::cos(angle), 4 * std::sin(angle), -s.object_mass * aerial_robot_model::kGravity / s.robot_names.size());
    in.vehicles.push_back(v);
  }
  return in;
}
GraspState state(const SceneConfig& s, double clearance = .104)
{
  ControlConfig c;
  c.force_filter_alpha = 1;
  GraspStateEstimator estimator(s, c);
  return estimator.update(input(s, clearance));
}
GraspState approachFixture(const SceneConfig& sc, const ControlConfig& config, Phase phase)
{
  auto st = state(sc, sc.staging_clearance);
  if (phase < Phase::Orient || phase > Phase::Approach) return st;
  const double gap = phase == Phase::Approach ? 0.0 : TaskConfig{}.precontact_distance;
  for (std::size_t i = 0; i < st.contacts.size(); ++i)
  {
    auto& v = st.input.vehicles[i];
    st.contacts[i].position = footCenterTarget(sc, i, st.input.object, config.ball_radius, gap);
    v.base.position = st.contacts[i].position - v.base.orientation * config.contact_offset;
    v.cog = v.base;
  }
  return st;
}
Reference supportReference()
{
  Reference r;
  r.phase = Phase::SupportTransfer;
  r.attitude_fraction = 1;
  r.normal_force = 4;
  r.normal_force_rate = 100;
  r.begin_support = true;
  r.object_z = .95;
  return r;
}
}  // namespace
TEST(Scene, RejectsInconsistentOrNonFiniteGeometry)
{
  auto s = scene();
  s.face_radii.pop_back();
  EXPECT_THROW(s.validate(), std::invalid_argument);
  s = scene();
  s.robot_names[1] = s.robot_names[0];
  EXPECT_THROW(s.validate(), std::invalid_argument);
  s = scene();
  s.object_mass = NAN;
  EXPECT_THROW(s.validate(), std::invalid_argument);
  s = scene();
  s.object_shape = "unknown";
  EXPECT_THROW(s.validate(), std::invalid_argument);
  ControlConfig c;
  c.z_kp = NAN;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.support_ramp_duration = 0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}
TEST(Scene, EllipsoidNormalUsesGradientAndObjectRotation)
{
  auto s = scene();
  s.surface_radii = Vec3(.35, .25, .3);
  auto in = input(s);
  in.object.orientation = Eigen::AngleAxisd(.2, Vec3::UnitX());
  GraspStateEstimator estimator(s, ControlConfig{});
  const auto st = estimator.update(in);
  for (const auto& c : st.contacts)
  {
    const Vec3 local = in.object.orientation.conjugate() * (c.position - in.object.position);
    const Vec3 expected = in.object.orientation * local.cwiseQuotient(s.surface_radii.cwiseProduct(s.surface_radii)).normalized();
    EXPECT_NEAR((c.normal - expected).norm(), 0, 1e-12);
    EXPECT_NEAR(c.normal.dot(c.tangent), 0, 1e-12);
  }
  s.surface_radii.z() = 0;
  EXPECT_THROW(s.validate(), std::invalid_argument);
}
TEST(SurfaceGeometry, CylinderSideAndSupport)
{
  auto s = scene(4);
  s.object_shape = "cylinder";
  s.surface_radius = .3;
  s.validate();
  EXPECT_NEAR((surfaceNormal(s, Vec3(.3, .4, .1), Vec3::UnitX()) - Vec3(.6, .8, 0)).norm(), 0, 1e-12);
  const Vec3 n = Vec3(1, 0, 1).normalized();
  EXPECT_NEAR(surfaceSupport(s, n, .2), .6 / std::sqrt(2.0), 1e-12);
  EXPECT_THROW(surfaceNormal(s, Vec3::Zero(), Vec3::UnitX()), std::invalid_argument);
}
TEST(SurfaceGeometry, FrustumPlaneIsConsistentAtBothRims)
{
  auto s = scene(4);
  s.object_shape = "frustum";
  s.surface_radius = .34;
  s.surface_top_radius = .26;
  s.object_bottom_offset = .6 * (.34*.34 + 2*.34*.26 + 3*.26*.26) /
                           (4 * (.34*.34 + .34*.26 + .26*.26));
  s.validate();
  const Vec3 low(.34, 0, -s.object_bottom_offset);
  const Vec3 high(.26, 0, .6 - s.object_bottom_offset);
  const Vec3 normal = surfaceNormal(s, (low + high) / 2, Vec3::UnitX());
  EXPECT_GT(normal.z(), 0);
  EXPECT_NEAR((normal - Vec3(1, 0, .08/.6).normalized()).norm(), 0, 1e-12);
  EXPECT_NEAR(normal.dot(low), normal.dot(high), 1e-12);
  EXPECT_NEAR(surfaceSupport(s, normal, 0), normal.dot(low), 1e-12);
  s.contact_height = -.29;  // Inside +/-h/2, but below the CoM-relative bottom.
  EXPECT_THROW(s.validate(), std::invalid_argument);
}
TEST(SurfaceGeometry, CapsuleSideEndsAndSeam)
{
  auto s = scene(4);
  s.object_shape = "capsule";
  s.surface_radius = .25;
  s.surface_length = .4;
  s.object_height = .5;
  s.validate();
  EXPECT_NEAR((surfaceNormal(s, Vec3(0, .25, 0), Vec3::UnitX()) - Vec3::UnitY()).norm(), 0, 1e-12);
  EXPECT_NEAR((surfaceNormal(s, Vec3(.45, 0, 0), Vec3::UnitY()) - Vec3::UnitX()).norm(), 0, 1e-12);
  EXPECT_NEAR((surfaceNormal(s, Vec3(.2 - 1e-8, .25, 0), Vec3::UnitX()) -
               surfaceNormal(s, Vec3(.2 + 1e-8, .25, 0), Vec3::UnitX())).norm(), 0, 1e-6);
  const Vec3 n = Vec3(1, 2, -1).normalized();
  const Vec3 p = Vec3(.2, 0, 0) + .25 * n;
  EXPECT_NEAR(surfaceSupport(s, n, 0), n.dot(p), 1e-12);
  s.object_height = .6;
  EXPECT_THROW(s.validate(), std::invalid_argument);
}
TEST(Scene, AdditionalSurfacesRotateNormalsAndUseSupportPlane)
{
  for (const auto& shape : {"cylinder", "frustum", "capsule"})
  {
    auto s = scene(4);
    s.object_shape = shape;
    s.surface_radius = .3;
    s.surface_top_radius = .2;
    s.surface_length = .4;
    s.object_bottom_offset = .25;
    auto in = input(s);
    in.object.orientation = Eigen::AngleAxisd(.2, Vec3::UnitY());
    ControlConfig config;
    config.distributed_tracking = true;
    GraspStateEstimator estimator(s, config);
    const auto st = estimator.update(in);
    GraspController controller(s, config);
    const auto out = controller.step(st, supportReference(), .01);
    for (std::size_t i = 0; i < st.contacts.size(); ++i)
    {
      const auto& c = st.contacts[i];
      const Vec3 local = in.object.orientation.conjugate() * (c.position - in.object.position);
      const Vec3 n = surfaceNormal(s, local, Vec3::UnitX());
      EXPECT_NEAR((c.normal - in.object.orientation * n).norm(), 0, 1e-12);
      EXPECT_NEAR(c.normal.dot(c.tangent), 0, 1e-12);
      EXPECT_NEAR(out.commands[i].minimum_radius, surfaceSupport(s, n, 0) +
          config.foot_contact_offset + config.ball_radius - config.max_compression - config.radius_margin, 1e-12);
    }
  }
}
TEST(Controller, LowerSphereApproachFollowsNormalAndWaitsForLastBee)
{
  auto sc = scene();
  sc.object_shape = "sphere";
  sc.surface_radii = Vec3::Constant(.30);
  sc.contact_height = -.12;
  ControlConfig config;
  auto st = state(sc);
  const Vec3 n = st.input.object.orientation * plannedContactNormal(sc, 0);
  const Vec3 start = footCenterTarget(sc, 0, st.input.object, config.ball_radius, .2);
  const Vec3 end = footCenterTarget(sc, 0, st.input.object, config.ball_radius, 0);
  EXPECT_LT(((end - start) + .2 * n).norm(), 1e-12);
  EXPECT_NEAR((end - start).z(), .08, 1e-12);
  Reference r;
  r.phase = Phase::Approach;
  r.touch_force = .35;
  r.attitude_fraction = 1;
  r.touching = {true, false, false};
  GraspController controller(sc,config);
  const auto out = controller.step(st,r,.01);
  EXPECT_EQ(out.commands[0].z_mode,ZMode::Acceleration);
  EXPECT_EQ(out.commands[1].z_mode,ZMode::Position);
}

TEST(Phase, TouchRequiresBothProximityAndLoadOnEveryBee)
{
  const auto sc = scene();
  ControlConfig config;
  TaskConfig task;
  GraspPhaseManager phases(sc, config, task);
  ControlStatus status;
  auto st = state(sc, sc.staging_clearance);
  phases.start(st, 0);
  double now = 0;
  while (now < 20 && phases.phase() != Phase::Approach)
  {
    st = approachFixture(sc, config, phases.phase());
    phases.step(st, status, now += .05);
  }
  ASSERT_EQ(phases.phase(), Phase::Approach);
  st = approachFixture(sc, config, Phase::Approach);
  st.contacts[0].position += task.precontact_distance * st.contacts[0].normal;
  st.contacts[1].normal_force = 0;
  auto ref = phases.step(st, status, now += .05);
  ASSERT_EQ(ref.touching.size(), 3u);
  EXPECT_FALSE(ref.touching[0]);
  EXPECT_FALSE(ref.touching[1]);
  EXPECT_TRUE(ref.touching[2]);
  phases.step(st, status, now += .5);
  EXPECT_EQ(phases.phase(), Phase::Approach);
  st = approachFixture(sc, config, Phase::Approach);
  phases.step(st, status, now += .05);
  phases.step(st, status, now += .15);
  EXPECT_EQ(phases.phase(), Phase::Preload);
}

TEST(Controller, ApproachDoesNotFollowOffSurfaceGradient)
{
  auto s = scene();
  s.surface_radii = Vec3(.35, .25, .3);
  auto st = state(s, .8);
  GraspController controller(s, {});
  Reference r;
  r.phase = Phase::Approach;
  r.radius_clearance = .8;
  r.object_z = s.object_position.z();
  const auto a = controller.step(st, r, .01);
  for (auto& c : st.contacts) { c.normal = Vec3::UnitY(); c.tangent = Vec3::UnitX(); }
  const auto b = controller.step(st, r, .01);
  for (std::size_t i = 0; i < a.commands.size(); ++i)
  {
    EXPECT_NEAR((a.commands[i].acceleration - b.commands[i].acceleration).norm(), 0, 1e-12);
    EXPECT_DOUBLE_EQ(a.commands[i].baselink_rpy.z(), b.commands[i].baselink_rpy.z());
  }
}
TEST(Controller, InclinedContactsAccountForPreloadVerticalForce)
{
  auto s = scene();
  ControlConfig config;
  config.distributed_tracking = true;
  GraspController controller(s, config);
  auto st = state(s);
  for (auto& c : st.contacts) c.normal = (c.normal + Vec3(0, 0, -.2)).normalized();
  auto r = supportReference();
  const auto out = controller.step(st, r, .01);
  double support = 0;
  for (const auto& cmd : out.commands)
  {
    EXPECT_NEAR(cmd.support_vector.dot(cmd.normal), 0, 1e-12);
    support += (-cmd.normal_force * cmd.normal + cmd.support_vector).z();
  }
  EXPECT_NEAR(support, out.status.vertical_force_target, 1e-10);
}
TEST(Controller, SupportTransferHoldsCapturedHeight)
{
  auto s = scene();
  auto st = state(s);
  ControlConfig config;
  config.distributed_tracking = true;
  GraspController controller(s, config);
  auto reference = supportReference();
  const double initial = st.input.object.position.z();
  controller.step(st, reference, .01);
  st.input.object.position.z() += .02;
  const auto output = controller.step(st, reference, .01);
  EXPECT_NEAR(output.status.reference_z, initial, 1e-12);
  double mean_vz = 0;
  for (const auto& command : output.commands) mean_vz += command.velocity.z();
  EXPECT_LT(mean_vz, 0);
}
TEST(Safety, SupportTransferChecksHeightAndSlip)
{
  auto s = scene();
  auto st = state(s);
  auto reference = supportReference();
  reference.object_z = st.input.object.position.z();
  ControlOutput output;
  output.status.reference_z = st.input.object.position.z();
  output.commands.resize(3);
  GraspSafetyMonitor monitor({});
  st.input.object.position.z() += .04;
  EXPECT_EQ(monitor.check(st, reference, output, .01), "support transfer height drift limit");
  st.input.object.position.z() -= .04;
  output.status.contact_geometry_errors.assign(3, Vec3(0, 0, .10));
  EXPECT_EQ(monitor.check(st, reference, output, .01), "mean contact slip limit");
}
TEST(Estimator, ActionReactionAndMomentTransport)
{
  auto s = scene();
  auto in = input(s);
  ControlConfig c;
  c.force_filter_alpha = 1;
  GraspStateEstimator estimator(s, c);
  auto out = estimator.update(in);
  EXPECT_NEAR(out.object_wrench.force.x(), 0, 1e-12);
  EXPECT_NEAR(out.object_wrench.force.y(), 0, 1e-12);
  EXPECT_NEAR(out.object_wrench.force.z(), s.object_mass * c.gravity, 1e-12);
  // Shifting a wrench's reference point must preserve its moment about origin.
  for (std::size_t i = 0; i < in.vehicles.size(); ++i)
  {
    const auto& contact = out.contacts[i];
    const Vec3 moment_origin = contact.bee_world.moment + contact.position.cross(contact.bee_world.force);
    const Vec3 expected = in.vehicles[i].cog.orientation * in.vehicles[i].observer_cog.moment +
                          in.vehicles[i].cog.position.cross(contact.bee_world.force);
    EXPECT_LT((moment_origin - expected).norm(), 1e-12);
  }
}
TEST(Estimator, CalibrationRemovesConstantBias)
{
  auto s = scene();
  auto in = input(s);
  GraspStateEstimator estimator(s, {});
  EXPECT_FALSE(estimator.finishCalibration());
  estimator.addCalibrationSample(in);
  ASSERT_TRUE(estimator.finishCalibration());
  const auto out = estimator.update(in);
  EXPECT_LT(out.object_wrench.force.norm(), 1e-12);
  EXPECT_LT(out.object_wrench.moment.norm(), 1e-12);
  for (const auto& c : out.contacts)
    EXPECT_NEAR(c.normal_force, 0, 1e-12);
}
TEST(Estimator, PerBeeCalibrationUpdatesForceAndMomentWithoutChangingOtherBees)
{
  const auto s = scene();
  auto in = input(s);
  in.vehicles[0].observer_cog.moment = Vec3(.1, -.2, .3);
  GraspStateEstimator estimator(s, {});
  const auto before = estimator.update(in);
  const auto baseline = in.vehicles[0].observer_cog;
  const Vec3 world = in.vehicles[0].cog.orientation * baseline.force;
  ASSERT_TRUE(estimator.applyCalibration(0, baseline, world));
  const auto after = estimator.update(in);
  EXPECT_LT(after.contacts[0].bee_world.force.norm(), 1e-12);
  EXPECT_LT(after.contacts[0].bee_world.moment.norm(), 1e-12);
  EXPECT_NEAR(after.contacts[0].normal_force, 0, 1e-12);
  EXPECT_LT((after.contacts[1].bee_world.force - before.contacts[1].bee_world.force).norm(), 1e-12);
  Wrench invalid = baseline;
  invalid.moment.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimator.applyCalibration(0, invalid, world));
  EXPECT_FALSE(estimator.applyCalibration(3, baseline, world));
  EXPECT_LT(estimator.update(in).contacts[0].bee_world.force.norm(), 1e-12);
  // A later load is measured relative to the successfully applied baseline.
  in.vehicles[0].observer_cog.force += Vec3(1, 0, 0);
  EXPECT_LT((estimator.update(in).contacts[0].bee_world.force -
             in.vehicles[0].cog.orientation * Vec3(1, 0, 0)).norm(), 1e-12);
}
TEST(Estimator, RepeatedPoseStampDoesNotChangeVelocity)
{
  auto s = scene();
  auto in = input(s);
  GraspStateEstimator estimator(s, {});
  estimator.update(in);
  in.object_stamp += .1;
  in.object.position.x() += .1;
  const auto a = estimator.update(in), b = estimator.update(in);
  EXPECT_NEAR(a.object_velocity.x(), .2, 1e-12);
  EXPECT_LT((a.object_velocity - b.object_velocity).norm(), 1e-12);
}
TEST(Controller, PausedTimeRetainsSupportAndIntegral)
{
  auto s = scene();
  auto st = state(s);
  GraspController controller(s, {});
  auto r = supportReference();
  ControlOutput a;
  for (int i = 0; i < 400; ++i)
    a = controller.step(st, r, .01);
  ASSERT_TRUE(a.status.support_ready);
  r.begin_support = false;
  r.phase = Phase::Lift;
  r.object_z = 1;
  for (int i = 0; i < 100; ++i)
    a = controller.step(st, r, .01);
  const auto b = controller.step(st, r, 0);
  EXPECT_GT(a.status.z_integral, 0);
  EXPECT_DOUBLE_EQ(a.status.z_integral, b.status.z_integral);
  EXPECT_DOUBLE_EQ(a.status.common_z_acceleration, b.status.common_z_acceleration);
}
TEST(Controller, LiftHoldTransportPreserveVerticalMemory)
{
  auto s = scene();
  auto st = state(s);
  GraspController controller(s, {});
  auto r = supportReference();
  ControlOutput a;
  for (int i = 0; i < 400; ++i)
    a = controller.step(st, r, .01);
  r.begin_support = false;
  r.phase = Phase::Lift;
  r.object_z = 1;
  for (int i = 0; i < 100; ++i)
    a = controller.step(st, r, .01);
  for (auto phase : { Phase::Hold, Phase::Transport })
  {
    r.phase = phase;
    const auto b = controller.step(st, r, 0);
    EXPECT_DOUBLE_EQ(a.status.z_integral, b.status.z_integral);
    EXPECT_DOUBLE_EQ(a.status.reference_z, b.status.reference_z);
    EXPECT_DOUBLE_EQ(a.status.common_z_acceleration, b.status.common_z_acceleration);
  }
  r.phase = Phase::Unload;
  r.touchdown = true;
  EXPECT_DOUBLE_EQ(controller.step(st, r, .01).status.z_integral, 0);
}
TEST(Controller, DifferentialVerticalCommandsPreserveCommonSupport)
{
  for (int n : { 3, 4, 5 })
  {
    auto s = scene(n);
    auto st = state(s);
    GraspController controller(s, {});
    st.contacts[0].relative_z += .01;
    auto r = supportReference();
    auto out = controller.step(st, r, .01);
    double mean = 0;
    for (const auto& cmd : out.commands)
      mean += cmd.acceleration.z() / n;
    EXPECT_NEAR(mean, out.status.common_z_acceleration, 1e-12);
  }
}
TEST(Phase, CompleteAutonomousSequenceAndLatchedAbort)
{
  auto s = scene();
  ControlConfig c;
  TaskConfig t;
  GraspPhaseManager phases(s, c, t);
  auto st = state(s, s.staging_clearance);
  ControlStatus status;
  double now = 10;
  phases.start(st, now);
  for (int step = 0; step < 10000 && phases.phase() != Phase::Complete; ++step)
  {
    now += .05;
    const Phase phase = phases.phase();
    double clearance = s.staging_clearance;
    if (phase >= Phase::Approach)
      clearance = .104;
    if (phase >= Phase::Retreat)
      clearance = c.ball_radius + t.retreat_clearance;
    st = (phase >= Phase::Orient && phase <= Phase::Approach) ? approachFixture(s, c, phase) : state(s, clearance);
    status.normal_force = phase >= Phase::Release ? 0 : phases.normalForceTarget();
    status.normal_force_targets = {status.normal_force - .6, status.normal_force + .6, status.normal_force};
    if (phase == Phase::Preload)
      for (std::size_t i = 0; i < st.contacts.size(); ++i)
        st.contacts[i].normal_force = .95 * status.normal_force_targets[i];
    status.support_ready = true;
    status.reference_z = s.object_position.z() + t.lift_height;
    if (phase >= Phase::Lift && phase < Phase::Descend)
      st.input.object.position.z() += t.lift_height;
    if (phase >= Phase::Transport)
      st.input.object.position.head<2>() = s.destination_position.head<2>();
    // Preserve formation around the moved object in the synthetic trace.
    const Vec3 translation = st.input.object.position - s.object_position;
    for (std::size_t i = 0; i < st.contacts.size(); ++i)
    {
      st.contacts[i].position += translation;
      st.input.vehicles[i].base.position += translation;
      if (phase >= Phase::Release)
        st.contacts[i].normal_force = 0;
    }
    phases.step(st, status, now);
    ASSERT_NE(phases.phase(), Phase::Aborted) << phases.fault();
  }
  EXPECT_EQ(phases.phase(), Phase::Complete);
  phases.abort("test fault");
  phases.step(st, status, now + 1);
  EXPECT_EQ(phases.phase(), Phase::Aborted);
  EXPECT_THROW(phases.start(st, now), std::logic_error);
}
TEST(Phase, TimeoutAndMissingPreloadDoNotAdvance)
{
  auto s = scene();
  TaskConfig task;
  task.staging_timeout = 1;
  GraspPhaseManager phases(s, {}, task);
  auto st = state(s);
  phases.start(st, 0);
  phases.step(st, {}, 2);
  EXPECT_EQ(phases.phase(), Phase::Aborted);
}
TEST(Safety, RejectsNonFiniteCommandsAndLeavingHover)
{
  auto s = scene();
  auto st = state(s);
  GraspSafetyMonitor monitor({});
  Reference r;
  ControlOutput out;
  out.commands.resize(3);
  out.commands[0].z = NAN;
  EXPECT_FALSE(monitor.check(st, r, out, .01).empty());
  out.commands[0].z = 0;
  st.input.vehicles[0].flight_state = 6;
  EXPECT_FALSE(monitor.check(st, r, out, .01).empty());
}
TEST(Safety, SustainedContactLossAndDrop)
{
  auto s = scene();
  auto st = state(s);
  GraspSafetyMonitor monitor({});
  Reference r;
  r.phase = Phase::Transport;
  r.object_z = st.input.object.position.z();
  ControlOutput out;
  out.commands.resize(3);
  for (auto& c : st.contacts)
    c.normal_force = 0;
  for (int i = 0; i < 20; ++i)
    EXPECT_TRUE(monitor.check(st, r, out, .01).empty());
  for (int i = 0; i < 10; ++i)
    monitor.check(st, r, out, .01);
  EXPECT_FALSE(monitor.check(st, r, out, .01).empty());
  GraspSafetyMonitor drop_monitor({});
  st.input.object.position.z() -= .04;
  EXPECT_FALSE(drop_monitor.check(st, r, out, .01).empty());
}
#ifdef BEE_TEST_DATA_DIR
TEST(ContactAttitude, FeetFaceThePlannedLowerSphereContact)
{
  auto sc = scene();
  sc.object_shape = "sphere";
  sc.surface_radii = Vec3::Constant(.3);
  sc.contact_height = -.12;
  const Vec3 point = plannedContactPoint(sc, 0);
  EXPECT_NEAR(point.norm(), .3, 1e-12);
  const Vec3 normal = plannedContactNormal(sc, 0);
  EXPECT_NEAR(normal.z(), -.4, 1e-12);
  const Eigen::Quaterniond object(Eigen::AngleAxisd(.35, Vec3::UnitY()));
  const Vec3 world_normal = object * normal;
  const Vec3 rpy = contactAttitude(world_normal, 1, 0);
  const Eigen::Quaterniond target = Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) *
                                    Eigen::AngleAxisd(rpy.x(), Vec3::UnitX());
  EXPECT_LT((target * Vec3::UnitZ() - world_normal).norm(), 1e-12);
  EXPECT_NEAR(contactAttitude(Vec3::UnitX(), 1, 0).x(), kPi / 2, 1e-12);
}

TEST(LegacyComparison, PressAccelerationFromPythonReference)
{
  std::ifstream file(std::string(BEE_TEST_DATA_DIR) + "/grasp_press_reference.csv");
  ASSERT_TRUE(file.good());
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line))
  {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double object_yaw, normal_force, tangent_error, ax, ay, target_yaw;
    ASSERT_TRUE(bool(row >> object_yaw >> normal_force >> tangent_error >> ax >> ay >> target_yaw));
    auto sc = scene();
    auto st = state(sc);
    const double angle = sc.face_angles[0] + object_yaw;
    auto& contact = st.contacts[0];
    contact.normal = Vec3(std::cos(angle), std::sin(angle), 0);
    contact.tangent = Vec3(-std::sin(angle), std::cos(angle), 0);
    contact.normal_force = normal_force;
    contact.tangent_position = tangent_error;
    contact.radial_velocity = .02;
    contact.tangent_velocity = -.03;
    st.input.vehicles[0].base.position = st.input.object.position + .304 * contact.normal;
    ControlConfig config;
    config.moment_force_threshold = 100;  // isolate the force servo
    GraspController controller(sc, config);
    Reference r;
    r.phase = Phase::Preload;
    r.normal_force = 4;
    r.normal_force_rate = 100;
    const auto out = controller.step(st, r, .05);
    EXPECT_NEAR(out.commands[0].acceleration.x(), ax, 1e-12);
    EXPECT_NEAR(out.commands[0].acceleration.y(), ay, 1e-12);
    EXPECT_NEAR(angleError(out.commands[0].baselink_rpy.z() - target_yaw), 0, 1e-12);
  }
}
TEST(LegacyComparison, VerticalTrajectoryFromPythonReference)
{
  std::ifstream file(std::string(BEE_TEST_DATA_DIR) + "/grasp_trajectory_reference.csv");
  ASSERT_TRUE(file.good());
  std::string line;
  std::getline(file, line);
  auto sc = scene();
  auto st = state(sc);
  GraspController controller(sc, {});
  auto r = supportReference();
  ControlOutput out;
  for (int i = 0; i < 400; ++i)
    out = controller.step(st, r, .01);
  ASSERT_TRUE(out.status.support_ready);
  r.begin_support = false;
  r.phase = Phase::Lift;
  while (std::getline(file, line))
  {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double dt, z, vz, az;
    ASSERT_TRUE(bool(row >> r.object_z >> dt >> z >> vz >> az));
    out = controller.step(st, r, dt);
    EXPECT_NEAR(out.status.reference_z, z, 1e-12);
    EXPECT_NEAR(out.status.reference_vz, vz, 1e-12);
    EXPECT_NEAR(out.status.reference_az, az, 1e-12);
  }
}
#endif

TEST(SurfaceGeometry, ConvexRockUsesPlannedFaceAndExactSupport)
{
  auto sc = scene();
  sc.object_shape = "rock";
  sc.contact_normals = {Vec3(1, 0, .1).normalized(), Vec3(0, 1, .2).normalized(),
                        Vec3(-1, 0, .05).normalized()};
  sc.surface_vertices = {Vec3(.3, .2, -.3), Vec3(-.3, .2, -.3),
                         Vec3(0, -.3, -.3), Vec3(.02, .01, .3)};
  EXPECT_NO_THROW(sc.validate());
  const Vec3 normal = surfaceNormal(sc, Vec3(1, 0, .1), Vec3::UnitX());
  EXPECT_NEAR((normal - sc.contact_normals[0]).norm(), 0, 1e-12);
  EXPECT_NEAR((surfaceNormal(sc, Vec3(1, 0, -.2), Vec3::UnitX()) - normal).norm(), 0, 1e-12);
  EXPECT_NEAR(surfaceSupport(sc, Vec3::UnitZ(), 123), .3, 1e-12);
  EXPECT_NEAR(surfaceSupport(sc, Vec3(1, 0, 1), 123), .32, 1e-12);
  sc.contact_normals.pop_back();
  EXPECT_THROW(sc.validate(), std::invalid_argument);
  sc.surface_vertices.clear();
  EXPECT_THROW(surfaceSupport(sc, Vec3::UnitZ(), 0), std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(WrenchAllocation, SupportsPayloadAndMomentWithinFriction)
{
  ControlConfig p;
  std::vector<ContactState> contacts(3);
  std::vector<Vec3> own(3,Vec3(0,0,p.robot_mass*p.gravity));
  Eigen::VectorXd previous(9);
  for (int i=0;i<3;++i)
  {
    const double angle=2*kPi*i/3;
    contacts[i].normal=Vec3(cos(angle),sin(angle),0);
    contacts[i].position=.2*contacts[i].normal;
    contacts[i].normal_force=4;
    previous.segment<3>(3*i)=-4*contacts[i].normal;
  }
  Wrench desired; desired.force=Vec3(0,0,4.905); desired.moment=Vec3(.005,-.005,.001);
  auto result=allocateWrench(contacts,Vec3::Zero(),desired,own,previous,4,1,p);
  ASSERT_TRUE(result.converged);
  EXPECT_LT(result.violation,1e-4);
  EXPECT_LT((result.achieved.force-desired.force).norm(),.06);
  EXPECT_LT((result.achieved.moment-desired.moment).norm(),.002);
  for (int i=0;i<3;++i)
  {
    const Vec3 f=result.force.segment<3>(3*i);
    EXPECT_GE(-f.dot(contacts[i].normal),4-1e-4);
    EXPECT_LE(tangentProjection(contacts[i].normal,f).norm(),p.allocation_mu*(-f.dot(contacts[i].normal))+1e-4);
  }
}
TEST(WrenchAllocation, UnachievableWrenchIsReportedAsResidual)
{
  ControlConfig p; p.allocation_mu=.45; std::vector<ContactState> contacts(1);
  contacts[0].normal=Vec3::UnitX(); contacts[0].normal_force=2;
  Eigen::VectorXd previous(3);previous<<-2,0,0;
  Wrench target;target.force=Vec3(-2,0,20);
  auto result=allocateWrench(contacts,Vec3::Zero(),target,{Vec3(0,0,18)},previous,2,1,p);
  ASSERT_TRUE(result.converged);
  EXPECT_LT(result.violation,1e-4);
  EXPECT_LT(result.achieved.force.z(),1);
  EXPECT_GT((target.force-result.achieved.force).norm(),19);
}
TEST(WrenchTracking, ForceErrorSignAndOwnAcceleration)
{
  ControlConfig p;p.contact_force_ki=0;p.contact_damping=0;
  ForceTrackingState integral;const Vec3 target(-4,0,2), external(4,0,-2), own(.1,0,.2);
  const Vec3 exact=forceTrackingAcceleration(Vec3::UnitX(),own,Vec3::Zero(),target,external,integral,.01,p);
  EXPECT_NEAR((exact-own-target/p.robot_mass).norm(),0,1e-10);
  const Vec3 low=forceTrackingAcceleration(Vec3::UnitX(),own,Vec3::Zero(),target,Vec3(4,0,-1.5),integral,.01,p);
  EXPECT_GT(low.z(),exact.z());
  const Vec3 high=forceTrackingAcceleration(Vec3::UnitX(),own,Vec3::Zero(),target,Vec3(4,0,-2.5),integral,.01,p);
  EXPECT_LT(high.z(),exact.z());
}
TEST(WrenchTracking, IntegralFreezesAtAccelerationLimit)
{
  ControlConfig p; ForceTrackingState integral;
  auto acceleration=forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),
      Vec3(-4,0,100),Vec3::Zero(),integral,.01,p);
  EXPECT_DOUBLE_EQ(integral.integral.norm(),0);
  EXPECT_LE(acceleration.z(),p.common_z_limit+p.relative_z_limit+p.tilt_limit+1e-9);
}

TEST(WrenchAllocation, UnequalPreloadTransitionRespectsSlew)
{
  ControlConfig p;
  std::vector<ContactState> contacts(3);
  std::vector<Vec3> own(3,Vec3(0,0,p.robot_mass*p.gravity));
  Eigen::VectorXd previous(9);
  for (int i=0;i<3;++i)
  {
    const double a=kPi/3+2*kPi*i/3;
    contacts[i].normal=Vec3(cos(a),sin(a),0);
    contacts[i].position=.174*contacts[i].normal;
    contacts[i].normal_force=3.924;
    previous.segment<3>(3*i)=-(3.324+.6*i)*contacts[i].normal;
  }
  Wrench desired;desired.force=Vec3(0,0,.05);
  const auto result=allocateWrench(contacts,Vec3::Zero(),desired,own,previous,3.924,.01,p);
  ASSERT_TRUE(result.converged);
  EXPECT_LT(result.violation,1e-4);
  EXPECT_LE((result.force-previous).lpNorm<Eigen::Infinity>(),.2001);
}
TEST(WrenchController, GroundedTransferDoesNotCommandFreeFall)
{
  auto sc=scene(); ControlConfig p;p.distributed_tracking=true;p.wrench_control=true;
  auto s=state(sc);GraspController controller(sc,p);
  // A completed approach aligns the contact with the face ray. This fixture
  // tests transfer/release, not an uncorrected 17.5 mm tangential staging offset.
  for(auto& c:s.contacts)c.position=s.input.object.position+.2*c.normal;
  Reference r=supportReference();
  s.object_wrench.moment=Vec3(.1,.1,0);
  // Establish unequal preload first, then switch allocation on the pedestal.
  r.begin_support=false;r.phase=Phase::Preload;
  for (int i=0;i<50;i++) controller.step(s,r,.01);
  r=supportReference();
  const auto result=controller.step(s,r,.01);
  ASSERT_TRUE(result.status.fault.empty()) << result.status.fault;
  for (const auto& c:result.commands)
  {
    EXPECT_TRUE(c.wrench_tracking);
    EXPECT_DOUBLE_EQ(c.observer_blend,0);
    EXPECT_LT(c.acceleration.norm(),.2);
  }
}
TEST(WrenchController, DownwardPreloadTransferStartsContinuouslyAndWaitsForObservedForce)
{
  auto sc=scene(4);sc.object_mass=.5;
  ControlConfig p;p.distributed_tracking=p.wrench_control=true;
  p.allocation_normal_error=p.allocation_observer_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  auto st=state(sc);
  for(auto& c:st.contacts)
  {
    c.normal=(c.normal+Vec3(0,0,.133333)).normalized();
    const Vec3 radial=Vec3(c.normal.x(),c.normal.y(),0).normalized();
    c.position=st.input.object.position+.3*radial;
    c.normal_force=7;
  }
  GraspController controller(sc,p);
  Reference r=supportReference();r.phase=Phase::Preload;r.begin_support=false;
  r.preload_vertical_force=sc.object_mass*(p.gravity+p.reference_acceleration_limit);
  ControlOutput out;
  for(int i=0;i<100;++i)out=controller.step(st,r,.01);
  ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
  Vec3 preload=Vec3::Zero();
  for(std::size_t i=0;i<out.commands.size();++i)
  {
    preload-=out.commands[i].normal_force*st.contacts[i].normal;
    st.contacts[i].normal_force=out.commands[i].normal_force;
  }
  ASSERT_LT(preload.z(),-1);
  st.object_wrench.force=preload;
  r=supportReference();
  out=controller.step(st,r,.01);
  ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
  EXPECT_GT(out.status.vertical_force_target,preload.z());
  EXPECT_LT(out.status.vertical_force_target,preload.z()+.1);
  // An unresponsive force observer must stop progress within the existing
  // force lead, even while both target and measurement remain negative.
  for(int i=0;i<200;++i)
  {
    out=controller.step(st,r,.01);
    ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
    EXPECT_LE(out.status.vertical_force_target,preload.z()+p.support_force_lead+1e-6);
    EXPECT_FALSE(out.status.support_ready);
  }
  // Resume measured tracking with lag. Crossing zero must not deadlock the
  // gate; full support still requires measured weight and the settling dwell.
  for(int i=0;i<1000 && !out.status.support_ready;++i)
  {
    Vec3 observed=Vec3::Zero();
    for(std::size_t j=0;j<out.commands.size();++j)
    {
      const auto& c=out.commands[j];
      observed+=-c.normal_force*c.normal+c.support_vector;
      st.contacts[j].normal_force=c.normal_force;
    }
    st.object_wrench.force+=.01/(.2+.01)*(observed-st.object_wrench.force);
    out=controller.step(st,r,.01);
    ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
  }
  EXPECT_TRUE(out.status.support_ready);
  EXPECT_NEAR(out.status.vertical_force_target,sc.object_mass*p.gravity,1e-6);
  st.object_velocity.z() = .01;
  out = controller.step(st, r, .01);
  ASSERT_TRUE(out.status.fault.empty()) << out.status.fault;
  EXPECT_NEAR(out.status.vertical_force_target, sc.object_mass * p.gravity -
              (sc.object_mass + sc.robot_names.size() * p.robot_mass) * p.object_kd * .01, 1e-6);
}
TEST(WrenchController, MovingDescentReferenceHasConsistentAcceleration)
{
  auto sc=scene();ControlConfig p;p.distributed_tracking=true;p.wrench_control=true;
  auto s=state(sc);GraspController controller(sc,p);
  // A completed approach aligns the contact with the face ray. This fixture
  // tests transfer/release, not an uncorrected 17.5 mm tangential staging offset.
  for(auto& c:s.contacts)c.position=s.input.object.position+.2*c.normal;
  { Reference preload=supportReference(); preload.phase=Phase::Preload; preload.begin_support=false;
    for(int i=0;i<100;++i) controller.step(s,preload,.01); }
  Reference r=supportReference();
  ControlOutput output;
  for(int i=0;i<250;i++) output=controller.step(s,r,.01);
  ASSERT_TRUE(output.status.support_ready);
  r.phase=Phase::Lift;r.begin_support=false;r.object_z=sc.object_position.z()+.05;
  for(int i=0;i<800;i++) {output=controller.step(s,r,.01);s.input.object.position.z()=output.status.reference_z;}
  r.phase=Phase::Descend;r.z_velocity_limit=.005;
  double last_v=output.status.reference_vz, integral_a=0;
  const double initial_v=last_v;
  for(int i=0;i<500;i++)
  {
    r.object_z-=.005*.01;
    output=controller.step(s,r,.01);
    EXPECT_NEAR(output.status.reference_az,(output.status.reference_vz-last_v)/.01,1e-9);
    EXPECT_LE(std::abs(output.status.reference_az),p.trajectory_acceleration+1e-9);
    integral_a+=output.status.reference_az*.01;last_v=output.status.reference_vz;
    s.input.object.position.z()=output.status.reference_z;
  }
  EXPECT_NEAR(integral_a,last_v-initial_v,1e-9);
  EXPECT_NEAR(last_v,-.005,1e-4);
}
TEST(WrenchTracking, TransferDoesNotWindUpForceIntegral)
{
  ControlConfig p;ForceTrackingState integral;integral.integral.z()=.2;
  for(int i=0;i<100;i++)
    forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),
        Vec3(-4,0,1),Vec3(4,0,-.5),integral,.01,p,false);
  EXPECT_DOUBLE_EQ(integral.integral.norm(),0);
  forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),
      Vec3(-4,0,1),Vec3(4,0,-.5),integral,.01,p,true);
  EXPECT_GT(integral.integral.z(),0);
}
TEST(WrenchController, ReleaseEliminatesInternalSqueeze)
{
  auto sc=scene();ControlConfig p;p.distributed_tracking=true;p.wrench_control=true;
  p.allocation_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  p.allocation_observer_normal_error=.5;
  auto s=state(sc);GraspController controller(sc,p);
  for (auto& c:s.contacts) c.normal_force=5;
  // A completed approach aligns the contact with the face ray. This fixture
  // tests transfer/release, not an uncorrected 17.5 mm tangential staging offset.
  for(auto& c:s.contacts)c.position=s.input.object.position+.2*c.normal;
  { Reference preload=supportReference(); preload.phase=Phase::Preload; preload.begin_support=false;
    for(int i=0;i<100;++i) controller.step(s,preload,.01); }
  Reference r=supportReference();ControlOutput out;
  for(int i=0;i<250;i++)out=controller.step(s,r,.01);
  ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
  r.phase=Phase::Release;r.begin_support=false;r.support_scale=0;r.normal_force=0;r.normal_force_rate=.8;
  double release_time=0;
  for(const auto& c:out.commands) release_time=std::max(release_time,c.normal_force/r.normal_force_rate);
  for(int i=0;i<int((release_time+.5)/.01);i++)
  {
    out=controller.step(s,r,.01);
    ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
    for(const auto& c:out.commands)EXPECT_GE(c.normal_force,0);
  }
  for(const auto& c:out.commands)
  {
    EXPECT_LT(c.normal_force,.001);
    EXPECT_LT(c.support_vector.norm(),.001);

  }
}

TEST(WrenchAllocation, EveryCornerOfThreeDimensionalForceErrorRemainsAdmissible)
{
  ControlConfig p;
  p.allocation_normal_error=.5;p.allocation_tangent_up_error=.13;p.allocation_tangent_side_error=.07;
  p.allocation_observer_normal_error=.5;
  std::vector<ContactState> contacts(1);
  auto& c=contacts[0];c.normal=Vec3(1,2,.3).normalized();c.normal_force=7;
  const Vec3 up=tangentProjection(c.normal,Vec3::UnitZ()).normalized();
  const Vec3 side=up.cross(c.normal);
  Eigen::VectorXd previous=-5*c.normal;
  Wrench desired;desired.force=-12*c.normal+3*up+2*side;
  const auto out=allocateWrench(contacts,Vec3::Zero(),desired,{Vec3(0,0,18)},previous,4,1,p);
  ASSERT_TRUE(out.converged);ASSERT_LT(out.violation,1e-4);
  const Vec3 command=out.force;
  for(int a:{-1,1})for(int b:{-1,1})for(int d:{-1,1})
  {
    const Vec3 actual=command-a*p.allocation_normal_error*c.normal+
        b*p.allocation_tangent_up_error*up+d*p.allocation_tangent_side_error*side;
    const double normal=-actual.dot(c.normal);
    EXPECT_GE(normal,4-1e-4);
    EXPECT_LE(normal,p.max_normal_force+1e-4);
    EXPECT_LE(std::abs(actual.dot(up))+std::abs(actual.dot(side)),p.allocation_mu*normal+1e-4);
    EXPECT_LE(std::abs(actual.dot(up))+std::abs(actual.dot(side)),
        p.allocation_mu*(c.normal_force-p.allocation_observer_normal_error)+1e-4);
  }
}

TEST(WrenchAllocation, InsufficientMeasuredContactCapacityIsNotClippedIntoFeasibility)
{
  ControlConfig p;p.allocation_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  p.allocation_observer_normal_error=.5;
  std::vector<ContactState> contacts(1);contacts[0].normal=Vec3::UnitX();contacts[0].normal_force=.6;
  Eigen::VectorXd previous(3);previous<<-4,0,0;
  Wrench target;target.force=previous;
  const auto out=allocateWrench(contacts,Vec3::Zero(),target,{Vec3(0,0,18)},previous,4,1,p);
  EXPECT_TRUE(!out.converged || out.violation>1e-4);
}

TEST(WrenchAllocation, CorrectedOwnAccelerationStillLeavesTheFeedbackReserve)
{
  ControlConfig p;p.allocation_thrust_limit=27;p.allocation_thrust_reserve=3;
  std::vector<ContactState> contacts(3);
  std::vector<Vec3> own(3,Vec3(0,0,p.robot_mass*p.gravity));
  Eigen::VectorXd previous(9);
  for(int i=0;i<3;++i)
  {
    const double a=2*kPi*i/3;
    contacts[i].normal=Vec3(cos(a),sin(a),0);contacts[i].position=.2*contacts[i].normal;
    contacts[i].normal_force=6;previous.segment<3>(3*i)=-4*contacts[i].normal;
  }
  AllocationOptions options;options.moving_object_mass=.5;
  options.object_inverse_inertia=Eigen::Matrix3d::Identity()/.02;
  Wrench desired;desired.force=Vec3(.5,0,6);desired.moment=Vec3(.01,-.01,.02);
  const auto out=allocateWrench(contacts,Vec3::Zero(),desired,own,previous,4,1,p,options);
  ASSERT_TRUE(out.converged);ASSERT_LT(out.violation,1e-4);
  const Vec3 da=(out.achieved.force-desired.force)/options.moving_object_mass;
  const Vec3 dalpha=options.object_inverse_inertia*(out.achieved.moment-desired.moment);
  bool near_limit=false;
  for(int i=0;i<3;++i)
  {
    const Vec3 thrust=own[i]+out.force.segment<3>(3*i)+
        p.robot_mass*(da+dalpha.cross(contacts[i].position));
    EXPECT_LE(thrust.lpNorm<1>(),p.allocation_thrust_limit-p.allocation_thrust_reserve+1e-3);
    near_limit=near_limit || thrust.lpNorm<1>()>p.allocation_thrust_limit-p.allocation_thrust_reserve-.01;
  }
  EXPECT_TRUE(near_limit);
}

TEST(WrenchTracking, CombinedFeedbackUsesReservedForceAndFreezesIntegral)
{
  ControlConfig p;p.press_limit=p.approach_limit=p.common_z_limit=20;
  p.contact_damping=3;
  const Vec3 own(.02,.01,.03),target(-4,.1,1.6);
  ForceTrackingState integral;
  const Vec3 acceleration=forceTrackingAcceleration(Vec3::UnitX(),own,Vec3(.5,-.4,.3),target,
      Vec3(2,1,-.5),integral,.01,p);
  const Vec3 correction=p.robot_mass*(acceleration-own)-target;
  EXPECT_NEAR(correction.lpNorm<1>(),p.allocation_thrust_reserve,1e-10);
  EXPECT_DOUBLE_EQ(integral.integral.norm(),0);
  EXPECT_LE((p.robot_mass*(acceleration+Vec3(0,0,p.gravity))).lpNorm<1>(),p.allocation_thrust_limit);
  p.allocation_thrust_reserve=0;
  const Vec3 no_feedback=forceTrackingAcceleration(Vec3::UnitX(),own,Vec3::Ones(),target,
      Vec3::Ones(),integral,.01,p);
  EXPECT_LT((no_feedback-own-target/p.robot_mass).norm(),1e-10);
}

TEST(WrenchTracking, ProtectiveOverrideAlsoRespectsTotalThrust)
{
  ControlConfig p;
  bee::control::ForceConfig force_config;
  force_config.mass=p.robot_mass; force_config.gravity=p.gravity; force_config.thrust_limit=p.allocation_thrust_limit;
  const Vec3 acceleration=bee::control::limitThrust(Vec3(15,-10,8),force_config);
  EXPECT_LE((p.robot_mass*(acceleration+Vec3(0,0,p.gravity))).lpNorm<1>(),p.allocation_thrust_limit+1e-10);
}

TEST(WrenchConfiguration, RejectsInvalidMarginsAndImpossiblePreload)
{
  ControlConfig p;p.allocation_mu=.45;p.distributed_tracking=true;p.wrench_control=true;
  p.allocation_tangent_side_error=-.1;EXPECT_THROW(p.validate(),std::invalid_argument);
  p.allocation_tangent_side_error=0;p.allocation_normal_error=p.max_normal_force/2;EXPECT_THROW(p.validate(),std::invalid_argument);
  p.allocation_normal_error=0;p.allocation_thrust_reserve=20;EXPECT_THROW(p.validate(),std::invalid_argument);
  p.allocation_thrust_reserve=3;p.allocation_normal_error=.5;p.max_normal_force=4.5;
  EXPECT_THROW(GraspPhaseManager(scene(),p,TaskConfig()),std::invalid_argument);
}

TEST(WrenchPreload, CoversAccelerationAndBothUncertaintyBounds)
{
  auto sc=scene(); ControlConfig p; p.allocation_mu=.45; p.distributed_tracking=p.wrench_control=true;
  p.allocation_normal_error=.5; p.allocation_observer_normal_error=.7;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  TaskConfig task; GraspPhaseManager phases(sc,p,task);
  const double load=sc.object_mass*(p.gravity+p.reference_acceleration_limit)/3;
  const double command=phases.normalForceTarget();
  EXPECT_LE(command,p.max_normal_force-p.allocation_normal_error);
  ContactState c; c.normal=Vec3::UnitX(); c.normal_force=task.preload_force_ratio*command;
  EXPECT_TRUE(preloadCapacityReady(c,command,load,p.allocation_mu,p));
  EXPECT_GE(p.allocation_mu*(command-p.allocation_normal_error)-.2,task.normal_force_margin*load);
  EXPECT_GE(p.allocation_mu*(command-p.allocation_observer_normal_error)-.2,task.normal_force_margin*load-1e-12);
  // The previous 4.424 N floor passes a 90% force-ratio test but lacks capacity.
  c.normal_force=4.424;
  EXPECT_FALSE(preloadCapacityReady(c,4.424,load,p.allocation_mu,p));
  const Vec3 tilted=Vec3(1,0,.13).normalized();
  EXPECT_GT(requiredPreload(tilted,load,p.allocation_mu,.7,.2),
            requiredPreload(Vec3::UnitX(),load,p.allocation_mu,.7,.2));
}

TEST(WrenchPreload, PhaseWaitsForEveryContactsObservedCapacity)
{
  auto sc=scene(); ControlConfig p; p.allocation_mu=.45; p.distributed_tracking=p.wrench_control=true;
  p.allocation_normal_error=p.allocation_observer_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  TaskConfig task; task.preload_force_ratio=.8; task.preload_duration=.2;
  GraspPhaseManager phases(sc,p,task); auto st=state(sc,sc.staging_clearance);
  ControlStatus status; double now=0; phases.start(st,now);
  for (int k=0;k<2000 && phases.phase()!=Phase::Preload;++k)
  {
    st=approachFixture(sc,p,phases.phase());
    now+=.05; phases.step(st,status,now);
    ASSERT_NE(phases.phase(),Phase::Aborted) << phases.fault();
  }
  ASSERT_EQ(phases.phase(),Phase::Preload);
  status.normal_force=phases.normalForceTarget();
  status.normal_force_targets.assign(3,status.normal_force);
  status.preload_plan_ready=true;
  status.preload_normal_targets.assign(3,status.normal_force);
  for(const auto& c:st.contacts)
    status.preload_forces.push_back(-status.normal_force*c.normal+
        Vec3(0,0,sc.object_mass*(p.gravity+p.reference_acceleration_limit)/3));
  for (auto& c:st.contacts) c.normal_force=status.normal_force;
  st.contacts[1].normal_force=4.424; // Ratio passes; robust support capacity fails.
  for(int k=0;k<20;++k) { now+=.05; phases.step(st,status,now); }
  EXPECT_EQ(phases.phase(),Phase::Preload);
  st.contacts[1].normal_force=status.normal_force;
  // A fresh normal must not redefine last cycle's already reached ramp target.
  for(auto& c:st.contacts)c.normal=(c.normal-Vec3(0,0,.001)).normalized();
  for(int k=0;k<10 && phases.phase()==Phase::Preload;++k) { now+=.05; phases.step(st,status,now); }
  EXPECT_EQ(phases.phase(),Phase::SupportTransfer);
}

TEST(GraspRealignment, WaitsForOperatorAndChecksAlignmentAgainOnResume)
{
  const auto sc = scene();
  ControlConfig control;
  TaskConfig task;
  GraspPhaseManager phases(sc, control, task);
  ControlStatus status;
  double now = 0;
  phases.start(state(sc, sc.staging_clearance), now);
  for (int k = 0; k < 2000 && phases.phase() != Phase::Preload; ++k)
  {
    now += .05;
    phases.step(approachFixture(sc, control, phases.phase()), status, now);
  }
  ASSERT_EQ(phases.phase(), Phase::Preload);
  now += task.lift_timeout + 1;
  phases.step(approachFixture(sc, control, Phase::Approach), status, now);
  EXPECT_FALSE(phases.advisory().empty());
  ASSERT_TRUE(phases.requestRealign(now));
  EXPECT_FALSE(phases.requestResume());
  const auto aligned = approachFixture(sc, control, Phase::Realign);
  auto settle = [&] {
    for (int i = 0; i < 60; ++i)
    {
      now += .05;
      const auto r = phases.step(aligned, status, now);
      EXPECT_EQ(r.phase, Phase::Realign);
      EXPECT_DOUBLE_EQ(r.normal_force, 0);
    }
  };
  settle();
  ASSERT_TRUE(phases.requestResume());
  auto drift = aligned;
  drift.contacts.front().position.x() += .1;
  now += .05;
  EXPECT_EQ(phases.step(drift,status,now).phase, Phase::Realign);
  settle();
  ASSERT_TRUE(phases.requestResume());
  now += .05;
  const auto r = phases.step(aligned,status,now);
  EXPECT_EQ(r.phase, Phase::Approach);
  EXPECT_DOUBLE_EQ(r.radius_clearance, task.precontact_distance);
  EXPECT_EQ(r.touching, std::vector<bool>(sc.robot_names.size(), false));
}

TEST(ContactQp, DegenerateActiveConstraintsAndInfeasibilityAreDistinguished)
{
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2); H(1,1)=.0003;
  Eigen::VectorXd rhs(2); rhs<<2,-1;
  Eigen::MatrixXd A(4,2); A<<1,0, 2,0, 0,1, 0,2;
  Eigen::VectorXd lo(4),hi(4); lo<<0,0,0,0; hi<<1,2,1,2;
  const auto result=solveContactQp(H,rhs,A,lo,hi,Eigen::VectorXd::Zero(2));
  ASSERT_TRUE(result.converged) << result.status;
  EXPECT_NEAR(result.x[0],1,2e-5); EXPECT_NEAR(result.x[1],0,2e-5);
  EXPECT_LT(result.stationarity,1e-4); EXPECT_LT(result.complementarity,1e-4);
  // Each individual interval is valid, but x<=1 and 2*x>=3 are inconsistent.
  lo[1]=3; hi[1]=4;
  const auto impossible=solveContactQp(H,rhs,A,lo,hi,result.x);
  EXPECT_FALSE(impossible.converged);
}

#ifdef BEE_TEST_DATA_DIR
TEST(WrenchAllocation, PreviouslyUnconvergedBoundaryProblem)
{
  // Synthetic unequal contact/tilt case, seed 3821, case 162. The previous
  // 1200-iteration solver ends with primal 9.82e-5 and dual 3.20e-4.
  // This is a solver regression, not a claimed replay of the failed flight.
  std::ifstream file(std::string(BEE_TEST_DATA_DIR)+"/grasp_qp_boundary.txt");
  ASSERT_TRUE(file.good()); std::string metadata; std::getline(file,metadata);
  Wrench desired;
  for(int j=0;j<3;++j) file>>desired.force[j];
  for(int j=0;j<3;++j) file>>desired.moment[j];
  std::vector<ContactState> contacts(3); Eigen::VectorXd previous(9);
  ControlConfig p; p.allocation_normal_error=p.allocation_observer_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  std::vector<Vec3> own(3,Vec3(0,0,p.robot_mass*p.gravity));
  for(int i=0;i<3;++i)
  {
    for(int j=0;j<3;++j) file>>contacts[i].normal[j];
    for(int j=0;j<3;++j) file>>contacts[i].position[j];
    file>>contacts[i].normal_force;
    for(int j=0;j<3;++j) file>>previous[3*i+j];
  }
  ASSERT_TRUE(bool(file));
  const auto result=allocateWrench(contacts,Vec3::Zero(),desired,own,previous,3.924,.025,p);
  ASSERT_TRUE(result.converged) << result.solver_status << " " << result.primal;
  EXPECT_LT(result.violation,2e-5); EXPECT_LT(result.stationarity,1e-4);
  EXPECT_LE((result.force-previous).lpNorm<Eigen::Infinity>(),.50002);
}
#endif

TEST(WrenchMotion, RigidMotionProjectionIncludesRotationAndCogOffsets)
{
  ControlConfig p;
  const std::vector<Vec3> arms={Vec3(.3,0,0),Vec3(-.3,0,0),Vec3(0,.3,0)};
  const std::vector<Vec3> offsets={Vec3(.005,0,.002),Vec3(0,.006,0),Vec3(-.004,0,0)};
  Vec3 acc(.1,.1,.1),alpha(.2,-.15,.1);
  ASSERT_TRUE(limitObjectMotion(arms,offsets,p,acc,alpha));
  for(int i=0;i<3;++i)
    EXPECT_TRUE(referenceMotionValid(Vec3::Zero(),acc+alpha.cross(arms[i])+offsets[i],p));
  // No rigid motion can cancel opposite 1 m/s2 offsets at the same contact.
  EXPECT_FALSE(limitObjectMotion({Vec3::Zero(),Vec3::Zero()},
      {Vec3(1,0,0),Vec3(-1,0,0)},p,acc,alpha));
}

TEST(WrenchPreload, PlannedTargetsIgnoreOldMomentOffsetsAndRequireObservedCapacity)
{
  auto sc=scene(4); ControlConfig p;p.distributed_tracking=p.wrench_control=true;
  p.allocation_normal_error=p.allocation_observer_normal_error=.5;
  p.allocation_tangent_up_error=p.allocation_tangent_side_error=.1;
  auto st=state(sc); st.object_wrench.moment=Vec3(.2,-.2,0);
  for(auto& c:st.contacts) {c.normal_force=0;c.position=st.input.object.position+.2*c.normal-Vec3(0,0,.12);}
  GraspController controller(sc,p);
  Reference r=supportReference();r.phase=Phase::Preload;r.begin_support=false;
  r.preload_vertical_force=sc.object_mass*(p.gravity+p.reference_acceleration_limit);
  ControlOutput out;
  for(int i=0;i<100;++i) out=controller.step(st,r,.01);
  ASSERT_TRUE(out.status.fault.empty())<<out.status.fault;
  ASSERT_TRUE(out.status.preload_plan_ready);
  Vec3 sum=Vec3::Zero(),moment=Vec3::Zero();
  for(int i=0;i<4;++i)
  {
    const auto& cmd=out.commands[i];const Vec3 force=out.status.preload_forces[i];
    EXPECT_NEAR(cmd.normal_force,-force.dot(st.contacts[i].normal),1e-5);
    EXPECT_FALSE(cmd.support_tracking);
    EXPECT_NEAR(cmd.support_vector.norm(),0,1e-12);
    EXPECT_FALSE(plannedContactCapacityReady(st.contacts[i],force,p));
    st.contacts[i].normal_force=r.preload_force_ratio*cmd.normal_force;
    EXPECT_TRUE(plannedContactCapacityReady(st.contacts[i],force,p));
    sum+=force;moment+=(st.contacts[i].position-st.input.object.position).cross(force);
  }
  EXPECT_NEAR(sum.z(),r.preload_vertical_force,.01);
  EXPECT_LT(moment.norm(),.001);
}

TEST(WrenchAllocation, AchievedWrenchRespectsFinalAccelerationLimit)
{
  auto sc=scene(4);auto st=state(sc);ControlConfig p;
  for(auto& c:st.contacts)c.normal_force=5;
  Eigen::VectorXd previous(12);std::vector<Vec3> own(4,Vec3(0,0,p.robot_mass*p.gravity));
  for(int i=0;i<4;++i)previous.segment<3>(3*i)=-4*st.contacts[i].normal+Vec3(0,0,1.22625);
  AllocationOptions options;options.moving_object_mass=.5;
  options.object_inverse_inertia=sc.object_inertia.inverse();options.constrain_acceleration=true;
  Wrench desired;desired.force=Vec3(0,0,4.905);desired.moment=Vec3(.04,.04,0);
  const auto result=allocateWrench(st.contacts,st.input.object.position,desired,own,previous,4,1,p,options);
  ASSERT_TRUE(result.converged)<<result.solver_status;
  const Vec3 da=(result.achieved.force-desired.force)/.5;
  const Vec3 dalpha=options.object_inverse_inertia*(result.achieved.moment-desired.moment);
  for(const auto& c:st.contacts)
    EXPECT_TRUE(referenceMotionValid(Vec3::Zero(),da+dalpha.cross(c.position-st.input.object.position),p));
}

TEST(WrenchAllocation, MotionLimitRejectsUnsupportedAirborneWrench)
{
  auto sc=scene(4);auto st=state(sc);ControlConfig p;
  for(auto& c:st.contacts)c.normal_force=.6;
  Eigen::VectorXd previous(12);std::vector<Vec3> own(4,Vec3(0,0,p.robot_mass*p.gravity));
  for(int i=0;i<4;++i)previous.segment<3>(3*i)=-4*st.contacts[i].normal;
  AllocationOptions options;options.moving_object_mass=.5;
  options.object_inverse_inertia=sc.object_inertia.inverse();
  Wrench desired;desired.force=Vec3(0,0,4.905);
  const auto nominal=allocateWrench(st.contacts,st.input.object.position,desired,own,previous,4,1,p,options);
  ASSERT_TRUE(nominal.converged);
  EXPECT_GT((nominal.achieved.force-desired.force).norm()/.5,p.reference_acceleration_limit);
  options.constrain_acceleration=true;
  const auto bounded=allocateWrench(st.contacts,st.input.object.position,desired,own,previous,4,1,p,options);
  EXPECT_FALSE(bounded.converged);
}

#ifdef BEE_TEST_DATA_DIR
namespace {
void readQpFixture(const std::string& name,Eigen::MatrixXd& H,Eigen::VectorXd& rhs,
                  Eigen::MatrixXd& A,Eigen::VectorXd& lo,Eigen::VectorXd& hi,Eigen::VectorXd& previous)
{
  std::ifstream file(std::string(BEE_TEST_DATA_DIR)+"/"+name);
  int n=0,m=0,status;file>>n>>m>>status;
  ASSERT_GT(n,0);ASSERT_GT(m,0);
  H.resize(n,n);rhs.resize(n);A.resize(m,n);lo.resize(m);hi.resize(m);previous.resize(n);
  auto read=[&](auto& x){for(int i=0;i<x.rows();++i)for(int j=0;j<x.cols();++j)file>>x(i,j);};
  read(H);read(rhs);read(A);read(lo);read(hi);read(previous);ASSERT_TRUE(bool(file));
}
}
TEST(ContactQp, RecordedSupportTransferDualStall)
{
  Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
  readQpFixture("grasp_qp_transfer_boundary.txt",H,rhs,A,lo,hi,previous);
  const auto result=solveContactQp(H,rhs,A,lo,hi,previous);
  ASSERT_TRUE(result.converged)<<result.status;
  EXPECT_LT(result.iterations,4000);
  EXPECT_LT(result.violation,2e-5);EXPECT_LT(result.stationarity,1e-4);
  EXPECT_LT(result.complementarity,1e-4);
}
TEST(WrenchMotion, RecordedCogMotionFitsReceiverWithoutOctahedronRestriction)
{
  Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
  readQpFixture("grasp_motion_boundary.txt",H,rhs,A,lo,hi,previous);
  ASSERT_EQ(A.rows(),32);
  std::vector<Vec3> arms,offsets;
  for(int i=0;i<4;++i)
  {
    Eigen::Matrix<double,3,6> B=Eigen::Matrix<double,3,6>::Zero();Vec3 offset=Vec3::Zero();int k=0;
    for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
    {
      const Vec3 sign(x,y,z);B+=sign*A.row(8*i+k)/8;offset-=sign*hi[8*i+k]/8;++k;
    }
    arms.push_back(Vec3(B(1,5),B(2,3),B(0,4)));offsets.push_back(offset);
  }
  const Eigen::VectorXd desired=H.ldlt().solve(rhs);
  Vec3 acc=desired.head<3>(),alpha=desired.tail<3>();ControlConfig p;
  ASSERT_TRUE(limitObjectMotion(arms,offsets,p,acc,alpha));
  for(int i=0;i<4;++i)EXPECT_TRUE(referenceMotionValid(Vec3::Zero(),acc+alpha.cross(arms[i])+offsets[i],p));
}
#endif
TEST(WrenchMotion, EveryAccelerationPolytopeVertexFitsEuclideanLimit)
{
  ControlConfig p;const auto& directions=accelerationDirections();int vertices=0;
  for(int i=0;i<int(directions.size());++i)for(int j=0;j<i;++j)for(int k=0;k<j;++k)
  {
    Eigen::Matrix3d A;A.row(0)=directions[i];A.row(1)=directions[j];A.row(2)=directions[k];
    if(std::abs(A.determinant())<1e-9)continue;
    const Vec3 v=A.inverse()*Vec3::Constant(allocationAccelerationLimit(p));
    bool inside=true;for(const auto& d:directions)inside=inside&&d.dot(v)<=allocationAccelerationLimit(p)+1e-10;
    if(inside){++vertices;EXPECT_LT(v.norm(),p.reference_acceleration_limit);}
  }
  EXPECT_GT(vertices,0);
}

#ifdef BEE_TEST_DATA_DIR
TEST(ContactQp, RecordedReleaseAndTransferMeetPhysicalTolerancesWithinBudget)
{
  for(const auto* name:{"grasp_qp_release_boundary.txt","grasp_qp_transfer_restart.txt",
                       "grasp_qp_release_restart.txt","grasp_qp_scaled_precision.txt",
                       "grasp_qp_frustum_transfer.txt","grasp_qp_rock_transfer.txt","grasp_qp_heavy_transport.txt"})
  {
    Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
    readQpFixture(name,H,rhs,A,lo,hi,previous);
    const auto result=solveContactQp(H,rhs,A,lo,hi,previous);
    ASSERT_TRUE(result.converged)<<name<<" status "<<result.status;
    EXPECT_TRUE(result.status==1 || result.status==2);EXPECT_LE(result.restarts,2);
    EXPECT_LE(result.iterations,4000);EXPECT_LT(result.violation,2e-5);EXPECT_LT(result.primal,2e-5);
    EXPECT_LT(result.stationarity,1e-4);EXPECT_LT(result.complementarity,1e-4);
  }
}
TEST(ContactQp, RecordedCylinderTimeLimitInputIsFeasible)
{
  Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
  readQpFixture("grasp_qp_cylinder_time_limit.txt",H,rhs,A,lo,hi,previous);
  const auto result=solveContactQp(H,rhs,A,lo,hi,previous);
  ASSERT_TRUE(result.converged)<<result.status;
  EXPECT_EQ(result.status,1);EXPECT_LE(result.iterations,4000);
  EXPECT_LT(result.violation,2e-5);EXPECT_LT(result.primal,2e-5);
  EXPECT_LT(result.stationarity,1e-4);EXPECT_LT(result.complementarity,1e-4);
}
TEST(ContactQp, RecordedReleaseConstraintsAllowRemovingContactForces)
{
  for(const auto* name:{"grasp_qp_pentagon_release.txt","grasp_qp_cylinder_release.txt"})
  {
    Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
    readQpFixture(name,H,rhs,A,lo,hi,previous);
    // Keep every recorded physical constraint. Once unloading is complete,
    // the release objective minimizes the remaining force, not a holding wrench.
    const auto result=solveContactQp(Eigen::MatrixXd::Identity(H.rows(),H.cols()),
        Eigen::VectorXd::Zero(rhs.size()),A,lo,hi,previous);
    ASSERT_TRUE(result.converged)<<name<<" status "<<result.status;
    EXPECT_LT(result.violation,2e-5);EXPECT_LT(result.primal,2e-5);
    EXPECT_LT(result.stationarity,1e-4);EXPECT_LT(result.complementarity,1e-4);
    EXPECT_LT(result.x.norm(),previous.norm());
  }
}
TEST(ContactQp, RecordedHoldingPlanUsesWellConditionedObjective)
{
  Eigen::MatrixXd H,A;Eigen::VectorXd rhs,lo,hi,previous;
  readQpFixture("grasp_qp_frustum_preload.txt",H,rhs,A,lo,hi,previous);
  ASSERT_TRUE(H.isDiagonal(1e-12));
  const auto result=solveContactQp(H,rhs,A,lo,hi,previous);
  ASSERT_TRUE(result.converged)<<result.status;
  EXPECT_EQ(result.status,1);EXPECT_LE(result.iterations,4000);
  EXPECT_LT(result.violation,2e-5);EXPECT_LT(result.primal,2e-5);
  EXPECT_LT(result.stationarity,1e-4);EXPECT_LT(result.complementarity,1e-4);
}
#endif

TEST(WrenchTracking, FilterSmoothsOnlyForceFeedback)
{
  ControlConfig p;p.contact_force_filter_tau=.1;p.contact_force_ki=0;
  p.contact_damping=3;p.press_limit=p.approach_limit=p.common_z_limit=20;
  ForceTrackingState state;
  const Vec3 own(.1,.2,.1),target(-4,0,1.2),external(4,.2,-1.2),velocity(0,.1,0);
  const Vec3 acceleration=forceTrackingAcceleration(Vec3::UnitX(),own,velocity,
      target,external,state,.01,p,false);
  EXPECT_NEAR(state.filtered.y(),p.contact_force_kp*.2/11,1e-12);
  EXPECT_NEAR((acceleration-own-target/p.robot_mass-state.filtered-p.contact_damping*velocity).norm(),0,1e-12);
  // This must not delay changes to FF or damping when force error is zero.
  state.reset();const Vec3 target2(-4,.2,1.3);
  const Vec3 instant=forceTrackingAcceleration(Vec3::UnitX(),own,velocity,target2,-target2,state,.01,p,false);
  EXPECT_NEAR((instant-own-target2/p.robot_mass-p.contact_damping*velocity).norm(),0,1e-12);
}
TEST(WrenchTracking, FilterConvergesResetsAndBypasses)
{
  ControlConfig p;p.contact_force_filter_tau=.1;p.contact_force_ki=0;p.contact_damping=0;
  ForceTrackingState state;
  for (int i=0;i<1000;++i)
    forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),Vec3(-4,0,1),Vec3(4,.2,-1),state,.01,p,false);
  EXPECT_NEAR(state.filtered.y(),p.contact_force_kp*.2,1e-10);
  state.reset();EXPECT_DOUBLE_EQ(state.filtered.norm(),0);
  p.contact_force_filter_tau=0;
  forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),Vec3(-4,0,1),Vec3(4,.2,-1),state,.01,p,false);
  EXPECT_DOUBLE_EQ(state.filtered.y(),p.contact_force_kp*.2);
}
TEST(WrenchTracking, FilterRetainsLimitsAndFreezesIntegral)
{
  ControlConfig p;p.contact_force_filter_tau=.1;p.contact_damping=0;
  ForceTrackingState state;
  for (int i=0;i<200;++i)
  {
    const Vec3 prior=state.integral;
    const Vec3 a=forceTrackingAcceleration(Vec3::UnitX(),Vec3::Zero(),Vec3::Zero(),
        Vec3(-4,0,1),Vec3(4,100,-1),state,.01,p,true);
    if(state.correction_saturated || state.acceleration_limited)
      EXPECT_DOUBLE_EQ((state.integral-prior).norm(),0);
    EXPECT_LE((p.robot_mass*(a+Vec3(0,0,p.gravity))).lpNorm<1>(),p.allocation_thrust_limit+1e-10);
  }
  EXPECT_TRUE(state.correction_saturated);
  state.reset();EXPECT_DOUBLE_EQ(state.integral.norm(),0);EXPECT_FALSE(state.correction_saturated);
  p.contact_force_filter_tau=-.1;EXPECT_THROW(p.validate(),std::invalid_argument);
}

TEST(GenericReference, OrdinaryMotionAndAttitudePassThrough)
{
  VehicleCommand c; c.baselink_rpy=Vec3(.1,.2,.3);
  aerial_robot_msgs::FlightNav nav; nav.pos_xy_nav_mode=nav.POS_MODE; nav.pos_z_nav_mode=nav.VEL_MODE;
  nav.target_pos_x=1.5; nav.target_vel_z=.02;
  auto out=makeBeeCommand(c,VehicleState{},ContactState{},ControlConfig{},nav,std_msgs::Header{},Vec3::Zero());
  EXPECT_FALSE(out.enable_force_control); EXPECT_FALSE(out.acceleration_bounds);
  EXPECT_EQ(out.navigation.pos_xy_nav_mode,nav.POS_MODE);
  EXPECT_EQ(out.navigation.pos_z_nav_mode,nav.VEL_MODE);
  EXPECT_FLOAT_EQ(out.navigation.target_pos_x,1.5);
  EXPECT_DOUBLE_EQ(out.baselink_rpy.y,.2);
  EXPECT_DOUBLE_EQ(out.baselink_rpy.z,.3);
  EXPECT_EQ(out.navigation.yaw_nav_mode,nav.NO_NAVIGATION);
}
TEST(GenericReference, FreeApproachKeepsHeightControlAndForcePI)
{
  ControlConfig p;
  p.distributed_tracking = p.wrench_control = true;
  VehicleCommand c;
  aerial_robot_msgs::FlightNav nav;
  nav.pos_xy_nav_mode = nav.ACC_MODE;
  nav.pos_z_nav_mode = nav.POS_MODE;
  c.position_target = Vec3(0.3, -0.2, 0.95);
  nav.target_pos_z = 0.95;
  const auto command = makeBeeCommand(c, VehicleState{}, ContactState{}, p, nav, std_msgs::Header{}, Vec3::Zero());
  EXPECT_TRUE(command.enable_force_control);
  EXPECT_TRUE(command.force_integral_enabled);
  EXPECT_FALSE(command.reset_force_feedback);
  EXPECT_EQ(command.navigation.pos_z_nav_mode, nav.POS_MODE);
  EXPECT_FLOAT_EQ(command.navigation.target_pos_z, nav.target_pos_z);
  EXPECT_EQ(command.navigation.pos_xy_nav_mode, nav.POS_VEL_MODE);
  EXPECT_DOUBLE_EQ(command.navigation.target_pos_x, c.position_target.x());
  EXPECT_DOUBLE_EQ(command.navigation.target_pos_y, c.position_target.y());
  EXPECT_DOUBLE_EQ(command.force.x, 0);
  EXPECT_DOUBLE_EQ(command.force.y, 0);
  EXPECT_DOUBLE_EQ(command.force.z, 0);
}

TEST(Phase, HeavyTriangleExceedsConfiguredPreloadBudget)
{
  auto sc = scene();
  sc.object_mass = 1.234;
  ControlConfig p;
  p.allocation_mu=.45;
  p.distributed_tracking = p.wrench_control = true;
  p.reference_acceleration_limit = 1;
  p.allocation_normal_error = p.allocation_observer_normal_error = .5;
  p.allocation_tangent_up_error = p.allocation_tangent_side_error = .1;
  EXPECT_NEAR(requiredPreload(Vec3::UnitX(), 1.2 * sc.object_mass * (p.gravity + 1) / 3,
                              p.allocation_mu, .5, .2), 12.79, .01);
  EXPECT_THROW((GraspPhaseManager(sc, p, TaskConfig{})), std::invalid_argument);
}

TEST(GenericReference, PreloadSendsForceAndMotionSeparately)
{
  ControlConfig p; VehicleCommand c; ContactState contact; VehicleState v;
  p.wrench_control=true; c.contact_tracking=true; c.normal=Vec3::UnitX(); c.normal_force=4;
  c.velocity=Vec3(.01,0,0); c.minimum_radius=.2; v.base.position.x()=1;
  contact.normal_force=3; contact.velocity=Vec3::Zero();
  aerial_robot_msgs::FlightNav nav;
  auto out=makeBeeCommand(c,v,contact,p,nav,std_msgs::Header{},Vec3::Zero());
  EXPECT_TRUE(out.enable_force_control); EXPECT_TRUE(out.force_integral_enabled);
  EXPECT_EQ(out.navigation.pos_z_nav_mode,nav.ACC_MODE);
  EXPECT_DOUBLE_EQ(out.force.x,-4); EXPECT_FLOAT_EQ(out.navigation.target_acc_x,0);
}
TEST(GenericReference, GeometryBecomesForceVectorAndGenericBounds)
{
  ControlConfig p; VehicleCommand c; ContactState contact; VehicleState v;
  c.contact_tracking=c.wrench_tracking=true;
  c.normal=Vec3(1,2,-1).normalized(); c.normal_force=4;
  c.support_vector=tangentProjection(c.normal,Vec3(0,0,1));
  c.velocity=Vec3(.01,.02,.03); c.acceleration=Vec3(.01,0,.02);
  c.minimum_radius=.2; v.base.position=c.normal;
  aerial_robot_msgs::FlightNav nav;
  auto out=makeBeeCommand(c,v,contact,p,nav,std_msgs::Header{},Vec3::Zero());
  EXPECT_TRUE(out.enable_force_control); EXPECT_TRUE(out.force_integral_enabled);
  const Vec3 force(out.force.x,out.force.y,out.force.z);
  EXPECT_LT((force-(-4*c.normal+c.support_vector)).norm(),1e-12);
  const Eigen::Quaterniond q(out.acceleration_orientation.w,out.acceleration_orientation.x,
                            out.acceleration_orientation.y,out.acceleration_orientation.z);
  EXPECT_LT((q*Vec3::UnitX()-c.normal).norm(),1e-12);
  EXPECT_NEAR(out.acceleration_max.y,p.approach_limit,1e-12);
  EXPECT_NEAR(out.acceleration_max.z,p.common_z_limit+p.relative_z_limit+p.tilt_limit+
              c.support_vector.dot(q*Vec3::UnitZ())/p.robot_mass,1e-12);
  EXPECT_NEAR(out.velocity_point.z,p.contact_offset.z(),1e-12);
  EXPECT_FALSE(out.reset_force_feedback);
}
TEST(GenericReference, CentralPenetrationProtectionSetsAnAccelerationConstraint)
{
  ControlConfig p; p.press_radial_kp=2;
  VehicleCommand c; ContactState contact; VehicleState v;
  c.contact_tracking=c.wrench_tracking=true; c.normal=Vec3::UnitX(); c.normal_force=4;
  c.minimum_radius=.4; v.base.position.x()=.39;
  aerial_robot_msgs::FlightNav nav;
  auto out=makeBeeCommand(c,v,contact,p,nav,std_msgs::Header{},Vec3::Zero());
  EXPECT_FALSE(out.reset_force_feedback);
  EXPECT_DOUBLE_EQ(out.force.x,0);
  EXPECT_DOUBLE_EQ(out.acceleration_min.x,out.acceleration_max.x);
  EXPECT_NEAR(out.acceleration_min.x,.12,1e-12);
  bee::control::ForceConfig generic;
  ForceTrackingState state;
  const Vec3 result=bee::control::forceTrackingAcceleration(Vec3::Zero(),Vec3::Zero(),Vec3(-4,0,0),
      Vec3(4,0,0),Eigen::Matrix3d::Identity(),Vec3(.12,-.8,-1.5),Vec3(.12,.8,1.5),true,
      state,.01,generic,false);
  EXPECT_NEAR(result.x(),.12,1e-12);
}
TEST(Calibration, CentralPhaseWaitsForControllerResults)
{
  auto sc=scene(); ControlConfig p; p.distributed_tracking=p.wrench_control=true;
  TaskConfig task; GraspPhaseManager phases(sc,p,task); auto st=state(sc,sc.staging_clearance);
  ControlStatus status; status.calibration_ready=false;
  double now=0; phases.start(st,now);
  for(int k=0;k<2000 && phases.phase()!=Phase::Calibrate;++k)
  {st=approachFixture(sc,p,phases.phase()); now+=.05; phases.step(st,status,now); ASSERT_NE(phases.phase(),Phase::Aborted);}
  ASSERT_EQ(phases.phase(),Phase::Calibrate);
  now+=task.baseline_duration+1; phases.step(st,status,now);
  EXPECT_EQ(phases.phase(),Phase::Calibrate);
  status.calibration_ready=true; now+=.05; phases.step(st,status,now);
  EXPECT_EQ(phases.phase(),Phase::Approach);
}

TEST(GenericReference, ZeroCorrectionBudgetDisablesFeedbackAndIntegral)
{
  bee::control::ForceConfig config; config.correction_reserve=0; config.integral_limit=0;
  EXPECT_NO_THROW(config.validate());
  ForceTrackingState state;
  const Vec3 target(1,0,0), acceleration(.01,.02,.03);
  const Vec3 out=bee::control::forceTrackingAcceleration(acceleration,Vec3::Ones(),target,
      Vec3::Zero(),Eigen::Matrix3d::Identity(),Vec3::Zero(),Vec3::Zero(),false,state,.025,config,true);
  EXPECT_LT((out-acceleration-target/config.mass).norm(),1e-12);
  EXPECT_TRUE(state.integral.isZero()); EXPECT_TRUE(state.correction_saturated);
}

TEST(ModelMass, IncludesFixedChildrenAndIgnoresDummyRoot)
{
  auto body = [](const std::string& name, double mass) {
    return "<link name='"+name+"'><inertial><mass value='"+std::to_string(mass)+
      "'/><inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial></link>";
  };
  const std::string xml = "<robot name='test'>"+body("root",99)+body("body",1.5)+body("payload",.25)+
    "<link name='marker'/><joint name='base' type='fixed'><parent link='root'/><child link='body'/></joint>"
    "<joint name='payload_joint' type='fixed'><parent link='body'/><child link='payload'/></joint>"
    "<joint name='marker_joint' type='fixed'><parent link='body'/><child link='marker'/></joint></robot>";
  EXPECT_DOUBLE_EQ(bee::control::modelMass(xml), 1.75);
  EXPECT_THROW(bee::control::modelMass("<robot name='empty'><link name='root'/></robot>"), std::invalid_argument);
  EXPECT_THROW(bee::control::modelMass("invalid"), std::invalid_argument);
}
