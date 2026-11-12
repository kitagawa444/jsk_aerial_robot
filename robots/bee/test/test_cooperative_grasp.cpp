#include <bee/cooperative_grasp/controller.h>
#include <bee/cooperative_grasp/phase_manager.h>
#include <bee/cooperative_grasp/safety_monitor.h>
#include <bee/cooperative_grasp/state_estimator.h>
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

using namespace bee::grasp;
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
                           Vec3(4 * std::cos(angle), 4 * std::sin(angle), -s.object_mass * 9.81 / s.robot_names.size());
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
Reference supportReference()
{
  Reference r;
  r.phase = Phase::SupportTransfer;
  r.roll = kPi / 2;
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
  ControlConfig c;
  c.z_kp = NAN;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.support_ramp_duration = 0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
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
  CooperativeGraspController controller(s, {});
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
  CooperativeGraspController controller(s, {});
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
    CooperativeGraspController controller(s, {});
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
    st = state(s, clearance);
    status.normal_force = phase >= Phase::Release ? 0 : phases.normalForceTarget();
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
    CooperativeGraspController controller(sc, config);
    Reference r;
    r.phase = Phase::Preload;
    r.normal_force = 4;
    r.normal_force_rate = 100;
    const auto out = controller.step(st, r, .05);
    EXPECT_NEAR(out.commands[0].acceleration.x(), ax, 1e-12);
    EXPECT_NEAR(out.commands[0].acceleration.y(), ay, 1e-12);
    EXPECT_NEAR(angleError(out.commands[0].yaw - target_yaw), 0, 1e-12);
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
  CooperativeGraspController controller(sc, {});
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
