#include <bee/cooperative_grasp/grasp_state_estimator.h>
#include <set>

namespace bee::grasp
{
Vec3 tangentProjection(const Vec3& n, const Vec3& v)
{
  return v - n * n.dot(v);
}

bool additionalSurface(const SceneConfig& s)
{
  return s.object_shape == "cylinder" || s.object_shape == "frustum" || s.object_shape == "capsule";
}
double bottomOffset(const SceneConfig& s)
{
  return s.object_bottom_offset < 0 ? s.object_height / 2 : s.object_bottom_offset;
}
// Object-frame outward normals. Cylinder/frustum grasping uses side surfaces,
// not end caps or rim edges. Capsule's segment is along object X.
Vec3 surfaceNormal(const SceneConfig& s, const Vec3& p, const Vec3& planar)
{
  Vec3 n = planar;
  if (s.object_shape == "rock")
  {
    if (s.contact_normals.empty()) throw std::invalid_argument("rock requires planned face normals");
    // Stick to the planned broad face; crossing an edge is a deviation, not an
    // invitation to switch contact faces during a grasp.
    double alignment = -2;
    for (const auto& face : s.contact_normals)
    {
      const double score = face.head<2>().normalized().dot(planar.head<2>());
      if (score > alignment) { n = face; alignment = score; }
    }
  }
  else if (s.object_shape == "capsule")
  {
    n = p - Vec3(std::clamp(p.x(), -s.surface_length / 2, s.surface_length / 2), 0, 0);
  }
  else if (s.object_shape == "cylinder" || s.object_shape == "frustum")
  {
    const double radius = p.head<2>().norm();
    if (radius < 1e-9) throw std::invalid_argument("side contact on surface axis");
    n = Vec3(p.x() / radius, p.y() / radius,
             s.object_shape == "frustum" ? (s.surface_radius - s.surface_top_radius) / s.object_height : 0);
  }
  else if (!s.surface_radii.isZero())
    n = p.cwiseQuotient(s.surface_radii.cwiseProduct(s.surface_radii));
  if (!n.allFinite() || n.norm() < 1e-9)
    throw std::invalid_argument("undefined surface normal");
  return n.normalized();
}
// Support-plane distance from CoM, h(n) = max_x n.dot(x), for unit n.
double surfaceSupport(const SceneConfig& s, const Vec3& n, double planar_radius)
{
  if (s.object_shape == "rock")
  {
    if (s.surface_vertices.empty()) throw std::invalid_argument("rock requires mesh vertices");
    double support = n.dot(s.surface_vertices.front());
    for (const auto& vertex : s.surface_vertices) support = std::max(support, n.dot(vertex));
    return support;
  }
  if (s.object_shape == "capsule")
    return s.surface_radius * n.norm() + s.surface_length / 2 * std::abs(n.x());
  if (s.object_shape == "cylinder")
    return s.surface_radius * n.head<2>().norm() + s.object_height / 2 * std::abs(n.z());
  if (s.object_shape == "frustum")
    return std::max(s.surface_radius * n.head<2>().norm() - bottomOffset(s) * n.z(),
                    s.surface_top_radius * n.head<2>().norm() + (s.object_height - bottomOffset(s)) * n.z());
  if (!s.surface_radii.isZero()) return s.surface_radii.cwiseProduct(n).norm();
  return planar_radius;
}

Vec3 plannedContactPoint(const SceneConfig& s, std::size_t i)
{
  const double angle = s.face_angles[i], h = s.contact_height;
  const Vec3 ray(std::cos(angle), std::sin(angle), 0);
  double radius = s.face_radii[i];
  if (!s.surface_radii.isZero())
  {
    const double section = 1 - std::pow(h / s.surface_radii.z(), 2);
    if (section <= 0) throw std::invalid_argument("contact height misses ellipsoid side");
    radius = std::sqrt(section) / ray.cwiseQuotient(s.surface_radii).norm();
  }
  else if (s.object_shape == "capsule")
  {
    const double section = std::sqrt(s.surface_radius * s.surface_radius - h * h);
    const double half = s.surface_length / 2;
    const double x = std::abs(ray.x()), y = std::abs(ray.y());
    radius = section * x <= half * y ? section / y :
        half * x + std::sqrt(std::max(0.0, section * section - half * half * y * y));
  }
  else if (s.object_shape == "frustum")
    radius = s.surface_radius + (s.surface_top_radius - s.surface_radius) * (h + bottomOffset(s)) / s.object_height;
  else if (s.object_shape == "rock")
  {
    const Vec3 n = surfaceNormal(s, h * Vec3::UnitZ(), ray);
    radius = (surfaceSupport(s, n, radius) - h * n.z()) / n.dot(ray);
  }
  return radius * ray + h * Vec3::UnitZ();
}

Vec3 plannedContactNormal(const SceneConfig& s, std::size_t i)
{
  const double a = s.face_angles[i];
  return surfaceNormal(s, plannedContactPoint(s, i), Vec3(std::cos(a), std::sin(a), 0));
}

Vec3 contactAttitude(const Vec3& n, double fraction, double fallback_heading)
{
  const double horizontal = n.head<2>().norm();
  return Vec3(fraction * std::atan2(horizontal, n.z()), 0,
              horizontal > 1e-9 ? std::atan2(n.y(), n.x()) + kPi / 2 : fallback_heading);
}

Vec3 footCenterTarget(const SceneConfig& scene, std::size_t i, const Pose& object,
                      double ball_radius, double gap)
{
  return object.position + object.orientation * (plannedContactPoint(scene, i) +
      (ball_radius + gap) * plannedContactNormal(scene, i));
}

void SceneConfig::validate() const
{
  const std::set<std::string> shapes = {"", "triangular_prism", "pentagonal_prism", "box", "rectangular_prism",
                                      "sphere", "ellipsoid", "cylinder", "frustum", "capsule", "rock"};
  if (!shapes.count(object_shape)) throw std::invalid_argument("unsupported contact surface shape");
  if (!surface_radii.allFinite() || (!surface_radii.isZero() && surface_radii.minCoeff() <= 0) ||
      !std::isfinite(object_bottom_offset) || !std::isfinite(contact_height) ||
      bottomOffset(*this) <= 0 || bottomOffset(*this) >= object_height ||
      contact_height <= -bottomOffset(*this) || contact_height >= object_height - bottomOffset(*this))
    throw std::invalid_argument("invalid curved contact geometry");
  if (additionalSurface(*this) &&
      (!surface_radii.isZero() || !std::isfinite(surface_radius) || surface_radius <= 0 ||
       !std::isfinite(surface_top_radius) || !std::isfinite(surface_length) ||
       (object_shape == "frustum" && (surface_top_radius <= 0 || object_bottom_offset <= 0)) ||
       (object_shape == "capsule" && (surface_length <= 0 || std::abs(object_height - 2 * surface_radius) > 1e-9))))
    throw std::invalid_argument("invalid cylinder/frustum/capsule geometry");
  const auto n = robot_names.size();
  if (object_shape == "rock")
  {
    if (surface_vertices.size() < 4 || contact_normals.size() != n)
      throw std::invalid_argument("incomplete convex contact geometry");
    for (const auto& v : surface_vertices)
      if (!v.allFinite()) throw std::invalid_argument("nonfinite mesh vertex");
    for (const auto& normal : contact_normals)
      if (!normal.allFinite() || std::abs(normal.norm() - 1) > 1e-6 || normal.head<2>().norm() < .1)
        throw std::invalid_argument("invalid convex contact normal");
  }
  if (n < 3 || n > 5 || face_angles.size() != n || face_radii.size() != n ||
      std::set<std::string>(robot_names.begin(), robot_names.end()).size() != n)
    throw std::invalid_argument("scene requires 3..5 distinct vehicles and matching contact geometry");
  for (std::size_t i = 0; i < n; ++i)
    if (robot_names[i].empty() || !std::isfinite(face_angles[i]) || !std::isfinite(face_radii[i]) || face_radii[i] <= 0)
      throw std::invalid_argument("invalid contact geometry");
  if (!std::isfinite(object_mass) || object_mass <= 0 || !std::isfinite(object_height) || object_height <= 0 ||
      !std::isfinite(staging_clearance) || staging_clearance <= 0 || !object_position.allFinite() ||
      !destination_position.allFinite())
    throw std::invalid_argument("invalid scene dimensions, mass or position");
}

GraspStateEstimator::GraspStateEstimator(SceneConfig scene, ControlConfig config)
  : scene_(std::move(scene)), config_(std::move(config))
{
  scene_.validate();
  config_.validate();
  reset();
}
void GraspStateEstimator::reset()
{
  baseline_.assign(scene_.robot_names.size(), Wrench{});
  world_baseline_.assign(scene_.robot_names.size(), Vec3::Zero());
  normal_forces_.assign(scene_.robot_names.size(), 0);
  previous_stamp_ = 0;
  velocity_.setZero();
  omega_.setZero();
  beginCalibration();
}
void GraspStateEstimator::beginCalibration()
{
  samples_ = 0;
  sums_.assign(scene_.robot_names.size(), Wrench{});
  world_sums_.assign(scene_.robot_names.size(), Vec3::Zero());
}
void GraspStateEstimator::addCalibrationSample(const InputState& input)
{
  if (input.vehicles.size() != sums_.size())
    throw std::invalid_argument("vehicle count mismatch");
  for (std::size_t i = 0; i < sums_.size(); ++i)
  {
    sums_[i].force += input.vehicles[i].observer_cog.force;
    sums_[i].moment += input.vehicles[i].observer_cog.moment;
    world_sums_[i] += input.vehicles[i].cog.orientation * input.vehicles[i].observer_cog.force;
  }
  ++samples_;
}
bool GraspStateEstimator::finishCalibration()
{
  if (!samples_)
    return false;
  for (std::size_t i = 0; i < sums_.size(); ++i)
  {
    baseline_[i].force = sums_[i].force / samples_;
    baseline_[i].moment = sums_[i].moment / samples_;
    world_baseline_[i] = world_sums_[i] / samples_;
    normal_forces_[i] = 0;
  }
  return true;
}
bool GraspStateEstimator::applyCalibration(std::size_t robot, const Wrench& baseline, const Vec3& world_force)
{
  if (robot >= baseline_.size() || !baseline.force.allFinite() || !baseline.moment.allFinite() ||
      !world_force.allFinite()) return false;
  baseline_[robot] = baseline;
  world_baseline_[robot] = world_force;
  normal_forces_[robot] = 0;
  return true;
}
GraspState GraspStateEstimator::update(const InputState& input)
{
  if (input.vehicles.size() != scene_.robot_names.size())
    throw std::invalid_argument("vehicle count mismatch");
  GraspState state;
  state.input = input;
  const double dt = input.object_stamp - previous_stamp_;
  if (previous_stamp_ > 0 && dt > 0)
  {
    const double alpha = config_.twist_filter_alpha;
    velocity_ += alpha * ((input.object.position - previous_object_.position) / dt - velocity_);
    Eigen::Quaterniond delta = input.object.orientation * previous_object_.orientation.conjugate();
    if (delta.w() < 0)
      delta.coeffs() *= -1;
    const Eigen::AngleAxisd rotation(delta.normalized());
    omega_ += alpha * (rotation.axis() * rotation.angle() / dt - omega_);
  }
  if (previous_stamp_ == 0 || dt > 0)
  {
    previous_stamp_ = input.object_stamp;
    previous_object_ = input.object;
  }
  state.object_velocity = velocity_;
  state.object_angular_velocity = omega_;
  for (std::size_t i = 0; i < input.vehicles.size(); ++i)
  {
    const auto& v = input.vehicles[i];
    ContactState c;
    const Vec3 offset = v.base.orientation * config_.contact_offset;
    c.position = v.base.position + offset;
    const double angle = scene_.face_angles[i];
    const Vec3 local = input.object.orientation.conjugate() * (c.position - input.object.position);
    const Vec3 normal = surfaceNormal(scene_, local, Vec3(std::cos(angle), std::sin(angle), 0));
    c.normal = input.object.orientation * normal;
    Vec3 up = tangentProjection(c.normal, Vec3::UnitZ());
    if (up.norm() < 1e-6) up = c.normal.unitOrthogonal();
    c.tangent = up.normalized().cross(c.normal).normalized();
    c.velocity = v.velocity_world + (v.base.orientation * v.angular_velocity_base).cross(offset);
    c.bee_world.force = config_.observer_force_sign * config_.observer_force_scale *
                        (v.cog.orientation * (v.observer_cog.force - baseline_[i].force));
    c.bee_world.moment = config_.observer_force_sign * config_.observer_torque_scale *
                             (v.cog.orientation * (v.observer_cog.moment - baseline_[i].moment)) -
                         (c.position - v.cog.position).cross(c.bee_world.force);
    c.local_moment = Vec3(c.bee_world.moment.dot(c.normal), c.bee_world.moment.dot(c.tangent), c.bee_world.moment.z());
    const Vec3 arm = c.position - input.object.position;
    const Vec3 relative_velocity = c.velocity - velocity_ - omega_.cross(arm);
    c.radius = arm.dot(c.normal);
    c.tangent_position = arm.dot(c.tangent);
    c.radial_velocity = relative_velocity.dot(c.normal);
    c.tangent_velocity = relative_velocity.dot(c.tangent);
    c.relative_z = arm.z();
    c.relative_vz = relative_velocity.z();
    // Preserve legacy normal-force calibration in world axes; the six-axis
    // wrench baseline above is transported in CoG axes.
    const Vec3 force_delta = v.cog.orientation * v.observer_cog.force - world_baseline_[i];
    const double measured =
        std::max(0.0, config_.observer_force_sign * config_.observer_force_scale * force_delta.dot(c.normal));
    normal_forces_[i] += config_.force_filter_alpha * (measured - normal_forces_[i]);
    c.normal_force = normal_forces_[i];
    state.object_wrench.force -= c.bee_world.force;
    state.object_wrench.moment -= c.bee_world.moment + arm.cross(c.bee_world.force);
    state.contacts.push_back(c);
  }
  return state;
}
}  // namespace bee::grasp
