# Generic Bee reference. Force/damping/box vectors use header.frame_id (world).
# navigation retains its own FlightNav control_frame and target conventions.
std_msgs/Header header
bool stop
aerial_robot_msgs/FlightNav navigation
# Absolute world-frame baselink RPY [rad]. Leave all navigation rotation modes at NO_NAVIGATION.
geometry_msgs/Vector3 baselink_rpy
# Force the Bee applies to its environment [N]. World/CoG, XY ACC or position/velocity; Z ACC or position with force feedforward.
bool enable_force_control
bool force_integral_enabled
geometry_msgs/Vector3 force
# Velocity of a point fixed in baselink; used for relative-motion damping.
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 velocity_point
# Optional acceleration box in the supplied orthonormal frame (world <- box).
# Also defines axes for directional force gains. Without bounds, use world axes.
bool acceleration_bounds
geometry_msgs/Quaternion acceleration_orientation
geometry_msgs/Vector3 acceleration_min
geometry_msgs/Vector3 acceleration_max
# Upper-level control may explicitly clear force feedback state.
bool reset_force_feedback
