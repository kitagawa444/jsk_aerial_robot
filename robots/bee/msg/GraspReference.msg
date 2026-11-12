# All vectors use world axes. Velocity is for the representative contact point.
std_msgs/Header header
bool stop
bool calibrate
bool contact_tracking
bool support_tracking
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 normal
geometry_msgs/Vector3 object_position
float64 normal_force
float64 minimum_radius
# Positive upward support force per Bee, used to bootstrap support transfer.
# Blend towards measured external-force compensation, never add both in full.
float64 support_force
float64 observer_blend
float64 support_scale
# Approach/retreat and attitude reference; contact tracking replaces acceleration.
aerial_robot_msgs/FlightNav navigation
geometry_msgs/Vector3 baselink_rpy
