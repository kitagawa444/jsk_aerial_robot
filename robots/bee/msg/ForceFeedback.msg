# Local vector-force loop diagnostics; published only on actual control updates.
# World frame. Force error is Bee-on-object target minus estimated Bee-on-object.
std_msgs/Header header
geometry_msgs/Vector3 force_error
# The following fields have acceleration units [m/s^2], before combined limits.
geometry_msgs/Vector3 pi_unfiltered
geometry_msgs/Vector3 pi_filtered
geometry_msgs/Vector3 integral
geometry_msgs/Vector3 velocity_damping
bool correction_saturated
bool acceleration_limited
bool state_reset
