# Complete runtime parameter set; all fields are finite and nonnegative.
# P axes follow BeeCommand.acceleration_orientation when bounds are used,
# otherwise world axes. I/damping are isotropic; integral limit is world per-axis.
geometry_msgs/Vector3 force_kp
float64 force_ki
# Integral acceleration limit [m/s^2 per axis].
float64 force_integral_limit
float64 velocity_damping
# Time constants [s]. Zero disables the corresponding low-pass filter.
float64 force_filter_tau
float64 external_filter_tau
