# Bee whole-body kinodynamic MPC

`mujoco_triple_transport_place_demo.py` supports two control strategies through
the private `~control_strategy` parameter.

- `legacy` (default): the existing observer/admittance transport controller.
- `whole_body_mpc` (alias: `science_robotics`): a reduced whole-body
  kinodynamic planner inspired by the finite-horizon architecture in the
  Science Robotics cooperative aerial manipulation work.

The planner jointly optimizes the prism pose/twist and one resultant contact
force for each Bee.  Each resultant represents the four rubber-ball contacts
on one foot plane.  The optimization enforces unilateral normal force,
friction cones, force slew, object velocity/acceleration, angular velocity,
vehicle thrust, and vehicle tilt constraints.  The existing state machine is
retained for contact acquisition, release, retreat, and failsafe handling.
During continuous contact, the momentum observer supplies the inner normal
force servo and low-pass contact-reaction correction; force-site values are
monitoring only.

At every replan, the demo recomputes the three object-to-contact arms from the
current object pose and each Bee's four-ball representative center.  It also
recomputes the object-to-Bee-CG arms from observer odometry.  Both sets are
passed as online MPC parameters and held constant only across the 0.6 s
prediction horizon.  Thus sliding or formation displacement changes the
predicted object moment and vehicle acceleration without regenerating the
acados solver.  The initial relative contact position remains a separate,
fixed reference for slip monitoring and the existing local correction.

The demo also estimates the six-axis model error from the difference between
the mocap-derived object acceleration and the acceleration predicted by the
last applied MPC sample.  A low-pass, bounded additive linear/angular
acceleration is passed into every node of the next horizon.  This makes the
optimizer redistribute its contact forces to reject persistent unmodelled
effects instead of applying the residual only as an after-the-fact Bee
command.  The estimate is reset during the pedestal support-force ramp so the
pedestal reaction is not learned as a grasp disturbance.

## Run

Online optimization needs MuJoCo time to remain close to wall time so the
observer/contact inner loops see realistic timing.  The default acados backend
generates and compiles its solver once at planner initialization, then runs a
three-sweep SQP-RTI update with partial-condensing HPIPM at every replan.

```bash
roslaunch bee mujoco_multi_module.launch \
  headless:=true launch_rviz:=false spawn_destination_pedestal:=true \
  real_time_factor:=1.0
```

Planner mode:

```bash
rosrun bee mujoco_triple_transport_place_demo.py \
  _control_strategy:=whole_body_mpc
```

Legacy mode remains the default:

```bash
rosrun bee mujoco_triple_transport_place_demo.py
```

## Main planner parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `~planner_horizon` | 0.6 s | finite prediction horizon |
| `~planner_nodes` | 6 | shooting intervals |
| `~planner_rate` | 5 Hz | requested replan rate |
| `~planner_object_acceleration_limit` | 0.02 m/s² | translational object acceleration bound |
| `~planner_horizontal_velocity_constraint_limit` | 0.08 m/s | recoverable OCP XY velocity region; trajectory speed is unchanged |
| `~planner_vertical_velocity_constraint_limit` | 0.08 m/s | recoverable OCP Z velocity region; trajectory speed is unchanged |
| `~planner_friction_coefficient` | scene value | contact friction used by the OCP |
| `~planner_friction_margin` | 0.90 | usable fraction of the friction cone |
| `~planner_normal_force_min/max` | 1/8 N | force bounds per Bee |
| `~planner_force_rate_limit` | 5 N/s | tangential/vertical force slew |
| `~planner_normal_force_rate_limit` | 2 N/s | normal force slew |
| `~planner_disturbance_time_constant` | 0.75 s | bandwidth of model-error learning |
| `~planner_disturbance_gain` | 1.0 | innovation gain before filtering |
| `~planner_disturbance_linear_limit` | 0.10 m/s² | norm bound on learned linear disturbance |
| `~planner_disturbance_angular_limit` | 0.50 rad/s² | norm bound on learned angular disturbance |
| `~planner_solver_backend` | `acados` | `acados`, `casadi`, or `scipy` |
| `~planner_casadi_nlp_solver` | `ipopt` | `ipopt` or `sqpmethod` |
| `~planner_stale_timeout` | 3 s | wall-time age before failsafe |
| `~planner_max_open_loop_time` | automatic | `0`: full horizon; positive: clamp sampling age |
| `~lift_progress_support_force_ratio` | 0.98 | fraction of object weight supported before the lift watchdog starts |

All existing mission parameters, including `~target_normal_force`, object
mass, destination, lift height, and transport limits remain valid.

## Diagnostics

- `/mujoco_triple_transport_place/planner/status`
- `/mujoco_triple_transport_place/planner/metrics`
- `/mujoco_triple_transport_place/planner/external_linear_acceleration`
- `/mujoco_triple_transport_place/planner/external_angular_acceleration`
- `/mujoco_triple_transport_place/planner/{bee1,bee2,bee3}/contact_force`

The metrics vector contains solver seconds, maximum friction-cone ratio, and
objective value.  The contact-force messages use the component order
`[face normal, face tangent, object Z]`.  The CSV wrench log also records the
measured and learned six-axis acceleration terms.
