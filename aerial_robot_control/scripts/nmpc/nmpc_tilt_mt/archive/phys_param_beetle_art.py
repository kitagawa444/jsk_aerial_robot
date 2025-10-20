import os
import yaml


physical_params = {
    "mass": 2.773,  # no battery: 2.773-0.821=1.952 ; with battery: 2.773; in sim: 2.99194 # kg
    "gravity": 9.798,  #9.80665 # m/s^2    Tokyo 9.798; in sim: 9.80665
    "inertia_diag": [0.04170, 0.03945, 0.07068],  #[ 0.0621273, 0.0927884, 0.130061 ] # kg m^2
    "dr1": 1,  # direction of motor 1
    "p1": [0.137712, 0.137882, 0.0297217],  # position of motor 1
    "dr2": -1,  # direction of motor 2
    "p2": [-0.137745, 0.137882, 0.0297217],  # position of motor 2
    "dr3": 1,  # direction of motor 3
    "p3": [-0.137745, -0.138284, 0.0297217],  # position of motor 3
    "dr4": -1,  # direction of motor 4
    "p4": [0.137712, -0.138284, 0.0297217],  # position of motor 4
    "kq_d_kt": 0.0153,  #0.0171998
    "num_rotors": 4,
    "t_rotor": 0.0942,  # real condition w/o propeller 0.0942 s; sim: try to set 0.0942 s
    "num_servos": 4,
    # the identified params in simulation are 1/1.619, 1/1.646, 1/1.530, 1/1.542. so the average is 1/1.584 = 0.6312 s
    # the identified params in real is 1/0.0501 ~ 20s (slowest servo of four servos); the weighted average is ~ 15s
    "t_servo": 0.085883,  # real in flight 0.044228, no propeller 0.085883; sim: 0.15858 sec
    # profile drag model
    "c0": -0.00278,
    "c1": -0.02147,
    "c2": 0.08134,
    "c3": 0.00470,
    "c4": -0.02439,
}

mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
dr3 = physical_params["dr3"]
p3_b = physical_params["p3"]
dr4 = physical_params["dr4"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

t_servo = physical_params["t_servo"]  # Time constant of servo
t_rotor = physical_params["t_rotor"]  # Time constant of rotor

c0 = physical_params["c0"]
c1 = physical_params["c1"]
c2 = physical_params["c2"]
c3 = physical_params["c3"]
c4 = physical_params["c4"]

# fmt: off
# concatenate the parameters to make a new list
physical_param_list = [
    mass, gravity, Ixx, Iyy, Izz,
    kq_d_kt,
    dr1, p1_b[0], p1_b[1], p1_b[2],
    dr2, p2_b[0], p2_b[1], p2_b[2],
    dr3, p3_b[0], p3_b[1], p3_b[2],
    dr4, p4_b[0], p4_b[1], p4_b[2],
    t_rotor, t_servo,
]
# fmt: on

physical_param_list.extend([0.0, 0.0, 0.0])
physical_param_list.extend([1.0, 0.0, 0.0, 0.0])  # to compatible with end-effectors.
