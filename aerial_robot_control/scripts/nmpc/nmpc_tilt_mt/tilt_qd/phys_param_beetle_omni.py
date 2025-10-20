"""
 Created by li-jinjie on 24-10-5.
"""

physical_params = {
    # with original ball and battery
    "mass": 3.0386,  # kg
    "gravity": 9.798,  #9.80665 # m/s^2    Tokyo 9.798; in sim: 9.80665
    "inertia_diag": [0.0627958, 0.0620796, 0.0948795],  # kg m^2
    "dr1": 1,  # direction of motor 1
    "p1": [0.194824, 0.194652, -0.00368224],  # position of motor 1
    "dr2": -1,  # direction of motor 2
    "p2": [-0.194837, 0.194652, -0.00368224],  # position of motor 2
    "dr3": 1,  # direction of motor 3
    "p3": [-0.194837, -0.195008, -0.00368224],  # position of motor 3
    "dr4": -1,  # direction of motor 4
    "p4": [0.194824, -0.195008, -0.00368224],  # position of motor 4
    "kq_d_kt": 0.0165,
    "num_rotors": 4,
    "t_rotor": 0.0942,
    "num_servos": 4,
    "t_servo": 0.0480,  # Dynamixel servo: XC330-T181 with self-tuned PID. As contrary, the previous kondo's value was 0.085883
    "ball_effector_p": [0, 0, 0.264],  # m
    "ball_effector_q": [1, 0, 0, 0],  # quaternion, qw, qx, qy, qz
}

mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
dr2 = physical_params["dr2"]
dr3 = physical_params["dr3"]
dr4 = physical_params["dr4"]
p1_b = physical_params["p1"]
p2_b = physical_params["p2"]
p3_b = physical_params["p3"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

t_servo = physical_params["t_servo"]  # time constant of servo
t_rotor = physical_params["t_rotor"]  # time constant of rotor

ball_effector_p = physical_params["ball_effector_p"]
ball_effector_q = physical_params["ball_effector_q"]  # qw, qx, qy, qz

# concatenate the parameters to make a new list
# fmt: off
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

# Add ball effector parameters
physical_param_list.extend(ball_effector_p)
physical_param_list.extend(ball_effector_q)
