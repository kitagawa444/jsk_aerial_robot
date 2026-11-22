"""Contact-axis interpretation of references produced by the grasp allocator."""
import numpy as np


def vector(v):
    return np.array([v.x, v.y, v.z], dtype=float)


def contact_normal(msg):
    if not msg.acceleration_bounds:
        raise ValueError('Grasp command has no contact frame')
    q = msg.acceleration_orientation
    x, y, z, w = np.array([q.x, q.y, q.z, q.w])
    if not np.isclose(x*x + y*y + z*z + w*w, 1., atol=1e-5):
        raise ValueError('Invalid contact frame quaternion')
    return np.array([1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y)])


def reference_topic(name, available):
    current = f'/{name}/command'
    if current in available:
        return current
    legacy = f'/{name}/controller/reference'
    return legacy if legacy in available else f'/{name}/grasp/reference'


def force_control_enabled(msg):
    return getattr(msg, 'enable_force_control', getattr(msg, 'force_tracking', getattr(msg, 'wrench_tracking', False)))


def bag_force_command(msg):
    if msg.stop:
        return None
    if hasattr(msg, 'force'):
        if not force_control_enabled(msg) or not msg.acceleration_bounds:
            return None
        nav = msg.navigation
        acceleration = np.array([nav.target_acc_x, nav.target_acc_y, nav.target_acc_z])
        return vector(msg.force), contact_normal(msg), acceleration, True
    # Old bags contain the message definition and can be read without its package.
    if msg.calibrate or not msg.contact_tracking:
        return None
    normal = vector(msg.normal)
    return (-msg.normal_force*normal + vector(msg.support_vector), normal,
            vector(msg.acceleration), msg.wrench_tracking and msg.support_tracking)
