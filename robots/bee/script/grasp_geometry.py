"""Offline geometry shared by scene layout and RViz (metres, CoM origin)."""
import math

EXTRA_SHAPES = ("cylinder", "frustum", "capsule", "rock")


def rock_geometry(height=.6, contact=0.):
    """Fixed asymmetric convex rock: broad side faces, bevels, flat support.

    Halfspaces and mesh are one source of truth. Integrate tetrahedra to obtain
    the exact uniform-density CoM and full inertia, then recenter all geometry.
    No random shape generation is performed between repeated trials.
    """
    import itertools
    import numpy as np
    if not math.isfinite(height) or height <= 0 or not math.isfinite(contact):
        raise RuntimeError('invalid rock height/contact')
    raw = np.array([
        [1, 0, .10, .34], [0, 1, .16, .31],
        [-1, 0, .04, .30], [0, -1, .07, .32],
        [1, 1, .12, .50], [-1, 1, .19, .46],
        [-1, -1, .09, .49], [1, -1, .15, .52],
        [.15, -.08, 1, .28], [-.13, .10, 1, .29],
        [0, 0, -1, .30],
        [1, 0, -.70, .47], [-1, 0, -.65, .44],
        [0, 1, -.75, .45], [0, -1, -.80, .47],
    ], dtype=float)
    normals = raw[:, :3] / np.linalg.norm(raw[:, :3], axis=1)[:, None]
    offsets = raw[:, 3] / np.linalg.norm(raw[:, :3], axis=1)
    vertices = []
    for triple in itertools.combinations(range(len(raw)), 3):
        matrix = normals[list(triple)]
        if abs(np.linalg.det(matrix)) < 1e-9:
            continue
        p = np.linalg.solve(matrix, offsets[list(triple)])
        if np.max(normals @ p - offsets) <= 1e-9 and not any(np.linalg.norm(p - q) < 1e-8 for q in vertices):
            vertices.append(p)
    vertices = np.array(vertices)
    scale = height / np.ptp(vertices[:, 2])
    vertices *= scale
    offsets *= scale
    faces = []
    for normal, offset in zip(normals, offsets):
        ids = np.where(np.abs(vertices @ normal - offset) < 1e-8)[0]
        if len(ids) < 3:
            continue
        centre = np.mean(vertices[ids], axis=0)
        u = vertices[ids[0]] - centre
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        ids = sorted(ids, key=lambda i: math.atan2(np.dot(vertices[i] - centre, v), np.dot(vertices[i] - centre, u)))
        faces.extend((int(ids[0]), int(ids[i]), int(ids[i+1])) for i in range(1, len(ids)-1))
    volume, first, second = 0., np.zeros(3), np.zeros((3, 3))
    for face in faces:
        a, b, c = vertices[list(face)]
        dv = float(np.dot(a, np.cross(b, c))) / 6
        total = a + b + c
        volume += dv
        first += dv * total / 4
        second += dv / 20 * (np.outer(total, total) + np.outer(a, a) + np.outer(b, b) + np.outer(c, c))
    if volume <= 0:
        raise RuntimeError('invalid rock mesh winding')
    centre = first / volume
    covariance = second / volume - np.outer(centre, centre)
    inertia = np.trace(covariance) * np.eye(3) - covariance
    vertices -= centre
    offsets -= normals @ centre
    bottom = -float(np.min(vertices[:, 2]))
    if not -bottom < contact < height - bottom:
        raise RuntimeError('rock contact height outside object')
    radii = []
    for i in range(4):
        ray = np.array([math.cos(i * math.pi / 2), math.sin(i * math.pi / 2), 0])
        reach = (offsets[i] - normals[i, 2] * contact) / np.dot(normals[i], ray)
        p = reach * ray + np.array([0, 0, contact])
        # Contact must lie well inside its intended broad face, not on a bevel.
        if np.min(np.delete(offsets - normals @ p, i)) < .08 * scale:
            raise RuntimeError('rock contact too close to a face edge')
        radii.append(float(reach))
    return dict(shape='rock', height=height, bottom=bottom, volume=float(volume),
                radius=0., top_radius=0., length=0., radii=radii,
                size=[height], vertices=vertices.tolist(), faces=[list(f) for f in faces],
                normals=normals.tolist(), offsets=offsets.tolist(),
                contact_normals=normals[:4].tolist(),
                unit_inertia=[float(inertia[i, j]) for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))])


def extra_shape_geometry(parameters):
    shape = str(parameters.get("object_shape", "")).lower()
    if shape not in EXTRA_SHAPES:
        return None
    height = float(parameters.get("object_height", .6))
    contact = float(parameters.get("contact_height", 0.0))
    if shape == 'rock':
        return rock_geometry(height, contact)
    radius = float(parameters.get("object_radius", .3))
    top = radius
    length = 0.0
    bottom = height / 2
    if shape == "frustum":
        radius = float(parameters.get("object_radius_bottom", .34))
        top = float(parameters.get("object_radius_top", .26))
    if not all(math.isfinite(v) and v > 0 for v in (height, radius, top)):
        raise RuntimeError("shape dimensions must be finite and positive")
    if shape == "frustum":
        total = radius ** 2 + radius * top + top ** 2
        bottom = height * (radius ** 2 + 2 * radius * top + 3 * top ** 2) / (4 * total)
        volume = math.pi * height * total / 3
        reach = radius + (top - radius) * (contact + bottom) / height
        radii = [reach] * 4
        size = [radius, top, height]
    elif shape == "capsule":
        # Horizontal, body-X axis. object_length includes both hemispheres.
        full_length = float(parameters.get("object_length", 1.0))
        length = full_length - 2 * radius
        if (not math.isfinite(full_length) or length <= 0 or
                abs(height - 2 * radius) > 1e-9):
            raise RuntimeError("capsule requires length > 2*radius and height == 2*radius")
        volume = math.pi * radius ** 2 * length + 4 * math.pi * radius ** 3 / 3
        cross_radius = math.sqrt(max(0.0, radius ** 2 - contact ** 2))
        radii = [length / 2 + cross_radius, cross_radius] * 2
        size = [radius, length]
    else:
        volume = math.pi * radius ** 2 * height
        radii = [radius] * 4
        size = [radius, height]
    if not math.isfinite(contact) or not -bottom < contact < height - bottom:
        raise RuntimeError("contact_height must be strictly inside the object's vertical bounds")
    return dict(shape=shape, height=height, bottom=bottom, volume=volume,
                radius=radius, top_radius=top, length=length, radii=radii, size=size)


def surface_triangles(geometry, segments=128, cap_steps=16):
    """CoM-centred frustum/capsule triangles for one RViz TRIANGLE_LIST."""
    g = geometry
    if g['shape'] == 'rock':
        return [g['vertices'][i] for face in g['faces'] for i in face]
    if g['shape'] == 'frustum':
        rings = [(-g['bottom'], g['radius']), (g['height'] - g['bottom'], g['top_radius'])]
        # Include end-cap centres as zero-radius rings.
        rings = [(rings[0][0], 0.0)] + rings + [(rings[-1][0], 0.0)]
        axis_x = False
    elif g['shape'] == 'capsule':
        rings = []
        for centre, start in ((-g['length'] / 2, -math.pi / 2), (g['length'] / 2, 0.0)):
            for i in range(cap_steps + 1):
                angle = start + i * math.pi / (2 * cap_steps)
                rings.append((centre + g['radius'] * math.sin(angle), g['radius'] * math.cos(angle)))
        axis_x = True
    else:
        raise ValueError('triangle surface is only needed for frustum/capsule')
    vertices = []
    for axial, radius in rings:
        for i in range(segments):
            u, v = radius * math.cos(2 * math.pi * i / segments), radius * math.sin(2 * math.pi * i / segments)
            vertices.append((axial, u, v) if axis_x else (u, v, axial))
    triangles = []
    for j in range(len(rings) - 1):
        for i in range(segments):
            a, b = j * segments + i, j * segments + (i + 1) % segments
            for face in ((a, b, b + segments), (a, b + segments, a + segments)):
                triangles.extend(vertices[k] for k in face)
    return triangles
