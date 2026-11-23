"""Kinematic alignment, surface sampling and temporal filtering of joint targets."""
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

from .mesh_geometry import homogeneous


def rigid_fit(source, target):
    """Rigid palm alignment in metres; no reflection or mesh scaling."""
    source, target = np.asarray(source), np.asarray(target)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Palm anchors must have matching (N,3) shapes")
    a, b = source - source.mean(0), target - target.mean(0)
    if not np.isfinite(target).all() or np.linalg.matrix_rank(a) < 2 or np.linalg.matrix_rank(b) < 2:
        raise ValueError("Degenerate palm anchors")
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target.mean(0) - rotation @ source.mean(0)
    return transform


def palm_pose(robot_knuckles, robot_tips, human_knuckles, human_wrist):
    """Align anatomical palm axes and MCP centres, not mechanical root origins.

    The knuckle centre sets translation. The palm width and length axes set
    rotation even when the mechanical wrist origin differs from the MANO wrist.
    """
    def basis(width, length):
        if not np.isfinite(width).all() or not np.isfinite(length).all() or np.linalg.norm(width) < 1e-6:
            raise ValueError("Degenerate palm width")
        x = width / np.linalg.norm(width)
        y = length - x * np.dot(length, x)
        if np.linalg.norm(y) < 1e-6:
            raise ValueError("Degenerate palm length")
        y /= np.linalg.norm(y)
        return np.column_stack([x, y, np.cross(x, y)])

    robot_knuckles, robot_tips, human_knuckles = map(np.asarray, (robot_knuckles, robot_tips, human_knuckles))
    robot_centre, human_centre = robot_knuckles.mean(0), human_knuckles.mean(0)
    rb = basis(robot_knuckles[0] - robot_knuckles[-1], robot_tips.mean(0) - robot_centre)
    hb = basis(human_knuckles[0] - human_knuckles[-1], human_centre - human_wrist)
    transform = np.eye(4)
    transform[:3, :3] = hb @ rb.T
    transform[:3, 3] = human_centre - transform[:3, :3] @ robot_centre
    return transform


def surface_samples(meshes, count):
    """Deterministic, area-weighted triangle samples, including triangle interiors."""
    rng = np.random.default_rng(20261112)
    output = {}
    for link, items in meshes.items():
        triangles = np.concatenate([mesh.triangles for mesh in items])
        area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                       triangles[:, 2] - triangles[:, 0]), axis=1)
        if not np.isfinite(area).all() or area.sum() <= 0:
            raise ValueError(f'Invalid visual mesh for {link}')
        picked = triangles[rng.choice(len(triangles), count, p=area / area.sum())]
        uv = rng.random((count, 2))
        uv[uv.sum(axis=1) > 1] = 1 - uv[uv.sum(axis=1) > 1]
        output[link] = picked[:, 0] + uv[:, :1] * (picked[:, 1] - picked[:, 0]) + uv[:, 1:] * (picked[:, 2] - picked[:, 0])
    return output


def smooth_rotations(matrices, sigma):
    """Local quaternion means on SO(3), including across the +/-pi branch cut."""
    if sigma <= 0 or not np.isfinite(sigma):
        raise ValueError('Smoothing sigma must be positive and finite')
    radius = int(np.ceil(3 * sigma))
    offsets = np.arange(-radius, radius + 1)
    weights = np.exp(-offsets ** 2 / (2 * sigma ** 2))
    return np.array([Rotation.from_matrix(matrices[np.clip(i + offsets, 0, len(matrices) - 1)])
                     .mean(weights).as_matrix() for i in range(len(matrices))])


def filter_trajectory(qpos, roots_cv, camera_world, sigma):
    world = np.linalg.inv(camera_world) @ roots_cv
    filtered = world.copy()
    filtered[:, :3, 3] = gaussian_filter1d(world[:, :3, 3], sigma, axis=0, mode='nearest')
    filtered[:, :3, :3] = smooth_rotations(world[:, :3, :3], sigma)
    q = gaussian_filter1d(qpos.astype(float), sigma, axis=0, mode='nearest')
    return q, filtered, world


def camera_extrinsics(source, ids):
    report = json.loads((source / 'geometry_report.json').read_text())
    camera = json.loads((Path(report['source_processed']) / 'camera.json').read_text())
    path = Path(report['camera_calibration'])
    calib = json.loads(path.read_text())['extrinsics'][camera['camera_id']]
    return np.array([homogeneous(calib[f'{int(i):05d}']) for i in ids]), path
