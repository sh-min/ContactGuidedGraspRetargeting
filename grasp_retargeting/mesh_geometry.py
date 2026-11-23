"""Read HRDexDB geometry and project penetrating MANO targets onto the object.

The projected vertices are retargeting targets, not a new valid MANO pose.
Open object meshes use an explicitly recorded convex-hull collision proxy.
Original object triangles are retained for rendering.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import pickle

import numpy as np
from scipy.spatial import ConvexHull
from scipy.ndimage import map_coordinates

TIP_IDS = [744, 320, 443, 554, 671]
JOINT_ORDER = [0, 13, 14, 15, 16, 1, 2, 3, 17, 4, 5, 6, 18, 10, 11, 12, 19, 7, 8, 9, 20]


def transform_points(points, transform):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def inverse_points(points, transform):
    return (np.asarray(points) - transform[:3, 3]) @ transform[:3, :3]


def homogeneous(value):
    value = np.asarray(value, float)
    if value.shape == (3, 4):
        value = np.vstack([value, [0, 0, 0, 1]])
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("Expected finite 3x4/4x4 rigid transform")
    if not np.allclose(value[3], [0, 0, 0, 1]) or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-4):
        raise ValueError("Invalid homogeneous rigid transform")
    if not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-4):
        raise ValueError("Reflected coordinate system")
    return value


def read_mano(path):
    vertices, faces = [], []
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith("f "):
                face = [int(v.split('/')[0]) - 1 for v in line.split()[1:]]
                if len(face) != 3:
                    raise ValueError("MANO OBJ must be triangulated")
                faces.append(face)
    vertices, faces = np.asarray(vertices), np.asarray(faces, np.int32)
    if vertices.shape != (778, 3) or not np.isfinite(vertices).all():
        raise ValueError(f"Wrong MANO topology: {path}")
    return vertices, faces


def mano_joints(vertices, regressor):
    joints16 = np.asarray(regressor @ vertices)
    return np.concatenate([joints16, vertices[TIP_IDS]])[JOINT_ORDER]


def load_mano_model(path):
    """Read the joint regressor, skinning weights and faces of a right-hand MANO model."""
    path = Path(path)
    if path.suffix.lower() == '.npz':
        with np.load(path, allow_pickle=False) as model:
            regressor, weights, faces = (model[key] for key in ('J_regressor', 'weights', 'f'))
    elif path.suffix.lower() == '.pkl':
        # Use only a trusted, licensed MANO pickle. The NPZ form contains numeric arrays.
        with path.open('rb') as stream:
            model = pickle.load(stream, encoding='latin1')
        regressor, weights, faces = (model[key] for key in ('J_regressor', 'weights', 'f'))
    else:
        raise ValueError('Expected a right-hand MANO .pkl or .npz model')
    # Standard pickle regressors are sparse; the portable NPZ uses a dense matrix.
    if hasattr(regressor, 'toarray'):
        regressor = regressor.toarray()
    regressor, weights, faces = map(np.asarray, (regressor, weights, faces))
    if (regressor.shape != (16, 778) or weights.shape != (778, 16)
            or faces.ndim != 2 or faces.shape[1] != 3 or not len(faces)
            or not np.issubdtype(faces.dtype, np.integer) or faces.min() < 0 or faces.max() >= 778
            or not np.isfinite(regressor).all() or not np.isfinite(weights).all()):
        raise ValueError('Unexpected MANO joint regressor, skinning weights or face topology')
    return regressor, weights, faces


def hull_plane_distance(points, equations):
    """Negative inside; exact distance to nearest supporting plane inside hull."""
    output = np.empty(len(points))
    for start in range(0, len(points), 512):
        block = np.asarray(points[start:start + 512], float)
        output[start:start + len(block)] = (block @ equations[:, :3].T + equations[:, 3]).max(axis=1)
    return output


def project_outside(points, proxy, margin=0.001, tolerance=0.0002):
    import trimesh
    if margin < 0 or tolerance < 0:
        raise ValueError("Projection margin/tolerance must be nonnegative")
    equations = ConvexHull(proxy.vertices).equations
    signed = hull_plane_distance(points, equations)
    penetrating = signed < -tolerance
    projected = np.asarray(points).copy()
    if penetrating.any():
        nearest, _, triangles = trimesh.proximity.closest_point(proxy, projected[penetrating])
        projected[penetrating] = nearest + margin * proxy.face_normals[triangles]
    return projected, penetrating, np.maximum(-signed, 0)


def make_collision_grid(proxy, resolution=48):
    """Convex support-plane field; interpolation is not a collision guarantee."""
    if resolution < 8:
        raise ValueError("Collision grid resolution must be >= 8")
    equations = ConvexHull(proxy.vertices).equations
    bounds = np.array(proxy.bounds) + [[-.015, -.015, -.015], [.015, .015, .015]]
    step = (bounds[1] - bounds[0]) / (resolution - 1)
    axes = [np.linspace(bounds[0, i], bounds[1, i], resolution) for i in range(3)]
    points = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    field = hull_plane_distance(points, equations).reshape((resolution,) * 3).astype(np.float32)
    return field, bounds[0], step, equations


def sample_collision_grid(points, field, grid_origin, step):
    coords = (np.asarray(points) - grid_origin) / step
    # Outside the bounding grid is outside the enclosed collision proxy.
    return map_coordinates(field, coords.T, order=1, mode="constant", cval=.1, prefilter=False)


def run(args):
    import trimesh
    source = args.processed.resolve()
    camera = json.loads((source / 'camera.json').read_text())
    if args.episode is None:
        run_manifest = json.loads((source / 'run.json').read_text())
        # The manifest points at the recorded episode's vid/<camera>.mp4.
        episode = (source / Path(run_manifest['source']).expanduser()).resolve().parents[1]
    else:
        episode = args.episode.expanduser().resolve()
    root = episode.parents[2]
    name = episode.parent.name
    mesh_path = root / 'assets/mesh_v2' / name / f'{name}.obj'
    pose_path = root / 'object_6d_pose_v2/human' / f'{name}_{episode.name}.npz'
    calibration_path = episode / 'cam_param/ego_calib.json'
    calibration = json.loads(calibration_path.read_text())
    extrinsics = calibration['extrinsics'][camera['camera_id']]
    stop = camera['frames'] if args.max_frames is None else args.start_frame + args.max_frames
    if not 0 <= args.start_frame < stop <= camera['frames']:
        raise ValueError('Requested interval lies outside the video')
    ids = np.arange(args.start_frame, stop, dtype=np.int32)
    with np.load(pose_path, allow_pickle=False) as poses:
        object_world_poses = []
        for t in ids:
            if f'frame_{t}' not in poses or f'{t:05d}' not in extrinsics:
                raise ValueError(f'Missing exact object/camera frame {t}')
            object_world_poses.append(homogeneous(poses[f'frame_{t}']))
    obj = trimesh.load(mesh_path, force='mesh', process=False)
    extent = float(np.max(obj.extents))
    if not .001 < extent < 2:
        raise ValueError('Object mesh must be in metres')
    proxy = obj.convex_hull
    regressor, weights, model_faces = load_mano_model(args.mano_model)
    part = weights.argmax(1)
    labels = np.full(778, -1, np.int8)
    for finger, joints in enumerate(([13, 14, 15], [1, 2, 3], [4, 5, 6], [10, 11, 12], [7, 8, 9])):
        labels[np.isin(part, joints)] = finger
    raw, corrected, joints, object_cv, contacts, contact_points, moved, penetration = [], [], [], [], [], [], [], []
    for t, object_world in zip(ids, object_world_poses):
        vertices, faces = read_mano(episode / 'hand/mano' / f'{t:05d}.obj')
        if not np.array_equal(np.sort(faces, axis=1), np.sort(model_faces, axis=1)):
            raise ValueError('Source MANO vertex/face ordering differs from the joint regressor')
        camera_world = homogeneous(extrinsics[f'{t:05d}'])
        local = inverse_points(vertices, object_world)
        projected, changed, depth = project_outside(local, proxy, args.margin, args.penetration_tolerance)
        nearest, distances, _ = trimesh.proximity.closest_point(proxy, local)
        # Contact is a geometric proximity target, not a tactile/force label.
        contact = (distances <= args.contact_distance) | changed
        pose = camera_world @ object_world
        raw.append(transform_points(vertices, camera_world))
        corrected.append(transform_points(projected, pose))
        joints.append(mano_joints(corrected[-1], regressor))
        object_cv.append(pose)
        contacts.append(contact)
        contact_points.append(nearest)
        moved.append(changed)
        penetration.append(depth)
        if len(raw) % 15 == 0:
            print(f'geometry {len(raw)}/{len(ids)}', flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    field, origin, step, equations = make_collision_grid(proxy)
    np.savez_compressed(args.output / 'geometry.npz', source_frame_ids=ids,
        raw_mano_cv=np.asarray(raw), corrected_targets_cv=np.asarray(corrected), joints_cv=np.asarray(joints),
        object_pose_cv=np.asarray(object_cv), mano_faces=faces, finger_labels=labels,
        contact_mask=np.asarray(contacts), contact_points_object=np.asarray(contact_points),
        moved_vertices=np.asarray(moved), mano_penetration_m=np.asarray(penetration),
        object_vertices=obj.vertices, object_faces=obj.faces, collision_field=field,
        grid_origin=origin, grid_step=step, hull_equations=equations,
        collision_proxy='convex_hull', K=np.asarray(camera['K']),
        fps=camera['fps'], image_size=[camera['width'], camera['height']])
    report = dict(source_processed=str(source), episode=str(episode), object_mesh=str(mesh_path),
        object_poses=str(pose_path), camera_calibration=str(calibration_path),
        source_frames=ids.tolist(), original_watertight=bool(obj.is_watertight),
        collision_proxy='convex_hull', convex_volume_ratio=float(proxy.volume / abs(obj.volume)),
        margin_m=args.margin, contact_distance_m=args.contact_distance,
        moved_vertex_count=int(np.sum(moved)), max_raw_mano_penetration_mm=float(np.max(penetration) * 1000),
        target_note='Projected vertices are optimization targets, not a valid articulated MANO pose.')
    (args.output / 'geometry_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--processed', type=Path, required=True)
    p.add_argument('--episode', type=Path, help='HRDexDB episode directory, overriding the path in run.json')
    p.add_argument('--mano-model', type=Path, required=True, help='Trusted right-hand MANO pickle or numeric NPZ')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-frame', type=int, default=0)
    p.add_argument('--max-frames', type=int, default=None)
    p.add_argument('--margin', type=float, default=.001)
    p.add_argument('--penetration-tolerance', type=float, default=.0002)
    p.add_argument('--contact-distance', type=float, default=.006)
    run(p.parse_args())
