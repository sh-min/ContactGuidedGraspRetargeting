"""Visual retargeting for native two-, three-, four- and five-finger MJCF hands.

These are geometric visual targets, not demonstrated physical grasps. The
Robotiq native tendon control is retained through a settled linkage lookup.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.distance import cdist

from .mesh_geometry import transform_points, sample_collision_grid, hull_plane_distance
from .kinematics import camera_extrinsics, filter_trajectory, palm_pose, rigid_fit, surface_samples
from .native_hands import NativeHand, HAND_IDS


def pinch_pose(robot_tips, human_tips, human_wrist):
    def basis(width, length):
        x = width / max(np.linalg.norm(width), 1e-8)
        y = length - x * np.dot(x, length); y /= max(np.linalg.norm(y), 1e-8)
        z = np.cross(x, y)
        if np.linalg.norm(z) < .9: raise ValueError('Degenerate pinch frame')
        return np.column_stack([x, y, z])
    rb = basis(robot_tips[1] - robot_tips[0], robot_tips.mean(axis=0))
    hb = basis(human_tips[1] - human_tips[0], human_tips.mean(axis=0) - human_wrist)
    pose = np.eye(4); pose[:3, :3] = hb @ rb.T
    pose[:3, 3] = human_tips.mean(axis=0) - pose[:3, :3] @ robot_tips.mean(axis=0)
    return pose


def run(args):
    hand = NativeHand(args.hand)
    output = args.output
    keys = ('source_frame_ids', 'joints_cv', 'contact_mask', 'finger_labels', 'contact_points_object',
            'collision_field', 'grid_origin', 'grid_step', 'object_pose_cv', 'hull_equations', 'fps')
    with np.load(output / 'geometry.npz', allow_pickle=False) as archive:
        data = {key: archive[key] for key in keys}
    hand.prepare_linkage(output)
    neutral_controls = np.clip(hand.entry['neutral_ctrl'], *hand.control_bounds.T)
    neutral_q = hand.controls_to_qpos(neutral_controls)
    hand.set_qpos(neutral_q)
    neutral_tips = hand.tips()
    neutral_knuckles = None
    if args.hand == 'leap_hand':
        neutral_knuckles = np.array([hand.data.xpos[hand.model.body(name).id] for name in ('if_bs', 'mf_bs', 'rf_bs')])
    elif args.hand == 'shadow_hand':
        neutral_knuckles = np.array([hand.data.xpos[hand.model.body(name).id]
                                     for name in ('rh_ffknuckle', 'rh_mfknuckle', 'rh_rfknuckle')])
    samples = surface_samples({str(gid): [hand.geom_mesh(gid)] for gid in hand.geom_ids}, 64)
    gid_finger = {}
    for f, group in enumerate(hand.entry['finger_groups']):
        bodies = {hand.model.body(name).id for name in group['body_names']}
        for gid in hand.geom_ids:
            if int(hand.model.geom_bodyid[gid]) in bodies: gid_finger[str(gid)] = f
    human_fingers = hand.entry['human_fingers']
    tips_indices = np.array([4, 8, 12, 16, 20])[human_fingers]
    field = data['collision_field'].astype(float)
    previous = np.clip((neutral_controls - hand.centre) / hand.half, -1, 1)
    controls_all, roots, converged = [], [], []

    def object_surfaces(root, pose):
        to_object = np.linalg.inv(pose) @ root
        return {gid: transform_points(vertices, to_object @ hand.geom_pose(int(gid))) for gid, vertices in samples.items()}

    for i, source_id in enumerate(data['source_frame_ids']):
        joints = data['joints_cv'][i]
        target_tips = joints[tips_indices]
        if args.hand == 'robotiq_2f85':
            base = pinch_pose(neutral_tips, target_tips, joints[0])
        elif args.hand in ('leap_hand', 'shadow_hand'):
            base = palm_pose(neutral_knuckles, neutral_tips[:3], joints[[5, 9, 13]], joints[0])
        else:
            base = rigid_fit(neutral_tips, target_tips)
        targets = {}
        for f, human_finger in enumerate(human_fingers):
            chosen = np.flatnonzero(data['contact_mask'][i] & (data['finger_labels'] == human_finger))
            if len(chosen):
                chosen = chosen[np.linspace(0, len(chosen)-1, min(8, len(chosen)), dtype=int)]
                targets[f] = data['contact_points_object'][i, chosen]
        def loss(x):
            controls = hand.centre + hand.half * x[:-3]
            hand.set_qpos(hand.controls_to_qpos(controls))
            root = base.copy(); root[:3, 3] += x[-3:] * .01
            cost = .4 * np.mean((transform_points(hand.tips(), root) - target_tips) ** 2) / .02**2
            surfaces = object_surfaces(root, data['object_pose_cv'][i])
            distances = sample_collision_grid(np.concatenate(list(surfaces.values())), field, data['grid_origin'], data['grid_step'])
            depth = np.maximum(.0007 - distances, 0) / .003
            tail = np.partition(depth, -min(32, len(depth)))[-min(32, len(depth)):]
            cost += 50 * np.mean(tail**2)
            for f, target in targets.items():
                selected = [points for gid, points in surfaces.items() if gid_finger.get(gid) == f]
                if selected:
                    cost += .3 * np.mean(cdist(target, np.concatenate(selected), 'sqeuclidean').min(axis=1)) / .005**2
            cost += .2 * np.mean((x[:-3] - previous)**2) + .1 * np.sum(x[-3:]**2)
            return float(cost)
        start = np.r_[previous, [0., 0., 0.]]
        fit = minimize(loss, start, method='L-BFGS-B', bounds=[(-1, 1)]*hand.model.nu + [(-3, 3)]*3,
                       options=dict(maxiter=args.max_iterations, eps=1e-5, ftol=1e-7))
        chosen = fit.x if np.isfinite(fit.fun) and fit.fun <= loss(start) else start
        previous = chosen[:-3].copy()
        root = base.copy(); root[:3, 3] += chosen[-3:] * .01
        controls_all.append(hand.centre + hand.half * previous); roots.append(root); converged.append(bool(fit.success))
        if i % 15 == 0: print(f'{args.hand} retarget {i+1}/{len(data["source_frame_ids"])}', flush=True)
    controls_all, roots = np.array(controls_all), np.array(roots)
    camera_world, _ = camera_extrinsics(output, data['source_frame_ids'])
    controls, root_world, _ = filter_trajectory(controls_all, roots, camera_world, args.sigma)
    roots_smoothed = camera_world @ root_world
    qpos = np.array([hand.controls_to_qpos(control) for control in controls])
    penetration = []
    for i, q in enumerate(qpos):
        hand.set_qpos(q)
        points = np.concatenate(list(object_surfaces(roots_smoothed[i], data['object_pose_cv'][i]).values()))
        penetration.append(float(np.maximum(-hull_plane_distance(points, data['hull_equations']), 0).max()))
    hand.save_render_cache(output, qpos)
    np.savez_compressed(output / 'trajectory.npz', qpos=qpos.astype(np.float32), controls=controls,
        root_pose_cv=roots_smoothed, joint_names=hand.joint_names, control_names=hand.control_names,
        source_frame_ids=data['source_frame_ids'],
        valid=np.ones(len(qpos), bool), embodiment=args.hand, side='right',
        source_revision=hand.entry['source']['revision'], fps=data['fps'],
        refinement_converged=converged, penetration_after_m=penetration,
        unsmoothed_controls=controls_all, unsmoothed_root_pose_cv=roots,
        native_control_bounds=hand.control_bounds, smoothing_sigma_frames=args.sigma)
    report = dict(hand=args.hand, fingers=hand.entry['finger_count'], frames=len(qpos),
        joint_positions=hand.model.nq, native_controls=hand.model.nu, human_finger_mapping=human_fingers,
        solver_converged_frames=int(np.sum(converged)), max_sampled_penetration_after_smoothing_mm=float(np.max(penetration)*1000),
        smoothing_sigma_frames=args.sigma,
        limitations=['Visual retargeting; not a verified physical grasp or robot demonstration.',
                     'Hand alignment/finger correspondence is a heuristic across morphologies.',
                     'Smoothing may reintroduce collision; inspect the saved penetration diagnostics.',
                     'Robotiq qpos is interpolated from a native linkage table; its control remains one tendon input.',
                     'Shadow Hand distal tendon targets use equal joint allocation for visual FK; physical simulation retains native tendons.'])
    (output / 'native_retarget_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hand', choices=HAND_IDS, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sigma', type=float, default=2.5)
    p.add_argument('--max-iterations', type=int, default=60)
    run(p.parse_args())
