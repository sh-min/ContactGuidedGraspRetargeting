"""Composite robot and measured object meshes using camera-space depth.

Visible object pixels come from the source video. Newly uncovered object
surfaces use approximate vertex colours sampled from this clip, not GT texture.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .mesh_geometry import transform_points
from .native_hands import HAND_IDS, RenderCache


def compose_depth(source, background, robot_rgb, robot_depth, object_rgb,
                  object_depth, human_depth, object_mask, tolerance=.0002):
    obj = object_depth > 0
    # Do not paint the original human hand back onto newly uncovered object regions.
    source_visible = obj & object_mask & ((human_depth <= 0) | (object_depth < human_depth - .001))
    robot_front = (robot_depth > 0) & (~obj | (robot_depth < object_depth - tolerance))
    result = background.copy()
    result[obj] = object_rgb[obj]
    result[source_visible] = source[source_visible]
    result[robot_front] = robot_rgb[robot_front]
    return result, source_visible, robot_front


class MeshRenderer:
    def __init__(self, model, geometry):
        import pyrender
        import trimesh
        self.pr, self.tm = pyrender, trimesh
        self.model, self.geometry = model, geometry
        width, height = map(int, geometry['image_size'])
        k = geometry['K']
        self.renderer = pyrender.OffscreenRenderer(width, height)
        self.scene = pyrender.Scene(bg_color=[0, 0, 0, 0], ambient_light=[.35, .35, .35])
        self.scene.add(pyrender.IntrinsicsCamera(k[0, 0], k[1, 1], k[0, 2], k[1, 2], znear=.01, zfar=20))
        self.scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=3))
        self.robot = [(link, self.scene.add(pyrender.Mesh.from_trimesh(mesh, smooth=False)))
                      for link, items in model.meshes().items() for mesh in items]
        self.object_node = None
        self.human_node = None
        self.cv_to_gl = np.diag([1., -1., -1., 1.])
        self.set_object_colors(np.full((len(geometry['object_vertices']), 4), 255, np.uint8))

    def set_object_colors(self, colors):
        if self.object_node is not None:
            self.scene.remove_node(self.object_node)
        mesh = self.tm.Trimesh(vertices=self.geometry['object_vertices'], faces=self.geometry['object_faces'],
                               vertex_colors=colors, process=False)
        self.object_node = self.scene.add(self.pr.Mesh.from_trimesh(mesh, smooth=False))

    def set_frame(self, i, q=None, root=None):
        if hasattr(self.model, 'set_frame_index'):
            self.model.set_frame_index(i)
        self.scene.set_pose(self.object_node, self.cv_to_gl @ self.geometry['object_pose_cv'][i])
        if self.human_node is not None:
            self.scene.remove_node(self.human_node)
        mesh = self.tm.Trimesh(vertices=self.geometry['raw_mano_cv'][i], faces=self.geometry['mano_faces'], process=False)
        self.human_node = self.scene.add(self.pr.Mesh.from_trimesh(mesh, smooth=False), pose=self.cv_to_gl)
        if q is not None:
            fk = self.model.fk(q)
            for link, node in self.robot:
                self.scene.set_pose(node, self.cv_to_gl @ root @ fk[link])

    def render(self, kind, depth_only=False):
        for _, node in self.robot:
            node.mesh.is_visible = kind == 'robot'
        self.object_node.mesh.is_visible = kind == 'object'
        self.human_node.mesh.is_visible = kind == 'human'
        flags = self.pr.RenderFlags.SKIP_CULL_FACES
        if depth_only:
            flags |= self.pr.RenderFlags.DEPTH_ONLY
        elif kind == 'object':
            flags |= self.pr.RenderFlags.FLAT
        return self.renderer.render(self.scene, flags=flags)

    def close(self):
        self.renderer.delete()


def video_frame(capture, index):
    import cv2
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
    ok, frame = capture.read()
    if not ok:
        raise ValueError(f'Cannot read exact source frame {index}')
    return frame[..., ::-1]


def estimate_object_colors(renderer, geometry, capture, masks):
    """Multi-view vertex colours; unseen vertices borrow the nearest observed colour."""
    import cv2
    vertices = geometry['object_vertices']
    k = geometry['K']
    width, height = map(int, geometry['image_size'])
    sums, counts = np.zeros_like(vertices), np.zeros(len(vertices), np.int32)
    selected = np.unique(np.linspace(0, len(geometry['source_frame_ids']) - 1, 8, dtype=int))
    for i in selected:
        t = geometry['source_frame_ids'][i]
        source = video_frame(capture, t)
        renderer.set_frame(i)
        object_depth = renderer.render('object', depth_only=True)
        human_depth = renderer.render('human', depth_only=True)
        cv_points = transform_points(vertices, geometry['object_pose_cv'][i])
        uvz = cv_points @ k.T
        uv = np.rint(uvz[:, :2] / np.maximum(uvz[:, 2:], 1e-8)).astype(int)
        inside = (cv_points[:, 2] > .01) & (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
        ids = np.flatnonzero(inside)
        x, y = uv[ids].T
        od, hd = object_depth[y, x], human_depth[y, x]
        mask = cv2.erode(np.asarray(masks[t], np.uint8), np.ones((3, 3), np.uint8)) > 0
        visible = mask[y, x] & (od > 0) & (np.abs(cv_points[ids, 2] - od) < .003) & ((hd <= 0) | (cv_points[ids, 2] < hd - .002))
        ids = ids[visible]
        sums[ids] += source[uv[ids, 1], uv[ids, 0]]
        counts[ids] += 1
    observed = counts > 0
    if not observed.any():
        raise ValueError('No object surface agrees with visible source pixels; check camera/object transforms')
    colors = np.zeros((len(vertices), 4), np.uint8)
    colors[observed, :3] = np.clip(sums[observed] / counts[observed, None], 0, 255).astype(np.uint8)
    nearest = cKDTree(vertices[observed]).query(vertices[~observed])[1]
    colors[~observed, :3] = colors[observed, :3][nearest]
    colors[:, 3] = 255
    return colors, observed


def labelled(image, text):
    import cv2
    output = image.copy()
    cv2.rectangle(output, (0, 0), (output.shape[1], 42), (20, 20, 20), -1)
    cv2.putText(output, text, (12, 29), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def run(args):
    os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
    import cv2
    import mediapy as media

    # Materialize the data needed for drawing once, rather than decompressing
    # every mesh sequence again for each rendered video frame.
    with np.load(args.output / 'geometry.npz', allow_pickle=False) as archive:
        keys = ('source_frame_ids', 'image_size', 'K', 'fps', 'object_vertices', 'object_faces',
                'raw_mano_cv', 'mano_faces', 'object_pose_cv')
        geometry = {key: archive[key] for key in keys}
    with np.load(args.output / 'trajectory.npz', allow_pickle=False) as archive:
        keys = ('source_frame_ids', 'source_revision', 'joint_names', 'valid', 'root_pose_cv', 'qpos')
        trajectory = {key: archive[key] for key in keys}
    model = RenderCache(args.output, args.native_hand)
    ids = geometry['source_frame_ids']
    width, height = map(int, geometry['image_size'])
    if not np.array_equal(ids, trajectory['source_frame_ids']) or trajectory['joint_names'].tolist() != model.joint_names:
        raise ValueError('Frame/joint order mismatch')
    if str(trajectory['source_revision']) != model.profile['source_revision']:
        raise ValueError('Robot geometry revision mismatch')
    if not np.all(np.diff(ids) == 1) or not trajectory['valid'].all():
        raise ValueError('Expected contiguous valid source frames')
    masks = np.load(args.processed / 'interaction_objects/object_mask.npy', mmap_mode='r')
    if masks.shape[1:] != (height, width) or len(masks) <= ids[-1]:
        raise ValueError('Source object mask dimensions differ')
    paths = [args.processed / 'video_L.mp4', args.processed / 'inpaint_processor/background.mkv']
    if args.baseline is not None:
        paths.append(args.baseline)
    panel_count = len(paths)  # Source, optional baseline, retargeted hand.
    comparison_size = (max(2, width * panel_count // 4 * 2), max(2, height // 4 * 2))
    temporary = args.output / 'mesh_depth.partial.mp4'
    comparison_tmp = args.output / 'comparison.partial.mp4'
    preview_indices = {0, len(ids) // 2, len(ids) - 1}
    ious, approximated, hidden = [], [], []
    with ExitStack() as stack:
        captures = []
        for path in paths:
            capture = cv2.VideoCapture(str(path))
            stack.callback(capture.release)
            if not capture.isOpened():
                raise FileNotFoundError(f'Cannot open input video: {path}')
            captures.append(capture)
        renderer = MeshRenderer(model, geometry)
        stack.callback(renderer.close)
        stack.callback(temporary.unlink, missing_ok=True)
        stack.callback(comparison_tmp.unlink, missing_ok=True)
        colors, observed = estimate_object_colors(renderer, geometry, captures[0], masks)
        renderer.set_object_colors(colors)
        np.savez_compressed(args.output / 'object_appearance.npz', vertex_rgba=colors, observed_vertices=observed,
                            note='Approximate vertex colour transfer; unknown surfaces use nearest observed vertex colour.')
        for cap in captures:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(ids[0]))
        with media.VideoWriter(str(temporary), shape=(height, width), fps=float(geometry['fps']), codec='h264') as writer, \
             media.VideoWriter(str(comparison_tmp), shape=comparison_size[::-1], fps=float(geometry['fps']), codec='h264') as compare:
            for i, t in enumerate(ids):
                frames = [cap.read() for cap in captures]
                if not all(ok for ok, _ in frames):
                    raise ValueError(f'Video ended at source frame {t}')
                images = [frame[..., ::-1] for _, frame in frames]
                if any(frame.shape != (height, width, 3) for frame in images):
                    raise ValueError('Source/background/baseline image sizes differ')
                source, background = images[:2]
                renderer.set_frame(i, trajectory['qpos'][i], trajectory['root_pose_cv'][i])
                robot_rgb, robot_depth = renderer.render('robot')
                object_rgb, object_depth = renderer.render('object')
                human_depth = renderer.render('human', depth_only=True)
                result, source_visible, robot_front = compose_depth(source, background, robot_rgb, robot_depth,
                    object_rgb, object_depth, human_depth, np.asarray(masks[t], bool))
                writer.add_image(result)
                panels = [labelled(source, 'Human demonstration')]
                if args.baseline is not None:
                    panels.append(labelled(images[2], 'Baseline'))
                panels.append(labelled(result, f'{args.native_hand} (frame {t})'))
                combined = np.concatenate(panels, axis=1)
                compare.add_image(cv2.resize(combined, comparison_size, interpolation=cv2.INTER_AREA))
                visible_object = (object_depth > 0) & ((human_depth <= 0) | (object_depth < human_depth - .001))
                union = np.count_nonzero(visible_object | masks[t])
                ious.append(np.count_nonzero(visible_object & masks[t]) / max(union, 1))
                approximated.append(int(((object_depth > 0) & ~source_visible & ~robot_front).sum()))
                hidden.append(int(((robot_depth > 0) & ~robot_front).sum()))
                if i in preview_indices:
                    cv2.imwrite(str(args.output / f'comparison_{t:05d}.png'), combined[..., ::-1])
                    overlay = source.copy()
                    for mask, color in (((object_depth > 0), (0, 255, 0)), ((human_depth > 0), (0, 180, 255))):
                        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(overlay, contours, -1, color, 1)
                    cv2.imwrite(str(args.output / f'geometry_overlay_{t:05d}.png'), overlay[..., ::-1])
                if i % 10 == 0:
                    print(f'render {i + 1}/{len(ids)}; object visible-mask IoU={ious[-1]:.3f}', flush=True)
        temporary.replace(args.output / 'mesh_depth.mp4')
        comparison_tmp.replace(args.output / 'comparison.mp4')
    report = dict(frames=len(ids), source_frames=ids.tolist(), fps=float(geometry['fps']), embodiment=args.native_hand,
        comparison_baseline=str(args.baseline) if args.baseline is not None else None,
        mean_visible_object_mask_iou=float(np.mean(ious)),
        vertex_colors_observed=int(observed.sum()), object_vertices=len(observed),
        mean_approximate_object_pixels=float(np.mean(approximated)), mean_robot_pixels_occluded_by_object=float(np.mean(hidden)),
        depth_order='Per-pixel object mesh depth versus articulated robot mesh depth, including thumb and palm.',
        object_appearance='Source RGB where observed; approximate multiview vertex colours on newly revealed surfaces.',
        limitations=['Collision geometry uses the convex hull of the object mesh.',
                     'Cached inpainting errors can remain in the background.',
                     'Offline smoothing uses future frames.'])
    (args.output / 'render_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--processed', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--baseline', type=Path, help='Optional comparison video aligned to the complete source timeline')
    p.add_argument('--native-hand', choices=HAND_IDS, required=True)
    run(p.parse_args())
