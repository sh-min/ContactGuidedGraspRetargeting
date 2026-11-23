"""Native Menagerie hands and replayable render meshes for grasp retargeting."""
import errno
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


def source_entries():
    """Read packaged metadata for the four pinned Menagerie hands."""
    return json.loads(Path(__file__).with_name('models.json').read_text(encoding='utf-8'))


_CATALOG = source_entries()
HAND_IDS = tuple(_CATALOG)
HUMAN_FINGERS = {name: entry['human_fingers'] for name, entry in _CATALOG.items()}


def model_dir():
    default = Path.home() / '.cache/grasp_retargeting/models'
    return Path(os.environ.get('CGR_MODEL_DIR', default)).expanduser().resolve()


def _asset_files(directory):
    candidates = [directory / 'LICENSE', *directory.glob('*.xml'), *(directory / 'assets').rglob('*')]
    return sorted(path.relative_to(directory) for path in candidates if path.is_file())


def _verify_assets(directory, entry):
    fingerprint = hashlib.sha256()
    for relative in _asset_files(directory):
        fingerprint.update(relative.as_posix().encode('utf-8') + b'\0')
        fingerprint.update(hashlib.sha256((directory / relative).read_bytes()).digest())
    if fingerprint.hexdigest() != entry['source']['assets_sha256']:
        raise ValueError(f"{entry['id']} model assets in {directory} differ from "
                         f"Menagerie revision {entry['source']['revision']}")


def prepared_entry(name):
    entry = source_entries()[name]
    xml = model_dir() / entry['source']['model_file']
    if not xml.is_file():
        raise FileNotFoundError(f'Missing {xml}. Run prepare-models with the pinned Menagerie checkout.')
    _verify_assets(xml.parent, entry)
    entry['mjcf_path'] = str(xml)
    return entry


def prepare_assets(menagerie: Path):
    """Copy pinned XML, meshes, textures and licenses from a Menagerie checkout."""
    root = model_dir()
    root.mkdir(parents=True, exist_ok=True)
    for name, entry in _CATALOG.items():
        relative = Path(entry['source']['model_file'])
        source = Path(menagerie).expanduser().resolve() / relative.parent
        destination = root / relative.parent
        if destination.is_dir():
            _verify_assets(destination, entry)
        else:
            with tempfile.TemporaryDirectory(dir=root) as temp:
                staged = Path(temp) / name
                staged.mkdir()
                for path in _asset_files(source):
                    target = staged / path
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source / path, target)
                _verify_assets(staged, entry)
                try:
                    staged.rename(destination)
                except OSError as exc:
                    if exc.errno not in (errno.EEXIST, errno.ENOTEMPTY):
                        raise
                    _verify_assets(destination, entry)
        print(f"{name}: {destination / relative.name}", flush=True)


def resolved_visual_rgba(model, data, geom_ids):
    """Resolve authored materials with MuJoCo's own CPU visualization stage.

    geom_rgba alone is only the fallback color and loses named materials.
    No model parameters, poses or physics states are modified here.
    """
    import mujoco
    options = mujoco.MjvOption()
    options.geomgroup[:] = 1
    scene = mujoco.MjvScene(model, maxgeom=max(1000, model.ngeom * 8))
    mujoco.mjv_updateScene(model, data, options, None, mujoco.MjvCamera(),
                          mujoco.mjtCatBit.mjCAT_ALL, scene)
    colors = {int(geom.objid): geom.rgba.copy() for geom in scene.geoms[:scene.ngeom]
              if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM}
    if any(gid not in colors for gid in geom_ids):
        raise ValueError('Native visual geometry is missing from the authored appearance scene')
    return {gid: colors[gid] for gid in geom_ids}


def position_control_matrix(xml, joint_names, control_names):
    """Map scalar joint positions to native joint/fixed-tendon position targets.

    This is a kinematic map, not a replacement for the physical actuators.
    Shadow's distal pair is controlled by its SUM, not by two independent inputs.
    Robotiq uses its native linkage lookup instead of this linear map.
    """
    joints = {name: i for i, name in enumerate(joint_names)}
    actuators = {a.get('name'): a for a in xml.find('actuator')}
    tendons = {t.get('name'): t for t in xml.findall('./tendon/fixed')}
    matrix = np.zeros((len(control_names), len(joint_names)))
    for i, name in enumerate(control_names):
        actuator = actuators[name]
        if actuator.tag not in ('position', 'plugin'):
            raise ValueError(f'Unsupported kinematic position actuator: {name}')
        gear = np.fromstring(actuator.get('gear', '1'), sep=' ')
        if len(gear) < 1 or not np.isclose(gear[0], 1):
            raise ValueError(f'Unexpected position actuator gear: {name}')
        if actuator.get('joint'):
            matrix[i, joints[actuator.get('joint')]] = 1
        elif actuator.get('tendon') in tendons:
            for joint in tendons[actuator.get('tendon')]:
                if joint.tag != 'joint': raise ValueError('Only fixed joint tendons are supported')
                matrix[i, joints[joint.get('joint')]] = float(joint.get('coef'))
        else:
            raise ValueError(f'Unknown actuator transmission: {name}')
    if np.linalg.matrix_rank(matrix) != len(control_names):
        raise ValueError('Native position controls must be independent')
    return matrix


class NativeHand:
    def __init__(self, name):
        import mujoco
        self.mj = mujoco
        self.name = name
        self.entry = prepared_entry(name)
        path = Path(self.entry['mjcf_path'])
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        if self.model.nq != self.model.nv or self.model.nq != self.model.njnt:
            raise ValueError('This adapter expects a fixed-base native hand with scalar joints')
        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        self.control_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        self.control_bounds = self.model.actuator_ctrlrange.copy()
        self.centre = self.control_bounds.mean(axis=1)
        self.half = np.diff(self.control_bounds, axis=1).ravel() / 2
        if not np.isfinite(self.control_bounds).all() or (self.half <= 0).any():
            raise ValueError('Native actuator control ranges must be finite and nonempty')
        self.bounds = self.model.jnt_range.copy()
        self.control_matrix = None
        if name != 'robotiq_2f85':
            self.control_matrix = position_control_matrix(ET.parse(path).getroot(), self.joint_names, self.control_names)
            self.control_inverse = np.linalg.pinv(self.control_matrix)
        self.tip_bodies = [self.model.body(f['tip_body']).id for f in self.entry['finger_groups']]
        self.tip_sites = [self.model.site(f['tip_site']).id if f['tip_site'] else None for f in self.entry['finger_groups']]
        # Prefer original visual geometry, retaining collision-only pads where necessary.
        self.geom_ids = []
        for body in range(1, self.model.nbody):
            geoms = np.flatnonzero(self.model.geom_bodyid == body)
            visual = [int(g) for g in geoms if self.model.geom_contype[g] == 0 and self.model.geom_conaffinity[g] == 0]
            self.geom_ids.extend(visual or geoms.tolist())
        self.lookup = None
        mujoco.mj_forward(self.model, self.data)
        self.visual_rgba = resolved_visual_rgba(self.model, self.data, self.geom_ids)

    def prepare_linkage(self, output):
        if self.name != 'robotiq_2f85': return
        self.model.opt.gravity[:] = 0
        values, qpos = [], []
        self.mj.mj_resetData(self.model, self.data)
        for control in np.linspace(*self.control_bounds[0], 65):
            self.data.ctrl[:] = control
            for _ in range(400): self.mj.mj_step(self.model, self.data)
            values.append(control); qpos.append(self.data.qpos.copy())
        self.lookup = np.array(values), np.array(qpos)
        np.savez_compressed(Path(output) / 'native_linkage_table.npz', controls=values, qpos=qpos,
                            note='Native tendon/equality equilibrium table; interpolated visual configurations, not independent joint commands.')

    def controls_to_qpos(self, controls):
        controls = np.clip(controls, *self.control_bounds.T)
        if self.name == 'robotiq_2f85':
            if self.lookup is None: raise RuntimeError('Prepare native gripper linkage table first')
            return np.array([np.interp(controls[0], self.lookup[0], self.lookup[1][:, j]) for j in range(self.model.nq)])
        # Minimum-norm allocation splits each Shadow distal tendon target equally.
        # Native dynamics can redistribute this pair under contact during training.
        q = np.clip(self.control_inverse @ controls, *self.bounds.T)
        if not np.allclose(self.control_matrix @ q, controls, atol=1e-5):
            raise ValueError('Position targets cannot be allocated within native joint bounds')
        return q

    def set_qpos(self, q):
        self.data.qpos[:] = q
        self.mj.mj_forward(self.model, self.data)

    def tips(self):
        positions = []
        for body, site in zip(self.tip_bodies, self.tip_sites):
            if site is not None:
                positions.append(self.data.site_xpos[site].copy())
            else:
                ids = np.flatnonzero((self.model.geom_bodyid == body) & ((self.model.geom_contype != 0) | (self.model.geom_conaffinity != 0)))
                positions.append(self.data.geom_xpos[ids].mean(axis=0) if len(ids) else self.data.xpos[body].copy())
        return np.array(positions)

    def geom_mesh(self, gid):
        import trimesh
        if gid not in self.visual_rgba:
            self.visual_rgba.update(resolved_visual_rgba(self.model, self.data, [gid]))
        kind, size = int(self.model.geom_type[gid]), self.model.geom_size[gid]
        types = self.mj.mjtGeom
        if kind == types.mjGEOM_MESH:
            mid = int(self.model.geom_dataid[gid])
            va, vn = self.model.mesh_vertadr[mid], self.model.mesh_vertnum[mid]
            fa, fn = self.model.mesh_faceadr[mid], self.model.mesh_facenum[mid]
            mesh = trimesh.Trimesh(vertices=self.model.mesh_vert[va:va+vn].copy(), faces=self.model.mesh_face[fa:fa+fn].copy(), process=False)
        elif kind == types.mjGEOM_BOX: mesh = trimesh.creation.box(extents=2*size)
        elif kind == types.mjGEOM_SPHERE: mesh = trimesh.creation.icosphere(subdivisions=2, radius=size[0])
        elif kind == types.mjGEOM_ELLIPSOID:
            mesh = trimesh.creation.icosphere(subdivisions=2); mesh.apply_scale(size)
        elif kind == types.mjGEOM_CYLINDER: mesh = trimesh.creation.cylinder(radius=size[0], height=2*size[1], sections=24)
        elif kind == types.mjGEOM_CAPSULE:
            mesh = trimesh.creation.capsule(radius=size[0], height=2*size[1], count=[12, 12])
        else: raise ValueError(f'Unsupported native visual geometry type {kind}')
        mesh.visual.face_colors = np.rint(np.clip(self.visual_rgba[gid] * 255, 0, 255)).astype(np.uint8)
        return mesh

    def geom_pose(self, gid):
        pose = np.eye(4); pose[:3, :3] = self.data.geom_xmat[gid].reshape(3, 3); pose[:3, 3] = self.data.geom_xpos[gid]
        return pose

    def save_render_cache(self, output, qpos):
        output = Path(output)
        assets = output / 'render_assets'; assets.mkdir(exist_ok=True)
        files, shared = [], {}
        for gid in self.geom_ids:
            signature = (int(self.model.geom_type[gid]), int(self.model.geom_dataid[gid]),
                         tuple(self.model.geom_size[gid]), tuple(self.visual_rgba[gid]))
            if signature not in shared:
                filename = f'mesh_{len(shared):03d}.ply'
                self.geom_mesh(gid).export(assets / filename); shared[signature] = filename
            files.append(shared[signature])
        poses = []
        for q in qpos:
            self.set_qpos(q); poses.append([self.geom_pose(gid) for gid in self.geom_ids])
        np.savez_compressed(output / 'native_render.npz', geom_poses_native=poses, qpos=qpos,
                            joint_names=self.joint_names, mesh_files=files, geom_ids=self.geom_ids,
                            geom_rgba=[self.visual_rgba[gid] for gid in self.geom_ids],
                            material_model_sha256=self.entry['source']['model_sha256'])
        (output / 'native_model.json').write_text(json.dumps(self.entry, indent=2) + '\n')


class RenderCache:
    """Replay native MuJoCo FK in the existing pyrender environment without loading MuJoCo there."""
    def __init__(self, output, expected_name):
        self.output = Path(output)
        self.entry = json.loads((self.output / 'native_model.json').read_text())
        if self.entry['id'] != expected_name: raise ValueError('Native render model mismatch')
        with np.load(self.output / 'native_render.npz', allow_pickle=False) as archive:
            self.data = {key: archive[key] for key in archive.files}
        self.joint_names = self.data['joint_names'].tolist()
        self.profile = dict(embodiment=expected_name, side='right', source_revision=self.entry['source']['revision'])
        self.index = None

    def meshes(self):
        import trimesh
        colors = None
        if 'geom_rgba' in self.data:
            if str(self.data['material_model_sha256']) != self.entry['source']['model_sha256']:
                raise ValueError('Cached appearance belongs to a different native model')
            colors = self.data['geom_rgba']
        appearance_file = self.output / 'native_appearance.json'
        if appearance_file.is_file():
            appearance = json.loads(appearance_file.read_text())
            if (appearance['embodiment'] != self.entry['id']
                    or appearance['model_sha256'] != self.entry['source']['model_sha256']
                    or appearance['mesh_files'] != self.data['mesh_files'].tolist()):
                raise ValueError('Appearance override differs from the recorded native model/geometry order')
            colors = np.asarray(appearance['rgba'])
        if colors is not None and (colors.shape != (len(self.data['mesh_files']), 4)
                                   or not np.isfinite(colors).all()
                                   or (colors < 0).any() or (colors > 1).any()):
            raise ValueError('Invalid native appearance colors')
        output = {}
        for i, name in enumerate(self.data['mesh_files']):
            mesh = trimesh.load(self.output / 'render_assets' / name, force='mesh', process=False)
            if colors is not None:
                mesh.visual.face_colors = np.rint(np.clip(colors[i] * 255, 0, 255)).astype(np.uint8)
            output[str(i)] = [mesh]
        return output

    def set_frame_index(self, index): self.index = index

    def fk(self, q):
        if self.index is None or not np.allclose(q, self.data['qpos'][self.index], atol=1e-6):
            raise ValueError('Native render cache and trajectory differ')
        return {str(i): pose for i, pose in enumerate(self.data['geom_poses_native'][self.index])}
