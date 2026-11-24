# Contact-Guided Grasp Retargeting

Retarget an annotated human grasp video to a two-finger Robotiq 2F-85, three-finger Shadow DEX-EE, four-finger LEAP Hand or five-finger Shadow Hand. The module produces a wrist pose and native joint trajectory for every selected frame. With a background plate and an object mask, it also renders the robot hand into the original camera view.

[국문 논문 초안 · 비교 그림 · 참고문헌](paper/README.md)

## How it works

1. Read the right-hand MANO mesh, object pose and camera calibration at each source frame. Identify human hand vertices near the object and project penetrating vertices onto a convex collision proxy. Keep the original object mesh for rendering.
2. Map the human digits to each robot’s native fingers. Fit the wrist and actuator targets using fingertip position, finger-surface contact distances, penetration and temporal continuity. Smooth wrist poses in the world frame and report sampled penetration after smoothing.
3. Render the native hand geometry with its authored material colors. Resolve hand–object occlusions by depth. Preserve visible object pixels from the video and estimate colors on newly exposed surfaces from other views in the clip.

Contact here is estimated from geometry. The resulting trajectories describe visual demonstrations. Physical grasp success requires a separate evaluation.

| CLI hand ID | Robot | Fingers | Native controls |
| --- | --- | ---: | ---: |
| `robotiq_2f85` | Robotiq 2F-85 | 2 | 1 |
| `shadow_dexee` | Shadow DEX-EE | 3 | 12 |
| `leap_hand` | LEAP Hand | 4 | 16 |
| `shadow_hand` | Shadow Hand (right) | 5 | 20 |

## Installation

On Linux with Python 3.10 or 3.11, run the following from the repository root. The video extra provides the Python rendering dependencies. Install FFmpeg (including ffprobe and the H.264 encoder) and an OpenGL/EGL implementation for frame preparation and rendering.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install '.[video]'
```

Prepare the pinned native hand models once. The sparse checkout fetches the four hand directories. The preparation command checks the model fingerprints and copies the required assets and model licenses into the local cache. Set `CGR_MODEL_DIR` to choose a different cache location.

```bash
git clone --filter=blob:none --no-checkout https://github.com/google-deepmind/mujoco_menagerie.git
git -C mujoco_menagerie sparse-checkout init --cone
git -C mujoco_menagerie sparse-checkout set robotiq_2f85 shadow_dexee leap_hand shadow_hand
git -C mujoco_menagerie checkout d7b481bac63d3d600ab404ee74f7474c5efe8a67
grasp-retarget prepare-models --menagerie ./mujoco_menagerie
```

## Inputs

Obtain an annotated HRDexDB human episode and the matching assets and annotations. Keep these paths together under the dataset sequence directory.

```text
<sequence>/
  assets/mesh_v2/<object>/<object>.obj
  object_6d_pose_v2/human/<object>_<episode>.npz
  human/<object>/<episode>/
    vid/<camera>.mp4
    cam_param/ego_calib.json
    hand/mano/00000.obj, 00001.obj, ...
```

Provide your own licensed right-hand MANO parameters via `--mano-model`. The geometry loader reads a trusted MANO pickle in a compatible MANO/Chumpy environment, or a numeric `.npz` with dense arrays named `J_regressor` (16 × 778), `weights` (778 × 16) and `f` (triangle vertex indices). To export these three arrays locally from a legacy MANO environment that can already read your original model:

```python
import pickle
import numpy as np

with open("MANO_RIGHT.pkl", "rb") as stream:
    model = pickle.load(stream, encoding="latin1")
regressor = model["J_regressor"]
if hasattr(regressor, "toarray"):
    regressor = regressor.toarray()
np.savez("mano_right.npz", J_regressor=np.asarray(regressor),
         weights=np.asarray(model["weights"]), f=np.asarray(model["f"]))
```

Use an existing processed clip, or rectify an episode to a fresh directory. Replace the paths and camera ID with those of your dataset. The command writes the normalized source video, RGB frames, camera intrinsics and a pointer to the episode.

```bash
EPISODE=/path/to/sequence/human/lemon/0
CAMERA=25452062
CLIP="$PWD/data/lemon"
MANO=/path/to/managed/parameters/mano_right.npz

python -m grasp_retargeting.prepare_video --episode "$EPISODE" --camera "$CAMERA" --output "$CLIP"
```

Geometry and retargeting use the source annotations. For **video rendering**, additionally supply the following two files from your segmentation and inpainting workflow, in the pixel coordinates of the rectified `video_L.mp4`:

| File relative to the processed clip | Contents |
| --- | --- |
| `interaction_objects/object_mask.npy` | Boolean visible-object mask of shape (source frames, height, width). |
| `inpaint_processor/background.mkv` | Cleaned source clip with the human hand and arm removed. Match the source frame order, frame count, resolution and frame rate. |

For example, segment the visible object and human arm/hand in the rectified frames with SAM 2. Protect the visible object mask when removing the human region with ProPainter. Existing DexStitch-style processed clips can supply both files directly.

## Run

Inspect the commands and selected frames with `plan`. This only reads metadata. The example selects source frames 75 through 134, inclusive.

```bash
grasp-retarget plan --hand leap_hand --processed "$CLIP" --episode "$EPISODE" \
    --mano-model "$MANO" --start-frame 75 --max-frames 60 --output runs/leap
```

Generate the geometry and trajectory first. To convert the same demonstration for another hand, change `--hand` and use a separate output directory.

```bash
grasp-retarget run --hand leap_hand --processed "$CLIP" --episode "$EPISODE" \
    --mano-model "$MANO" --start-frame 75 --max-frames 60 --output runs/leap --stop-after retarget
```

Once the mask and background are available, render from the saved trajectory with the same inputs. For a single uninterrupted conversion, omit `--stop-after` in the previous command.

```bash
grasp-retarget run --hand leap_hand --processed "$CLIP" --episode "$EPISODE" \
    --mano-model "$MANO" --start-frame 75 --max-frames 60 --output runs/leap --resume --from-stage render
```

The default frame range covers the full clip. The optional `--baseline` accepts a comparison video aligned to source frame zero. Use the same baseline choice throughout a resumed run. If a processed clip has moved, `--episode` overrides its saved dataset location.

## Outputs

| File | Contents |
| --- | --- |
| `geometry.npz` | Frame IDs, MANO targets, object geometry and per-vertex contact candidates. |
| `trajectory.npz` | Native `controls`, joint positions `qpos`, joint and actuator names, `root_pose_cv`, frame IDs and post-smoothing penetration. The root pose maps the hand root into the OpenCV camera frame (x right, y down, z forward), in metres. |
| `native_retarget_report.json` | Solver convergence, finger correspondence and maximum sampled penetration. |
| `mesh_depth.mp4` | Robot hand composited into the source scene. |
| `comparison.mp4` | Human and robot views side by side, with a third panel if a baseline is supplied. |

Robotiq has one tendon command and eight joint positions. Shadow Hand also couples some joints through tendons. Use the saved actuator names and controls when interpreting the trajectories.

## Tests

```bash
python -m unittest discover -s tests -v
```

## Sources

The video conversion draws on the contact-aware robotic inpainting module in DexStitch. This repository implements the annotated mesh-based multi-hand retargeting path. [DexStitch paper](https://arxiv.org/abs/2610.17789)

Human episodes, camera calibration and object geometry follow the HRDexDB release. MANO parameters and datasets are obtained separately under their respective terms. [HRDexDB dataset](https://huggingface.co/datasets/HRDexDB/HRDexDB) · [MANO project](https://mano.is.tue.mpg.de/)

Robot models, geometry and per-model license information come from the pinned MuJoCo Menagerie revision. [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/d7b481bac63d3d600ab404ee74f7474c5efe8a67)
