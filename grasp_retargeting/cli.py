"""Retarget an annotated grasp sequence to a native robot hand."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys

from .native_hands import HAND_IDS, prepare_assets, source_entries

STAGES = ("geometry", "retarget", "render")


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    commands = cli.add_subparsers(dest="command", required=True)
    models = commands.add_parser("prepare-models", help="Copy the pinned hand models from MuJoCo Menagerie")
    models.add_argument("--menagerie", type=Path, required=True)
    for name, help_text in (("plan", "Print commands without running or writing"),
                            ("run", "Run the selected conversion stages")):
        sub = commands.add_parser(name, help=help_text)
        sub.add_argument("--hand", choices=HAND_IDS, required=True)
        sub.add_argument("--processed", type=Path, required=True, help="Prepared clip containing camera.json")
        sub.add_argument("--mano-model", type=Path, required=True, help="Right-hand MANO parameters")
        sub.add_argument("--episode", type=Path, help="HRDexDB episode; overrides the path recorded in run.json")
        sub.add_argument("--output", type=Path, help="New output directory (default: runs/<clip>_<hand>_<frames>)")
        sub.add_argument("--start-frame", type=int, default=0)
        sub.add_argument("--max-frames", type=int, help="Default: all remaining frames")
        sub.add_argument("--sigma", type=float, default=2.5, help="Smoothing width, in frames")
        sub.add_argument("--max-iterations", type=int, default=60)
        sub.add_argument("--baseline", type=Path, help="Optional comparison video aligned to source frame zero")
        sub.add_argument("--from-stage", choices=STAGES, default="geometry")
        sub.add_argument("--stop-after", choices=STAGES, default="render")
        sub.add_argument("--resume", action="store_true", help="Continue an existing output using the same inputs")
    return cli


def build_plan(args):
    clip = args.processed.expanduser().resolve()
    camera_path = clip / "camera.json"
    camera = json.loads(camera_path.read_text())
    frames = camera["frames"]
    start = args.start_frame
    stop = frames if args.max_frames is None else start + args.max_frames
    if not 0 <= start < stop <= frames:
        raise ValueError(f"Requested frames [{start}, {stop}) outside clip of {frames} frames")
    if not math.isfinite(args.sigma) or not 0 < args.sigma <= 30 or args.max_iterations < 1:
        raise ValueError("Expected 0 < sigma <= 30 and at least one solver iteration")
    first, last = STAGES.index(args.from_stage), STAGES.index(args.stop_after)
    if first > last:
        raise ValueError("--from-stage must precede or equal --stop-after")

    episode = args.episode
    if episode is None:
        source = Path(json.loads((clip / "run.json").read_text())["source"]).expanduser()
        episode = (clip / source).parents[1]
    episode = episode.expanduser().resolve()
    mano = args.mano_model.expanduser().resolve()
    baseline = args.baseline.expanduser().resolve() if args.baseline else None
    default_name = f"{clip.name}_{args.hand}_f{start:04}_{stop:04}"
    output = (args.output or Path("runs") / default_name).expanduser().resolve()
    if output == clip or output == episode or output in clip.parents or output in episode.parents:
        raise ValueError("Choose an output directory separate from the inputs")

    image_py = os.environ.get("CGR_IMAGE_PYTHON", sys.executable)
    native_py = os.environ.get("CGR_NATIVE_PYTHON", sys.executable)
    steps = {
        "geometry": [image_py, "-u", "-m", "grasp_retargeting.mesh_geometry", "--processed", str(clip),
                     "--output", str(output), "--episode", str(episode), "--mano-model", str(mano),
                     "--start-frame", str(start), "--max-frames", str(stop-start)],
        "retarget": [native_py, "-u", "-m", "grasp_retargeting.native_retarget", "--output", str(output),
                     "--hand", args.hand, "--sigma", str(args.sigma), "--max-iterations", str(args.max_iterations)],
        "render": [image_py, "-u", "-m", "grasp_retargeting.mesh_preview", "--processed", str(clip),
                   "--output", str(output), "--native-hand", args.hand],
    }
    if baseline:
        steps["render"] += ["--baseline", str(baseline)]
    manifest = {
        "hand": args.hand, "model_source": source_entries()[args.hand]["source"],
        "processed": str(clip), "episode": str(episode), "mano_model": str(mano),
        "camera_sha256": hashlib.sha256(camera_path.read_bytes()).hexdigest(),
        "baseline": str(baseline) if baseline else None, "start_frame": start,
        "frames": stop-start, "sigma": args.sigma, "max_iterations": args.max_iterations,
    }
    return output, manifest, [(stage, steps[stage]) for stage in STAGES[first:last+1]]


@contextmanager
def exclusive_output(path):
    import fcntl

    path.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"Another conversion is writing to {path}") from exc
        yield
    finally:
        os.close(fd)


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.command == "prepare-models":
        prepare_assets(args.menagerie.expanduser().resolve())
        return
    output, manifest, steps = build_plan(args)
    print(f"Output: {output}", flush=True)
    for stage, command in steps:
        print(f"[{stage}] {shlex.join(command)}", flush=True)
    if args.command == "plan":
        return
    if args.from_stage != "geometry" and not args.resume:
        cli.error("--from-stage retarget/render requires --resume")
    manifest["mano_sha256"] = hashlib.sha256(Path(manifest["mano_model"]).read_bytes()).hexdigest()
    with exclusive_output(output):
        record = output / "native_run.json"
        if args.resume:
            if not record.is_file() or json.loads(record.read_text()) != manifest:
                raise ValueError("Resume requires a matching run manifest, inputs and solver settings")
        elif any(output.iterdir()):
            raise FileExistsError(f"Choose an empty output directory or use --resume: {output}")
        else:
            temporary = record.with_suffix(".tmp")
            temporary.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary.replace(record)
        env = os.environ.copy()
        env.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        for _, command in steps:
            subprocess.run(command, check=True, env=env)
