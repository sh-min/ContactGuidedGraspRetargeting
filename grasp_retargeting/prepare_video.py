"""Prepare a calibrated HRDexDB clip and preserve its source frame numbers."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np


def video_info(path):
    result = subprocess.run([
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
        "stream=width,height,avg_frame_rate,nb_frames", "-of", "json", str(path),
    ], check=True, capture_output=True, text=True)
    stream = json.loads(result.stdout)["streams"][0]
    numerator, denominator = map(float, stream["avg_frame_rate"].split("/"))
    fps = numerator / denominator
    if fps <= 0 or not np.isfinite(fps):
        raise ValueError("Invalid source frame rate")
    return dict(width=int(stream["width"]), height=int(stream["height"]), fps=fps,
                frames=int(stream["nb_frames"]))


def camera_parameters(episode, camera_id, info, width, video_space):
    calibration = json.loads((Path(episode) / "cam_param/ego_calib.json").read_text())
    entry = calibration["intrinsics"][camera_id]
    if (entry["width"], entry["height"]) != (info["width"], info["height"]):
        raise ValueError("Source video size does not match ego camera calibration")
    if width < 64 or width % 2:
        raise ValueError("--width must be an even integer >= 64")
    height = int(round(width * info["height"] / info["width"] / 2)) * 2
    key = "original_intrinsics" if video_space == "raw" else "intrinsics_undistort"
    source_k = np.asarray(entry[key], float)
    focal = float((source_k[0, 0] + source_k[1, 1]) / 2 * width / info["width"])
    target_k = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])
    distortion = np.asarray(entry["dist_params"], float) if video_space == "raw" else np.zeros(4)
    return dict(source_K=source_k.tolist(), K=target_k.tolist(), distortion=distortion.tolist(),
                width=width, height=height, fps=info["fps"], frames=info["frames"],
                camera_id=camera_id, video_space=video_space, source_size=[info["width"], info["height"]])


def run(args):
    import cv2
    source = args.episode / "vid" / f"{args.camera}.mp4"
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f"Choose a fresh output directory: {args.output}")
    info = video_info(source)
    camera = camera_parameters(args.episode, args.camera, info, args.width, args.video_space)
    width, height = camera["width"], camera["height"]
    rgb = args.output / "rgb"
    if rgb.exists() and any(rgb.iterdir()):
        raise FileExistsError(f"Prepared frames already exist in {rgb}; use --from-stage hand or a fresh output")
    rgb.mkdir(parents=True, exist_ok=True)
    map_x, map_y = cv2.initUndistortRectifyMap(
        np.array(camera["source_K"]), np.array(camera["distortion"]), None,
        np.array(camera["K"]), (width, height), cv2.CV_32FC1,
    )
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {source}")
    count = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frame = cv2.remap(frame, map_x, map_y, interpolation=cv2.INTER_LINEAR)
            # One consistent JPEG sequence feeds every model and the canonical video.
            if not cv2.imwrite(str(rgb / f"rgb_frame{count:05d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 97]):
                raise RuntimeError(f"Cannot write frame {count}")
            count += 1
    finally:
        capture.release()
    if count != info["frames"]:
        raise ValueError(f"Decoded {count} frames; expected {info['frames']}")
    subprocess.run([
        "ffmpeg", "-v", "error", "-n", "-framerate", str(info["fps"]), "-start_number", "0",
        "-i", str(rgb / "rgb_frame%05d.jpg"), "-an", "-c:v", "libx264", "-crf", "16",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output / "video_L.mp4"),
    ], check=True)
    (args.output / "camera.json").write_text(json.dumps(camera, indent=2) + "\n")
    (args.output / "run.json").write_text(json.dumps({"source": str(source.resolve())}, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--camera", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--video-space", choices=["raw", "undistorted"], default="undistorted")
    run(parser.parse_args())
