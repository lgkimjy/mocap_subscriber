#!/usr/bin/env python3
"""Inspect and plot a go2/box1 mocap NPZ log."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def latest_npz(log_dir: Path) -> Path:
    files = sorted(log_dir.glob("*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {log_dir}")
    return files[-1]


def fmt_vec(values: np.ndarray) -> str:
    return "[" + " ".join(f"{float(v):.5f}" for v in values) + "]"


def print_pose_sample(label: str, position: np.ndarray, quaternion: np.ndarray, index: int) -> None:
    print(f"{label}_position[{index}] = {fmt_vec(position[index])}")
    print(f"{label}_quaternion_wxyz[{index}] = {fmt_vec(quaternion[index])}")


def quaternion_wxyz_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0 or not np.isfinite(norm):
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def square_xy_limits(
    go2_position: np.ndarray,
    box1_position: np.ndarray,
    go2_valid: np.ndarray,
    box1_valid: np.ndarray,
) -> tuple[float, float, float, float, float]:
    points = []
    if go2_valid.any():
        points.append(go2_position[go2_valid, :2])
    if box1_valid.any():
        points.append(box1_position[box1_valid, :2])
    if not points:
        half = 0.5
        return -half, half, -half, half, half

    xy = np.vstack(points)
    xy = xy[np.isfinite(xy).all(axis=1)]
    if xy.size == 0:
        half = 0.5
        return -half, half, -half, half, half

    x_min, y_min = np.min(xy, axis=0)
    x_max, y_max = np.max(xy, axis=0)
    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    half = 0.5 * max(float(x_max - x_min), float(y_max - y_min), 1e-3)
    half *= 1.15
    # Keep world origin in view.
    half = max(half, abs(cx) * 1.05 + 0.05, abs(cy) * 1.05 + 0.05, 0.5)
    return cx - half, cx + half, cy - half, cy + half, half


def auto_axis_scale(view_half: float, go2_position: np.ndarray, box1_position: np.ndarray, valid: np.ndarray) -> float:
    # Prefer a fraction of the visible window so axes stay readable.
    scale = 0.12 * view_half
    if valid.any():
        go2 = go2_position[valid, :2]
        box1 = box1_position[valid, :2]
        separation = float(np.linalg.norm(np.nanmean(go2, axis=0) - np.nanmean(box1, axis=0)))
        if np.isfinite(separation) and separation > 0.0:
            scale = max(scale, 0.25 * separation)
    return float(np.clip(scale, 0.15, 0.45 * view_half))


def plot_body_axes(
    ax, position: np.ndarray, quaternion: np.ndarray, valid: np.ndarray, interval: int, scale: float, alpha: float
) -> None:
    indices = np.flatnonzero(valid)
    if indices.size == 0:
        return
    indices = indices[:: max(interval, 1)]

    origins = position[indices, :2]
    x_dirs = []
    y_dirs = []
    for q in quaternion[indices]:
        rotation = quaternion_wxyz_to_rotation_matrix(q)
        x_dirs.append(rotation[:2, 0])
        y_dirs.append(rotation[:2, 1])
    x_dirs = np.asarray(x_dirs) * scale
    y_dirs = np.asarray(y_dirs) * scale

    ax.quiver(
        origins[:, 0],
        origins[:, 1],
        x_dirs[:, 0],
        x_dirs[:, 1],
        color="red",
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=0.0045,
        alpha=alpha,
    )
    ax.quiver(
        origins[:, 0],
        origins[:, 1],
        y_dirs[:, 0],
        y_dirs[:, 1],
        color="blue",
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=0.0045,
        alpha=alpha,
    )


def save_top_view_plot(
    data: np.lib.npyio.NpzFile, npz_path: Path, plot_dir: Path | None, interval: int, axis_scale: float | None
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    go2_position = data["go2_position"]
    box1_position = data["box1_position"]
    go2_quaternion = data["go2_quaternion_wxyz"]
    box1_quaternion = data["box1_quaternion_wxyz"]
    go2_valid = data["go2_valid"].astype(bool)
    box1_valid = data["box1_valid"].astype(bool)
    both_valid = go2_valid & box1_valid

    output_dir = plot_dir if plot_dir is not None else npz_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{npz_path.stem}_top_view.png"

    x0, x1, y0, y1, view_half = square_xy_limits(go2_position, box1_position, go2_valid, box1_valid)
    scale = axis_scale if axis_scale is not None else auto_axis_scale(
        view_half, go2_position, box1_position, both_valid
    )
    world_scale = max(scale * 1.25, 0.18 * view_half)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(f"Top View: {npz_path.name}")
    ax.set_xlabel("world x")
    ax.set_ylabel("world y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)

    ax.quiver(
        [0.0],
        [0.0],
        [world_scale],
        [0.0],
        color="red",
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=0.006,
    )
    ax.quiver(
        [0.0],
        [0.0],
        [0.0],
        [world_scale],
        color="blue",
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=0.006,
    )
    ax.text(world_scale * 1.08, 0.0, "world x", color="red", va="center")
    ax.text(0.0, world_scale * 1.08, "world y", color="blue", ha="center")

    if go2_valid.any():
        ax.plot(go2_position[go2_valid, 0], go2_position[go2_valid, 1], color="tab:orange", label="go2 path")
        ax.scatter(go2_position[go2_valid, 0], go2_position[go2_valid, 1], s=18, color="tab:orange", alpha=0.45)
    if box1_valid.any():
        ax.plot(box1_position[box1_valid, 0], box1_position[box1_valid, 1], color="tab:purple", label="box1 path")
        ax.scatter(box1_position[box1_valid, 0], box1_position[box1_valid, 1], s=18, color="tab:purple", alpha=0.45)

    plot_body_axes(ax, go2_position, go2_quaternion, go2_valid, interval, scale, alpha=0.85)
    plot_body_axes(ax, box1_position, box1_quaternion, box1_valid, interval, scale, alpha=0.70)

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="best")
    if not go2_valid.any() and not box1_valid.any():
        ax.text(0.5, 0.5, "no valid samples", transform=ax.transAxes, ha="center", va="center")

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Read, summarize, and plot a go2/box1 mocap NPZ log.")
    parser.add_argument("npz", nargs="?", help="Path to .npz file. Defaults to latest file in --log-dir.")
    parser.add_argument("--log-dir", default="demo/go2-box", help="Directory used when npz path is omitted.")
    parser.add_argument("--list-keys", action="store_true", help="Print all arrays in the file.")
    parser.add_argument("--no-plot", action="store_true", help="Skip top-view figure generation.")
    parser.add_argument("--plot-dir", help="Directory for the generated top-view PNG. Defaults to the NPZ directory.")
    parser.add_argument("--axis-interval", type=int, default=30, help="Draw body axes every N logged samples.")
    parser.add_argument("--axis-scale", type=float, help="Length of plotted axes in world units.")
    args = parser.parse_args()

    path = Path(args.npz) if args.npz else latest_npz(Path(args.log_dir))
    data = np.load(path)

    print(f"file: {path}")
    if args.list_keys:
        print("arrays:")
        for key in data.files:
            print(f"  {key}: shape={data[key].shape}, dtype={data[key].dtype}")

    time_seconds = data["time_seconds"]
    frame_number = data["frame_number"]
    go2_valid = data["go2_valid"].astype(bool)
    box1_valid = data["box1_valid"].astype(bool)
    both_valid = go2_valid & box1_valid

    print(f"samples: {time_seconds.shape[0]}")
    if time_seconds.size > 0:
        print(f"duration_seconds: {float(time_seconds[-1] - time_seconds[0]):.5f}")
        print(f"frame_range: {int(frame_number[0])} -> {int(frame_number[-1])}")
    print(f"go2_valid: {int(go2_valid.sum())}")
    print(f"box1_valid: {int(box1_valid.sum())}")
    print(f"both_valid: {int(both_valid.sum())}")

    if both_valid.any():
        first = int(np.flatnonzero(both_valid)[0])
        last = int(np.flatnonzero(both_valid)[-1])
        print(f"first_both_valid_index: {first}")
        print_pose_sample("go2", data["go2_position"], data["go2_quaternion_wxyz"], first)
        print_pose_sample("box1", data["box1_position"], data["box1_quaternion_wxyz"], first)
        if last != first:
            print(f"last_both_valid_index: {last}")
            print_pose_sample("go2", data["go2_position"], data["go2_quaternion_wxyz"], last)
            print_pose_sample("box1", data["box1_position"], data["box1_quaternion_wxyz"], last)

    if not args.no_plot:
        plot_dir = Path(args.plot_dir) if args.plot_dir else None
        plot_path = save_top_view_plot(data, path, plot_dir, args.axis_interval, args.axis_scale)
        print(f"top_view_plot: {plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
