#!/usr/bin/env python3
"""Project the captured LiDAR clouds onto their paired images using
the extrinsic written by livox_camera_calib, so the calibration
quality can be eyeballed."""

from pathlib import Path

import cv2
import numpy as np

WS = Path(__file__).resolve().parent.parent
DATA = WS / "data"
EXTR = DATA / "results" / "extrinsic.txt"
OUT = DATA / "results" / "viz"
OUT.mkdir(parents=True, exist_ok=True)

# See3cam intrinsics (from fast_livo/config/camera_see3cam.yaml)
K = np.array([
    [691.464783, 0.0,        666.939107],
    [0.0,        692.229814, 356.419891],
    [0.0,        0.0,        1.0],
])
D = np.array([-0.298524, 0.068119, 0.000204, 0.000403, 0.0])

MAX_PTS = 200_000   # subsample for speed; output stays dense enough to read


def read_pcd_binary_xyzi(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        header = b""
        n_points = None
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"POINTS"):
                n_points = int(line.split()[1])
            if line.startswith(b"DATA"):
                break
        raw = f.read()
    arr = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)[:n_points]
    return arr


def read_extrinsic(path: Path):
    M = np.loadtxt(path, delimiter=",")
    return M[:3, :3], M[:3, 3]


def overlay_one(pcd_path: Path, img_path: Path, out_path: Path,
                R: np.ndarray, t: np.ndarray):
    pts = read_pcd_binary_xyzi(pcd_path)[:, :3]
    img = cv2.imread(str(img_path))
    h, w = img.shape[:2]

    if pts.shape[0] > MAX_PTS:
        idx = np.random.default_rng(0).choice(pts.shape[0], MAX_PTS, replace=False)
        pts = pts[idx]

    pts_cam = (R @ pts.T).T + t
    front = pts_cam[:, 2] > 0.1
    pts_cam = pts_cam[front]
    if pts_cam.shape[0] == 0:
        print(f"  WARN: no points in front of camera for {pcd_path.name}")
        return

    # cv2.projectPoints handles distortion
    pts_2d, _ = cv2.projectPoints(
        pts_cam.astype(np.float64),
        np.zeros(3), np.zeros(3),
        K, D,
    )
    pts_2d = pts_2d.reshape(-1, 2)

    us = pts_2d[:, 0].astype(int)
    vs = pts_2d[:, 1].astype(int)
    in_bounds = (us >= 0) & (us < w) & (vs >= 0) & (vs < h)
    us, vs = us[in_bounds], vs[in_bounds]
    depths = pts_cam[in_bounds, 2]

    if us.size == 0:
        print(f"  WARN: no projected points land in image for {pcd_path.name}")
        return

    # Color by depth (jet-like via cv2.applyColorMap on a 1-channel ramp)
    d_min, d_max = 0.5, 15.0
    norm = np.clip((depths - d_min) / (d_max - d_min), 0, 1)
    ramp = (norm * 255).astype(np.uint8)
    colors = cv2.applyColorMap(ramp.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    overlay = img.copy()
    # Plot 3x3 dots so points are visible at 1280x720
    for du in (-1, 0, 1):
        for dv in (-1, 0, 1):
            uu = np.clip(us + du, 0, w - 1)
            vv = np.clip(vs + dv, 0, h - 1)
            overlay[vv, uu] = colors

    cv2.imwrite(str(out_path), overlay)
    print(f"  {out_path.name}: {us.size:>6d} pts on image (of {pts.shape[0]} sampled)")


def main():
    R, t = read_extrinsic(EXTR)
    print(f"R =\n{R}")
    print(f"t = {t}")
    print()
    for i in range(8):
        pcd = DATA / "pcd" / f"scene_{i:02d}.pcd"
        img = DATA / "images" / f"scene_{i:02d}.png"
        out = OUT / f"proj_scene_{i:02d}.png"
        if not pcd.exists() or not img.exists():
            print(f"skip scene_{i:02d}: missing input")
            continue
        overlay_one(pcd, img, out, R, t)


if __name__ == "__main__":
    main()
