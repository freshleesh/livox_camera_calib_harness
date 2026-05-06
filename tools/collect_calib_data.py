#!/usr/bin/env python3
"""Capture paired (accumulated PCD, image) scenes for livox_camera_calib.

Workflow per scene:
  1. Park the vehicle / hold sensors still.
  2. Press ENTER. The script accumulates LiDAR for --secs seconds and grabs
     the latest CompressedImage at the end.
  3. Move to a different viewpoint and repeat (5~10 scenes recommended).

Usage:
  source /opt/ros/humble/setup.bash
  python3 collect_calib_data.py \
      --lidar /livox/lidar \
      --image /camera/image_raw/compressed \
      --secs 20
"""

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def msg_to_xyzi(msg: PointCloud2) -> np.ndarray:
    """Return Nx4 float32 [x,y,z,intensity], handling both rclpy APIs."""
    raw = pc2.read_points(
        msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
    )
    if isinstance(raw, np.ndarray) and raw.dtype.names is not None:
        return np.column_stack(
            [raw["x"], raw["y"], raw["z"], raw["intensity"]]
        ).astype(np.float32)
    pts = np.array(list(raw), dtype=np.float32)
    return pts.reshape(-1, 4) if pts.size else pts


def write_pcd_binary_xyzi(points: np.ndarray, path: Path) -> None:
    n = points.shape[0]
    points32 = np.ascontiguousarray(points, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points32.tobytes())


class CalibCollector(Node):
    def __init__(
        self,
        lidar_topic: str,
        image_topic: str,
        out_dir: Path,
        accum_secs: float,
    ) -> None:
        super().__init__("calib_collector")
        self.out_dir = out_dir
        self.pcd_dir = out_dir / "pcd"
        self.img_dir = out_dir / "images"
        self.pcd_dir.mkdir(parents=True, exist_ok=True)
        self.img_dir.mkdir(parents=True, exist_ok=True)

        self.accum_secs = accum_secs
        self.scene_idx = self._next_scene_idx()

        self._accumulating = False
        self._accum_points: list[np.ndarray] = []
        self._accum_lock = threading.Lock()
        self._latest_image: np.ndarray | None = None
        self._image_lock = threading.Lock()
        self._lidar_msg_count = 0
        self._image_msg_count = 0

        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, 10)
        self.create_subscription(
            CompressedImage, image_topic, self._image_cb, 10
        )

        self.get_logger().info(f"lidar topic : {lidar_topic}")
        self.get_logger().info(f"image topic : {image_topic}")
        self.get_logger().info(f"output dir  : {out_dir}")
        self.get_logger().info(f"accumulate  : {accum_secs:.1f} s per scene")
        self.get_logger().info(f"next scene  : {self.scene_idx:02d}")

    def _next_scene_idx(self) -> int:
        existing = sorted(self.pcd_dir.glob("scene_*.pcd"))
        if not existing:
            return 0
        return int(existing[-1].stem.split("_")[1]) + 1

    def _lidar_cb(self, msg: PointCloud2) -> None:
        self._lidar_msg_count += 1
        with self._accum_lock:
            if not self._accumulating:
                return
            pts = msg_to_xyzi(msg)
            if pts.size:
                self._accum_points.append(pts)

    def _image_cb(self, msg: CompressedImage) -> None:
        self._image_msg_count += 1
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("failed to decode CompressedImage")
            return
        with self._image_lock:
            self._latest_image = img

    def topics_alive(self) -> tuple[bool, bool]:
        return self._lidar_msg_count > 0, self._image_msg_count > 0

    def collect_scene(self) -> bool:
        idx = self.scene_idx
        with self._image_lock:
            have_image = self._latest_image is not None
        if not have_image:
            self.get_logger().error(
                "no image received yet — check image topic before capturing"
            )
            return False

        with self._accum_lock:
            self._accum_points = []
            self._accumulating = True

        self.get_logger().info(
            f"[scene {idx:02d}] accumulating {self.accum_secs:.1f}s — HOLD STILL"
        )
        deadline = time.monotonic() + self.accum_secs
        while time.monotonic() < deadline:
            time.sleep(0.1)

        with self._accum_lock:
            self._accumulating = False
            chunks = self._accum_points
            self._accum_points = []

        if not chunks:
            self.get_logger().error(
                f"[scene {idx:02d}] no LiDAR points captured — check lidar topic"
            )
            return False

        all_pts = np.vstack(chunks)
        pcd_path = self.pcd_dir / f"scene_{idx:02d}.pcd"
        write_pcd_binary_xyzi(all_pts, pcd_path)

        with self._image_lock:
            img = self._latest_image
        img_path = self.img_dir / f"scene_{idx:02d}.png"
        cv2.imwrite(str(img_path), img)

        self.get_logger().info(
            f"[scene {idx:02d}] {all_pts.shape[0]:>9d} pts -> "
            f"{pcd_path.name}, {img_path.name}"
        )
        self.scene_idx += 1
        return True


DEFAULT_OUT = Path(__file__).resolve().parent.parent / "data"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Collect paired PCD + image scenes for livox_camera_calib"
    )
    parser.add_argument("--lidar", default="/livox/lidar")
    parser.add_argument("--image", default="/camera/image_raw/compressed")
    parser.add_argument("--out", default=DEFAULT_OUT, type=Path)
    parser.add_argument("--secs", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = CalibCollector(args.lidar, args.image, args.out, args.secs)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    print("=" * 60)
    print(" press ENTER to capture a scene (sensors must be still)")
    print(' type "s"   + ENTER to print topic stats')
    print(' type "q"   + ENTER to quit')
    print("=" * 60)

    try:
        while True:
            line = input().strip().lower()
            if line == "q":
                break
            if line == "s":
                lid_ok, img_ok = node.topics_alive()
                print(
                    f"  lidar received: {lid_ok}, image received: {img_ok}"
                )
                continue
            node.collect_scene()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
