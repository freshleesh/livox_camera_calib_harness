# livox_camera_calib_harness

Docker harness + ROS2 capture tooling around
[HKU-MARS livox_camera_calib](https://github.com/hku-mars/livox_camera_calib)
(targetless LiDAR-camera extrinsic calibration).

The upstream tool is **ROS1** and expects pre-recorded `.pcd` + image files.
This repo wraps it for use from a **ROS2 Humble** system without polluting the
host install:

| Stage | Where it runs | What this repo provides |
| --- | --- | --- |
| Capture paired (accumulated PCD, image) scenes | host (ROS2) | `tools/collect_calib_data.py` |
| Run the actual extrinsic calibration | Docker (ROS1 Noetic) | `docker/Dockerfile`, `docker/build.sh`, `docker/run.sh` |

## Layout

```
docker/   noetic-desktop-full image with ceres / eigen / pcl / cv_bridge
tools/    standalone rclpy script for capturing data on the host
data/     captured PCD/PNG go here (gitignored)
src/      livox_camera_calib is cloned here by setup.sh (gitignored)
setup.sh  one-shot: clones livox_camera_calib + creates data dirs
```

## Typical workflow

### One-time, on any machine

```bash
git clone git@github.com:freshleesh/livox_camera_calib_harness.git calib_ws
cd calib_ws
bash setup.sh
```

### On the sensor machine (ROS2 host) — capture data

```bash
# deps usually already present if FAST-LIVO2 / livox driver are installed
sudo apt install -y python3-opencv ros-humble-sensor-msgs-py

source /opt/ros/humble/setup.bash
python3 tools/collect_calib_data.py \
  --lidar /livox/lidar \
  --image /camera/image_raw/compressed \
  --secs 20
```

Press `ENTER` to capture a scene (sensors must be **stationary** during the
20 s accumulation), move the platform, repeat 5–10 times. Output:

```
data/pcd/scene_00.pcd, data/pcd/scene_01.pcd, ...
data/images/scene_00.png, data/images/scene_01.png, ...
```

### On the calibration machine — run extrinsic calibration

```bash
bash docker/build.sh         # 5–15 min, once
bash docker/run.sh           # enter the container
# inside:
catkin_make && source devel/setup.bash
# edit src/livox_camera_calib/config/calib.yaml so image_file/pcd_file
# point at data/images/scene_NN.png and data/pcd/scene_NN.pcd, then:
roslaunch livox_camera_calib calib.launch
```

The result lands in `src/livox_camera_calib/result/extrinsic.txt` (path is
configurable in `calib.yaml`).

## Notes

- The upstream tool was tested with **Livox Avia / Horizon**. MID360 works but
  needs longer accumulation (20–30 s) and edge-rich scenes for good results.
- The capture script saves binary PCD with `x y z intensity` fields, which is
  the format the calibration tool expects.
- `data/` and `src/livox_camera_calib/` are gitignored — clone fresh on each
  machine via `setup.sh`.
