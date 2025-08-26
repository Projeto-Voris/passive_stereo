
# Passive Stereo ROS2

This repository provides a simple ROS 2 pipeline for **passive stereo depth estimation** using [Retinify](https://docs.retinify.ai/) and conversion to 3D colored point clouds.

---

## Features

* 🔹 **Disparity Estimation**
  A node that runs the Retinify disparity network on rectified stereo pairs and publishes the disparity image.

* 🔹 **Disparity → PointCloud2**
  A node that converts the disparity map into a `sensor_msgs/PointCloud2`, using camera calibration parameters.
  The left rectified image is used as the **RGB texture** of the 3D point cloud.

* 🔹 ROS 2 native (tested on Humble + JetPack 6.2 / x86_64).

---
### Dependencies

* ROS 2 Humble
* OpenCV 4.5.4
* [Retinify](https://docs.retinify.ai/installation.html)
---

## Installation

Clone into your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Projeto-Voris/passive_stereo.git
cd ..
colcon build
source install/setup.bash
```



## Nodes

### `retinify_disp`

Runs Retinify disparity estimation.

**Subscribed topics**:

* `/stereo/left/image_raw` (`sensor_msgs/Image`)
* `/stereo/right/image_raw` (`sensor_msgs/Image`)
* `/stereo/left/camera_info` (`sensor_msgs/CameraInfo`)
*  `/stereo/right/camera_info` (`sensor_msgs/CameraInfo`)

**Published topics**:

* `/stereo/disparity/image` (`stereo_msgs/DisparityImage`)
* `/stereo/disparity/debug/image` (`sensor_msgs/Image`)
* `/stereo/left/rect_image` (`sensor_msgs/Image`)
* `/stereo/right/rect_image` (`sensor_msgs/Image`)

**Parameters**

| Parameter                   | Description                                    | Value    |
|----------------------------|------------------------------------------------|----------|
| `publish_rectified`             | Publish rectified images                       | `True`   |
| `debug_image`   | Publish debug disparity image                  | `0.12`   |
| `m_occupancy_max_thresh`   | Maximum occupancy threshold                    | `0.97`   |

---

### `disp_to_pointcloud`

Converts disparity to colored point cloud from left rectified image.

**Subscribed topics**:

* `/stereo/disparity` (`stereo_msgs/DisparityImage`)
* `/stereo/left/image_rect` (`sensor_msgs/Image`)
* `/stereo/left/camera_info` (`sensor_msgs/CameraInfo`)

**Published topics**:

* `/stereo/points2` (`sensor_msgs/PointCloud2`)
* 
**Parameters**

| Parameter                   | Description                                    | Value    |
|----------------------------|------------------------------------------------|----------|
| `sampling_factor`   | Publish debug disparity image                  | `0.8`   |
| `crop_factor`   | Maximum occupancy threshold                    | `0.9`   |

---

## Usage

1. Launch your stereo camera drivers (e.g. FLIR, RealSense, ZED) that publish raw images and camera info.
2. Run:

   ```bash
   ros2 launch passive_stereo stereo.launch.py
   ```

## License

MIT License. See [LICENSE](LICENSE) for details.

