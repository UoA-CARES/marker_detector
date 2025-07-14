# 🧱 Node: `ArucoDetectorNode`

## ✅ Description

The `ArucoDetectorNode` subscribes to a camera stream (`/camera/image_raw`) and calibration data (`/camera/camera_info`), detects markers in the image, estimates each marker's 3D pose, and publishes the results.

### ▶️ How to Run

To run the node directly with ROS 2:

```sh
ros2 run marker_detector aruco_detector_node
```

Or, to launch with parameters and remappings, use your launch file (see below).

## 📤 Published Topics

- `/aruco_poses` (`geometry_msgs/PoseArray`)  
  3D pose of all detected markers in the image frame.

## 📥 Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/Image`)  
  Camera image stream.
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)  
  Camera intrinsic matrix and distortion coefficients.

## 🧩 Parameters

| Name             | Type   | Description                                                    | Default         |
|------------------|--------|----------------------------------------------------------------|-----------------|
| `aruco_dict_id`  | string | OpenCV dictionary name (e.g., `"DICT_4X4_50"`, `"DICT_5X5_100"`) | `"DICT_4X4_50"` |
| `marker_length`  | float  | Marker side length in meters (e.g., 0.1)                       | `0.1`           |

Supported dictionary names correspond to OpenCV constants like `DICT_4X4_50`, `DICT_5X5_100`, etc.

---

## 🚀 Launch Example

Here’s how to include the node in a launch file using the ROS 2 Python launch API.

### `launch/marker_detector_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marker_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[
                {'aruco_dict_id': 'DICT_4X4_50'},  # OpenCV dictionary name as string
                {'marker_length': 0.1},            # in meters
            ],
            remappings=[
                ('/camera/image_raw', '/my_camera/image_raw'),
                ('/camera/camera_info', '/my_camera/camera_info'),
            ],
            output='screen'
        )
    ])
```