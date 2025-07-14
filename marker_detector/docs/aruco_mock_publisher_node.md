# 🧪 Node: `ArucoMockPublisherNode`

## ✅ Description

The `ArucoMockPublisherNode` is a ROS 2 utility node that publishes synthetic camera images containing a single ArUco marker, along with matching camera calibration data. This is useful for testing and validating ArUco detection and pose estimation pipelines without requiring real hardware.

## 📤 Published Topics

- `/camera/image_raw` (`sensor_msgs/Image`)  
  Synthetic color image with a centered ArUco marker.
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)  
  Camera calibration info matching the synthetic image.

## ⚙️ Details

- Publishes a 640x480 color image with a single ArUco marker (ID 0, 200x200 px) centered on a white background.
- Camera intrinsics are set to simple pinhole values (fx=600, fy=600, cx=320, cy=240) with no distortion.
- All messages use the frame ID `camera_frame`.
- Publishes at 1 Hz by default.

## ▶️ How to Run

```sh
ros2 run marker_detector aruco_mock_publisher_node
```

## 🧩 Example Use Case

Use this node to simulate a camera for developing and testing your ArUco marker detection pipeline, or to validate downstream consumers of `/camera/image_raw` and `/camera/camera_info` topics.

---

## 📝 Example Output

- A synthetic image with a black ArUco marker on a white background, published to `/camera/image_raw`.
- Matching camera calibration data published to `/camera/camera_info`.

---

## 🚀 Launch Example

You can include this node in a launch file as follows:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marker_detector',
            executable='aruco_mock_publisher_node',
            name='aruco_mock_publisher_node',
            output='screen'
        )
    ])
```