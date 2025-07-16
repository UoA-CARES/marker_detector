# marker_detector

A lightweight ROS 2 package for detecting ArUco and STag markers in camera streams. The main functionality is provided by the `MarkerDetectorNode`, which detects markers in real time, estimates their poses using camera intrinsics, and publishes the results as a `Markers`.

# Installation Instructions
`git clone`the repository into your desired ros2 workspace directory on your local machine.

Run `pip3 install -r requirements' in the **root directory** of this package.

`colcon build` your ros2 workspace.

# Node: `MarkerDetectorNode`

The `MarkerDetectorNode` subscribes to a camera stream (`/camera/image_raw`) and calibration data (`/camera/camera_info`), detects markers in the image, estimates each marker's 3D pose, and publishes the results via topic and transforms.

## Usage

To run the node directly with ROS 2:

```sh
ros2 run marker_detector marker_detector_node
```

```sh
ros2 launch marker_detector marker_detector.launch.py
```

Or, to launch with parameters and remappings, use your launch file (see below).

## Published Topics

- `/marker_poses` (`marker_detector_interfaces/Markers`)  
  3D pose of all detected markers in the image frame.

## Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/Image`)  
  Camera image stream.
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)  
  Camera intrinsic matrix and distortion coefficients.

## Parameters

| Name             | Type   | Description                                                    | Default         |
|------------------|--------|----------------------------------------------------------------|-----------------|
| `marker_type`  | string | Marker Type - Aruco or STag | `"Aruco"` |
| `dict_id`  | int | Marker dictionary name | 0 = `"DICT_4X4_50"` |
| `marker_length`  | float  | Marker side length in meters (e.g., 0.1)                       | `0.1`           |
| `camera_name`  | str  | Name of the camera to subscribe too                       | `camera`           |
---

## Launch Example

Here’s how to include the node in a launch file using the ROS 2 Python launch API.

### `launch/marker_detector_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='marker_detector_node',
            name='marker_detector',
            parameters=[
                {'marker_type': 'Aruco'},
                {'dict_id': 0},
                {'marker_size': 0.1}
                {'camera_name': 'camera'}
            ],
            remappings=[
                ('/camera/image_raw', '/your_camera/image_raw'),
                ('/camera/camera_info', '/your_camera/camera_info')
            ]
        )
    ])
```

## Node: `ArucoMockPublisherNode`

A utility node that publishes synthetic camera images containing a single ArUco marker and matching camera calibration data. This is useful for testing and validating ArUco detection and pose estimation pipelines without requiring real hardware.

- Publishes a 640x480 color image with a single ArUco marker (ID 0, 200x200 px) centered on a white background.
- Camera intrinsics are set to simple pinhole values (fx=600, fy=600, cx=320, cy=240) with no distortion.
- All messages use the frame ID `camera_frame`.
- Publishes at 1 Hz by default.


