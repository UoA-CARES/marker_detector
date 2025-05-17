# 📦 marker_detector

A lightweight ROS 2 package for detecting ArUco markers in camera streams. The main functionality is provided by the `ArucoDetectorNode`, which detects markers in real time, estimates their poses using camera intrinsics, and publishes the results as a `PoseArray`.

---

## 🔍 Features

- Detects ArUco markers using OpenCV
- Estimates 3D pose (position + orientation) of each detected marker
- Publishes poses to a `geometry_msgs/PoseArray`
- Supports configurable ArUco dictionary and marker size
- Designed for use with onboard cameras (e.g., drones, robots)

---

## 🧱 Node: `ArucoDetectorNode`

A ROS 2 node that subscribes to camera images and calibration data, detects ArUco markers in real time, estimates their 3D poses, and publishes the results as a `geometry_msgs/PoseArray`.  
This node is designed for integration with real camera hardware and supports configurable marker dictionary and size.

See [docs/aruco_detector_node.md](docs/aruco_detector_node.md) for full details on topics, parameters, and launch instructions.

---

## 🧪 Node: `ArucoMockPublisherNode`

A utility node that publishes synthetic camera images containing a single ArUco marker and matching camera calibration data. This is useful for testing and validating ArUco detection and pose estimation pipelines without requiring real hardware.

- Publishes a 640x480 color image with a single ArUco marker (ID 0, 200x200 px) centered on a white background.
- Camera intrinsics are set to simple pinhole values (fx=600, fy=600, cx=320, cy=240) with no distortion.
- All messages use the frame ID `camera_frame`.
- Publishes at 1 Hz by default.

See [docs/aruco_mock_publisher_node.md](docs/aruco_mock_publisher_node.md) for details and launch instructions.
