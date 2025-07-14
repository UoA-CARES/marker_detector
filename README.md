# marker_detector

A lightweight ROS 2 package for detecting ArUco markers in camera streams. The main functionality is provided by the `ArucoDetectorNode`, which detects markers in real time, estimates their poses using camera intrinsics, and publishes the results as a `PoseArray`.

---

## Features

- Detects ArUco markers using OpenCV
- Estimates 3D pose (position + orientation) of each detected marker
- Publishes poses to a `marker_detector/Markers`
- Supports configurable ArUco and STag dictionary and marker size
- Designed for use with onboard cameras (e.g., drones, robots)
---

## Package: `marker_detector`

A ROS 2 package that subscribes to camera images and calibration data, detects ArUco or STag markers in real time, estimates their 3D poses, and publishes the results as a `geometry_msgs/PoseArray`.  
This node is designed for integration with real camera hardware and supports configurable marker dictionary and size.

See [marker_detector/README.md](marker_detector/README.md) for full details on topics, parameters, and launch instructions.

---

## Package: `marker_detector_interfaces`
Custom message definition for Markers with pose and IDs. 
You can install the message definitions without the overhead of the full package for non-camera facing nodes. 

See [marker_detector_interfaces/README.md](marker_detector_interfaces/README.md) for full details on topics, parameters, and launch instructions.

---