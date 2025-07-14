import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from marker_detector.detectors.aruco_detector import ArucoDetector
from marker_detector.detectors.stag_detector import STagDetector
from marker_detector_interfaces.msg import Markers


def cv_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])

    # Convert rotation vector to quaternion
    rot_mat, _ = cv2.Rodrigues(rvec)

    r = R.from_matrix(rot_mat)
    qx, qy, qz, qw = r.as_quat()  # returns [x, y, z, w]

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def pose_to_transform(
    header: Header, marker_id: np.ndarray, pose: Pose
) -> TransformStamped:
    t = TransformStamped()
    t.header = header
    t.child_frame_id = f"marker_{marker_id[0]}"

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    return t


class MarkerDetectorNode(Node):
    """
    A ROS 2 node that detects ArUco/Stag markers in incoming camera images and
    publishes their 3D poses as a geometry_msgs/PoseArray message.

    Subscriptions:
        - /camera/image_raw (sensor_msgs/Image): Camera stream
        - /camera/camera_info (sensor_msgs/CameraInfo): Camera intrinsics

    Publications:
        - /marker_poses (geometry_msgs/PoseArray): Poses of detected ArUco/STag markers

    Parameters:
        - marker_type (str): Type of marker to detect ('Aruco' or 'STag')
        - dict_id (int): Marker dictionary ID (e.g. 0 = DICT_4X4_50)
        - marker_length (float): Marker size in meters

    Launch file usage (Python-style):
    --------------------------------
    Node(
        package='your_package',
        executable='marker_detector_node',
        name='marker_detector',
        parameters=[
            {'marker_type': 'Aruco'},
            {'dict_id': 0},
            {'marker_size': 0.1}
        ],
        remappings=[
            ('/camera/image_raw', '/your_camera/image_raw'),
            ('/camera/camera_info', '/your_camera/camera_info')
        ]
    )
    """

    def __init__(self):
        super().__init__("marker_detector_node")

        self.bridge = CvBridge()

        # Declare and retrieve ROS parameters
        self.declare_parameter("marker_type", "Aruco")
        self.declare_parameter("marker_size", 0.1)
        self.declare_parameter("dict_id", 0)

        marker_type = (
            self.get_parameter("marker_type").get_parameter_value().string_value
        )
        marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        dict_id = self.get_parameter("dict_id").get_parameter_value().integer_value

        if marker_type == "Aruco":
            self.get_logger().info("Using ArucoDetector")
            self.marker_detector = ArucoDetector(
                marker_size=marker_size, dictionary_id=dict_id
            )
        elif marker_type == "STag":
            self.get_logger().info("Using STagDetector")
            self.marker_detector = STagDetector(
                marker_size=marker_size, library_hd=dict_id
            )
        else:
            self.get_logger().error(
                f"Unsupported marker type: {marker_type}. Supported types are 'Aruco' and 'STag'."
            )
            raise ValueError(f"Unsupported marker type: {marker_type}")

        # Camera intrinsics will be filled from /camera_info
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers for image and camera calibration
        self.image_sub = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "camera/camera_info", self.camera_info_callback, 10
        )

        # Publisher for pose array
        self.pose_pub = self.create_publisher(Markers, "marker_poses", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("MarkerDetectorNode started")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None

        self.get_logger().info(
            "Received camera intrinsics. Unsubscribing from camera_info..."
        )

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("Camera info not received yet.")
            return

        self.get_logger().debug("Received image message")
        self.get_logger().debug(f"Image width: {msg.width}, height: {msg.height}")
        try:

            if msg.data is None or len(msg.data) == 0:
                self.get_logger().warn("Empty image data")
                return

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().debug("Finished Bridge conversion")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.get_logger().debug("Converting to grayscale complete")

        # Detect markers in image
        marker_ids, r_vecs, t_vecs = self.marker_detector.get_marker_poses(
            gray_img, self.camera_matrix, self.dist_coeffs
        )
        self.get_logger().debug("Detecting markers complete")

        self.get_logger().debug(f"Detected marker IDs: {marker_ids} {type(marker_ids)}")
        self.get_logger().debug(f"Estimated poses: {r_vecs} {t_vecs}")

        if marker_ids is not None:

            pose_array = Markers()
            pose_array.header = msg.header  # Sync timestamp and frame ID

            for marker_id, rvec, tvec in zip(marker_ids, r_vecs, t_vecs):
                self.get_logger().debug(f"Processing marker ID: {marker_id[0]}")
                self.get_logger().debug(f"Rotation vector: {rvec}")
                self.get_logger().debug(f"Translation vector: {tvec}")

                tvec = tvec[0]
                rvec = rvec[0]

                pose = cv_to_pose(rvec, tvec)

                pose_array.marker_ids.append(int(marker_id[0]))
                pose_array.poses.append(pose)

                # Publish the transform for each marker
                marker_transform = pose_to_transform(msg.header, marker_id, pose)
                self.tf_broadcaster.sendTransform(marker_transform)

                self.get_logger().debug(f"Detected marker ID: {marker_id[0]}")
                self.get_logger().debug(
                    f"Pose: {pose.position.x}, {pose.position.y}, {pose.position.z}"
                )
                self.get_logger().debug(
                    f"Orientation: {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w}"
                )
                self.get_logger().debug("-----------------------------------")

            # Publish all poses in one message
            self.pose_pub.publish(pose_array)
            self.get_logger().debug("Published PoseArray message")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
