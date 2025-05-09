import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    """
    A ROS 2 node that detects ArUco markers in incoming camera images and
    publishes their 3D poses as a geometry_msgs/PoseArray message.

    Subscriptions:
        - /camera/image_raw (sensor_msgs/Image): Camera stream
        - /camera/camera_info (sensor_msgs/CameraInfo): Camera intrinsics

    Publications:
        - /aruco_poses (geometry_msgs/PoseArray): Poses of detected ArUco markers

    Parameters:
        - aruco_dict_id (int): OpenCV ArUco dictionary ID (e.g. 0 = DICT_4X4_50)
        - marker_length (float): Marker size in meters

    Launch file usage (Python-style):
    --------------------------------
    Node(
        package='your_package',
        executable='aruco_detector_node',
        name='aruco_detector',
        parameters=[
            {'aruco_dict_id': 0},
            {'marker_length': 0.1}
        ],
        remappings=[
            ('/camera/image_raw', '/your_camera/image_raw'),
            ('/camera/camera_info', '/your_camera/camera_info')
        ]
    )
    """

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.bridge = CvBridge()

        # Declare and retrieve ROS parameters
        self.declare_parameter('aruco_dict_id', 0)
        self.declare_parameter('marker_length', 0.1)

        self.aruco_dict_id = self.get_parameter('aruco_dict_id').get_parameter_value().integer_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        # ArUco dictionary and detection parameters
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # Camera intrinsics will be filled from /camera_info
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers for image and camera calibration
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)

        # Publisher for pose array
        self.pose_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)

    def camera_info_callback(self, msg: CameraInfo):
        """Handles camera calibration info. Extracts K and D matrices."""
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Received camera intrinsics. Unsubscribing from camera_info...')
        self.camera_info_sub.destroy()  # Only need this once

    def image_callback(self, msg: Image):
        """Handles incoming images, detects ArUco markers, estimates pose, and publishes a PoseArray."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Camera info not received yet.')
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Detect markers in image
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.detector_params)

        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            pose_array = PoseArray()
            pose_array.header = msg.header  # Sync timestamp and frame ID

            for i in range(len(ids)):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

                # Convert rotation vector to quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rot_mat)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw

                pose_array.poses.append(pose)

            # Publish all poses in one message
            self.pose_pub.publish(pose_array)

    def rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
        """Convert a rotation matrix to a quaternion (x, y, z, w)."""
        q = np.empty((4,))
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q[0], q[1], q[2], q[3]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
