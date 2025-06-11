import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoMockPublisherNode(Node):
    """
    A ROS 2 node that publishes a synthetic camera image containing a single ArUco marker
    and corresponding camera calibration info for testing marker detection pipelines.

    - Publishes a synthetic color image (with a centered ArUco marker) to 'camera/image_raw'.
    - Publishes matching CameraInfo to 'camera/camera_info'.
    - Marker ID, size, and camera intrinsics are hardcoded for repeatable testing.
    - Useful for validating ArUco detection and pose estimation nodes without real hardware.

    Topics:
        - camera/image_raw (sensor_msgs/Image): Synthetic camera image with ArUco marker.
        - camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info.

    Example usage:
        ros2 run marker_detector aruco_mock_publisher_node
    """
    def __init__(self):
        super().__init__('aruco_mock_publisher_node')
        self.bridge = CvBridge()

        # Publishers for image and camera_info
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Timer to publish periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Prepare synthetic camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = 640
        self.camera_info_msg.height = 480

        # Simple pinhole camera intrinsics (fx, fy, cx, cy)
        fx = 600.0
        fy = 600.0
        cx = 320.0
        cy = 240.0
        self.camera_info_msg.k = [fx, 0.0, cx,
                                  0.0, fy, cy,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

        # CameraInfo additional fields (for completeness)
        self.camera_info_msg.distortion_model = 'plumb_bob'
        self.camera_info_msg.header.frame_id = 'camera_frame'

        # Generate an ArUco marker image once, size 200x200 pixels
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.marker_id = 0
        self.marker_size_px = 200
        self.marker_img = np.zeros((self.marker_size_px, self.marker_size_px), dtype=np.uint8)
        self.marker_img = cv2.aruco.drawMarker(aruco_dict, self.marker_id, self.marker_size_px, self.marker_img, 1)

        # Create a blank color image and place the marker on it at center
        self.image_width = 640
        self.image_height = 480
        self.color_image = np.ones((self.image_height, self.image_width, 3), dtype=np.uint8) * 255  # white background

        # Place marker at center
        x_offset = (self.image_width - self.marker_size_px) // 2
        y_offset = (self.image_height - self.marker_size_px) // 2
        self.color_image[y_offset:y_offset+self.marker_size_px, x_offset:x_offset+self.marker_size_px, 0] = self.marker_img
        self.color_image[y_offset:y_offset+self.marker_size_px, x_offset:x_offset+self.marker_size_px, 1] = self.marker_img
        self.color_image[y_offset:y_offset+self.marker_size_px, x_offset:x_offset+self.marker_size_px, 2] = self.marker_img

        self.get_logger().info('ArucoMockPublisherNode initialized')

    def timer_callback(self):
        # Update header timestamps
        now = self.get_clock().now().to_msg()
        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.header.frame_id = 'camera_frame'

        # Publish camera info once or repeatedly
        self.camera_info_pub.publish(self.camera_info_msg)

        # Convert the synthetic image to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(self.color_image, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_frame'

        self.image_pub.publish(img_msg)
        self.get_logger().info('Published mock image and camera_info')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMockPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
