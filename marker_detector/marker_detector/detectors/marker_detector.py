from abc import ABC

import cv2
import numpy as np


class MarkerDetector(ABC):
    """Base class for marker detectors like ArucoDetector and STagDetector"""

    def __init__(self, marker_size: float):
        self.marker_size = marker_size

    def detect_markers(self, image) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError(
            "Subclasses must implement the get_marker_poses method"
        )

    def get_marker_poses(
        self, image, camera_matrix, camera_distortion
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:

        marker_ids, corners = self.detect_markers(image)

        r_vecs, t_vecs = np.array([]), np.array([])
        if len(corners) > 0:
            r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, camera_matrix, camera_distortion
            )

        return marker_ids, r_vecs, t_vecs


# cv2.aruco.drawDetectedMarkers(
#     cv_image, corners, marker_ids, borderColor=(0, 0, 255)
# )

# for r_vec, t_vec in zip(rvecs, tvecs):
#     cv2.drawFrameAxes(
#         cv_image,
#         self.camera_matrix,
#         self.dist_coeffs,
#         r_vec,
#         t_vec,
#         self.marker_length / 2.0,
#         3,
#     )

# cv2.imshow("Frame", cv_image)
# cv2.waitKey(100)
