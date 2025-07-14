import cv2
import numpy as np

from marker_detector.detectors.marker_detector import MarkerDetector


class ArucoDetector(MarkerDetector):
    def __init__(self, marker_size: float, dictionary_id: int = cv2.aruco.DICT_4X4_50):
        super().__init__(marker_size)
        self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def detect_markers(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:

        (corners, marker_ids, _) = cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.aruco_params
        )

        return marker_ids, corners
