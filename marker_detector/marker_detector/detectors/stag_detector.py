import numpy as np
import stag

from marker_detector.detectors.marker_detector import MarkerDetector


class STagDetector(MarkerDetector):
    """Detector for STag markers"""

    def __init__(
        self, marker_size: float, library_hd: int = 21, error_correction: int = -1
    ):
        super().__init__(marker_size)
        self.dictionary = library_hd
        self.error_correction = error_correction

    def detect_markers(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        (corners, marker_ids, _) = stag.detectMarkers(
            image, self.dictionary, self.error_correction
        )

        return marker_ids, corners
