import cv2
import numpy as np

from config import (
    GREEN_HSV_LOW,
    GREEN_HSV_HIGH,
    BLOB_AREA_MIN,
    STABLE_FRAME_COUNT,
)


class SignalWatcher:
    """
    Watches the video feed for a steady green LED signal.
    Applies HSV masking and requires the signal to persist across
    multiple consecutive frames before reporting a confirmed detection.
    """

    def __init__(self) -> None:
        self._streak = 0

    def _find_dominant_blob(self, mask) -> tuple[bool, tuple | None]:
        """
        Scans contours in the mask and returns (found, centroid).
        Only considers blobs above the minimum area threshold.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_area = 0
        centroid = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < BLOB_AREA_MIN or area <= best_area:
                continue
            best_area = area
            m = cv2.moments(cnt)
            if m["m00"] != 0:
                centroid = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))

        return best_area > 0, centroid

    def analyze(self, frame):
        """
        Process a single video frame and return detection results.

        Returns:
            visible    : green blob is present in this frame
            confirmed  : signal has been stable long enough to act on
            overlay    : annotated copy of the frame for debugging
            mask       : binary mask of the detected green region
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lo = np.array(GREEN_HSV_LOW, dtype=np.uint8)
        hi = np.array(GREEN_HSV_HIGH, dtype=np.uint8)

        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        visible, centroid = self._find_dominant_blob(mask)

        self._streak = (self._streak + 1) if visible else 0
        confirmed = self._streak >= STABLE_FRAME_COUNT

        overlay = self._draw_overlay(frame.copy(), visible, centroid)
        return visible, confirmed, overlay, mask

    def _draw_overlay(self, frame, visible: bool, centroid) -> None:
        label = "SIGNAL: ON" if visible else "SIGNAL: OFF"
        color = (0, 255, 0) if visible else (0, 0, 255)

        cv2.putText(frame, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"Stable frames: {self._streak}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if centroid is not None:
            cv2.circle(frame, centroid, 10, (0, 255, 0), 2)

        return frame