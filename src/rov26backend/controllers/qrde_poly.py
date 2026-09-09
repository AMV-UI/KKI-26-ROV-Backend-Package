import logging
import queue
import threading

import cv2
import numpy as np
from qrdet import QRDetector

from rov26backend.controllers.polygon_debouncer import QRDebouncer
from rov26backend.models.polygon_state import PolygonState

logger = logging.getLogger("ROV.mixer")


class QRPolygonFinder:
    def __init__(self, frame_queue, polygon_state: PolygonState):
        self.frame_queue = frame_queue
        self.polygon_state = polygon_state
        self._thread = None
        self._is_running = threading.Event()
        self.debouncer = QRDebouncer()

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def run(self):
        self.detector = QRDetector(model_size="m", conf_th=0.5)

        while self._is_running.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                detections = self.detector.detect(image=frame, is_bgr=True)

                new_corners = []
                for detection in detections:
                    polygon = np.array(detection["polygon_xy"], dtype=np.int32)
                    perimeter = cv2.arcLength(polygon, True)
                    corners = cv2.approxPolyDP(polygon, 0.05 * perimeter, True)
                    new_corners.append(corners)

                # Inside qrde_poly.py -> run()
                if len(new_corners) > 0:
                    # Extract the first polygon and flatten it from (4, 1, 2) to (4, 2)
                    poly = new_corners[0].reshape(-1, 2)
                    smoothed_corners = self.debouncer.update(poly, frame.shape)

                    with self.polygon_state as polygon:
                        # Wrap back in a list for downstream cv2.polylines compatibility
                        polygon.qr_polygon = (
                            [smoothed_corners] if smoothed_corners is not None else []
                        )
                else:
                    # Explicitly pass None so the debouncer registers a missing frame
                    smoothed_corners = self.debouncer.update(None, frame.shape)

                    with self.polygon_state as polygon:
                        polygon.qr_polygon = []

            except queue.Empty:
                continue
