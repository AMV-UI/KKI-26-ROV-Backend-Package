import queue
import threading
import cv2
import numpy as np
import logging
from qrdet import QRDetector

from rov26backend.models.vision_state import VisionState

logger = logging.getLogger("ROV.vision")


class QRDebouncer:
    """Smoothing polygon"""

    def __init__(
        self, required_streak=3, max_missing_frames=4, max_jump_pixels=75, alpha=0.3
    ):
        self.required_streak = required_streak
        self.max_missing_frames = max_missing_frames
        self.max_jump_pixels = max_jump_pixels
        self.alpha = alpha

        self.current_streak = 0
        self.missing_frames = 0
        self.is_confirmed = False

        self.tracking_polygon = None
        self.persistent_polygon = None

    def update(self, detected_polygon, frame_shape):
        if detected_polygon is not None and self.tracking_polygon is not None:
            old_center = np.mean(self.tracking_polygon, axis=0)
            new_center = np.mean(detected_polygon, axis=0)
            jump_distance = np.linalg.norm(new_center - old_center)

            if jump_distance > self.max_jump_pixels:
                detected_polygon = None
            else:
                detected_polygon = (self.alpha * detected_polygon) + (
                    (1 - self.alpha) * self.tracking_polygon
                )
                detected_polygon = np.int32(detected_polygon)

        if detected_polygon is not None:
            self.current_streak += 1
            self.missing_frames = 0
            self.tracking_polygon = detected_polygon

            if self.current_streak >= self.required_streak:
                self.is_confirmed = True
                self.persistent_polygon = detected_polygon
        else:
            self.missing_frames += 1

            if self.missing_frames > self.max_missing_frames:
                self.current_streak = 0
                self.is_confirmed = False
                self.tracking_polygon = None

        if self.persistent_polygon is not None:
            clamped_polygon = np.copy(self.persistent_polygon)
            max_y, max_x = frame_shape[0] - 1, frame_shape[1] - 1

            clamped_polygon[:, 0] = np.clip(clamped_polygon[:, 0], 0, max_x)
            clamped_polygon[:, 1] = np.clip(clamped_polygon[:, 1], 0, max_y)

            return clamped_polygon

        return None


class QRTrackerPipeline:
    """Class gabungan: AI Detector (qrdet) + Debouncer untuk menghaluskan Polygon."""

    def __init__(self, model_size="n", conf_th=0.5):
        self.detector = QRDetector(model_size=model_size, conf_th=conf_th)
        self.debouncer = QRDebouncer()

    def process_frame(self, frame):
        """Menerima frame BGR, deteksi dengan AI, lalu di-debounce."""
        detections = self.detector.detect(image=frame, is_bgr=True)

        raw_box = None
        if detections:
            polygon = np.array(detections[0]["polygon_xy"], dtype=np.int32)
            perimeter = cv2.arcLength(polygon, True)
            corners = cv2.approxPolyDP(polygon, 0.05 * perimeter, True)

            if len(corners.shape) == 3:
                corners = corners.reshape(-1, 2)
            raw_box = corners

        smoothed_polygon = self.debouncer.update(raw_box, frame.shape)
        return smoothed_polygon


class QRDetectorWorker:
    """Worker Thread yang menyambung dengan FrontCamera.
    
    Tugas: Mengambil frame dari FrontCamera Queue -> Deteksi & Debounce -> Update ke VisionState.
    """

    def __init__(self, frame_queue: queue.Queue, vision_state: VisionState):
        self.frame_queue = frame_queue
        self.vision_state = vision_state
        self.pipeline = QRTrackerPipeline(model_size="n", conf_th=0.5)

        self.is_running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self.thread.start()

    def _worker(self):
        """Loop background yang me-pull frame dari FrontCamera."""
        while self.is_running:
            try:
                # Ambil frame yang dikirim oleh FrontCamera
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if frame is None:
                break

            # Process frame memakai AI & Debouncer
            smoothed_polygon = self.pipeline.process_frame(frame)

            # Update hasilnya langsung ke shared object VisionState
            with self.vision_state as vs:
                if smoothed_polygon is not None:
                    vs.qr_polygon = [
                        (float(pt[0]), float(pt[1])) for pt in smoothed_polygon
                    ]
                else:
                    vs.qr_polygon = []

    def stop(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()