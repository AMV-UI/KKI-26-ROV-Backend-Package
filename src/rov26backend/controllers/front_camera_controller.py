import logging
import queue
import sys
import threading
import time

import cv2
import numpy as np
from pyzbar.pyzbar import decode

from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.controllers.polygon_debouncer import QRDebouncer
from rov26backend.controllers.solvePnP import solvePnP
from rov26backend.models.polygon_state import PolygonState
from rov26backend.models.vision_state import VisionState

logger = logging.getLogger("ROV.cam")


class FrontCamera(BaseCamera):
    def __init__(
        self,
        vision_state: VisionState,
        auto_event: threading.Event,
        frame_queue,
        polygon_state: PolygonState,
        **kwargs,
    ):
        default_cam_id = (
            "CNFHH52R10643003DBB0_Integrated_Webcam_HD"
            # "046d_C270_HD_WEBCAM_55E22480"
            if sys.platform == "linux"
            else "7&2C094952&0&0000"
        )
        super().__init__(
            camera_id=kwargs.get("front_camera_id") or default_cam_id,
            stream_url="rtsp://localhost:8554/live/frontcam",
        )

        self.vision_state = vision_state
        self.auto_event = auto_event
        self.frame_queue = frame_queue
        self.polygon_state = polygon_state
        self.pnp_solver = solvePnP(vision_state)
        self.qrbouncer = QRDebouncer()

        self.qr_text = "NOT_FOUND"
        self.last_qr_read = time.time()

    def update_frame(self, frame):
        """Memasukkan frame terbaru secara thread-safe dan kirim ke AI Queue."""
        if frame is None:
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()

        # Masukkan frame terbaru ke Queue AI secara non-blocking
        try:
            self.ai_frame_queue.get_nowait()  # Buang frame lama jika AI masih sibuk
        except queue.Empty:
            pass
        self.ai_frame_queue.put(frame.copy())

    def process_and_publish(self, frame):
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            self.frame_queue.get()
            self.frame_queue.put_nowait(frame)

        if time.time() - self.last_qr_read > 0.5:
            decoded_objects = decode(frame)
            if decoded_objects:
                for obj in decoded_objects:
                    data = obj.data.decode("utf-8")
                    if data in ["A", "B", "C", "D"]:
                        self.qr_text = data
                        break
            else:
                self.qr_text = "NOT_FOUND"
            self.last_qr_read = time.time()

        ai_poly, poly_shape = self.polygon_state.get_latest().qr_polygon

        # 2. Ambil polygon QR yang SUDAH di-update oleh AI Worker dari VisionState
        with self.vision_state as vision_state:
            vision_state.qr_side = self.qr_text

            raw_polygon = self.qrbouncer.update(ai_poly, poly_shape)

            if raw_polygon is not None:
                actual_poly = raw_polygon
                points = [(float(pt[0]), float(pt[1])) for pt in actual_poly]

        if raw_polygon is None:
            self.pnp_solver.process([])
            return

        actual_poly = raw_polygon

        # 1. Cast the array to int32 for OpenCV drawing functions
        actual_poly_int = np.int32(actual_poly)

        # 2. Draw using the integer array wrapped in a list
        cv2.polylines(frame, [actual_poly_int], True, (255, 0, 0), 3)

        success, rvec, tvec = self.pnp_solver.process(points)

        if success:
            cv2.drawFrameAxes(
                frame,
                self.pnp_solver.camera_matrix,
                self.pnp_solver.dist_coeffs,
                rvec,
                tvec,
                length=3.0,
                thickness=3,
            )

            tx, ty, tz = tvec.flatten()
            xyz_text = f"X:{tx:.1f} Y:{ty:.1f} Z:{tz:.1f}cm Data:{self.qr_text}"

            # Safely extract scalars using the inner array (int conversion is handled)
            text_x = int(actual_poly[0][0])
            text_y = max(int(actual_poly[0][1]) - 15, 20)

            cv2.putText(
                frame,
                xyz_text,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

    def stop(self):
        self.is_running = False
        if hasattr(self, "ai_worker"):
            self.ai_worker.stop()
        super().stop()
