import queue
import sys

import cv2  # Added OpenCV import
from pyzbar.pyzbar import decode

from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.models.qrdat_state import QrdatState


class BottomCamera(BaseCamera):
    """
    Child class for the Bottom (Down) Camera.
    Handles specific downward-facing computer vision.
    """

    def __init__(
        self,
        vision_state: QrdatState, 
        polygon_state, frame_state,
        **kwargs,
    ):
        default_cam_id = (
            "Generic_HD_camera_20201212000000"
            if sys.platform == "linux"
            else "7&C0B9667&0&0000"
        )
        super().__init__(
            camera_id=kwargs.get("bottom_camera_id") or default_cam_id,
            stream_url="rtsp://localhost:8554/live/bottomcam",
            fps=30,
            thread_name = "Bottom Cam"
        )
        self.qr_text = "NOT_FOUND"
        self.vision_state = vision_state
        self.polygon_state = polygon_state
        self.frame_queue = frame_state


    def process_and_publish(self, frame):
        ai_poly, poly_shape = self.polygon_state.get_latest().qr_polygon

        # Simpan frame untuk QRDetector
        try:
            self.frame_queue.put_nowait(frame.copy())
        except queue.Full:
            self.frame_queue.get()
            self.frame_queue.put_nowait(frame.copy())

        # Kalau QR tidak terdeteksi qrdet
        if ai_poly is None:
            with self.vision_state as vision_state:
                vision_state.qr_side = "NOT_FOUND"
            return

        # =========================
        # 1. Buat bounding box QR
        # =========================
        x, y, w, h = cv2.boundingRect(ai_poly)

        # Pastikan koordinat tidak keluar frame
        frame_h, frame_w = frame.shape[:2]

        x1 = max(x, 0)
        y1 = max(y, 0)
        x2 = min(x + w, frame_w)
        y2 = min(y + h, frame_h)

        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return

        # =========================
        # 2. Preprocessing ROI
        # =========================
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

        thresh = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            2
        )

        # =========================
        # 3. Decode pakai pyzbar
        # =========================
        decoded_objects = decode(thresh)

        self.qr_text = "NOT_FOUND"

        for obj in decoded_objects:
            self.qr_text = obj.data.decode("utf-8")
            break

        # =========================
        # 4. Update state
        # =========================
        with self.vision_state as vision_state:
            vision_state.qr_side = self.qr_text
        
        