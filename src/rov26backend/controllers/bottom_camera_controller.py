import queue
import sys

import cv2  # Added OpenCV import
from pyzbar.pyzbar import decode

from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.models.qrdat_state import QrdatState


import cv2
import numpy as np


def order_points(points):
    """
    Order 4 QR corners as:
    top-left, top-right, bottom-right, bottom-left
    """
    points = np.array(points, dtype=np.float32)

    # Sum: smallest = top-left, largest = bottom-right
    s = points.sum(axis=1)
    top_left = points[np.argmin(s)]
    bottom_right = points[np.argmax(s)]

    # Difference: smallest = top-right, largest = bottom-left
    diff = np.diff(points, axis=1).flatten()
    top_right = points[np.argmin(diff)]
    bottom_left = points[np.argmax(diff)]

    return np.array(
        [top_left, top_right, bottom_right, bottom_left],
        dtype=np.float32,
    )


def warp_qr(frame, polygon, output_size=300):
    """
    Take QR polygon from qrdet and warp it into
    a square, front-facing image.
    """

    if polygon is None:
        return None

    points = np.array(polygon, dtype=np.float32)

    # Need 4 corners
    if len(points) != 4:
        return None

    rect = order_points(points)

    dst = np.array(
        [
            [0, 0],
            [output_size - 1, 0],
            [output_size - 1, output_size - 1],
            [0, output_size - 1],
        ],
        dtype=np.float32,
    )

    matrix = cv2.getPerspectiveTransform(rect, dst)

    warped = cv2.warpPerspective(
        frame,
        matrix,
        (output_size, output_size),
    )

    return warped

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
            return

        # =========================
        # WARP QR
        # =========================

        warped = warp_qr(
            frame,
            ai_poly,
            output_size=300,
        )

        if warped is None:
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
        
        