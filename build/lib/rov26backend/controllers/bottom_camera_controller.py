import sys

import cv2  # Added OpenCV import
from pyzbar.pyzbar import decode

from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.models.vision_state import VisionState


class BottomCamera(BaseCamera):
    """
    Child class for the Bottom (Down) Camera.
    Handles specific downward-facing computer vision.
    """

    def __init__(
        self,
        vision_state: VisionState,
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
        )
        self.qr_text = "NOT_FOUND"
        self.vision_state = vision_state

    def process_and_publish(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
        )
        decoded_objects = decode(thresh)

        if decoded_objects:
            for obj in decoded_objects:
                data = obj.data.decode("utf-8")
                if data in ["A", "B", "C", "D"]:
                    self.qr_text = data
                    break

        with self.vision_state as vision_state:
            vision_state.qr_side = self.qr_text

        # Note: Any drawings (like cv2.rectangle for the QR bounding box)
        # should still be done on the original `frame` object so they appear in the stream.
