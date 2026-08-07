#!/usr/bin/env python3

import logging
import cv2
from core_perception.base_camera_controller import BaseCameraNode


class BottomCameraNode(BaseCameraNode):

    def __init__(self, webcam=False):
        super().__init__(
            node_name="bottom_camera",
            stream_url="rtsp://localhost:8554/live/bottomcam",
            webcam=webcam,
            cameraIndex=1
        )

    def process_and_publish(self, frame):
        # Contoh pemrosesan khusus kamera bawah (misal: penanda visual/garis)
        cv2.putText(
            frame,
            "BOTTOM CAM",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 0),
            2,
        )