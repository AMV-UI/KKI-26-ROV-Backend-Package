#!/usr/bin/env python3

import logging
import cv2
from pyzbar.pyzbar import decode
from core_perception.base_camera_controller import BaseCameraNode


class FrontCameraNode(BaseCameraNode):

    def __init__(self, webcam=False):
        super().__init__(
            node_name="front_camera",
            stream_url="rtsp://localhost:8554/live/frontcam",
            webcam=webcam,
            cameraIndex=0
        )

    def process_and_publish(self, frame):
        # Deteksi QR Code pada Front Camera
        decoded_objects = decode(frame)

        for obj in decoded_objects:
            data = obj.data.decode("utf-8")
            logging.info(f"[{self.node_name}] QR Code Detected: {data}")

            points = obj.polygon
            if len(points) == 4:
                pts = [(pt.x, pt.y) for pt in points]
                for i in range(4):
                    cv2.line(frame, pts[i], pts[(i + 1) % 4], (0, 255, 0), 3)

                text_x = pts[0][0]
                text_y = max(pts[0][1] - 10, 20)

                cv2.putText(
                    frame,
                    data,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )