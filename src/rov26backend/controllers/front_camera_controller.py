import cv2
from pyzbar.pyzbar import decode
from rov26backend.models.vision_state import VisionState
from rov26backend.controllers.base_camera_controller import BaseCamera


class FrontCamera(BaseCamera):
    def __init__(self, vision_state: VisionState):
        super().__init__(
            node_name="front_camera",
            camera_id="046d_C270_HD_WEBCAM_55E22480",
            stream_url="rtsp://localhost:8554/live/frontcam",
        )
        self.vision_state = vision_state

    def process_and_publish(self, frame):
        decoded_objects = decode(frame)

        for obj in decoded_objects:
            data = obj.data.decode("utf-8")

            if data in ["A", "B", "C", "D"]:
                self.vision_state.update(qr_side=data)
            else:
                self.vision_state.update(qr_side="NOT_FOUND")

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
