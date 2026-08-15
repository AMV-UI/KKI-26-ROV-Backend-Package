from rov26backend.controllers.base_camera_controller import BaseCamera
import sys


class BottomCamera(BaseCamera):
    """
    Child class for the Bottom (Down) Camera.
    Handles specific downward-facing computer vision.
    """

    def __init__(
        self,
        camera_id="046d_C270_HD_WEBCAM_55E22480"
        if sys.platform == "linux"
        else "7&2C094952&0&0000",
    ):
        super().__init__(
            camera_id=camera_id,
            stream_url="rtsp://localhost:8554/live/bottomcam",
            fps=30,
        )

    def process_and_publish(self, frame):
        # Insert bottom-specific image processing here (e.g. line tracking, box detection)
        # Anything drawn onto `frame` here will automatically be streamed.
        pass
