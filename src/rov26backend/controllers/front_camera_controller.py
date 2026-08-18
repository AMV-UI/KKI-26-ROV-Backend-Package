import sys
import threading
import time
import cv2
from pyzbar.pyzbar import decode
from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.controllers.solvePnP import solvePnP
from rov26backend.models.vision_state import VisionState


class FrontCamera(BaseCamera):
    def __init__(
        self,
        vision_state: VisionState,
        camera_id="046d_C270_HD_WEBCAM_55E22480"
        # camera_id="CNFHH52R10643003DBB0_Integrated_Webcam_HD"
        if sys.platform == "linux"
        else "7&2C094952&0&0000",
    ):
        super().__init__(
            camera_id=camera_id,
            stream_url="rtsp://localhost:8554/live/frontcam",
        )
        self.vision_state = vision_state
        self.pnp_solver = solvePnP(vision_state)

        # Threading state
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.is_running = True

        # Thread terpisah untuk pemrosesan vision
        self.worker_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.worker_thread.start()

    def update_frame(self, frame):
        """Memasukkan frame terbaru dari camera capture thread secara safe."""
        with self.frame_lock:
            self.latest_frame = frame.copy() if frame is not None else None

    def _process_loop(self):
        """Loop pemrosesan yang berjalan independen di background thread."""
        while self.is_running:
            frame_to_process = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame.copy()

            if frame_to_process is not None:
                self.process_and_publish(frame_to_process)
                time.sleep(0.01)  # Beri sedikit jeda agar CPU tidak 100% usage
            else:
                time.sleep(0.02)

    def process_and_publish(self, frame):
        decoded_objects = decode(frame)

        if not decoded_objects:
            with self.vision_state as vision_state:
                vision_state.qr_side = "NOT_FOUND"
                vision_state.qr_polygon = []
            self.pnp_solver.process([])
            return

        for obj in decoded_objects:
            data = obj.data.decode("utf-8")
            points = [(pt.x, pt.y) for pt in obj.polygon]

            side_status = data if data in ["A", "B", "C", "D"] else "NOT_FOUND"
            with self.vision_state as vision_state:
                vision_state.qr_side = side_status
                vision_state.qr_polygon = points

            # Hitung SolvePnP dan update tvec/rvec ke VisionState
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
                xyz_text = f"X:{tx:.1f} Y:{ty:.1f} Z:{tz:.1f}cm"
                cv2.putText(
                    frame,
                    xyz_text,
                    (points[0][0], max(points[0][1] - 15, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

    def stop(self):
        self.is_running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join()
