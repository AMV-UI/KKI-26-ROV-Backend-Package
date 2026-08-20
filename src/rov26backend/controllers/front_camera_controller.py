import sys
import threading
import time
import cv2
from pyzbar.pyzbar import decode
from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.controllers.solvePnP import solvePnP
from rov26backend.models.vision_state import VisionState
from rov26backend.controllers.qrcode_as_apriltag import QRPolygonFinder
import logging

logger = logging.getLogger("ROV.cam")


class FrontCamera(BaseCamera):
    def __init__(
        self,
        vision_state: VisionState,
        auto_event: threading.Event,
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
        self.auto_event = auto_event
        self.pnp_solver = solvePnP(vision_state)
        self.qr_polygon_finder = QRPolygonFinder()

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
        raw_polygon = self.qr_polygon_finder.get_polygon_from_frame(frame)

        qr_text = "NOT_FOUND"

        if not self.auto_event.is_set():
            decoded_objects = decode(frame)
            if decoded_objects:
                for obj in decoded_objects:
                    data = obj.data.decode("utf-8")
                    if data in ["A", "B", "C", "D"]:
                        qr_text = data
                        break

        with self.vision_state as vision_state:
            vision_state.qr_side = qr_text

            if raw_polygon is not None:
                points = [(float(pt[0]), float(pt[1])) for pt in raw_polygon]
                vision_state.qr_polygon = points
            else:
                vision_state.qr_polygon = []

        if raw_polygon is None:
            self.pnp_solver.process([])
            return

        cv2.polylines(frame, [raw_polygon], True, (255, 0, 0), 3)

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
            xyz_text = f"X:{tx:.1f} Y:{ty:.1f} Z:{tz:.1f}cm Data:{qr_text}"

            text_x = int(raw_polygon[0][0])
            text_y = max(int(raw_polygon[0][1]) - 15, 20)

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
        if self.worker_thread.is_alive():
            self.worker_thread.join()
