import sys
import threading
import time
import queue
import cv2
import logging
from pyzbar.pyzbar import decode

from rov26backend.controllers.base_camera_controller import BaseCamera
from rov26backend.controllers.solvePnP import solvePnP
from rov26backend.models.vision_state import VisionState
# Import worker AI terpisah dari file qr_detector kamu
from rov26backend.controllers.qrcode_as_apriltag import QRDetectorWorker 

logger = logging.getLogger("ROV.cam")


class FrontCamera(BaseCamera):
    def __init__(
        self,
        vision_state: VisionState,
        auto_event: threading.Event,
        **kwargs,
    ):
        #default_cam_id = "046d_C270_HD_WEBCAM_55E22480" if sys.platform == "linux" else "7&2C094952&0&0000"
        default_cam_id = 0
        super().__init__(
            camera_id = kwargs.get('front_camera_id') or default_cam_id,
            stream_url="rtsp://localhost:8554/live/frontcam",
        )

        self.vision_state = vision_state
        self.auto_event = auto_event
        self.pnp_solver = solvePnP(vision_state)

        self.qr_text = "NOT_FOUND"
        self.last_qr_read = time.time()

        # 1. Inisialisasi Single Queue untuk tempat passing frame ke AI
        self.ai_frame_queue = queue.Queue(maxsize=1)

        # Threading state
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.is_running = True

        # 2. Inisialisasi AI Worker terpisah & passing Queue kamera ke dalamnya
        self.ai_worker = QRDetectorWorker(
            frame_queue=self.ai_frame_queue, 
            vision_state=self.vision_state
        )
        self.ai_worker.start()

        # Thread terpisah untuk pemrosesan vision (Read PyZBar + PnP + UI)
        self.worker_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.worker_thread.start()

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

    def _process_loop(self):
        """Loop pemrosesan cepat untuk PnP dan Render overlay."""
        while self.is_running:
            frame_to_process = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame.copy()

            if frame_to_process is not None:
                self.process_and_publish(frame_to_process)
                time.sleep(0.01)
            else:
                time.sleep(0.02)

    def process_and_publish(self, frame):
        # 1. Dekode Teks QR periodik via PyZbar
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

        # 2. Ambil polygon QR yang SUDAH di-update oleh AI Worker dari VisionState
        with self.vision_state as vision_state:
            vision_state.qr_side = self.qr_text
            points = vision_state.qr_polygon  # List koordinat hasil AI + Debouncer

        if not points or len(points) < 4:
            self.pnp_solver.process([])
            return

        import numpy as np
        raw_polygon = np.array(points, dtype=np.int32)

        # 3. Draw Polylines & SolvePnP
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
            xyz_text = f"X:{tx:.1f} Y:{ty:.1f} Z:{tz:.1f}cm Data:{self.qr_text}"

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
        if hasattr(self, 'ai_worker'):
            self.ai_worker.stop()
        super().stop()