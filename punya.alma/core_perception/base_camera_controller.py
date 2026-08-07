#!/usr/bin/env python3

import logging
import subprocess
import time
import traceback
import cv2
from cv2_enumerate_cameras import enumerate_cameras

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] [%(levelname)s] %(message)s"
)


class BaseCameraNode:
    """Parent class handling common camera operations:
    capture setup, frame reading, processing delegation, and direct FFmpeg
    streaming.
    """

    def __init__(
        self,
        node_name,
        stream_url,
        fps=30,
        width=640,
        height=480,
        webcam=False,
        cameraIndex=0
    ):
        self.node_name = node_name
        self.stream_url = stream_url
        self.fps = fps
        self.show_result = False
        self.vid_writer = None
        self.is_running = False

        for camera_info in enumerate_cameras():
            print(camera_info)

        self.cap = cv2.VideoCapture(cameraIndex)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # --- Initialize FFmpeg Pipeline ---
        # --- Initialize FFmpeg Pipeline ---
        ffmpeg_cmd = [
            "ffmpeg",
            "-loglevel", "quiet",
            "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(self.fps),
            "-i", "-",                     
            "-c:v", "libx264",
            "-preset", "ultrafast",        
            "-tune", "zerolatency",
            "-bf", "0",                    # MATIKAN B-Frames (Penyebab utama OBS freeze!)
            "-g", "1",                     # SETIAP FRAME ADALAH KEYFRAME (Anti Freeze 100%)
            "-pix_fmt", "yuv420p",
            "-rtsp_transport", "tcp",
            "-f", "rtsp",
            self.stream_url
        ]
        
        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, bufsize=10**8
        )

        logging.info(
            f"Initialized {node_name} streaming to {stream_url} at {fps} FPS"
        )

    def visualize(self, img, window_name="Output", scale=0.6):
        """Display annotated frame"""
        display_img = cv2.resize(
            img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        cv2.imshow(window_name, display_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            logging.info("Exit requested by user (q pressed).")
            return True
        return False

    def run(self):
        self.is_running = True
        frame_duration = 1.0 / self.fps

        try:
            while self.is_running:
                loop_start = time.time()

                success, frame = self.cap.read()
                if not success or frame is None or frame.size == 0:
                    time.sleep(0.005)
                    continue

                # 1. Olah frame di child class (misal QR code/teks)
                self.process_and_publish(frame)

                # 2. Kirim frame ke FFmpeg jika proses masih hidup
                if self.ffmpeg_process and self.ffmpeg_process.poll() is None:
                    try:
                        self.ffmpeg_process.stdin.write(frame.tobytes())
                        self.ffmpeg_process.stdin.flush()  # Wajib flush agar buffer tidak tertahan
                    except (BrokenPipeError, OSError):
                        logging.error(f"[{self.node_name}] FFmpeg Pipe broken!")
                        break

                # 3. GUI Window jika show_result True
                if self.show_result:
                    if self.visualize(frame, window_name=self.node_name):
                        self.is_running = True

                # Penjagaan FPS Real-Time
                elapsed = time.time() - loop_start
                sleep_time = frame_duration - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            logging.error(f"[{self.node_name}] Error in run loop: {e}")
        finally:
            self.cleanup()

    def process_and_publish(self, frame):
        """To be overridden by child classes"""
        raise NotImplementedError("Child classes must implement this method")

    def cleanup(self):
        """Release hardware resources and close FFmpeg"""
        logging.info("Cleaning up resources...")
        self.cap.release()
        if self.vid_writer:
            self.vid_writer.release()
        if self.ffmpeg_process:
            try:
                if self.ffmpeg_process.stdin:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=2.0)
            except Exception:
                self.ffmpeg_process.kill()
        cv2.destroyAllWindows()