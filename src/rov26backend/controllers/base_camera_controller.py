import cv2
import subprocess
import imageio_ffmpeg
from rov26backend.utils.device_fetching import get_webcam_device_idx
import logging
import sys
import threading
import time

logger = logging.getLogger("ROV.cam")


class BaseCamera:
    """
    Parent class handling common camera operations:
    capture setup, frame reading, processing delegation, and direct FFmpeg streaming.
    """

    def __init__(
        self,
        stream_url,
        fps=30,
        width=640,
        height=480,
        camera_id="",
    ):
        self.camera_id = camera_id

        self.stream_url = stream_url.replace("localhost", "127.0.0.1")

        self.width = width
        self.height = height
        self.fps = fps

        self.show_result = False
        self.vid_writer = None

        self.ffmpeg_process = None
        self._thread = None
        self._is_running = threading.Event()

        self._init_video_cap()

    def _init_video_cap(self):
        try:
            if self.camera_id == "":
                self.camera_idx = 0
            else:
                self.camera_idx = get_webcam_device_idx(self.camera_id)
            self.cap = (
                cv2.VideoCapture(self.camera_idx, cv2.CAP_DSHOW)
                if sys.platform != "linux"
                else cv2.VideoCapture(self.camera_idx)
            )
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception as e:
            logger.error(f"ERROR IN INIT CAMERA: {e}")
            self.cap = None

    def _start_ffmpeg(self, frame):
        """Dynamically starts FFmpeg right before the first frame is sent."""
        height, width, _ = frame.shape
        ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()

        ffmpeg_cmd = [
            ffmpeg_exe,
            "-y",
            "-thread_queue_size",
            "512",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(self.fps),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "superfast",
            "-tune",
            "zerolatency",
            "-crf",
            "18",
            "-b:v",
            "2.5M",
            "-pix_fmt",
            "yuv420p",
            "-rtsp_transport",
            "tcp",
            "-f",
            "rtsp",
            self.stream_url,
        ]

        logger.info(f"Starting FFmpeg for {self.stream_url} at {width}x{height}")

        self.ffmpeg_log_file = open(f"rov_{self.camera_id}_ffmpeg.log", "a")

        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=self.ffmpeg_log_file,
            stderr=self.ffmpeg_log_file,
            bufsize=10**8,
        )

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def run(self):
        while self._is_running.is_set():
            try:
                success, frame = self.cap.read()
                if not success:
                    logger.info("Video capture fail, retrying in 1 second...")
                    time.sleep(1)
                    self._init_video_cap()
                    continue

                self.process_and_publish(frame)

                if self.ffmpeg_process is None:
                    self._start_ffmpeg(frame)
                if self.ffmpeg_process.poll() is None:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    self.ffmpeg_process.stdin.flush()
                else:
                    logger.error("FFmpeg crashed! Clearing process.")
                    self.ffmpeg_process = None

            except BrokenPipeError:
                logger.error("Broken pipe: FFmpeg shut down unexpectedly.")
                self.ffmpeg_process = None
            except Exception as e:
                logger.error(f"Camera Loop Error: {e}")

    def process_and_publish(self, frame):
        """To be overridden by child classes"""
        raise NotImplementedError("Child classes must implement this method")

    def cleanup(self):
        """Release hardware resources and close FFmpeg"""
        if self.cap:
            self.cap.release()
        if self.vid_writer:
            self.vid_writer.release()
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()

        if hasattr(self, "ffmpeg_log_file") and not self.ffmpeg_log_file.closed:
            self.ffmpeg_log_file.close()
