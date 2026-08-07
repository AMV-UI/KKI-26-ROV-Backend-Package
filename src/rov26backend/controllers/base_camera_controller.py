import cv2
import subprocess
import imageio_ffmpeg
from rov26backend.utils.device_fetching import get_webcam_device_idx
import logging

logger = logging.getLogger("ROV.vis")


class BaseCamera:
    """
    Parent class handling common camera operations:
    capture setup, frame reading, processing delegation, and direct FFmpeg streaming.
    """

    def __init__(
        self,
        node_name,
        stream_url,
        fps=30,
        width=640,
        height=480,
        camera_id="",
    ):

        if camera_id == "":
            self.camera_idx = 0
        else:
            self.camera_idx = get_webcam_device_idx(camera_id)

        # Force IPv4 resolution to prevent FFmpeg connection hangs
        self.stream_url = stream_url.replace("localhost", "127.0.0.1")

        self.cap = cv2.VideoCapture(self.camera_idx)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.fps = fps
        self.show_result = False
        self.vid_writer = None

        self.ffmpeg_process = None

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
            f"{width}x{height}",  # Dynamically matches the real webcam output
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
        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, bufsize=10**8
        )

    def run(self):
        """Internal loop called by timer. Reads frame, delegates to child, and streams."""
        try:
            success, frame = self.cap.read()
            if not success:
                return

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
        self.cap.release()
        if self.vid_writer:
            self.vid_writer.release()
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()
