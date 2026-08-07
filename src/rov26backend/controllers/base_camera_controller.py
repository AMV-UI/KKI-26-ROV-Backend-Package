import cv2
import subprocess
import imageio_ffmpeg  # Added this import
from rov26backend.utils.device_fetching import get_webcam_device_idx


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
        webcam=False,
        camera_id="",
    ):

        if camera_id == "":
            self.camera_idx = 0
        else:
            self.camera_idx = get_webcam_device_idx(camera_id)

        self.stream_url = stream_url

        self.cap = cv2.VideoCapture(self.camera_idx)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.fps = fps
        self.show_result = False
        self.vid_writer = None

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
            "-r",
            str(self.fps),
            "-g",
            "15",
            "-forced-idr",
            "1",
            "-fflags",
            "+genpts+igndts",
            "-max_muxing_queue_size",
            "1024",
            "-rtsp_transport",
            "tcp",
            "-f",
            "rtsp",
            self.stream_url,
        ]

        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, bufsize=10**8
        )

    def run(self):
        """Internal loop called by timer. Reads frame, delegates to child, and streams."""
        try:
            success, frame = self.cap.read()
            if not success:
                return

            # 1. Let the child node (Front/Bottom) process the frame in-place
            self.process_and_publish(frame)

            # 2. Stream the processed frame directly to FFmpeg
            if self.ffmpeg_process.poll() is None:
                self.ffmpeg_process.stdin.write(frame.tobytes())
                self.ffmpeg_process.stdin.flush()
            else:
                pass

        except BrokenPipeError:
            pass
        except Exception:
            pass

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
