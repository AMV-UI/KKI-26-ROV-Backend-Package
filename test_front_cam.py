import logging
import subprocess
import sys
import time
import cv2
from src.rov26backend.controllers.front_camera_controller import FrontCamera
from src.rov26backend.models.vision_state import VisionState

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)

logger = logging.getLogger("ROV.test_front_cam")


def start_ffmpeg_rtsp_publisher(
    width=1920, height=1080, fps=30, rtsp_url="rtsp://localhost:8554/live/frontcam"
):
    """Membuka pipeline FFmpeg untuk mengirimkan raw frame OpenCV ke MediaMTX via

    RTSP.
    """
    command = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",  # Menerima masukan pipa dari stdin
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-f",
        "rtsp",
        rtsp_url,
    ]
    try:
        process = subprocess.Popen(command, stdin=subprocess.PIPE)
        logger.info("FFmpeg RTSP pipeline started to %s", rtsp_url)
        return process
    except Exception as e:
        logger.error(
            "Gagal menjalankan FFmpeg. Pastikan FFmpeg terinstall di sistem! Error: %s",
            e,
        )
        return None


def main():
    logger.info("Starting Threaded Front Camera & MediaMTX Publisher...")

    vision_state = VisionState()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        logger.error("Gagal membuka kamera laptop!")
        return

    try:
        camera = FrontCamera(vision_state)
        logger.info("FrontCamera controller initialized.")
    except Exception as e:
        logger.error("Failed to initialize FrontCamera: %s", e)
        return

    # Inisialisasi FFmpeg untuk stream ke MediaMTX
    rtsp_process = start_ffmpeg_rtsp_publisher(
        width=1920, height=1080, fps=30
    )

    window_name = "ROV Front Camera Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # 1. Kirim frame ke background thread untuk diproses PnP & QR
            camera.update_frame(frame)

            # 2. Ambil snapshot data hasil kalkulasi dari VisionState
            state = vision_state.get_latest()
            qr_side = state.get("qr_side", "NOT_FOUND")
            tvec = state.get("tvec", None)
            euler = state.get("euler_angles", {})

            # 3. Visualisasi Overlay pada frame utama
            cv2.putText(
                frame,
                f"Side: {qr_side}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

            if tvec is not None:
                pnp_text = f"X:{tvec[0]:.1f} Y:{tvec[1]:.1f} Z:{tvec[2]:.1f} cm | Yaw:{euler.get('yaw', 0):.1f} deg"
                cv2.putText(
                    frame,
                    pnp_text,
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            # 4. Stream frame hasil overlay ke MediaMTX via FFmpeg (Pipe)
            if rtsp_process and rtsp_process.poll() is None:
                try:
                    rtsp_process.stdin.write(frame.tobytes())
                except Exception as stream_err:
                    logger.warning("Error writing frame to RTSP stream: %s", stream_err)

            # 5. Tampilkan ke lokal window
            cv2.imshow(window_name, frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                logger.info("Exit key pressed.")
                break

    finally:
        # Cleanup Resources
        camera.stop()
        cap.release()
        if rtsp_process:
            rtsp_process.stdin.close()
            rtsp_process.wait()
        cv2.destroyAllWindows()
        logger.info("Testing finished cleanly.")


if __name__ == "__main__":
    main()