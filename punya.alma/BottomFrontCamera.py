import logging
import threading
import time
import cv2
from core_perception.bottom_camera_controller import BottomCameraNode
from core_perception.front_camera_controller import FrontCameraNode

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] [%(levelname)s] %(message)s"
)


def main():
    logging.info("Memulai Front Camera dan Bottom Camera secara paralel...")

    # 1. Inisialisasi objek kamera (cukup 1x diinisialisasi)
    front_cam = FrontCameraNode(webcam=True)
    bottom_cam = BottomCameraNode(webcam=True)

    front_cam.show_result = True
    bottom_cam.show_result = True

    # 2. Buat Thread untuk masing-masing kamera (menargetkan method .run)
    t_front = threading.Thread(
        target=front_cam.run, name="FrontCameraThread", daemon=True
    )
    t_bottom = threading.Thread(
        target=bottom_cam.run, name="BottomCameraThread", daemon=True
    )

    # 3. Jalankan kedua thread secara bersamaan (paralel)
    t_front.start()
    t_bottom.start()

    logging.info("Kedua kamera berhasil berjalan. Tekan Ctrl+C untuk keluar.")

    # 4. Jaga main thread agar tetap hidup dan mendengarkan KeyboardInterrupt (Ctrl+C)
    try:
        while t_front.is_alive() or t_bottom.is_alive():
            time.sleep(0.5)

    except KeyboardInterrupt:
        logging.info("Menerima sinyal berhenti (Ctrl+C)...")

    finally:
        # 5. Hentikan perulangan di dalam masing-masing class dan bersihkan resource
        logging.info("Memberhentikan thread kamera...")
        front_cam.is_running = False
        bottom_cam.is_running = False

        # Tunggu thread benar-benar selesai
        t_front.join(timeout=1.0)
        t_bottom.join(timeout=1.0)

        # Bersihkan resource OpenCV & FFmpeg
        front_cam.cleanup()
        bottom_cam.cleanup()

        cv2.destroyAllWindows()
        logging.info("Program selesai, seluruh resource berhasil dibersihkan.")


# Jalankan fungsi main
if __name__ == "__main__":
    main()