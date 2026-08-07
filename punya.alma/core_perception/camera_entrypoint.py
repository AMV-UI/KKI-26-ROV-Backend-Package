#!/usr/bin/env python3

import argparse
import logging
import sys
import threading
import traceback
import cv2

# Mengimpor class kamera (versi Python murni tanpa ROS)
from core_perception.bottom_camera_controller import BottomCameraNode
from core_perception.front_camera_controller import FrontCameraNode

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] [%(levelname)s] %(message)s"
)


def main(args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description="Camera nodes entry point")
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Run in dev mode (webcam only, no bottom cam)",
    )

    parsed_args = parser.parse_args(args=args)

    threads = []
    front_cam = None
    bottom_cam = None

    try:
        if parsed_args.dev:
            logging.info("Running in DEV mode (Front Webcam only)...")
            front_cam = FrontCameraNode(webcam=True)
            # Jalankan kamera di thread terpisah
            t_front = threading.Thread(
                target=front_cam.run, name="FrontCamThread"
            )
            threads.append(t_front)
        else:
            logging.info("Running in PRODUCTION mode (Front & Bottom Cam)...")
            front_cam = FrontCameraNode()
            bottom_cam = BottomCameraNode()

            t_front = threading.Thread(
                target=front_cam.run, name="FrontCamThread"
            )
            t_bottom = threading.Thread(
                target=bottom_cam.run, name="BottomCamThread"
            )

            threads.append(t_front)
            threads.append(t_bottom)

        # Mulai semua thread kamera
        for t in threads:
            t.daemon = True  # Otomatis mati jika main program di-kill
            t.start()

        logging.info("Camera threads started. Press Ctrl+C to stop.")

        # Jaga main thread tetap hidup sampai user menekan Ctrl+C
        for t in threads:
            while t.is_alive():
                t.join(timeout=1.0)

    except KeyboardInterrupt:
        logging.info("Shutting down camera nodes (Ctrl+C pressed)...")
    except Exception:
        logging.error(f"Error in main loop: {traceback.format_exc()}")
    finally:
        logging.info("Cleaning up camera resources...")

        # Hentikan loop di dalam masing-masing class kamera
        if front_cam:
            front_cam.is_running = False
            front_cam.cleanup()

        if bottom_cam:
            bottom_cam.is_running = False
            bottom_cam.cleanup()

        # Tunggu hingga thread selesai dibersihkan
        for t in threads:
            if t.is_alive():
                t.join()

        cv2.destroyAllWindows()
        logging.info("Shutdown complete.")


if __name__ == "__main__":
    main()