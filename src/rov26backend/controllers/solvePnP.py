import logging
import cv2
import numpy as np
import time

logger = logging.getLogger("ROV.vision")


class solvePnP:
    def __init__(self, vision_state):
        self.vision_state = vision_state

        # Camera Matrix (Kalibrasi 1080p)
        self.camera_matrix = np.array(
            [
                [1261.616988791761, 0.0, 261.6271004104672],
                [0.0, 1251.9246045054717, 266.21319058087386],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        # Koordinat 3D Fisik QR Code (Ukuran 4cm x 4cm)
        self.object_points = np.array(
            [[-2, 2, 0], [2, 2, 0], [2, -2, 0], [-2, -2, 0]], dtype=np.float32
        )

        # Distortion Coefficients
        self.dist_coeffs = np.array(
            [
                3.95003433e-01,
                8.73422864e00,
                1.57939148e-02,
                -5.80052436e-02,
                -1.41160080e02,
            ],
            dtype=np.float32,
        )

    def orderPoints(self, polygon_pts):
        pts = np.array(polygon_pts, dtype=np.float32)

        # Sorts 4 points into: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect

    def process(self, polygon_pts):
        if not polygon_pts or len(polygon_pts) != 4:
            return False, None, None

        rect = self.orderPoints(polygon_pts)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            rect,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if success:
            # Hitung Euler Angles (Roll, Pitch, Yaw) dari rvec
            rmat, _ = cv2.Rodrigues(rvec)
            sy = np.sqrt(rmat[0, 0] * rmat[0, 0] + rmat[1, 0] * rmat[1, 0])

            is_singular = sy < 1e-6
            if not is_singular:
                roll = np.arctan2(rmat[2, 1], rmat[2, 2])
                pitch = np.arctan2(-rmat[2, 0], sy)
                yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
            else:
                roll = np.arctan2(-rmat[1, 2], rmat[1, 1])
                pitch = np.arctan2(-rmat[2, 0], sy)
                yaw = 0.0

            euler = {
                "roll": float(np.degrees(roll)),
                "pitch": float(np.degrees(pitch)),
                "yaw": float(np.degrees(yaw)),
            }

            # Update ke VisionState
            with self.vision_state as vision_state:
                vision_state.tvec = tvec.flatten().tolist()
                vision_state.rvec = rvec.flatten().tolist()
                vision_state.euler_angles = euler

            logger.debug(f"""
                         tvec: {tvec.flatten().tolist()}
                         rvec: {rvec.flatten().tolist()}
                         euler: {euler}
                         """)
            logger.debug(self.vision_state.get_latest().tvec)
            return True, rvec, tvec

        time.sleep(0.01)

        return False, None, None
