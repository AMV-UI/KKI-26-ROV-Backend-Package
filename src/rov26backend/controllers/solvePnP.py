import logging
import cv2
import numpy as np

logger = logging.getLogger("ROV.solvePnP")


class solvePnP:

    def __init__(self, vision_state):
        self.vision_state = vision_state

        # Camera Matrix (Kalibrasi 1080p)
        self.camera_matrix = np.array(
            [
                [1164.0743995447119, 0.0, 980.03650102852032],
                [0.0, 1158.2462609239342, 562.2397226536624],
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
                -0.081632802945328112,
                0.029020913802887096,
                -0.0015049443976139964,
                -0.00088117174435944933,
                -0.053633582926546026,
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
            self.vision_state.update(
                tvec=tvec.flatten().tolist(),  # [tx, ty, tz]
                rvec=rvec.flatten().tolist(),  # [rx, ry, rz]
                euler_angles=euler,
            )
            return True, rvec, tvec

        return False, None, None