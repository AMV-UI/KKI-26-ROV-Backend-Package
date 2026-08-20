import cv2
import numpy as np


class QRDebouncer:
    def __init__(
        self, required_streak=3, max_missing_frames=2, max_jump_pixels=50, alpha=0.3
    ):
        self.required_streak = required_streak
        self.max_missing_frames = max_missing_frames
        self.max_jump_pixels = max_jump_pixels
        self.alpha = alpha  # NEW: EMA smoothing factor

        self.current_streak = 0
        self.missing_frames = 0
        self.is_confirmed = False
        self.last_known_polygon = None

    def update(self, detected_polygon):
        if detected_polygon is not None and self.last_known_polygon is not None:
            old_center = np.mean(self.last_known_polygon, axis=0)
            new_center = np.mean(detected_polygon, axis=0)
            jump_distance = np.linalg.norm(new_center - old_center)

            if jump_distance > self.max_jump_pixels:
                detected_polygon = None
            else:
                # --- NEW: EMA SMOOTHING ---
                # Interpolate between the old polygon and the new one
                detected_polygon = (self.alpha * detected_polygon) + (
                    (1 - self.alpha) * self.last_known_polygon
                )
                detected_polygon = np.int32(detected_polygon)

        # --- STANDARD DEBOUNCING LOGIC ---
        if detected_polygon is not None:
            self.current_streak += 1
            self.missing_frames = 0
            self.last_known_polygon = detected_polygon

            if self.current_streak >= self.required_streak:
                self.is_confirmed = True
        else:
            self.missing_frames += 1

            if self.missing_frames > self.max_missing_frames:
                self.current_streak = 0
                self.is_confirmed = False
                self.last_known_polygon = None

        if self.is_confirmed:
            return self.last_known_polygon
        else:
            return None


class QRPolygonFinder:
    def __init__(
        self,
    ):
        self.debouncer = QRDebouncer()

    def preprocess_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )

        return thresh

    def get_polygon_from_frame(self, frame):
        preprocessed_frame = self.preprocess_frame(frame)

        contours, hierarchy = cv2.findContours(
            preprocessed_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        raw_box = None

        if hierarchy is not None:
            hierarchy = hierarchy[0]
            raw_patterns = []

            for i, contour in enumerate(contours):
                child_idx = hierarchy[i][2]
                if child_idx != -1:
                    grandchild_idx = hierarchy[child_idx][2]
                    if grandchild_idx != -1:
                        area = cv2.contourArea(contour)
                        if area > 100:
                            peri = cv2.arcLength(contour, True)
                            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                            # 1. RELAXED SKEW CHECK:
                            # We trust the hierarchy (parent->child->grandchild) more than geometry now.
                            # We just ensure it has 4 sides and a much looser aspect ratio to allow for 45-degree trapezoids.
                            if len(approx) == 4:
                                x, y, w, h = cv2.boundingRect(approx)
                                aspect_ratio = float(w) / h
                                if 0.3 <= aspect_ratio <= 3.0:
                                    raw_patterns.append(contour)

            distinct_patterns = []
            known_centers = []

            for contour in raw_patterns:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # We use half the width of the contour as a dynamic distance threshold
                    _, _, w, _ = cv2.boundingRect(contour)
                    min_dist = w / 2

                    # Check if this center is too close to an already found corner
                    is_duplicate = False
                    for prev_cX, prev_cY in known_centers:
                        # Calculate Euclidean distance between centers
                        dist = np.sqrt((cX - prev_cX) ** 2 + (cY - prev_cY) ** 2)
                        if dist < min_dist:
                            is_duplicate = True
                            break

                    if not is_duplicate:
                        known_centers.append((cX, cY))
                        distinct_patterns.append(contour)

            # Use distinct_patterns instead of the raw list
            if len(known_centers) >= 3:
                # Take the first 3 centers and convert to numpy array
                pts = np.array(known_centers[:3], dtype=np.float32)

                # --- NEW: COLLINEARITY / AREA CHECK ---
                x1, y1 = pts[0]
                x2, y2 = pts[1]
                x3, y3 = pts[2]

                # Calculate the area of the triangle formed by the 3 points
                triangle_area = 0.5 * abs(
                    x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
                )

                # If the area is tiny, the points are in a straight line. Reject them.
                # A 1000 pixel area is a very safe minimum for a readable QR code triangle.
                if triangle_area > 500:
                    # Find distances between all three points
                    dist_01 = np.linalg.norm(pts[0] - pts[1])
                    dist_02 = np.linalg.norm(pts[0] - pts[2])
                    dist_12 = np.linalg.norm(pts[1] - pts[2])

                    # The longest distance is the hypotenuse.
                    if dist_12 > dist_01 and dist_12 > dist_02:
                        tl, p1, p2 = pts[0], pts[1], pts[2]
                    elif dist_02 > dist_01 and dist_02 > dist_12:
                        tl, p1, p2 = pts[1], pts[0], pts[2]
                    else:
                        tl, p1, p2 = pts[2], pts[0], pts[1]

                    # Estimate the missing 4th center (Bottom-Right) using vector math
                    br = p1 + p2 - tl

                    # Combine into an array of 4 points
                    four_points = np.array([tl, p1, br, p2], dtype=np.int32)

                    # Sort the points clockwise based on their angles from the centroid
                    center = np.mean(four_points, axis=0)
                    angles = np.arctan2(
                        four_points[:, 1] - center[1], four_points[:, 0] - center[0]
                    )
                    sorted_points = four_points[np.argsort(angles)]

                    raw_box = sorted_points

        return self.debouncer.update(raw_box)

        # if confirmed_box is not None:
        #     cv2.polylines(frame, [confirmed_box], True, (255, 0, 0), 3)
        #     cv2.putText(
        #         frame,
        #         "QR Polygon",
        #         (confirmed_box[0][0], confirmed_box[0][1] - 10),
        #         cv2.FONT_HERSHEY_SIMPLEX,
        #         0.5,
        #         (255, 0, 0),
        #         2,
        #     )
        #     return frame
