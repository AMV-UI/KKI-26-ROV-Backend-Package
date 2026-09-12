import numpy as np


class QRDebouncer:
    def __init__(
        self, required_streak=3, max_missing_frames=4, max_jump_pixels=75, alpha=0.3
    ):
        self.required_streak = required_streak
        self.max_missing_frames = max_missing_frames
        self.max_jump_pixels = max_jump_pixels
        self.alpha = alpha

        self.current_streak = 0
        self.missing_frames = 0
        self.is_confirmed = False

        self.tracking_polygon = None
        self.persistent_polygon = None

    def update(self, detected_polygon, frame_shape):
        if detected_polygon is not None and self.tracking_polygon is not None:
            if detected_polygon.shape != self.tracking_polygon.shape:
                detected_polygon = None
            else:
                old_center = np.mean(self.tracking_polygon, axis=0)
                new_center = np.mean(detected_polygon, axis=0)
                jump_distance = np.linalg.norm(new_center - old_center)

                if jump_distance > self.max_jump_pixels:
                    detected_polygon = None
                else:
                    detected_polygon = (self.alpha * detected_polygon) + (
                        (1 - self.alpha) * self.tracking_polygon
                    )
                    detected_polygon = np.int32(detected_polygon)

        if detected_polygon is not None:
            self.current_streak += 1
            self.missing_frames = 0
            self.tracking_polygon = detected_polygon

            if self.current_streak >= self.required_streak:
                self.is_confirmed = True
                self.persistent_polygon = detected_polygon
        else:
            self.missing_frames += 1

            if self.missing_frames > self.max_missing_frames:
                self.current_streak = 0
                self.is_confirmed = False
                self.tracking_polygon = None
                self.persistent_polygon = None

        if self.persistent_polygon is not None:
            clamped_polygon = np.copy(self.persistent_polygon)
            max_y, max_x = frame_shape[0] - 1, frame_shape[1] - 1

            clamped_polygon[:, 0] = np.clip(clamped_polygon[:, 0], 0, max_x)
            clamped_polygon[:, 1] = np.clip(clamped_polygon[:, 1], 0, max_y)

            return clamped_polygon

        return None
