import threading


class VisionState:
    def __init__(self):
        self.lock = threading.Lock()

        self.qr_side = "NOT_FOUND"
        self.qr_polygon = []

    def update(self, **kwargs):
        """
        Safely updates only the provided fields.
        Example: state.update(roll=12.5, pitch=5.2)
        """
        with self.lock:
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, value)

    def get_latest(self):
        """
        Safely grabs a snapshot of all current values for the gRPC stream.
        """
        with self.lock:
            return {
                "qr_side": self.qr_side,
                "qr_polygon": self.qr_polygon
            }
