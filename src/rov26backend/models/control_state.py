import threading


class ControlState:
    def __init__(self):

        self.lock = threading.Lock()
        self.forward = 1500
        self.lateral = 1500
        self.vertical = 1500
        self.yaw = 1500
        self.servo = 1500
        self.target_mode = None
        self.arm_toggle = False

    def update(self, **kwargs):
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
                "forward": self.forward,
                "lateral": self.lateral,
                "vertical": self.vertical,
                "yaw": self.yaw,
                "servo": self.servo,
                "target_mode": self.target_mode,
                "arm_toggle": self.arm_toggle,
            }
