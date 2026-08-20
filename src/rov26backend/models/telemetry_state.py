import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any
import copy
import logging

logger = logging.getLogger("ROV.gRPC")


@dataclass
class TelemetryState:
    mode: str = "MANUAL"
    battery: float = 0.0
    timestamp = datetime.fromtimestamp(0, tz=timezone.utc)

    # Attitude & Depth
    depth: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yawspeed: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0

    # RC Channels (1500 is standard MAVLink neutral)
    forward_rc: int = 1500
    lateral_rc: int = 1500
    vertical_rc: int = 1500
    yaw_rc: int = 1500

    # Motor Efforts
    mot1_eff: int = 0
    mot2_eff: int = 0
    mot3_eff: int = 0
    mot4_eff: int = 0
    mot5_eff: int = 0
    mot6_eff: int = 0

    # Flight Controller Status
    fc_cpu_load: bool = False
    fc_gyro_health: bool = False
    fc_acc_health: bool = False
    fc_compass_health: bool = False
    fc_baro_health: bool = False
    armed: bool = False
    servo_effort: int = 2500

    def __post_init__(self):
        # __post_init__ runs after the dataclass sets up the fields.
        # By not type-hinting _lock, it is excluded from asdict() and dataclass fields.
        self._lock = threading.Lock()

    def update(self, **kwargs: Any) -> None:
        """Maintains dynamic partial updates."""
        with self._lock:
            for key, value in kwargs.items():
                # Prevent accidental creation of new fields or overriding the lock
                if hasattr(self, key) and key != "_lock":
                    setattr(self, key, value)

    def get_latest(self) -> "TelemetryState":
        """Returns a thread-safe snapshot of the current state as an object."""
        with self._lock:
            # We copy so the receiver doesn't accidentally modify the live state
            # (We drop the lock during the copy so the snapshot doesn't have a locked lock)
            snapshot = copy.copy(self)
            snapshot._lock = threading.Lock()  # Reset lock on the copy
            return snapshot

    # --- Optional: Context Manager for better LSP autocomplete ---
    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logger.debug(f"""
                     Mot 1 Effort: {self.mot1_eff}
                     Mot 2 Effort: {self.mot2_eff}
                     Mot 3 Effort: {self.mot3_eff}
                     Mot 4 Effort: {self.mot4_eff}
                     Mot 5 Effort: {self.mot5_eff}
                     Mot 6 Effort: {self.mot6_eff}
                     """)

        self._lock.release()
