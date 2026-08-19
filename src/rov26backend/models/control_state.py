import threading
from dataclasses import dataclass
from typing import Any
import copy


@dataclass
class ControlState:
    forward: int = 1500
    lateral: int = 1500
    vertical: int = 1500
    yaw: int = 1500
    servo: int = 1700
    target_mode: str = None
    arm_toggle: bool = False

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

    def get_latest(self) -> "ControlState":
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
        self._lock.release()
