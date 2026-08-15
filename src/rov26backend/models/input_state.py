import threading
from dataclasses import dataclass
from typing import Any
import copy


@dataclass
class InputState:
    l_analog_x: float = 0.0
    l_analog_y: float = 0.0
    r_analog_x: float = 0.0
    r_analog_y: float = 0.0
    rt_analog: float = 0.0
    lt_analog: float = 0.0
    rb: bool = False
    lb: bool = False
    btn_up: bool = False
    btn_right: bool = False
    btn_left: bool = False
    btn_down: bool = False
    dpad_vert: int = 0

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

    def get_latest(self) -> "InputState":
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
