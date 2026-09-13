import logging
import threading
import time

import pandas as pd

logger = logging.getLogger("ROV.mixer")


# 1. Create a thread-safe shared state object
class SharedKeyboardState:
    def __init__(self):
        self.active_key = None
        self.lock = threading.Lock()
        self.running = True

    def set_key(self, key):
        with self.lock:
            try:
                self.active_key = key.char  # For standard alphanumeric keys
            except AttributeError:
                self.active_key = key.name  # For special keys (esc, space, etc.)

    def clear_key_on_release(self, key):
        with self.lock:
            try:
                released_key = key.char
            except AttributeError:
                released_key = key.name

            # Only set to None if the released key is the one currently stored
            if self.active_key == released_key:
                self.active_key = None

    def get_key(self):
        with self.lock:
            return self.active_key

    def clear_key(self):
        with self.lock:
            self.active_key = None


class ControlRecorder:
    def __init__(self):
        self.control_columns = [
            "forward",
            "lateral",
            "vertical",
            "yaw",
            "servo",
            "target_mode",
        ]

        self.recorded_data = []
        self.playback_df = None
        self.playback_index = 0

        self.delay = 0.1
        self.last_run_time = 0.0

        self.is_recording = False
        self.is_playing = False

    def toggle_record(self):
        if not self.is_recording:
            logger.info("Recording...")
            self.is_recording = True
            self.recorded_data = []
        else:
            self.is_recording = False
            if self.recorded_data:
                df = pd.DataFrame(self.recorded_data, columns=self.control_columns)
                df.to_parquet("control.parquet")
            self.recorded_data = []
            logger.info("Recording finished")

    def toggle_playback(self):
        if not self.is_playing:
            logger.info("Starting playback...")
            try:
                self.playback_df = pd.read_parquet("control.parquet")
                self.playback_index = 0
                self.is_playing = True
            except FileNotFoundError:
                print("No control.parquet file found.")
        else:
            logger.info("Playback finished")
            self.is_playing = False
            self.playback_df = None

    def process_data(self, control_state):
        current_time = time.monotonic()
        if current_time - self.last_run_time < self.delay:
            return

        self.last_run_time = current_time

        with control_state as control:
            if self.is_recording:
                self.recorded_data.append(
                    [
                        control.forward,
                        control.lateral,
                        control.vertical,
                        control.yaw,
                        control.servo,
                        control.target_mode,
                    ]
                )

            elif self.is_playing and self.playback_df is not None:
                if self.playback_index < len(self.playback_df):
                    row = self.playback_df.iloc[self.playback_index]

                    control.forward = row["forward"]
                    control.lateral = row["lateral"]
                    control.vertical = row["vertical"]
                    control.yaw = row["yaw"]
                    control.servo = row["servo"]
                    control.target_mode = row["target_mode"]

                    self.playback_index += 1
                else:
                    self.toggle_playback()
