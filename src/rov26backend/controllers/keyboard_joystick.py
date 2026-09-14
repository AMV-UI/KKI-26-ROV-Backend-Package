import logging
import threading
import time

from rov26backend.controllers.keyboardshit import SharedKeyboardState
from rov26backend.models.input_state import InputState

logger = logging.getLogger("ROV.keyboard_joystick")


class KeyboardJoystickController:
    def __init__(self, input_state: InputState, keyboard_state: SharedKeyboardState):
        self.input_state = input_state
        self.keyboard_state = keyboard_state
        self._thread = None
        self._is_running = threading.Event()

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.monitor, daemon=True)
            self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def monitor(self):
        while self._is_running.is_set():
            # Retrieves the current active key from the shared lock state[cite: 3]
            key = self.keyboard_state.get_key()

            # Use the context manager to safely update the shared state[cite: 1]
            with self.input_state as state:
                # 1. Reset all states to default to handle key releases[cite: 1, 3]
                state.l_analog_x = 0.0
                state.l_analog_y = 0.0
                state.r_analog_x = 0.0
                state.r_analog_y = 0.0
                state.rt_analog = 0.0
                state.lt_analog = 0.0
                state.rb = False
                state.lb = False
                state.btn_up = False
                state.btn_right = False
                state.btn_left = False
                state.btn_down = False
                state.recorded_depth = False
                state.dpad_vert = 0
                state.dpad_hor = 0

                # 2. Map standard keys (key.char) and special keys (key.name)[cite: 1, 3]
                if key == "w":
                    state.l_analog_y = -1.0
                elif key == "s":
                    state.l_analog_y = 1.0
                elif key == "a":
                    state.l_analog_x = -1.0
                elif key == "d":
                    state.l_analog_x = 1.0
                elif key == "up":
                    state.r_analog_y = -1.0
                elif key == "down":
                    state.r_analog_y = 1.0
                elif key == "left":
                    state.r_analog_x = -1.0
                elif key == "right":
                    state.r_analog_x = 1.0
                elif key == "q":
                    state.lt_analog = 1.0
                elif key == "e":
                    state.rt_analog = 1.0
                elif key == "u":
                    state.lb = True
                elif key == "o":
                    state.rb = True
                elif key == "i":
                    state.btn_up = True
                elif key == "k":
                    state.btn_down = True
                elif key == "j":
                    state.btn_left = True
                elif key == "l":
                    state.btn_right = True
                elif key == "t":
                    state.dpad_vert = 1
                elif key == "g":
                    state.dpad_vert = -1
                elif key == "f":
                    state.dpad_hor = -1
                elif key == "h":
                    state.dpad_hor = 1
                elif key == "enter":
                    state.recorded_depth = True

            # Prevent thread from maxing out CPU
            time.sleep(0.05)
