import threading
import logging
import time
import XInput

from rov26backend.models.input_state import InputState

logger = logging.getLogger("ROV.joystick")


class PxnP5JoystickWindows:
    def __init__(self, input_state: InputState):
        self.input_state = input_state
        self._thread = None
        self._is_running = threading.Event()
        self.controller_index = 0
        self.is_connected = False

    def _check_connection(self):
        """Queries which controllers are connected."""
        connected_controllers = XInput.get_connected()

        if connected_controllers[0]:
            self.is_connected = True
            logger.info("XInput Gamepad connected on slot 1.")
        else:
            self.is_connected = False
            logger.warning("No gamepad detected! Waiting for connection...")

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
        self._check_connection()

        while self._is_running.is_set():
            try:
                if not self.is_connected:
                    self._check_connection()
                    time.sleep(1.0)
                    continue

                state = XInput.get_state(self.controller_index)

                if state is None:
                    self.is_connected = False
                    continue

                sticks = XInput.get_thumb_values(state)

                triggers = XInput.get_trigger_values(state)

                buttons = XInput.get_button_values(state)

                with self.input_state as input_state:
                    input_state.l_analog_x = sticks[0][0]
                    input_state.l_analog_y = sticks[0][1]
                    input_state.r_analog_x = sticks[1][0]

                    input_state.lt_analog = triggers[0]
                    input_state.rt_analog = triggers[1]

                    input_state.rb = buttons.get("RIGHT_SHOULDER", False)
                    input_state.lb = buttons.get("LEFT_SHOULDER", False)

                    if buttons.get("DPAD_UP"):
                        input_state.dpad_vert = 1
                    elif buttons.get("DPAD_DOWN"):
                        input_state.dpad_vert = -1
                    else:
                        input_state.dpad_vert = 0

                    input_state.btn_down = buttons.get("A", False)
                    input_state.btn_right = buttons.get("B", False)
                    input_state.btn_left = buttons.get("X", False)
                    input_state.btn_up = buttons.get("Y", False)

            except Exception as e:
                logger.warning(f"Error reading gamepad: {e}")
                self.is_connected = False
                time.sleep(1.0)

            time.sleep(0.02)
