from rov26backend.models.input_state import InputState
from inputs import get_gamepad
import logging
import threading
import time
import inputs

# --- FIX FOR LINUX 'inputs' LIBRARY BUG ---
# This overwrites the broken LED scanning function with a dummy function that does nothing.
inputs.DeviceManager._find_leds = lambda self: None
# ------------------------------------------

logger = logging.getLogger("ROV.joystick")


class PxnP5JoystickLinux:
    def __init__(self, input_state: InputState):
        self.input_state = input_state
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

    def _normalize_stick(self, x):
        return (x - 128) / 128

    def _normalize_brakes(self, x):
        return x / 255

    def monitor(self):
        while self._is_running.is_set():
            try:
                events = get_gamepad()
                for event in events:
                    if event.code != "SYN_REPORT":
                        logger.debug(f"Event: {event.code}: {event.state}")
                    with self.input_state as input_state:
                        if event.code == "ABS_X":
                            input_state.l_analog_x = self._normalize_stick(event.state)
                        elif event.code == "ABS_Y":
                            input_state.l_analog_y = (
                                self._normalize_stick(event.state) * -1
                            )
                        elif event.code == "ABS_Z":
                            input_state.r_analog_x = self._normalize_stick(event.state)
                        elif event.code == "ABS_GAS":
                            input_state.rt_analog = self._normalize_brakes(event.state)
                        elif event.code == "ABS_BRAKE":
                            input_state.lt_analog = self._normalize_brakes(event.state)
                        elif event.code == "BTN_TR":
                            input_state.rb = event.state == 1
                        elif event.code == "BTN_TL":
                            input_state.lb = event.state == 1
                        elif event.code == "ABS_HAT0Y":
                            input_state.dpad_vert = event.state
                        elif event.code == "BTN_NORTH":
                            input_state.btn_left = event.state
                        elif event.code == "BTN_SOUTH":
                            input_state.btn_down = event.state
                        elif event.code == "BTN_EAST":
                            input_state.btn_right = event.state
                        elif event.code == "BTN_WEST":
                            input_state.btn_up = event.state

            except Exception as e:
                logger.warn(f"Error reading gamepad: {e}")
                time.sleep(1.0)
                inputs.devices = inputs.DeviceManager()
