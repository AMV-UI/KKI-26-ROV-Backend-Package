from inputs import get_gamepad
from rov26backend.models.button import ModeButton
import threading


class JoystickController:
    def __init__(self):
        self.axes = {
            "ABS_X": 128,
            "ABS_Y": 128,
            "ABS_Z": 128,
            "ABS_RZ": 128,
            "ABS_BRAKE": 0,
            "ABS_GAS": 0,
        }

        self.btn = {"BTN_NORTH": 0, "BTN_WEST": 0, "BTN_EAST": 0, "BTN_SOUTH": 0}

        self.btn_ctrl = {
            "BTN_EAST": ModeButton("ALT_HOLD"),
            "BTN_WEST": ModeButton("STABILIZE"),
            "BTN_SOUTH": ModeButton("MANUAL"),
            "BTN_NORTH": ModeButton("HOLD"),
        }

        self.smoothing_factor = 0.2
        self.pwm_center = 1500
        self.pwm_range = 400
        self.pwm_min = 1300
        self.pwm_max = 1700

        self.arm1 = 0
        self.arm2 = 0

        self.current_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]

        self.lock = threading.Lock()

    def log(self, msg):
        print(f"Joystick: {msg}")

    def log_err(self, msg):
        print(f"[ERR] Joystick: {msg}")

    def _normalize_stick(self, val):
        """Converts 0-255 stick value to -1.0 to 1.0 (128 is center)"""
        return (val - 128) / 128.0

    def _normalize_trigger(self, val):
        """Converts 0-255 trigger value to 0.0 to 1.0"""
        return val / 255.0

    def monitor(self):
        """
        Call this method in a dedicated thread to continuously read joystick events,
        or call it sequentially if you have set get_gamepad() to be non-blocking.
        """
        try:
            events = get_gamepad()
            for event in events:
                print(event.code)
                print(event.state)
                if event.ev_type == "Absolute":
                    if event.code in self.axes:
                        self.axes[event.code] = event.state

                if event.ev_type == "Key":
                    if event.code in self.btn.keys():
                        self.btn[event.code] = event.state

                if event.code == "BTN_TR":
                    self.arm1 = event.state
                elif event.code == "BTN_TL":
                    self.arm2 = event.state

            self._calculate_targets()

        except Exception as e:
            self.log_err(f"Error reading gamepad: {e}")

    def _calculate_targets(self):
        """Translates raw axis inputs to target PWM channels (1300-1700)."""
        raw_lateral = self._normalize_stick(self.axes["ABS_X"])
        raw_forward = self._normalize_stick(self.axes["ABS_Y"]) * -1
        raw_yaw = self._normalize_stick(self.axes["ABS_Z"])

        raw_up = self._normalize_trigger(self.axes["ABS_GAS"])
        raw_down = self._normalize_trigger(self.axes["ABS_BRAKE"])
        raw_vertical = raw_up - raw_down

        forward = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_forward * self.pwm_range))),
        )
        lateral = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_lateral * self.pwm_range))),
        )
        vertical = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_vertical * self.pwm_range))),
        )
        yaw = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_yaw * self.pwm_range))),
        )

        with self.lock:
            self.target_manual_control = [forward, lateral, vertical, yaw]

    def update_smoothed_controls(self, control_state):
        """
        Applies a low-pass filter to the controls.
        Call this iteratively in your main control loop (e.g., every 0.01s).
        """
        for i in range(4):
            self.current_manual_control[i] += self.smoothing_factor * (
                self.target_manual_control[i] - self.current_manual_control[i]
            )

        control_state.update(forward=int(self.current_manual_control[0]))
        control_state.update(lateral=int(self.current_manual_control[1]))
        control_state.update(vertical=int(self.current_manual_control[2]))
        control_state.update(yaw=int(self.current_manual_control[3]))

        for name, ctrl in self.btn_ctrl.items():
            if ctrl.toggle(self.btn[name]):
                control_state.update(target_mode=ctrl.mode_name)

        if self.arm1 == 1 and self.arm2 == 1:
            control_state.update(arm_toggle=True)

        return control_state
