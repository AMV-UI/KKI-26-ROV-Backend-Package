from inputs import get_gamepad
from rov26backend.models.button import ModeButton
from rov26backend.models.simul_press_button import SimulPressButton
import threading
import logging
import time
import inputs

# --- FIX FOR LINUX 'inputs' LIBRARY BUG ---
# This overwrites the broken LED scanning function with a dummy function that does nothing.
inputs.DeviceManager._find_leds = lambda self: None
# ------------------------------------------

logger = logging.getLogger("ROV.joystick")


class JoystickController:
    def __init__(self):
        self._detect_gamepad_mode()
        self.axes = {
            "ABS_X": 128,
            "ABS_Y": 128,
            "ABS_Z": 128,
            "ABS_RZ": 128,
            "ABS_BRAKE": 128,
            "ABS_GAS": 128,
        }

        self.btn = {"BTN_NORTH": 0, "BTN_WEST": 0, "BTN_EAST": 0, "BTN_SOUTH": 0}

        self.lb_btn = False
        self.rb_btn = False

        self.arm_btn = SimulPressButton()

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

        self.MAX_SLEW_PER_SEC = 400  # unit PWM per detik, sesuaikan
        self.last_servo_time = time.time()
        self.servo_pwm = 2500

        self.arm1 = ModeButton("")
        self.arm2 = ModeButton("")

        self.current_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]

        self.lock = threading.Lock()

        self.servo_btn = 0

    def _detect_gamepad_mode(self):
        """Checks if the gamepad is XInput (Xbox) or DirectInput and sets defaults."""
        self.is_xinput = False

        if inputs.devices.gamepads:
            pad = inputs.devices.gamepads[0]
            if any([x in pad.name.lower() for x in ["xbox", "x-box"]]):
                self.is_xinput = True
                logger.info(f"Gamepad detected as XInput: {pad.name}")
            else:
                logger.info(f"Gamepad detected as DirectInput: {pad.name}")

        # Set the neutral resting state based on the mode
        if self.is_xinput:
            self.axes = {
                "ABS_X": 0,
                "ABS_Y": 0,
                "ABS_RX": 0,
                "ABS_RY": 0,
                "ABS_Z": 0,
                "ABS_RZ": 0,
            }
        else:
            self.axes = {
                "ABS_X": 128,
                "ABS_Y": 128,
                "ABS_Z": 128,
                "ABS_RZ": 128,
                "ABS_BRAKE": 0,
                "ABS_GAS": 0,
            }

    def _normalize_stick(self, val):
        """Converts stick value to -1.0 to 1.0"""
        if self.is_xinput:
            return val / 32767.0
        return (val - 128) / 128.0

    def _normalize_trigger(self, val):
        """Converts trigger value to 0.0 to 1.0"""
        return val / 255.0

    def monitor(self):
        """
        Call this method in a dedicated thread to continuously read joystick events,
        or call it sequentially if you have set get_gamepad() to be non-blocking.
        """
        try:
            events = get_gamepad()
            for event in events:
                if event.code != "SYN_REPORT":
                    logger.debug(f"Event: {event.code}: {event.state}")
                if event.ev_type == "Absolute":
                    if event.code in self.axes:
                        self.axes[event.code] = event.state

                if event.ev_type == "Key":
                    with self.lock:
                        if event.code in self.btn:
                            self.btn[event.code] = event.state
                        elif event.code == "BTN_TR":
                            self.lb_btn = event.state == 1
                        elif event.code == "BTN_TL":
                            self.rb_btn = event.state == 1
                if event.code == "ABS_HAT0Y":
                    self.servo_btn = event.state

            self._calculate_targets()

        except Exception as e:
            logger.warn(f"Error reading gamepad: {e}")
            time.sleep(1.0)
            inputs.devices = inputs.DeviceManager()
            self._detect_gamepad_mode()

    def _calculate_targets(self):
        """Translates raw axis inputs to target PWM channels (1300-1700)."""

        if self.is_xinput:
            raw_lateral = self._normalize_stick(self.axes["ABS_X"])
            raw_forward = self._normalize_stick(self.axes["ABS_Y"]) * -1
            raw_yaw = self._normalize_stick(self.axes["ABS_RX"])
            raw_up = self._normalize_trigger(self.axes["ABS_RZ"])
            raw_down = self._normalize_trigger(self.axes["ABS_Z"])
        else:
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
        with self.lock:
            target_snapshot = self.target_manual_control[:]
            btn_snapshot = self.btn.copy()
            current_servo_btn = self.servo_btn
            lb = self.lb_btn
            rb = self.rb_btn

        for i in range(4):
            self.current_manual_control[i] += self.smoothing_factor * (
                target_snapshot[i] - self.current_manual_control[i]
            )

            logger.debug(f"""SENDING RC:
                         forward: {int(self.current_manual_control[0])}
                         lateral: {int(self.current_manual_control[1])}
                         vertical:{int(self.current_manual_control[2])}
                         yaw:     {int(self.current_manual_control[3])}""")

        control_state.update(
            forward=int(self.current_manual_control[0]),
            lateral=int(self.current_manual_control[1]),
            vertical=int(self.current_manual_control[2]),
            yaw=int(self.current_manual_control[3]),
        )

        for name, ctrl in self.btn_ctrl.items():
            if ctrl.toggle(btn_snapshot[name]):
                control_state.update(target_mode=ctrl.mode_name)

        if self.arm_btn.toggle(lb, rb):
            control_state.update(arm_toggle=True)

        now = time.time()
        dt = now - self.last_servo_time
        self.last_servo_time = now

        delta = current_servo_btn * self.MAX_SLEW_PER_SEC * dt

        # min 500
        # open 1500
        # close tight on 1700
        self.servo_pwm = max(1700, min(2500, self.servo_pwm + delta))

        logger.debug(f"SERVO: {self.servo_pwm}")
        control_state.update(servo=int(self.servo_pwm))

        return control_state
