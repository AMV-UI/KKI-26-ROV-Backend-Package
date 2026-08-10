import threading
import logging
import time

# Import the Windows-specific XInput library
import XInput

from rov26backend.models.button import ModeButton
from rov26backend.models.simul_press_button import SimulPressButton

logger = logging.getLogger("ROV.joystick")


class JoystickController:
    def __init__(self):
        self.controller_index = 0
        self._check_connection()

        # Target and Current States
        self.current_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]

        # Button States
        self.btn_states = {
            "BTN_SOUTH": 0,  # A Button
            "BTN_EAST": 0,  # B Button
            "BTN_WEST": 0,  # X Button
            "BTN_NORTH": 0,  # Y Button
        }
        self.lb_btn = False
        self.rb_btn = False
        self.servo_btn = 0

        # Button Logic Handlers
        self.arm_btn = SimulPressButton()
        self.btn_ctrl = {
            "BTN_EAST": ModeButton("ALT_HOLD"),
            "BTN_WEST": ModeButton("STABILIZE"),
            "BTN_SOUTH": ModeButton("MANUAL"),
            "BTN_NORTH": ModeButton("HOLD"),
        }

        # PWM configurations
        self.smoothing_factor = 0.2
        self.pwm_center = 1500
        self.pwm_range = 400
        self.pwm_min = 1300
        self.pwm_max = 1700

        self.MAX_SLEW_PER_SEC = 400
        self.last_servo_time = time.time()
        self.servo_pwm = 2500

        self.lock = threading.Lock()

    def _check_connection(self):
        """Queries which controllers are connected."""
        # Returns a tuple of 4 booleans for the 4 possible controller slots
        connected_controllers = XInput.get_connected()

        if connected_controllers[0]:
            self.is_connected = True
            logger.info("XInput Gamepad connected on slot 1.")
        else:
            self.is_connected = False
            logger.warning("No gamepad detected! Waiting for connection...")

    def monitor(self):
        """
        Runs continuously in the joystick_loop thread.
        Queries the current state of the XInput controller frame-by-frame.
        """
        try:
            if not self.is_connected:
                self._check_connection()
                time.sleep(1.0)
                return

            # Grab the current hardware state of the controller
            state = XInput.get_state(self.controller_index)

            # If state returns None, the controller was disconnected
            if state is None:
                self.is_connected = False
                return

            self._calculate_targets(state)

            # ~50Hz polling rate to match your Pixhawk control loop
            time.sleep(0.02)

        except Exception as e:
            logger.warning(f"Error reading gamepad: {e}")
            self.is_connected = False
            time.sleep(1.0)

    def _apply_deadzone(self, val, deadzone=0.15):
        """Zeroes out tiny stick drifts."""
        return val if abs(val) > deadzone else 0.0

    def _calculate_targets(self, state):
        """Reads raw states from XInput and translates them to PWM values."""

        # XInput-Python automatically normalizes sticks to (-1.0 to 1.0)
        # Format: ((LeftX, LeftY), (RightX, RightY))
        sticks = XInput.get_thumb_values(state)

        # XInput-Python automatically normalizes triggers to (0.0 to 1.0)
        # Format: (LeftTrigger, RightTrigger)
        triggers = XInput.get_trigger_values(state)

        # Returns a dictionary of booleans for all buttons
        buttons = XInput.get_button_values(state)

        # Map axes
        raw_lateral = self._apply_deadzone(sticks[0][0])  # Left Stick X
        raw_forward = self._apply_deadzone(
            sticks[0][1]
        )  # Left Stick Y (Up is positive in XInput)
        raw_yaw = self._apply_deadzone(sticks[1][0])  # Right Stick X

        raw_down_trigger = triggers[0]  # Left Trigger
        raw_up_trigger = triggers[1]  # Right Trigger

        raw_vertical = self._apply_deadzone(raw_up_trigger - raw_down_trigger)

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

            # Map standard buttons
            self.btn_states["BTN_SOUTH"] = 1 if buttons.get("A") else 0
            self.btn_states["BTN_EAST"] = 1 if buttons.get("B") else 0
            self.btn_states["BTN_WEST"] = 1 if buttons.get("X") else 0
            self.btn_states["BTN_NORTH"] = 1 if buttons.get("Y") else 0

            self.lb_btn = buttons.get("LEFT_SHOULDER", False)
            self.rb_btn = buttons.get("RIGHT_SHOULDER", False)

            # Map D-PAD for servo control
            if buttons.get("DPAD_UP"):
                self.servo_btn = 1
            elif buttons.get("DPAD_DOWN"):
                self.servo_btn = -1
            else:
                self.servo_btn = 0

    def update_smoothed_controls(self, control_state):
        with self.lock:
            target_snapshot = self.target_manual_control[:]
            btn_snapshot = self.btn_states.copy()
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

        # min 1700
        # max 2500
        self.servo_pwm = max(1700, min(2500, self.servo_pwm + delta))

        logger.debug(f"SERVO: {self.servo_pwm}")
        control_state.update(servo=int(self.servo_pwm))

        return control_state
