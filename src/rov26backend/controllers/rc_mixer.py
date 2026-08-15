from rov26backend.models.button import PressButton
from rov26backend.models.simul_press_button import SimulPressButton
from rov26backend.models.input_state import InputState
from rov26backend.models.control_state import ControlState
import time
import threading
import logging

logger = logging.getLogger("ROV.mixer")


class ROV26RcMixer:
    def __init__(
        self,
        input_state: InputState,
        control_state: ControlState,
        smoothing_factor=0.025,
        pwm_center=1500,
        pwm_range=400,
        pwm_min=1300,
        pwm_max=1700,
        MAX_SLEW_PER_SEC=400,
    ):
        self.smoothing_factor = smoothing_factor
        self.MAX_SLEW_PER_SEC = MAX_SLEW_PER_SEC

        self.current_forward = 1500
        self.current_lateral = 1500
        self.current_vertical = 1500
        self.current_yaw = 1500
        self.servo_pwm = 1700

        self.target_forward = 1500
        self.target_lateral = 1500
        self.target_vertical = 1500
        self.target_yaw = 1500

        self.pwm_center = pwm_center
        self.pwm_range = pwm_range
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max

        self.target_mode = None
        self.arm_toggle = False
        self.last_servo_time = time.time()

        self.depth_hold_btn = PressButton()
        self.manual_btn = PressButton()
        self.stabilize_btn = PressButton()
        self.autonomous_btn = PressButton()
        self.arm_btn = SimulPressButton()

        self.input_state = input_state
        self.control_state = control_state

        self._thread = None
        self._is_running = threading.Event()

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.stream_rc, daemon=True)
            self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def stream_rc(self):
        while self._is_running.is_set():
            self.update_control_from_inputs()
            time.sleep(0.01)

    def update_control_from_inputs(self):
        inputs = self.input_state.get_latest()
        self._update_motor_inputs(inputs)
        self._update_mode_inputs(inputs)
        self._update_servo_inputs(inputs)

        logger.debug(f"""
                      Sending Control:
                      forward: {self.current_forward}
                      lateral: {self.current_lateral}
                      vertical: {self.current_vertical}
                      yaw: {self.current_yaw}
                      servo: {self.servo_pwm}
                      target_mode: {self.target_mode}
                      """)

        with self.control_state as control:
            control.forward = int(self.current_forward)
            control.lateral = int(self.current_lateral)
            control.vertical = int(self.current_vertical)
            control.yaw = int(self.current_yaw)
            control.servo = int(self.servo_pwm)
            control.target_mode = self.target_mode

    def _update_servo_inputs(self, inputs: InputState):
        now = time.time()
        dt = now - self.last_servo_time
        self.last_servo_time = now

        delta = inputs.dpad_vert * self.MAX_SLEW_PER_SEC * dt

        self.servo_pwm = max(1700, min(2500, self.servo_pwm + delta))

    def _update_mode_inputs(self, inputs: InputState):
        if self.manual_btn.toggle(inputs.btn_down):
            self.target_mode = "MANUAL"
        elif self.stabilize_btn.toggle(inputs.btn_right):
            self.target_mode = "STABILIZE"
        elif self.stabilize_btn.toggle(inputs.btn_left):
            self.target_mode = "ALT_HOLD"
        elif self.autonomous_btn.toggle(inputs.btn_up):
            self.target_mode = None

        if self.arm_btn.toggle(inputs.lb, inputs.rb):
            self.arm_toggle = True

    def _update_motor_inputs(self, inputs: InputState):
        raw_lateral = inputs.l_analog_x
        raw_forward = inputs.l_analog_y
        raw_yaw = inputs.r_analog_x
        raw_up = inputs.rt_analog
        raw_down = inputs.lt_analog
        raw_vertical = raw_up - raw_down

        target_forward = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_forward * self.pwm_range))),
        )
        target_lateral = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_lateral * self.pwm_range))),
        )
        target_vertical = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_vertical * self.pwm_range))),
        )
        target_yaw = max(
            self.pwm_min,
            min(self.pwm_max, int(self.pwm_center + (raw_yaw * self.pwm_range))),
        )

        self.current_forward += self.smoothing_factor * (
            target_forward - self.current_forward
        )
        self.current_lateral += self.smoothing_factor * (
            target_lateral - self.current_lateral
        )
        self.current_vertical += self.smoothing_factor * (
            target_vertical - self.current_vertical
        )
        self.current_yaw += self.smoothing_factor * (target_yaw - self.current_yaw)
