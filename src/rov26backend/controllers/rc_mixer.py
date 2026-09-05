import logging
import threading
import time

from rov26backend.models.button import PressButton
from rov26backend.models.control_state import ControlState
from rov26backend.models.input_state import InputState
from rov26backend.models.simul_press_button import SimulPressButton

logger = logging.getLogger("ROV.mixer")


class ROV26RcMixer:
    def __init__(
        self,
        input_state: InputState,
        control_state: ControlState,
        auto_event: threading.Event,
        **kwargs,
    ):
        self.smoothing_factor = kwargs.get("smoothing_factor") or 0.025
        self.servo_open = kwargs.get("servo_open") or 1880
        self.servo_close = kwargs.get("servo_close") or 2390
        self.servo_target = self.servo_open

        self.auto_event = auto_event

        self.last_servo_time = time.time()

        self.MAX_SLEW_PER_SEC = 400

        self.servo_pwm = self.servo_open

        self.current_forward = 1500
        self.current_lateral = 1500
        self.current_vertical = 1500
        self.current_yaw = 1500

        self.target_forward = 1500
        self.target_lateral = 1500
        self.target_vertical = 1500
        self.target_yaw = 1500

        self.pwm_center = kwargs.get("pwm_center") or 1500
        self.pwm_range = kwargs.get("pwm_range") or 400
        self.pwm_min = kwargs.get("pwm_min") or 1100
        self.pwm_max = kwargs.get("pwm_max") or 1900

        self.target_mode = None
        self.arm_toggle = False

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
            time.sleep(0.02)

    def _update_poll_auto_stop(self, inputs):
        if self.autonomous_btn.toggle(inputs.btn_up):
            self.auto_event.clear()

    def update_control_from_inputs(self):
        inputs = self.input_state.get_latest()
        if not self.auto_event.is_set():
            self._update_motor_inputs(inputs)
            self._update_mode_inputs(inputs)
            self._update_servo_inputs(inputs)
            # self._update_servo_inputs_analog(inputs)

        if self.auto_event.is_set():
            self._update_poll_auto_stop(inputs)

        # logger.debug(f"""
        #               Sending Control:
        #               forward: {self.current_forward}
        #               lateral: {self.current_lateral}
        #               vertical: {self.current_vertical}
        #               yaw: {self.current_yaw}
        #               servo: {self.servo_pwm}
        #               """)

        is_fwd_stronger = abs(self.current_forward - 1500) > abs(
            self.current_lateral - 1500
        )

        if not self.auto_event.is_set():
            with self.control_state as control:
                control.forward = int(self.current_forward if is_fwd_stronger else 1500)
                control.lateral = int(
                    self.current_lateral if not is_fwd_stronger else 1500
                )
                control.vertical = int(self.current_vertical)
                control.yaw = int(self.current_yaw)

    def _update_servo_inputs(self, inputs: InputState):
        if inputs.dpad_vert == 1:
            self.servo_target = self.servo_open
        elif inputs.dpad_vert == -1:
            self.servo_target = self.servo_close
        # self.servo_pwm += self.smoothing_factor * (self.servo_target - self.servo_pwm)
        self.servo_pwm = self.servo_target

        with self.control_state as control:
            control.servo = int(self.servo_pwm)

    def _update_servo_inputs_analog(self, inputs: InputState):
        # if inputs.dpad_vert == -1:
        #     self.servo_target += 1
        # elif inputs.dpad_vert == 1:
        #     self.servo_target -= 1
        # self.servo_pwm += self.smoothing_factor * (self.servo_target - self.servo_pwm)

        if inputs.dpad_vert == -1:
            self.servo_pwm += 1
        elif inputs.dpad_vert == 1:
            self.servo_pwm -= 1

        with self.control_state as control:
            control.servo = int(self.servo_pwm)

    def _update_mode_inputs(self, inputs: InputState):
        with self.control_state as control:
            if self.manual_btn.toggle(inputs.btn_down):
                control.target_mode = "MANUAL"
            elif self.stabilize_btn.toggle(inputs.btn_right):
                control.target_mode = "STABILIZE"
            elif self.depth_hold_btn.toggle(inputs.btn_left):
                control.target_mode = "ALT_HOLD"
            elif self.autonomous_btn.toggle(inputs.btn_up):
                control.target_mode = "AUTO"

            if self.arm_btn.toggle(inputs.lb, inputs.rb):
                control.arm_toggle = True

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
