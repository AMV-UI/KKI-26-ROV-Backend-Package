import json
import logging
import os
import queue
import threading
import time
from enum import Enum

from rov26backend.controllers.keyboardshit import ControlRecorder, SharedKeyboardState
from rov26backend.models.button import PressButton, PressButtonTarget
from rov26backend.models.control_state import ControlState
from rov26backend.models.input_state import InputState
from rov26backend.models.simul_press_button import SimulPressButton

logger = logging.getLogger("ROV.mixer")

# bullshit pain
MOTOR_COORDS = {
    1: ("right", 100),  # Top Right
    2: ("left", 100),  # Top Left
    5: ("right", 380),  # Mid Right
    6: ("left", 380),  # Mid Left
    3: ("right", 660),  # Bottom Right
    4: ("left", 660),  # Bottom Left
}

# Per-motor parameters
PARAMS_CONFIG = [
    ("MIN", 1000, 1500, 1, int),
    ("MAX", 1500, 2000, 1, int),
    ("THROTTLE", -100.0, 100.0, 0.01, float),
    ("YAW", -1000.0, 200.0, 0.01, float),
    ("FORWARD", -100.0, 100.0, 0.01, float),
    ("LATERAL", -100.0, 100.0, 0.01, float),
    ("ROLL", -100.0, 100.0, 0.01, float),
    ("PITCH", -100.0, 100.0, 0.01, float),
]

# Global PID parameters
GLOBAL_PARAMS_CONFIG = [
    ("ATC_RAT_YAW_P", 0.0, 1.0, 0.01, float),
    ("ATC_RAT_YAW_I", 0.0, 0.1, 0.001, float),
    ("ATC_RAT_YAW_D", 0.0, 0.05, 0.001, float),
    ("ATC_ANG_YAW_P", 0.0, 10.0, 0.1, float),
]
# =================================================


class TuneMode(Enum):
    MANUAL = 1
    AUTO = 2


class ROV26RcMixer:
    def __init__(
        self,
        input_state: InputState,
        control_state: ControlState,
        param_queue: queue.Queue,
        auto_event: threading.Event,
        keyboard_state: SharedKeyboardState,
        **kwargs,
    ):
        self.smoothing_factor = kwargs.get("smoothing_factor") or 1
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

        self.manual_tune_file_name = "pwm_manual.json"
        self.auto_tune_file_name = "pwm_auto.json"

        self.depth_hold_btn = PressButton()
        self.manual_btn = PressButton()
        self.stabilize_btn = PressButton()
        self.autonomous_btn = PressButton()
        self.arm_btn = SimulPressButton()
        self.manual_tune_btn = PressButtonTarget(-1)
        self.auto_tune_btn = PressButtonTarget(1)
        self.record_btn = PressButton()
        self.playback_btn = PressButton()

        self.input_state = input_state
        self.control_state = control_state
        self.keyboard_state = keyboard_state
        self.param_queue = param_queue

        self.control_recorder = ControlRecorder()

        self._thread = None
        self._is_running = threading.Event()

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.stream_rc, daemon=True)
            self._thread.start()

        self._set_tune_manual()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def _set_tune_manual(self):
        manual_config = self._load_config(self.manual_tune_file_name)
        self._dump2queue(manual_config)

    def _set_tune_auto(self):
        manual_config = self._load_config(self.auto_tune_file_name)
        self._dump2queue(manual_config)

    def _dump2queue(self, book):
        for group_id, limits in book.items():
            for param_name, val in limits.items():
                if group_id == "GLOBAL":
                    param_id = param_name
                else:
                    param_id = f"MOT_{group_id}_{param_name}"

                self.param_queue.put((param_id, val))

    def _load_config(self, SAVE_FILE):
        defaults = {
            m: {p[0]: (1500 if p[4] == int else 0.0) for p in PARAMS_CONFIG}
            for m in MOTOR_COORDS
        }

        # Add Global parameter defaults
        defaults["GLOBAL"] = {p[0]: 0.0 for p in GLOBAL_PARAMS_CONFIG}

        if os.path.exists(SAVE_FILE):
            try:
                with open(SAVE_FILE, "r") as f:
                    data = json.load(f)
                    for k, v in data.items():
                        if k == "GLOBAL":
                            defaults["GLOBAL"].update(v)
                        else:
                            try:
                                motor_id = int(k)
                                if motor_id in defaults:
                                    defaults[motor_id].update(v)
                            except ValueError:
                                pass  # Ignore invalid keys
            except Exception as e:
                logger.error(f"Warning: Failed to load {SAVE_FILE}: {e}")
        return defaults

    def stream_rc(self):
        while self._is_running.is_set():
            self.update_control_from_inputs()

            time.sleep(0.02)

    def _update_poll_auto_stop(self, inputs):
        if self.autonomous_btn.toggle(inputs.btn_up):
            self.auto_event.clear()

    def update_control_from_inputs(self):
        inputs = self.input_state.get_latest()
        if not self.auto_event.is_set() and not self.control_recorder.is_playing:
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

        if not self.auto_event.is_set() and not self.control_recorder.is_playing:
            with self.control_state as control:
                control.forward = int(self.current_forward if is_fwd_stronger else 1500)
                control.lateral = int(
                    self.current_lateral if not is_fwd_stronger else 1500
                )
                control.vertical = int(self.current_vertical)
                control.yaw = int(self.current_yaw)

        if self.record_btn.toggle(self.keyboard_state.get_key() == "r"):
            self.control_recorder.toggle_record()
        elif self.playback_btn.toggle(self.keyboard_state.get_key() == "p"):
            self.control_recorder.toggle_playback()

        self.control_recorder.process_data(self.control_state)

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

            if self.manual_btn.toggle(inputs.dpad_hor):
                self._set_tune_manual()
            elif self.auto_tune_btn.toggle(inputs.dpad_hor):
                self._set_tune_auto()

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
