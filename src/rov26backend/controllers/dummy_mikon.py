from rov26backend.models.control_state import ControlState
from rov26backend.models.telemetry_state import TelemetryState
import logging
import serial.tools.list_ports
import sys
import threading
import time
import queue


logger = logging.getLogger("ROV.px4")


class DummyPixhawk:
    def __init__(
        self,
        control_state: ControlState,
        telemetry_state: TelemetryState,
        auto_event: threading.Event,
        param_queue: queue.Queue,
    ):
        self.master = None
        self.pxmode = "MANUAL"
        self.mav = None
        self.rc_chans = None
        self._thread = None
        self.param_queue = param_queue
        self._is_running = threading.Event()
        self.control_state = control_state
        self.telemetry_state = telemetry_state
        self.auto_event = auto_event

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self.control_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread:
            self._thread.join()

    def control_loop(self):
        hz = 20
        period = 1.0 / hz

        while self._is_running.is_set():
            if self.master is None:
                self._init_serial()
                time.sleep(1)
                continue

            start_time = time.time()

            latest_control_state = self.control_state.get_latest()

            self.rc_channels_override_send(
                1500,  # CH1
                1500,  # CH2
                latest_control_state.vertical,  # CH3
                latest_control_state.yaw,  # CH4
                latest_control_state.forward,  # CH5
                latest_control_state.lateral,  # CH6
                0,  # CH7
                latest_control_state.servo,
            )

            target_mode = latest_control_state.target_mode
            if target_mode is not None:
                if target_mode == "AUTO":
                    self.set_mode("ALT_HOLD")
                    self.auto_event.set()
                    logger.info("Mode set to : Auto")
                else:
                    self.set_mode(target_mode)
                    self.auto_event.clear()
                    logger.info(f"Mode set to : {self.pxmode}")
                with self.control_state as control:
                    control.target_mode = None

            try:
                param_id, param_value = self.param_queue.get_nowait()
                self.send_param_update(param_id, param_value)
            except queue.Empty:
                pass

            self._pump_mavlink_messages()
            self.request_pixhawk_to_telemetry()

            latest_telemetry_state = self.telemetry_state.get_latest()

            arm_toggle = latest_control_state.arm_toggle
            if arm_toggle:
                if latest_telemetry_state.armed:
                    self.disarm(block=False)
                else:
                    self.arm(block=False)
                with self.control_state as control:
                    control.arm_toggle = False

            elapsed = time.time() - start_time
            time.sleep(max(0, period - elapsed))

    def _get_serial_ports(self):
        dirs = []
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if sys.platform == "win32":
                dirs.append(port.device)
            else:
                if "ttyACM" in port.device:
                    dirs.append(port.device)

        return dirs

    def _init_serial(self):
        logger.info("Pixhawk found")
        self.master = "master"

    def arm(self, block=True):
        logger.info("Arming motors ...")
        if block:
            logger.info("Motor Armed!")

    def disarm(self, block=True):
        logger.info("Disarming motors ...")
        if block:
            logger.info("Motor Disarmed!")

    def send_param_update(self, param, value):
        """Constructs and sends the MAVLink parameter setting message."""
        logger.info(f"Sent {param} with value {value}")

    def request_pixhawk_to_telemetry(self):
        with self.telemetry_state as ts:
            ts.forward_rc = 0
            ts.vertical_rc = 0
            ts.lateral_rc = 0
            ts.yaw_rc = 0

            ts.mot1_eff = 0
            ts.mot2_eff = 0
            ts.mot3_eff = 0
            ts.mot4_eff = 0
            ts.mot5_eff = 0
            ts.mot6_eff = 0
            ts.servo_effort = 0

            ts.armed = 0

            ts.rollspeed = 0
            ts.yawspeed = 0
            ts.pitchspeed = 0
            ts.roll = 0
            ts.yaw = 0
            ts.pitch = 0

            ts.fc_cpu_load = 0
            ts.fc_gyro_health = 0
            ts.fc_acc_health = 0
            ts.fc_compass_health = 0
            ts.fc_baro_health = 0

            ts.depth = 0

            ts.mode = self.get_mode()

        return

    def set_mode(self, mode):
        pass

    def rc_channels_override_send(self, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8):
        logger.debug(f"""
                     Sending RC:
                     ch1: {ch1}
                     ch2: {ch2}
                     ch3: {ch3}
                     ch4: {ch4}
                     ch5: {ch5}
                     ch6: {ch6}
                     ch7: {ch7}
                     ch8: {ch8}
                     """)

    def _pump_mavlink_messages(self):
        pass

    def get_mode(self):
        return "MANUAL"
