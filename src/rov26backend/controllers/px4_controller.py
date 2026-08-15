from pymavlink import mavutil
from rov26backend.models.control_state import ControlState
from rov26backend.models.telemetry_state import TelemetryState
import logging
import serial.tools.list_ports
import sys
import threading
import time


logger = logging.getLogger("ROV.px4")


class PixhawkController:
    def __init__(self, control_state: ControlState, telemetry_state: TelemetryState):
        self.master = None
        self.pxmode = "MANUAL"
        self.mav = None
        self.rc_chans = None
        self._thread = None
        self._is_running = threading.Event()
        self.control_state = control_state
        self.telemetry_state = telemetry_state

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
                self.set_mode(target_mode)
                with self.control_state as control:
                    control.target_mode = None

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
        ports = self._get_serial_ports()
        if not ports:
            logger.warn("No USB serial ports found (Pixhawk)")
            return
        logger.info(f"Available USB ports: {ports}")
        for port in ports:
            try:
                self.master = mavutil.mavlink_connection(port, baud=57600)
                self.master.wait_heartbeat()
                logger.info(f"Pixhawk found on {port}")
                self.master.mav.heartbeat_send(0, 0, 0, 0, 0)
                self.arm()
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                    10,
                    1,
                )
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
                    10,
                    1,
                )
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    10,
                    1,
                )

                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                    10,
                    1,
                )

                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                    1,
                    1,
                )
                return
            except Exception as e:
                logger.warn(f"Failed to connect to Pixhawk on {port}: {e}")
                self.master = None

        logger.warn("Pixhawk not found on any port")

    def arm(self, block=True):
        if self.master is None:
            return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        logger.info("Arming motors ...")
        if block:
            self.master.motors_armed_wait()
            logger.info("Motor Armed!")

    def disarm(self, block=True):
        if self.master is None:
            return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        logger.info("Disarming motors ...")
        if block:
            self.master.motors_disarmed_wait()
            logger.info("Motor Disarmed!")

    def request_pixhawk_to_telemetry(self):
        try:
            heartbeat = self.master.messages.get("HEARTBEAT", None)
            msg_coor = self.master.messages.get("GLOBAL_POSITION_INT", None)
            # system_time = self.master.messages.get("SYSTEM_TIME", None)
            attitude = self.master.messages.get("ATTITUDE", None)
            sys_status = self.master.messages.get("SYS_STATUS", None)
            rc_msg = self.master.messages.get("RC_CHANNELS", None)
            servo_msg = self.master.messages.get("SERVO_OUTPUT_RAW", None)

            with self.telemetry_state as ts:
                if rc_msg is not None:
                    ts.forward_rc = rc_msg.chan5_raw
                    ts.vertical_rc = rc_msg.chan3_raw
                    ts.lateral_rc = rc_msg.chan6_raw
                    ts.yaw_rc = rc_msg.chan4_raw

                if servo_msg is not None:
                    ts.mot1_eff = servo_msg.servo1_raw
                    ts.mot2_eff = servo_msg.servo2_raw
                    ts.mot3_eff = servo_msg.servo3_raw
                    ts.mot4_eff = servo_msg.servo4_raw
                    ts.mot5_eff = servo_msg.servo5_raw
                    ts.mot6_eff = servo_msg.servo6_raw
                    ts.servo_effort = servo_msg.servo9_raw

                if heartbeat is not None:
                    ts.armed = (heartbeat.base_mode & 128) > 0

                if attitude is not None:
                    ts.rollspeed = attitude.rollspeed
                    ts.yawspeed = attitude.yawspeed
                    ts.pitchspeed = attitude.pitchspeed
                    ts.roll = attitude.roll
                    ts.yaw = attitude.yaw
                    ts.pitch = attitude.pitch

                if sys_status is not None:
                    ts.fc_cpu_load = sys_status.load > 0.8
                    health = sys_status.onboard_control_sensors_health
                    ts.fc_gyro_health = (health & 1) > 0
                    ts.fc_acc_health = (health & 2) > 0
                    ts.fc_compass_health = (health & 4) > 0
                    ts.fc_baro_health = (health & 8) > 0

                if msg_coor is not None:
                    ts.depth = msg_coor.relative_alt / 1000.0

                ts.mode = self.get_mode()

            # if system_time is not None:
            #     telemetry_state.update(timestamp=None)

            return

        except Exception as error:
            self.log_err(f"Error in request_pixhawk: {error}")
            return

    def set_mode(self, mode):
        self.pxmode = mode
        if self.pxmode not in self.master.mode_mapping():
            logger.warn(f"Unknown Mode : {self.pxmode}")
            return

        mode_id = self.master.mode_mapping()[self.pxmode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        logger.info(f"Mode set to : {self.pxmode}")

    def rc_channels_override_send(self, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            ch1,
            ch2,
            ch3,
            ch4,
            ch5,
            ch6,
            ch7,
            ch8,
            0,
            0,
        )
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
        while True:
            msg = self.master.recv_msg()
            if msg is None:
                break

    def get_mode(self):
        if "HEARTBEAT" in self.master.messages:
            latest_heartbeat = self.master.messages["HEARTBEAT"]
            mode_id = latest_heartbeat.custom_mode
            mav_type = self.master.field("HEARTBEAT", "type", None)
            mode_map_num = mavutil.mode_mapping_bynumber(mav_type)
            mode_name = (
                mode_map_num.get(mode_id, "UNKNOWN") if mode_map_num else "UNKNOWN"
            )
            return mode_name
        return "MANUAL"
