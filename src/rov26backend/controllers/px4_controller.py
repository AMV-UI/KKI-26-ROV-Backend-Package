import os
import fnmatch
from pymavlink import mavutil


class PixhawkController:
    def __init__(self):
        self.master = None
        self.pxmode = "MANUAL"
        self.mav = None
        self.rc_chans = None

        self.servo_pwm = 1500

        self._init_serial()

    def log(self, msg):
        print(f"PX4: {msg}")

    def log_err(self, msg):
        print(f"[ERR] PX4: {msg}")

    def _get_serial_ports(self):
        dirs = []
        list_of_files = os.listdir("/dev")
        pattern = "ttyACM*"
        for entry in list_of_files:
            if fnmatch.fnmatch(entry, pattern):
                dirs.append(f"/dev/{entry}")
        return dirs

    def _init_serial(self):
        ports = self._get_serial_ports()
        if not ports:
            self.log_err("No USB serial ports found (Pixhawk)")
            return
        self.log(f"Available USB ports: {ports}")
        for port in ports:
            try:
                self.master = mavutil.mavlink_connection(port, baud=57600)
                self.master.wait_heartbeat()
                self.log(f"Pixhawk found on {port}")
                self.master.mav.heartbeat_send(0, 0, 0, 0, 0)
                self._px_arm()
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
                self.log_err(f"Failed to connect to Pixhawk on {port}: {e}")
                self.master = None

        self.log_err("Pixhawk not found on any port")

    def _px_arm(self, block=True):
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
        self.log("Arming motors ...")
        if block:
            self.master.motors_armed_wait()
            self.log("Motor Armed!")

    def _px_disarm(self, block=True):
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
        self.log("Disarming motors ...")
        if block:
            self.master.motors_disarmed_wait()
            self.log("Motor Disarmed!")

    def request_pixhawk_to_telemetry(self, telemetry_state):
        try:
            heartbeat = self.master.messages.get("HEARTBEAT", None)
            msg_coor = self.master.messages.get("GLOBAL_POSITION_INT", None)
            # system_time = self.master.messages.get("SYSTEM_TIME", None)
            attitude = self.master.messages.get("ATTITUDE", None)
            sys_status = self.master.messages.get("SYS_STATUS", None)
            rc_msg = self.master.messages.get("RC_CHANNELS", None)
            servo_msg = self.master.messages.get("SERVO_OUTPUT_RAW", None)

            if rc_msg is not None:
                telemetry_state.update(forward_rc=rc_msg.chan5_raw)
                telemetry_state.update(vertical_rc=rc_msg.chan3_raw)
                telemetry_state.update(lateral_rc=rc_msg.chan6_raw)
                telemetry_state.update(yaw_rc=rc_msg.chan4_raw)

            if servo_msg is not None:
                telemetry_state.update(mot1_eff=servo_msg.servo1_raw)
                telemetry_state.update(mot2_eff=servo_msg.servo2_raw)
                telemetry_state.update(mot3_eff=servo_msg.servo3_raw)
                telemetry_state.update(mot4_eff=servo_msg.servo4_raw)
                telemetry_state.update(mot5_eff=servo_msg.servo5_raw)
                telemetry_state.update(mot6_eff=servo_msg.servo6_raw)

            if heartbeat is not None:
                telemetry_state.update(armed=(heartbeat.base_mode & 128) > 0)

            if attitude is not None:
                telemetry_state.update(rollspeed=attitude.rollspeed)
                telemetry_state.update(yawspeed=attitude.yawspeed)
                telemetry_state.update(pitchspeed=attitude.pitchspeed)
                telemetry_state.update(roll=attitude.roll)
                telemetry_state.update(yaw=attitude.yaw)
                telemetry_state.update(pitch=attitude.pitch)

            if sys_status is not None:
                telemetry_state.update(fc_cpu_load=sys_status.load)
                health = sys_status.onboard_control_sensors_health
                telemetry_state.update(fc_gyro_health=(health & 1) > 0)
                telemetry_state.update(fc_acc_health=(health & 2) > 0)
                telemetry_state.update(fc_compass_health=(health & 4) > 0)
                telemetry_state.update(fc_baro_health=(health & 8) > 0)

            if msg_coor is not None:
                telemetry_state.update(depth=msg_coor.relative_alt / 1000.0)

            # if system_time is not None:
            #     telemetry_state.update(timestamp=None)

            return

        except Exception as error:
            self.log_err(f"Error in request_pixhawk: {error}")
            return

    def _px_set_mode(self, mode):
        self.pxmode = mode
        if self.pxmode not in self.master.mode_mapping():
            self.log_err(f"Unknown Mode : {self.pxmode}")
            return

        mode_id = self.master.mode_mapping()[self.pxmode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.log(f"Mode set to : {self.pxmode}")

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
        )

    def _pump_mavlink_messages(self):
        while True:
            msg = self.master.recv_msg()
            if msg is None:
                break

    def _get_cur_mode(self):
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

    def __str__(self):
        if self.master is not None:
            return "Logs appear here"
        else:
            return "Pixhawk hasnt started yet"
