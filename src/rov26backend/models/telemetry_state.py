import threading


from datetime import datetime, timezone


class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()

        self.mode = "MANUAL"
        self.battery = 0.0
        self.timestamp = datetime.fromtimestamp(0, tz=timezone.utc)

        # Attitude & Depth
        self.depth = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yawspeed = 0.0
        self.rollspeed = 0.0
        self.pitchspeed = 0.0

        # RC Channels (1500 is standard MAVLink neutral)
        self.forward_rc = 1500
        self.lateral_rc = 1500
        self.vertical_rc = 1500
        self.yaw_rc = 1500

        # Motor Efforts
        self.mot1_eff = 0
        self.mot2_eff = 0
        self.mot3_eff = 0
        self.mot4_eff = 0
        self.mot5_eff = 0
        self.mot6_eff = 0

        # Flight Controller Status
        self.fc_cpu_load = False
        self.fc_gyro_health = False
        self.fc_acc_health = False
        self.fc_compass_health = False
        self.fc_baro_health = False
        self.armed = False
        self.servo_effort = 2500

    def update(self, **kwargs):
        """
        Safely updates only the provided fields.
        Example: state.update(roll=12.5, pitch=5.2)
        """
        with self.lock:
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, value)

    def get_latest(self):
        """
        Safely grabs a snapshot of all current values for the gRPC stream.
        """
        with self.lock:
            return {
                "mode": self.mode,
                "battery": self.battery,
                "timestamp": self.timestamp,
                "depth": self.depth,
                "yaw": self.yaw,
                "roll": self.roll,
                "pitch": self.pitch,
                "yawspeed": self.yawspeed,
                "rollspeed": self.rollspeed,
                "pitchspeed": self.pitchspeed,
                "forward_rc": self.forward_rc,
                "lateral_rc": self.lateral_rc,
                "vertical_rc": self.vertical_rc,
                "yaw_rc": self.yaw_rc,
                "mot1_eff": self.mot1_eff,
                "mot2_eff": self.mot2_eff,
                "mot3_eff": self.mot3_eff,
                "mot4_eff": self.mot4_eff,
                "mot5_eff": self.mot5_eff,
                "mot6_eff": self.mot6_eff,
                "fc_cpu_load": self.fc_cpu_load,
                "fc_gyro_health": self.fc_gyro_health,
                "fc_acc_health": self.fc_acc_health,
                "fc_compass_health": self.fc_compass_health,
                "fc_baro_health": self.fc_baro_health,
                "armed": self.armed,
                "servo_effort": self.servo_effort
            }
