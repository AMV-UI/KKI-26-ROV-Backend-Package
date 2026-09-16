import logging
import threading
import time

from rov26backend.controllers.direction_maintainers import (
    ForwardMaintainer,
    LateralMaintainer,
)
from rov26backend.models.control_state import ControlState
from rov26backend.models.depth_state import DepthState
from rov26backend.models.vision_state import VisionState

logger = logging.getLogger("ROV.auto")


class Rov26Autonomous:
    def __init__(
        self,
        control_state: ControlState,
        vision_state: VisionState,
        depth_state: DepthState,
        auto_event: threading.Event,
        **kwargs,
    ):
        self.target_x = kwargs.get("target_x") or 2.1
        self.target_y = kwargs.get("target_y") or -3.9
        self.target_z = kwargs.get("target_z") or 18.0
        self.target_yaw = kwargs.get("target_yaw") or 0.0

        self._thread = None
        self._is_running = threading.Event()
        self.auto_event = auto_event
        self.control_state = control_state
        self.vision_state = vision_state
        self.depth_state = depth_state

        self.maintainers = [
            LateralMaintainer(
                self.target_x,
                vision_state,
                control_state,
                auto_event,
                lateral_kp=kwargs.get("lateral_kp"),
                lateral_ki=kwargs.get("lateral_ki"),
                lateral_kd=kwargs.get("lateral_kd"),
                deadzone=kwargs.get("lateral_deadzone"),
            ),
            ForwardMaintainer(
                self.target_z,
                vision_state,
                control_state,
                auto_event,
                forward_kp=kwargs.get("forward_kp"),
                forward_ki=kwargs.get("forward_ki"),
                forward_kd=kwargs.get("forward_kd"),
                deadzone=kwargs.get("forward_deadzone"),
            ),
        ]

        self._fallback_thread = None

        # self.maintainers[0].maintainer = self.maintainers[1]
        self.maintainers[1].maintainer = self.maintainers[0]
        logger.info("Rov26Autonomous worker tracking instance ready.")

    def start(self):
        if self._thread is None:
            logger.info("Starting autonomous control manager thread...")
            self._is_running.set()
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        logger.info("Signaling autonomous manager thread to stop...")
        self._is_running.clear()
        self.auto_event.clear()
        if self._thread:
            self._thread.join()
            self._thread = None
        logger.info("Autonomous manager thread fully stopped.")

    # otw
    def target_depth(self):
        recorded_depth = self.depth_state.get_latest().recorded_depth
        if recorded_depth is not None:
            logger.info(
                f"Autonomous Phase 1: Descending to recorded depth of {recorded_depth:.2f} m."
            )
            return True

        logger.info("No recorded depth available.")
        return False

    def forward_and_grip(self):
        logger.info("Autonomous Phase 2: Approaching payload and engaging gripper.")

        GRIP_DISTANCE = 26.0  # cm, sesuaikan dengan posisi ideal gripper

        while self._is_running.is_set() and self.auto_event.is_set():
            latest_vision_state = self.vision_state.get_latest()
            tvec = latest_vision_state.tvec

            # QR tidak terdeteksi
            if tvec[0] == 0 and tvec[1] == 0 and tvec[2] == 0:
                with self.control_state as control:
                    control.forward = 1500

                logger.warning("Target lost! QR marker not detected.")
                time.sleep(0.05)
                continue

            x, y, z = tvec[0], tvec[1], tvec[2]
            logger.info(f"Payload detected | x={x:.2f} cm, y={y:.2f} cm, z={z:.2f} cm")

            # Masih terlalu jauh -> maju
            if z > GRIP_DISTANCE:
                with self.control_state as control:
                    control.forward = 1900
            # Sudah cukup dekat -> berhenti lalu grip
            else:
                logger.info(f"Grip distance reached: {z:.2f} cm. Stopping ROV.")

                with self.control_state as control:
                    control.forward = 1500
                time.sleep(0.5)
                with self.control_state as control:
                    control.servo = 2230

                logger.info("Payload grip engaged.")
                break
            time.sleep(0.05)

        # Pastikan forward berhenti
        with self.control_state as control:
            control.forward = 1500

    def auto_opt_1(self):
        logger.info("Autonomous Phase 3: Taking the pay load off the hook.")
        # mundur 3 detik
        logger.info("Phase 3.1: Reversing away from the pipe.")
        with self.control_state as control:
            control.forward = 1100
            control.lateral = 1500
            control.yaw = 1500
            control.servo = 2570
        time.sleep(4.5)

        # stop bentar
        logger.info("Phase 3.2: Stabilizing after reversing.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.yaw = 1500
        time.sleep(0.5)

        # naik 6 detik
        logger.info("Phase 3.3: Ascending to release the payload.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1900
            control.yaw = 1500
        time.sleep(10)

        # Stop and stabilize
        logger.info("Phase 3.4: Payload release maneuver complete.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500

    def auto_opt_2(self):
        logger.info("Autonomous Phase 3: Taking the pay load off the hook.")
        # mundur 3 detik sampe mentok ke ujung pipe
        logger.info("Phase 3.1: Reversing away from the pipe.")
        with self.control_state as control:
            control.forward = 1100
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
            control.servo = 2550
        time.sleep(3)

        for i in range(3):
            # naik 2 detik
            logger.info(f"Phase 3.{i + 2}: Ascending to release the payload.")
            with self.control_state as control:
                control.forward = 1500
                control.lateral = 1500
                control.vertical = 1900
                control.yaw = 1500
            time.sleep(2)

            # mundur 2 detik
            logger.info(f"Phase 3.{i + 3}: Reversing to release the payload.")
            with self.control_state as control:
                control.forward = 1100
                control.lateral = 1500
                control.vertical = 1600
                control.yaw = 1500
            time.sleep(2)

        # naik terakhir kali
        logger.info("Phase 3.8: Final ascending to release the payload.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1900
            control.yaw = 1500
        time.sleep(5)

    def run(self):
        logger.info("Autonomous execution thread processing loops active.")
        while self._is_running.is_set():
            if self.auto_event.is_set():
                logger.info("Autonomous sequence triggered via auto_event flag.")
                def give_up():
                    logger.info("GIVING UP AUTO AND RISING")
                    with self.control_state as control:
                        control.vertical = 1900
                        control.servo = 2550
                    self.auto_event.clear()

                with self.control_state as control:
                    control.servo = 2100

                self._fallback_thread = threading.Timer(180, give_up)

                self._fallback_thread.start()

                with self.control_state as control:
                    control.forward = 1300
                time.sleep(4.5)
                with self.control_state as control:
                    control.forward = 1500


                with self.control_state as control:
                    control.target_mode = "ALT_HOLD"

                time.sleep(0.5)

                # gunakan depth yang direkam saat manual
                if not self.target_depth():
                    logger.warning("No recorded depth. Aborting autonomous sequence.")
                    self.auto_event.clear()
                    continue

                with self.control_state as control:
                    control.depth_set = self.depth_state.get_latest().recorded_depth

                time.sleep(6)

                with self.control_state as control:
                    control.vertical = 1500


                logger.info(
                    "Autonomous Phase 2: Deploying 6DOF close-loop coordinate hold."
                )
                with self.vision_state as v:
                    v.tvec = [0, 0, 0]
                    v.rvec = [0, 0, 0]
                    v.euler_angles = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
                    v.qr_polygon = None

                oncer = True
                
                self.maintainers[0].pid.setpoint = 4
                self.maintainers[0].deadzone = 4

                time.sleep(0.5)
                while self._is_running.is_set() and self.auto_event.is_set():
                    all_maintained = True
                    for maintainer in self.maintainers:
                        all_maintained = (
                            maintainer.control_until_target() and all_maintained
                        )

                        with self.control_state as control:
                            control.forward = 1500
                            control.lateral = 1500
                            control.vertical = 1500
                            control.yaw = 1500
                            control.servo = 2100

                        time.sleep(0.5)

                    if all_maintained:
                        logger.info(
                            "All directional maintenance modules verified stabilized inside deadzones!"
                        )
                        break
                    time.sleep(0.01)

                    if oncer:
                        self.maintainers[0].pid.setpoint = 1
                        self.maintainers[0].deadzone = 1
                        oncer = False

                self.auto_opt_1()
                self.auto_event.clear()
                logger.info(
                    "Autonomous mission routing complete. Returning control context to baseline system."
                )
                with self.control_state as control:
                    control.forward = 1500
                    control.lateral = 1500
                    control.vertical = 1900
                    control.yaw = 1500
                    control.servo = 2570

                self._fallback_thread.cancel()

            time.sleep(0.01)
