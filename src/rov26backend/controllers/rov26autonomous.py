import logging
import threading
import time

from rov26backend.controllers.direction_maintainers import (
    ForwardMaintainer,
    LateralMaintainer,
    VerticalMaintainer,
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
        self.target_x = kwargs.get("target_x") or 0.7
        self.target_y = kwargs.get("target_y") or -3.9
        self.target_z = kwargs.get("target_z") or 24.0
        self.target_yaw = kwargs.get("target_yaw") or 0.0

        self._thread = None
        self._is_running = threading.Event()
        self.auto_event = auto_event
        self.control_state = control_state
        self.vision_state = vision_state
        self.depth_state = depth_state
        self.vertical_maintainer = VerticalMaintainer(
            self.target_y,
            vision_state,
            control_state,
            depth_state,
            auto_event,
            vertical_kp=kwargs.get("vertical_kp"),
            vertical_ki=kwargs.get("vertical_ki"),
            vertical_kd=kwargs.get("vertical_kd"),
            deadzone=kwargs.get("vertical_deadzone"),
        )

        self.maintainers = [
            LateralMaintainer(
                self.target_x,
                vision_state,
                control_state,
                auto_event,
                vertical_maintainer=self.vertical_maintainer,  # Pass instance here
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
                vertical_maintainer=self.vertical_maintainer,  # Pass instance here
                forward_kp=kwargs.get("forward_kp"),
                forward_ki=kwargs.get("forward_ki"),
                forward_kd=kwargs.get("forward_kd"),
                deadzone=kwargs.get("forward_deadzone"),
            ),
        ]
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


    #otw
    def target_depth(self):
        recorded_depth = self.depth_state.get_latest().recorded_depth
        if recorded_depth is not None:
            self.vertical_maintainer.pid.target = recorded_depth
            self.vertical_maintainer.target = recorded_depth

            logger.info(
                f"Autonomous Phase 1: Descending to recorded depth of {recorded_depth:.2f} m."
            )
            with self.control_state as control:
                control.target_mode = "ALT_HOLD"
            return True 

        logger.info("No recorded depth available.")
        return False


    def forward_and_grip(self):
        logger.info(
            "Autonomous Phase 2: Approaching payload and engaging gripper."
        )

        GRIP_DISTANCE = 15.1    # cm, sesuaikan dengan posisi ideal gripper

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
            logger.info(
                f"Payload detected | "
                f"x={x:.2f} cm, "
                f"y={y:.2f} cm, "
                f"z={z:.2f} cm"
            )

            # Masih terlalu jauh -> maju
            if z > GRIP_DISTANCE:
                with self.control_state as control:
                    control.forward = 1600
            # Sudah cukup dekat -> berhenti lalu grip
            else:
                logger.info(
                    f"Grip distance reached: {z:.2f} cm. Stopping ROV."
                )

                with self.control_state as control:
                    control.forward = 1500
                time.sleep(0.5)
                with self.control_state as control:
                    control.servo = 1800

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
            control.forward = 1450
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(3)

        # stop bentar
        logger.info("Phase 3.2: Stabilizing after reversing.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(0.5)

        # naik 6 detik
        logger.info("Phase 3.3: Ascending to release the payload.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1600
            control.yaw = 1500
        time.sleep(6)

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
            control.forward = 1450
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(3)

        for i in range(3):
            # naik 2 detik
            logger.info(f"Phase 3.{i + 2}: Ascending to release the payload.")
            with self.control_state as control:
                control.forward = 1500
                control.lateral = 1500
                control.vertical = 1550
                control.yaw = 1500
            time.sleep(2)

            # mundur 2 detik
            logger.info(f"Phase 3.{i + 3}: Reversing to release the payload.")
            with self.control_state as control:
                control.forward = 1450
                control.lateral = 1500
                control.vertical = 1600
                control.yaw = 1500
            time.sleep(2)

        # naik terakhir kali
        logger.info("Phase 3.8: Final ascending to release the payload.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1550
            control.yaw = 1500
        time.sleep(5)


    def run(self):
        logger.info("Autonomous execution thread processing loops active.")
        while self._is_running.is_set():
            if self.auto_event.is_set():
                logger.info("Autonomous sequence triggered via auto_event flag.")

                #mundur
                with self.control_state as control:
                    control.forward = 1425
                time.sleep(6)
                with self.control_state as control:
                    control.forward = 1500

                # gunakan depth yang direkam saat manual
                if not self.target_depth():
                    logger.warning(
                        "No recorded depth. Aborting autonomous sequence."
                    )
                    self.auto_event.clear()
                    continue

                self.vertical_maintainer.control_until_timeout(6)


                #Ini buat apa dah masih bingung
                logger.info(
                    "Autonomous Phase 2: Deploying 6DOF close-loop coordinate hold."
                )
                while self._is_running.is_set() and self.auto_event.is_set():
                    all_maintained = True
                    for maintainer in self.maintainers:
                        all_maintained = (
                            maintainer.control_until_target() and all_maintained
                        )

                        with self.control_state as control:
                            control.forward = 1500
                            control.lateral = 1500
                            control.yaw = 1500
                            control.servo = 1700

                        self.vertical_maintainer.control_until_timeout(6)

                    if all_maintained:
                        logger.info(
                            "All directional maintenance modules verified stabilized inside deadzones!"
                        )
                        break
                    time.sleep(0.01)

                self.forward_and_grip()
                self.auto_opt_1()

                self.auto_event.clear()
                logger.info(
                    "Autonomous mission routing complete. Returning control context to baseline system."
                )
                with self.control_state as control:
                    control.forward = 1500
                    control.lateral = 1500
                    control.vertical = 1500
                    control.yaw = 1500
                    control.servo = 1700

            time.sleep(0.01)