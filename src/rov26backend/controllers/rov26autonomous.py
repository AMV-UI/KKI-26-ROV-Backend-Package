import threading
import argparse
from rov26backend.models.vision_state import VisionState
from rov26backend.models.control_state import ControlState
from rov26backend.controllers.direction_maintainers import (
    ForwardMaintainer,
    LateralMaintainer,
    VerticalMaintainer,
    YawMaintainer,
)
import logging
import time

logger = logging.getLogger("ROV.auto")


class Rov26Autonomous:
    def __init__(
        self,
        control_state: ControlState,
        vision_state: VisionState,
        auto_event: threading.Event,
        **kwargs,
    ):
        self.target_x = kwargs.get('target_x') or 0.0
        self.target_y = kwargs.get('target_y') or 0.0
        self.target_z = kwargs.get('target_z') or 0.0
        self.target_yaw = kwargs.get('target_yaw') or 0.0

        self._thread = None
        self._is_running = threading.Event()
        self.auto_event = auto_event
        self.control_state = control_state
        self.vision_state = vision_state

        self.maintainers = [
            VerticalMaintainer(
                self.target_y,
                vision_state,
                control_state,
                auto_event,
                vertical_kp=kwargs.get("vertical_kp"),
                vertical_ki=kwargs.get("vertical_ki"),
                vertical_kd=kwargs.get("vertical_kd"),
                deadzone=kwargs.get("vertical_deadzone"),
            ),
            LateralMaintainer(
                self.target_x, vision_state, control_state, auto_event,
                lateral_kp=kwargs.get("lateral_kp"),
                lateral_ki=kwargs.get("lateral_ki"),
                lateral_kd=kwargs.get("lateral_kd"),
                deadzone=kwargs.get("lateral_deadzone"),
            ),
            YawMaintainer(
                self.target_yaw, vision_state, control_state, auto_event,
                yaw_kp=kwargs.get("yaw_kp"),
                yaw_ki=kwargs.get("yaw_ki"),
                yaw_kd=kwargs.get("yaw_kd"),
                deadzone=kwargs.get("yaw_deadzone"),
            ),
            ForwardMaintainer(
                self.target_z, vision_state, control_state, auto_event,
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

    def descend_until_qr_found(self):
        logger.info(
            "Autonomous Phase 1: Commencing vertical descent looking for QR Target..."
        )
        while self._is_running.is_set() and self.auto_event.is_set():
            latest_vision_state = self.vision_state.get_latest()
            tvec = latest_vision_state.tvec

            logger.debug("Descent Loop")

            with self.control_state as control:
                if tvec[0] != 0 or tvec[1] != 0 or tvec[2] != 0:
                    control.vertical = 1500
                    logger.info("Target locked! QR marker detected")
                    break
                else:
                    control.vertical = 1450
            time.sleep(0.01)


    def auto_opt_1(self):
        logger.info(
            "Autonomous Phase 3: Taking the pay load off the hook."
        )
        #mundur 3 detik
        logger.info("Phase 3.1: Reversing away from the pipe.")
        with self.control_state as control:
            control.forward = 1450
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(3)

        #stop bentar
        logger.info("Phase 3.2: Stabilizing after reversing.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(0.5)

        #naik .. detik
        logger.info("Phase 3.3: Ascending to release the payload.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1600
            control.yaw = 1500
        time.sleep(6)

        #Stop and stabilize
        logger.info("Phase 3.4: Payload release maneuver complete.")
        with self.control_state as control:
            control.forward = 1500
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500

    
    def auto_opt_2(self):
        logger.info(
            "Autonomous Phase 3: Taking the pay load off the hook."
        )
        #mundur 3 detik sampe mentok ke ujung pipe
        logger.info("Phase 3.1: Reversing away from the pipe.")
        with self.control_state as control:
            control.forward = 1450
            control.lateral = 1500
            control.vertical = 1500
            control.yaw = 1500
        time.sleep(3)

        for i in range(3):
            #naik 2 detik
            logger.info(f"Phase 3.{i+2}: Ascending to release the payload.")
            with self.control_state as control:
                control.forward = 1500
                control.lateral = 1500
                control.vertical = 1550
                control.yaw = 1500
            time.sleep(2)

            #mundur 2 detik
            logger.info(f"Phase 3.{i+3}: Reversing to release the payload.")
            with self.control_state as control:
                control.forward = 1450
                control.lateral = 1500
                control.vertical = 1600
                control.yaw = 1500
            time.sleep(2)

        #naik terakhir kali
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
                self.descend_until_qr_found() 

                logger.info(
                    "Autonomous Phase 2: Deploying 6DOF close-loop coordinate hold."
                )
                while self._is_running.is_set() and self.auto_event.is_set():
                    all_maintained = True
                    for maintainer in self.maintainers:
                        all_maintained = (
                            all_maintained and maintainer.control_until_target()
                        )

                    if all_maintained:
                        logger.info(
                            "All directional maintenance modules verified stabilized inside deadzones!"
                        )
                        break
                    time.sleep(0.01)

                #self.auto_opt_1()
                #self.auto_opt_2()

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
