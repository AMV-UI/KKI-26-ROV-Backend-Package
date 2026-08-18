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
        args: argparse.Namespace,
    ):
        self.target_x = args.target_x
        self.target_y = args.target_y
        self.target_z = args.target_z
        self.target_yaw = args.target_yaw

        self._thread = None
        self._is_running = threading.Event()
        self.auto_event = auto_event
        self.control_state = control_state
        self.vision_state = vision_state

        vertical_kwargs = {}
        if hasattr(args, "vertical_kp"):
            vertical_kwargs["kp"] = args.vertical_kp
        if hasattr(args, "vertical_ki"):
            vertical_kwargs["ki"] = args.vertical_ki
        if hasattr(args, "vertical_kd"):
            vertical_kwargs["kd"] = args.vertical_kd
        if hasattr(args, "vertical_deadzone"):
            vertical_kwargs["deadzone"] = args.vertical_deadzone

        lateral_kwargs = {}
        if hasattr(args, "lateral_kp"):
            lateral_kwargs["kp"] = args.lateral_kp
        if hasattr(args, "lateral_ki"):
            lateral_kwargs["ki"] = args.lateral_ki
        if hasattr(args, "lateral_kd"):
            lateral_kwargs["kd"] = args.lateral_kd
        if hasattr(args, "lateral_deadzone"):
            lateral_kwargs["deadzone"] = args.lateral_deadzone

        yaw_kwargs = {}
        if hasattr(args, "yaw_kp"):
            yaw_kwargs["kp"] = args.yaw_kp
        if hasattr(args, "yaw_ki"):
            yaw_kwargs["ki"] = args.yaw_ki
        if hasattr(args, "yaw_kd"):
            yaw_kwargs["kd"] = args.yaw_kd
        if hasattr(args, "yaw_deadzone"):
            yaw_kwargs["deadzone"] = args.yaw_deadzone

        forward_kwargs = {}
        if hasattr(args, "forward_kp"):
            forward_kwargs["kp"] = args.forward_kp
        if hasattr(args, "forward_ki"):
            forward_kwargs["ki"] = args.forward_ki
        if hasattr(args, "forward_kd"):
            forward_kwargs["kd"] = args.forward_kd
        if hasattr(args, "forward_deadzone"):
            forward_kwargs["deadzone"] = args.forward_deadzone

        self.maintainers = [
            VerticalMaintainer(
                self.target_y, vision_state, control_state, **vertical_kwargs
            ),
            LateralMaintainer(
                self.target_x, vision_state, control_state, **lateral_kwargs
            ),
            YawMaintainer(self.target_yaw, vision_state, control_state, **yaw_kwargs),
            ForwardMaintainer(
                self.target_z, vision_state, control_state, **forward_kwargs
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
            qr_status = latest_vision_state.qr_side

            logger.debug(f"Descent Loop | Target status: {qr_status}")

            with self.control_state as control:
                if qr_status != "NOT_FOUND":
                    control.vertical = 1500
                    logger.info(
                        f"Target locked! QR marker detected: {qr_status}. Halting descent."
                    )
                    break
                else:
                    control.vertical = 1450
            time.sleep(0.01)

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

                self.auto_event.clear()
                logger.info(
                    "Autonomous mission routing complete. Returning control context to baseline system."
                )
            time.sleep(0.01)
