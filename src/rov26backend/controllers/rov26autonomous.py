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

    def run(self):
        logger.info("Autonomous execution thread processing loops active.")
        while self._is_running.is_set():
            if self.auto_event.is_set():
                logger.info("Autonomous sequence triggered via auto_event flag.")

                with self.control_state as control:
                    control.forward = 1425

                time.sleep(6)

                with self.control_state as control:
                    control.forward = 1500

                self.vertical_maintainer.control_until_timeout(6)

                with self.control_state as control:
                    control.target_mode = "ALT_HOLD"

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
