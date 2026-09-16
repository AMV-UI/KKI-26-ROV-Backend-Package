import logging
import time

from simple_pid import PID

from rov26backend.models.control_state import ControlState
from rov26backend.models.vision_state import VisionState

logger = logging.getLogger("ROV.auto")


class DirectionMaintainer:
    def __init__(
        self,
        target,
        vision_state: VisionState,
        control_state: ControlState,
        auto_event,
        kp=1,
        ki=0,
        kd=0,
        deadzone=0.5,
        timeout=2.5,
        minom=None,
        minom_timeout=1,
        forward=False,
    ):
        self.pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=target, output_limits=(-400, 400))
        self.auto_event = auto_event
        self.target = target
        self.control_state = control_state
        self.vision_state = vision_state
        self.deadzone = deadzone
        self.timeout = timeout
        self.maintainer = None
        self.minom = minom
        self.minom_timeout = minom_timeout
        logger.info(
            f"{self.__class__.__name__} initialized. Target: {self.target}, "
            f"PID: ({kp}, {ki}, {kd}), Deadzone: {self.deadzone}"
        )

    def control_until_target(self):
        current_val = self.get_current()
        error = abs(current_val - self.target)
        if error <= self.deadzone:
            logger.info(
                f"[{self.__class__.__name__}] Target already maintained. Current: {current_val:.3f}, Target: {self.pid.setpoint}"
            )
            return True

        logger.info(
            f"[{self.__class__.__name__}] Starting track. Current: {current_val:.3f} -> Target: {self.pid.setpoint}"
        )

        long_maintained = False
        last_not_maintained = time.time()
        last_maintained = time.time()
        other_maintained = time.time()
        last_not_minom = time.time()

        while not long_maintained and self.auto_event.is_set():
            current = self.get_current()
            output = self.pid(current)

            # Minom threshold check with continuous duration timeout
            if self.minom is not None and current < self.minom:
                if time.time() - last_not_minom >= self.minom_timeout:
                    logger.info(
                        f"[{self.__class__.__name__}] Minom threshold met for >={self.minom_timeout}s. Current: {current:.3f}"
                    )
                    return True
            else:
                last_not_minom = time.time()

            logger.debug(
                f"[{self.__class__.__name__}] Tracking loop | Current: {current:.3f}, Error: {abs(current - self.target):.3f}, PID Output: {output:.3f}"
            )
            self.control_to(int(1500 + output))

            maintained = abs(current - self.target) < self.deadzone
            if not maintained:
                last_not_maintained = time.time()
            else:
                last_maintained = time.time()

            time_since_not_maintained = time.time() - last_not_maintained

            long_maintained = time_since_not_maintained > 0.5

            if time.time() - last_maintained > self.timeout:
                logger.info(
                    f"[{self.__class__.__name__}] Loop ended. long_maintained={long_maintained}, "
                    f"auto_event={self.auto_event.is_set()}, current={current:.3f}"
                )
                return long_maintained

            time.sleep(0.01)

        logger.info(
            f"[{self.__class__.__name__}] Target successfully reached! Settled at: {self.get_current():.3f}"
        )
        return False

    def control_to(self, value):
        raise NotImplementedError("method must be overriden by child")

    def get_current(self):
        raise NotImplementedError("method must be overriden by child")


class ForwardMaintainer(DirectionMaintainer):
    def __init__(
        self,
        target,
        vision_state: VisionState,
        control_state: ControlState,
        auto_event,
        **kwargs,
    ):
        super().__init__(
            target,
            vision_state,
            control_state,
            auto_event,
            kwargs.get("forward_kp") or -2.0,
            kwargs.get("forward_ki") or 0,
            kwargs.get("forward_kd") or 0.0,
            kwargs.get("forward_deadzone") or 3.5,
            timeout=2,
            minom=18,
            minom_timeout=1.0,
        )

    def control_to(self, value):
        with self.control_state as control:
            current = self.get_current()
            if current != 0:
                control.forward = int(value)
            else:
                control.forward = 1500

    def get_current(self):
        return self.vision_state.get_latest().tvec[2]


class LateralMaintainer(DirectionMaintainer):
    def __init__(
        self,
        target,
        vision_state: VisionState,
        control_state: ControlState,
        auto_event,
        **kwargs,
    ):
        super().__init__(
            target,
            vision_state,
            control_state,
            auto_event,
            kwargs.get("lateral_kp") or -5.0,
            kwargs.get("lateral_ki") or 0.0,
            kwargs.get("lateral_kd") or 0.0,
            kwargs.get("lateral_deadzone") or 2.0,
            timeout=4,
        )

    def control_to(self, value):
        with self.control_state as control:
            current = self.get_current()
            if current != 0:
                control.lateral = int(value)
            else:
                control.lateral = 1500

    def get_current(self):
        return self.vision_state.get_latest().tvec[0]
