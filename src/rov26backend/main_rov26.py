import os
import queue
import sys
import threading

import typer

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
os.environ["YOLO_VERBOSE"] = "False"

import grpc

from rov26backend.controllers.bottom_camera_controller import BottomCamera
from rov26backend.controllers.front_camera_controller import FrontCamera
from rov26backend.controllers.gcs_controller import RosGrpcServicer
from rov26backend.controllers.joystick_controller import PxnP5JoystickLinux
from rov26backend.generated.server_pb2_grpc import add_ServerServicer_to_server

if sys.platform == "win32":
    from rov26backend.controllers.joystick_windows import PxnP5JoystickWindows

import concurrent.futures
import logging
import time
from typing import Annotated

from rov26backend.config import log_listener
from rov26backend.controllers.dummy_mikon import DummyPixhawk
from rov26backend.controllers.px4_controller import PixhawkController
from rov26backend.controllers.qrde_poly import QRPolygonFinder
from rov26backend.controllers.rc_mixer import ROV26RcMixer
from rov26backend.controllers.rov26autonomous import Rov26Autonomous
from rov26backend.controllers.rov26tuner import LivePWMOverlayTuner
from rov26backend.models.control_state import ControlState
from rov26backend.models.input_state import InputState
from rov26backend.models.polygon_state import PolygonState
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState

logger = logging.getLogger("ROV.main")

app = typer.Typer()


@app.command()
def rov(
    no: Annotated[list[str], typer.Option()] = [],
    smoothing_factor: float = None,
    servo_open: int = None,
    servo_close: int = None,
    pwm_center: int = None,
    pwm_range: int = None,
    pwm_min: int = None,
    pwm_max: int = None,
    dummy_mikon: bool = False,
    tune_motor: bool = False,
    front_cam_id: str = None,
    bottom_cam_id: str = None,
    grpc_port: int = 50051,
    target_x: float = None,
    target_y: float = None,
    target_z: float = None,
    vertical_kp: float = None,
    vertical_ki: float = None,
    vertical_kd: float = None,
    vertical_deadzone: float = None,
    forward_kp: float = None,
    forward_ki: float = None,
    forward_kd: float = None,
    forward_deadzone: float = None,
    lateral_kp: float = None,
    lateral_ki: float = None,
    lateral_kd: float = None,
    lateral_deadzone: float = None,
    yaw_kp: float = None,
    yaw_ki: float = None,
    yaw_kd: float = None,
    yaw_deadzone: float = None,
):
    auto_event = threading.Event()

    input_state = InputState()
    control_state = ControlState()
    telemetry_state = TelemetryState()
    vision_state = VisionState()
    mikon_param_queue = queue.Queue(maxsize=60)
    frame_queue = queue.Queue(maxsize=1)

    polygon_state = PolygonState()
    frame_queue = queue.Queue(maxsize=1)

    qr_finder = QRPolygonFinder(frame_queue, polygon_state)
    qr_finder.start()

    if "front_cam" not in no:
        # Pass the shared queues/states to FrontCamera
        front_camera = FrontCamera(
            vision_state,
            auto_event,
            frame_queue=frame_queue,
            polygon_state=polygon_state,
            front_cam_id=front_cam_id,
        )
        front_camera.start()

    joystick = None
    rc_mixer = None
    mikon = None
    front_camera = None
    bottom_camera = None
    server = None
    tuner = None

    if "joystick" not in no:
        if sys.platform == "linux":
            joystick = PxnP5JoystickLinux(input_state)
        else:
            joystick = PxnP5JoystickWindows(input_state)
        joystick.start()

    if "mixer" not in no:
        rc_mixer = ROV26RcMixer(
            input_state,
            control_state,
            auto_event,
            smoothing_factor=smoothing_factor,
            servo_open=servo_open,
            servo_close=servo_close,
            pwm_center=pwm_center,
            pwm_max=pwm_max,
            pwm_min=pwm_min,
            pwm_range=pwm_range,
        )
        rc_mixer.start()

    if "auto" not in no:
        autonomous = Rov26Autonomous(
            control_state,
            vision_state,
            auto_event,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            vertical_kp=vertical_kp,
            vertical_ki=vertical_ki,
            vertical_kd=vertical_kd,
            vertical_deadzone=vertical_deadzone,
            forward_kp=forward_kp,
            forward_ki=forward_ki,
            forward_kd=forward_kd,
            forward_deadzone=forward_deadzone,
            lateral_kp=lateral_kp,
            lateral_ki=lateral_ki,
            lateral_kd=lateral_kd,
            lateral_deadzone=lateral_deadzone,
            yaw_kp=yaw_kp,
            yaw_ki=yaw_ki,
            yaw_kd=yaw_kd,
            yaw_deadzone=yaw_deadzone,
        )
        autonomous.start()

    if "mikon" not in no:
        if dummy_mikon:
            mikon = DummyPixhawk(
                control_state, telemetry_state, auto_event, mikon_param_queue
            )
        else:
            mikon = PixhawkController(
                control_state, telemetry_state, auto_event, mikon_param_queue
            )
        mikon.start()

    if "bottom_cam" not in no:
        bottom_camera = BottomCamera(bottom_cam_id=bottom_cam_id)
        bottom_camera.start()

    if "grpc" not in no:
        server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))
        add_ServerServicer_to_server(
            RosGrpcServicer(telemetry_state, vision_state), server
        )
        server.add_insecure_port(f"[::]:{grpc_port}")
        server.start()
        logger.info(f"gRPC Server running on port {grpc_port}.")

    if tune_motor:
        tuner = LivePWMOverlayTuner(mikon_param_queue)
        tuner.start()

    logger.info("Main script active. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Ctrl+C detected! Shutting down ROV backend...")
    finally:
        if server:
            server.stop(grace=0)
        if bottom_camera:
            bottom_camera.stop()
        if front_camera:
            front_camera.stop()
        if rc_mixer:
            rc_mixer.stop()
        if joystick:
            joystick.stop()
        if mikon:
            mikon.stop()
        if autonomous:
            autonomous.stop()
        if tuner:
            tuner.stop()
        if qr_finder:
            qr_finder.stop()

        logger.info("Waiting for threads to exit...")
        logger.info("All threads stopped. Goodbye.")
        log_listener.stop()
