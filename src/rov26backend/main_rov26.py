import os
import sys
import threading

# 1. Force pure-Python Protobuf
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

# 3. NOW load OpenCV/Vision modules

# 4. THEN load gRPC/Protobuf
import grpc
from rov26backend.controllers.gcs_controller import RosGrpcServicer
from rov26backend.generated.server_pb2_grpc import add_ServerServicer_to_server

# 5. Load the rest of your hardware controllers
from rov26backend.controllers.front_camera_controller import FrontCamera
from rov26backend.controllers.bottom_camera_controller import BottomCamera
from rov26backend.controllers.joystick_controller import PxnP5JoystickLinux

if sys.platform == "win32":
    from rov26backend.controllers.joystick_windows import PxnP5JoystickWindows

from rov26backend.controllers.rc_mixer import ROV26RcMixer
from rov26backend.controllers.px4_controller import PixhawkController
from rov26backend.controllers.rov26autonomous import Rov26Autonomous
from rov26backend.config import args, log_listener

from rov26backend.models.input_state import InputState
from rov26backend.models.control_state import ControlState
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState

# Standard libraries
import time
import concurrent.futures
import logging

logger = logging.getLogger("ROV.main")


def main():
    auto_event = threading.Event()

    input_state = InputState()
    control_state = ControlState()
    telemetry_state = TelemetryState()
    vision_state = VisionState()

    # Hardware controllers (initialized as None to manage shutdowns cleanly)
    joystick = None
    rc_mixer = None
    mikon = None
    front_camera = None
    bottom_camera = None
    server = None

    # --- 1. Joystick Initialization ---
    if not args.no_joystick:
        if sys.platform == "linux":
            joystick = PxnP5JoystickLinux(input_state)
        else:
            joystick = PxnP5JoystickWindows(input_state)
        joystick.start()

    # --- 2. RC Mixer Initialization ---
    if not args.no_mixer:
        mixer_kwargs = {
            key.replace("mixer_", ""): value
            for key, value in vars(args).items()
            if key.startswith("mixer_")
        }

        rc_mixer = ROV26RcMixer(input_state, control_state, auto_event, **mixer_kwargs)
        rc_mixer.start()

    autonomous = Rov26Autonomous(control_state, vision_state, auto_event, args)
    autonomous.start()

    # --- 3. Mikon / Pixhawk Initialization ---
    if not args.no_mikon:
        mikon = PixhawkController(control_state, telemetry_state, auto_event)
        mikon.start()

    # --- 4. Front Camera Initialization ---
    if not args.no_front_cam:
        front_cam_kwargs = {
            key.replace("front_cam_", "camera_"): value
            for key, value in vars(args).items()
            if key.startswith("front_cam_")
        }
        front_camera = FrontCamera(vision_state, auto_event, **front_cam_kwargs)
        front_camera.start()

    # --- 5. Bottom Camera Initialization ---
    if not args.no_bottom_cam:
        bottom_cam_kwargs = {
            key.replace("bottom_cam_", "camera_"): value
            for key, value in vars(args).items()
            if key.startswith("bottom_cam_")
        }
        bottom_camera = BottomCamera(**bottom_cam_kwargs)
        bottom_camera.start()

    # --- 6. gRPC Server Initialization ---
    if not args.no_grpc:
        grpc_port = args.grpc_port if hasattr(args, "grpc_port") else 50051

        server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))
        add_ServerServicer_to_server(
            RosGrpcServicer(telemetry_state, vision_state), server
        )
        server.add_insecure_port(f"[::]:{grpc_port}")
        server.start()
        logger.info(f"gRPC Server running on port {grpc_port}.")

    logger.info("Main script active. Press Ctrl+C to stop.")
    auto_event.set()

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

        logger.info("Waiting for threads to exit...")
        logger.info("All threads stopped. Goodbye.")
        log_listener.stop()


if __name__ == "__main__":
    main()
