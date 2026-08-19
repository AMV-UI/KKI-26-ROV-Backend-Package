import os
import sys
import argparse
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

from rov26backend.models.input_state import InputState
from rov26backend.models.control_state import ControlState
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState

# Standard libraries
import time
import concurrent.futures
import logging
from logging.handlers import QueueHandler, QueueListener
import queue

logger = logging.getLogger("ROV.main")


def setup_logging():
    """Sets up the asynchronous logging architecture."""
    log_queue = queue.Queue()

    # --- Console Handler (INFO only) ---
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter("%(levelname)s - %(message)s")
    console_handler.setFormatter(console_formatter)

    file_formatter = logging.Formatter(
        "%(asctime)s - %(threadName)s - %(levelname)s - %(message)s"
    )

    px4_handler = logging.FileHandler(os.path.abspath("rov_pixhawk.log"))
    px4_handler.setLevel(logging.DEBUG)
    px4_handler.setFormatter(file_formatter)
    px4_handler.addFilter(logging.Filter("ROV.px4"))

    joy_handler = logging.FileHandler(os.path.abspath("rov_joystick.log"))
    joy_handler.setLevel(logging.DEBUG)
    joy_handler.setFormatter(file_formatter)
    joy_handler.addFilter(logging.Filter("ROV.joystick"))

    mixer_handler = logging.FileHandler(os.path.abspath("rov_mixer.log"))
    mixer_handler.setLevel(logging.DEBUG)
    mixer_handler.setFormatter(file_formatter)
    mixer_handler.addFilter(logging.Filter("ROV.mixer"))

    vision_handler = logging.FileHandler(os.path.abspath("rov_vision.log"))
    vision_handler.setLevel(logging.DEBUG)
    vision_handler.setFormatter(file_formatter)
    vision_handler.addFilter(logging.Filter("ROV.vision"))

    auto_handler = logging.FileHandler(os.path.abspath("rov_auto.log"))
    auto_handler.setLevel(logging.DEBUG)
    auto_handler.setFormatter(file_formatter)
    auto_handler.addFilter(logging.Filter("ROV.auto"))

    grpc_handler = logging.FileHandler(os.path.abspath("rov_grpc.log"))
    grpc_handler.setLevel(logging.DEBUG)
    grpc_handler.setFormatter(file_formatter)
    grpc_handler.addFilter(logging.Filter("ROV.gRPC"))

    listener = QueueListener(
        log_queue,
        console_handler,
        px4_handler,
        joy_handler,
        grpc_handler,
        mixer_handler,
        vision_handler,
        auto_handler,
        respect_handler_level=True,
    )
    listener.start()

    log_obj = logging.getLogger("ROV")
    log_obj.setLevel(logging.DEBUG)
    log_obj.addHandler(QueueHandler(log_queue))
    log_obj.propagate = False

    return log_obj, listener


def parse_arguments():
    """Parses CLI arguments dynamically using argparse.SUPPRESS to respect default kwargs."""
    parser = argparse.ArgumentParser(description="ROV26 Backend Main Node")

    # Toggle Flags (Enable / Disable components)
    parser.add_argument(
        "--no-joystick", action="store_true", help="Disable the Joystick thread"
    )
    parser.add_argument(
        "--no-mixer", action="store_true", help="Disable the RC Mixer thread"
    )
    parser.add_argument(
        "--no-mikon", action="store_true", help="Disable the Pixhawk (Mikon) thread"
    )
    parser.add_argument(
        "--no-front-cam", action="store_true", help="Disable the Front Camera thread"
    )
    parser.add_argument(
        "--no-bottom-cam", action="store_true", help="Disable the Bottom Camera thread"
    )
    parser.add_argument(
        "--no-grpc", action="store_true", help="Disable the gRPC Server"
    )

    # Mixer Config (Using SUPPRESS so the object's constructor defaults take over if not specified)
    parser.add_argument(
        "--mixer-smoothing-factor",
        type=float,
        default=argparse.SUPPRESS,
        help="RC Mixer smoothing factor for low pass filter",
    )
    parser.add_argument(
        "--mixer-servo-open",
        type=float,
        default=argparse.SUPPRESS,
        help="RC Mixer pwm for opening servo",
    )
    parser.add_argument(
        "--mixer-servo-close",
        type=float,
        default=argparse.SUPPRESS,
        help="RC Mixer pwm for closing servo",
    )
    parser.add_argument(
        "--mixer-pwm-center",
        type=int,
        default=argparse.SUPPRESS,
        help="RC Mixer PWM center value",
    )
    parser.add_argument(
        "--mixer-pwm-range",
        type=int,
        default=argparse.SUPPRESS,
        help="RC Mixer PWM range",
    )
    parser.add_argument(
        "--mixer-pwm-min",
        type=int,
        default=argparse.SUPPRESS,
        help="RC Mixer PWM minimum",
    )
    parser.add_argument(
        "--mixer-pwm-max",
        type=int,
        default=argparse.SUPPRESS,
        help="RC Mixer PWM maximum",
    )
    parser.add_argument(
        "--mixer-max-slew",
        type=int,
        default=argparse.SUPPRESS,
        help="RC Mixer Max slew per second",
    )

    # Vision Config
    parser.add_argument(
        "--front-cam-id",
        type=str,
        default=argparse.SUPPRESS,
        help="Device ID/Path for Front Camera",
    )
    parser.add_argument(
        "--bottom-cam-id",
        type=str,
        default=argparse.SUPPRESS,
        help="Device ID/Path for Bottom Camera",
    )

    # Server Config
    parser.add_argument(
        "--grpc-port",
        type=int,
        default=argparse.SUPPRESS,
        help="Port to run the gRPC server on",
    )

    # ==========================================
    # Autonomous Target Config
    # ==========================================
    parser.add_argument(
        "--target-x", type=float, default=0.0, help="Target X coordinate"
    )
    parser.add_argument(
        "--target-y", type=float, default=0.0, help="Target Y coordinate"
    )
    parser.add_argument(
        "--target-z", type=float, default=0.0, help="Target Z coordinate"
    )
    parser.add_argument(
        "--target-yaw", type=float, default=0.0, help="Target Yaw angle"
    )

    # ==========================================
    # Autonomous PID Configs
    # ==========================================
    # Forward
    parser.add_argument(
        "--forward-kp", type=float, default=argparse.SUPPRESS, help="Forward PID Kp"
    )
    parser.add_argument(
        "--forward-ki", type=float, default=argparse.SUPPRESS, help="Forward PID Ki"
    )
    parser.add_argument(
        "--forward-kd", type=float, default=argparse.SUPPRESS, help="Forward PID Kd"
    )
    parser.add_argument(
        "--forward-deadzone",
        type=float,
        default=argparse.SUPPRESS,
        help="Forward Deadzone",
    )

    # Lateral
    parser.add_argument(
        "--lateral-kp", type=float, default=argparse.SUPPRESS, help="Lateral PID Kp"
    )
    parser.add_argument(
        "--lateral-ki", type=float, default=argparse.SUPPRESS, help="Lateral PID Ki"
    )
    parser.add_argument(
        "--lateral-kd", type=float, default=argparse.SUPPRESS, help="Lateral PID Kd"
    )
    parser.add_argument(
        "--lateral-deadzone",
        type=float,
        default=argparse.SUPPRESS,
        help="Lateral Deadzone",
    )

    # Vertical
    parser.add_argument(
        "--vertical-kp", type=float, default=argparse.SUPPRESS, help="Vertical PID Kp"
    )
    parser.add_argument(
        "--vertical-ki", type=float, default=argparse.SUPPRESS, help="Vertical PID Ki"
    )
    parser.add_argument(
        "--vertical-kd", type=float, default=argparse.SUPPRESS, help="Vertical PID Kd"
    )
    parser.add_argument(
        "--vertical-deadzone",
        type=float,
        default=argparse.SUPPRESS,
        help="Vertical Deadzone",
    )

    # Yaw
    parser.add_argument(
        "--yaw-kp", type=float, default=argparse.SUPPRESS, help="Yaw PID Kp"
    )
    parser.add_argument(
        "--yaw-ki", type=float, default=argparse.SUPPRESS, help="Yaw PID Ki"
    )
    parser.add_argument(
        "--yaw-kd", type=float, default=argparse.SUPPRESS, help="Yaw PID Kd"
    )
    parser.add_argument(
        "--yaw-deadzone", type=float, default=argparse.SUPPRESS, help="Yaw Deadzone"
    )

    return parser.parse_args()


def main():
    logger, log_listener = setup_logging()
    args = parse_arguments()

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

        rc_mixer = ROV26RcMixer(input_state, control_state, **mixer_kwargs)
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
        front_camera = FrontCamera(vision_state, **front_cam_kwargs)
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
