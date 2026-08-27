import argparse
import os
import logging
from logging.handlers import QueueHandler, QueueListener
import queue


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
    parser.add_argument("--forward-kp", type=float, default=0.0, help="Forward PID Kp")
    parser.add_argument("--forward-ki", type=float, default=0.0, help="Forward PID Ki")
    parser.add_argument("--forward-kd", type=float, default=0.0, help="Forward PID Kd")
    parser.add_argument(
        "--forward-deadzone",
        type=float,
        default=0.5,
        help="Forward Deadzone",
    )

    # Lateral
    parser.add_argument("--lateral-kp", type=float, default=0.0, help="Lateral PID Kp")
    parser.add_argument("--lateral-ki", type=float, default=0.0, help="Lateral PID Ki")
    parser.add_argument("--lateral-kd", type=float, default=0.0, help="Lateral PID Kd")
    parser.add_argument(
        "--lateral-deadzone",
        type=float,
        default=0.5,
        help="Lateral Deadzone",
    )

    # Vertical
    parser.add_argument(
        "--vertical-kp", type=float, default=0.0, help="Vertical PID Kp"
    )
    parser.add_argument(
        "--vertical-ki", type=float, default=0.0, help="Vertical PID Ki"
    )
    parser.add_argument(
        "--vertical-kd", type=float, default=0.5, help="Vertical PID Kd"
    )
    parser.add_argument(
        "--vertical-deadzone",
        type=float,
        default=0.5,
        help="Vertical Deadzone",
    )

    # Yaw
    parser.add_argument("--yaw-kp", type=float, default=0.0, help="Yaw PID Kp")
    parser.add_argument("--yaw-ki", type=float, default=0.0, help="Yaw PID Ki")
    parser.add_argument("--yaw-kd", type=float, default=0.0, help="Yaw PID Kd")
    parser.add_argument("--yaw-deadzone", type=float, default=0.5, help="Yaw Deadzone")

    return parser.parse_args()


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


args = parse_arguments()
log_obj, log_listener = setup_logging()
