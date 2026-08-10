import os

# 1. Force pure-Python Protobuf
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"


# 3. NOW load OpenCV/Vision modules
from rov26backend.models.vision_state import VisionState
from rov26backend.controllers.front_camera_controller import FrontCamera
from rov26backend.controllers.bottom_camera_controller import BottomCamera

# 4. THEN load gRPC/Protobuf
import grpc
from rov26backend.controllers.gcs_controller import RosGrpcServicer
from rov26backend.generated.server_pb2_grpc import add_ServerServicer_to_server

# 5. Load the rest of your hardware controllers
from rov26backend.controllers.px4_controller import PixhawkController
from rov26backend.controllers.joystick_controller import JoystickController
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.control_state import ControlState

# 4. Standard libraries
import time
import threading
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
        respect_handler_level=True,
    )
    listener.start()

    logger = logging.getLogger("ROV")
    logger.setLevel(logging.DEBUG)
    logger.addHandler(QueueHandler(log_queue))

    logger.propagate = False

    return logger, listener


def joystick_loop(joystick, shutdown_event):
    """Runs the joystick polling loop."""
    threading.current_thread().name = "Joystick"
    logger.info("Joystick thread started.")

    while not shutdown_event.is_set():
        joystick.monitor()


def control_loop(
    px4_controller: PixhawkController,
    joystick,
    control_state,
    telemetry_state,
    shutdown_event,
):
    """Runs the Pixhawk communication loop."""
    threading.current_thread().name = "Control"
    logger.info("Control loop started.")

    hz = 20
    period = 1.0 / hz

    while not shutdown_event.is_set():
        if px4_controller.master is None:
            px4_controller._init_serial()
            time.sleep(1)
            continue

        start_time = time.time()

        joystick.update_smoothed_controls(control_state)
        latest_control_state = control_state.get_latest()

        px4_controller.rc_channels_override_send(
            1500,  # CH1
            1500,  # CH2
            latest_control_state["vertical"],  # CH3
            latest_control_state["yaw"],  # CH4
            latest_control_state["forward"],  # CH5
            latest_control_state["lateral"],  # CH6
            0,  # CH7
            latest_control_state["servo"],
        )

        target_mode = latest_control_state["target_mode"]
        if target_mode is not None:
            px4_controller.set_mode(target_mode)
            control_state.update(target_mode=None)

        px4_controller._pump_mavlink_messages()
        px4_controller.request_pixhawk_to_telemetry(telemetry_state)

        latest_telemetry_state = telemetry_state.get_latest()

        arm_toggle = latest_control_state["arm_toggle"]
        if arm_toggle:
            if latest_telemetry_state["armed"]:
                px4_controller.disarm(block=False)
            else:
                px4_controller.arm(block=False)
            control_state.update(arm_toggle=False)

        elapsed = time.time() - start_time
        time.sleep(max(0, period - elapsed))


def front_cam_loop(front_cam, shutdown_event):

    while not shutdown_event.is_set():
        front_cam.run()


def bottom_cam_loop(bottom_cam, shutdown_event):
    while not shutdown_event.is_set():
        bottom_cam.run()


def main():
    logger, log_listener = setup_logging()

    shutdown_event = threading.Event()

    telemetry_state = TelemetryState()
    vision_state = VisionState()

    joystick_controller = JoystickController()
    px4_controller = PixhawkController()
    front_cam = FrontCamera(vision_state)
    bottom_cam = BottomCamera()

    control_state = ControlState()

    control_thread = threading.Thread(
        target=control_loop,
        args=(
            px4_controller,
            joystick_controller,
            control_state,
            telemetry_state,
            shutdown_event,
        ),
        daemon=True,
    )
    joystick_thread = threading.Thread(
        target=joystick_loop, args=(joystick_controller, shutdown_event), daemon=True
    )

    front_cam_thread = threading.Thread(
        target=front_cam_loop, args=(front_cam, shutdown_event)
    )

    bottom_cam_thread = threading.Thread(
        target=bottom_cam_loop, args=(bottom_cam, shutdown_event)
    )

    joystick_thread.start()
    control_thread.start()
    front_cam_thread.start()
    bottom_cam_thread.start()

    # while True:
    #     joystick_controller.update_smoothed_controls(control_state)
    #     time.sleep(0.01)

    server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))
    add_ServerServicer_to_server(RosGrpcServicer(telemetry_state, vision_state), server)
    server.add_insecure_port("[::]:50051")
    server.start()

    logger.info("Server running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Ctrl+C detected! Shutting down ROV backend...")
    finally:
        shutdown_event.set()

        server.stop(grace=0)

        logger.info("Waiting for threads to exit...")
        if joystick_thread.is_alive():
            joystick_thread.join(timeout=2.0)
        if control_thread.is_alive():
            control_thread.join(timeout=2.0)
        if front_cam_thread.is_alive():
            front_cam_thread.join(timeout=2.0)
        if bottom_cam_thread.is_alive():
            bottom_cam_thread.join(timeout=2.0)

        logger.info("All threads stopped. Goodbye.")

        log_listener.stop()


if __name__ == "__main__":
    main()
