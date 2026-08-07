from rov26backend.controllers.px4_controller import PixhawkController
from rov26backend.controllers.joystick_controller import JoystickController
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState
from rov26backend.models.control_state import ControlState
from rov26backend.controllers.gcs_controller import RosGrpcServicer
from rov26backend.generated.server_pb2_grpc import add_ServerServicer_to_server

import time
import threading
import grpc
import concurrent.futures
import logging
from logging.handlers import QueueHandler, QueueListener
import queue

logger = logging.getLogger("ROV.main")


def setup_logging():
    """Sets up the asynchronous logging architecture."""
    log_queue = queue.Queue()

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter("%(levelname)s - %(message)s")
    console_handler.setFormatter(console_formatter)

    file_handler = logging.FileHandler("rov_telemetry.log")
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter(
        "%(asctime)s - %(threadName)s - %(levelname)s - %(message)s"
    )
    file_handler.setFormatter(file_formatter)

    listener = QueueListener(log_queue, console_handler, file_handler)
    listener.start()

    logger = logging.getLogger("ROV")
    logger.setLevel(logging.DEBUG)
    logger.addHandler(QueueHandler(log_queue))

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
            px4_controller._px_set_mode(target_mode)
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


def main():
    logger, log_listener = setup_logging()

    shutdown_event = threading.Event()

    joystick_controller = JoystickController()
    px4_controller = PixhawkController()

    telemetry_state = TelemetryState()
    vision_state = VisionState()
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
    )
    joystick_thread = threading.Thread(
        target=joystick_loop, args=(joystick_controller, shutdown_event)
    )

    joystick_thread.start()
    control_thread.start()

    server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))
    add_ServerServicer_to_server(RosGrpcServicer(telemetry_state, vision_state), server)
    server.add_insecure_port("[::]:50051")
    server.start()

    logger.info("Server running. Press Ctrl+C to stop.")

    try:
        server.wait_for_termination()
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

        logger.info("All threads stopped. Goodbye.")

        log_listener.stop()


if __name__ == "__main__":
    main()
