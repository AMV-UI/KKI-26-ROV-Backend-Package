import logging
import os
import queue
from logging.handlers import QueueHandler, QueueListener


def setup_logging():
    """Sets up the asynchronous logging architecture."""
    log_queue = queue.Queue()

    os.remove("rov_mixer.log")

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


log_obj, log_listener = setup_logging()
