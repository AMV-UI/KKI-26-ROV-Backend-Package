from rov26backend.controllers.px4_controller import PixhawkController
from rov26backend.controllers.joystick_controller import JoystickController
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState
from rov26backend.models.control_state import ControlState
from rov26backend.controllers.gcs_controller import RosGrpcServicer
import time
import threading
import grpc
import concurrent.futures
from rov26backend.generated.server_pb2_grpc import add_ServerServicer_to_server


def joystick_loop(joystick):  # cuz the stupid one from inputs blocks :(
    print("Joystick thread started.")
    while True:
        joystick.monitor()


def control_loop(
    px4_controller: PixhawkController, joystick, control_state, telemetry_state
):
    print("Control loop started.")

    hz = 20
    period = 1.0 / hz

    while True:
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

        toggle_arm = latest_control_state["toggle_arm"]
        if toggle_arm:
            if latest_telemetry_state["armed"]:
                px4_controller._px_disarm(block=False)
            else:
                px4_controller._px_arm(block=False)
            control_state.update(arm_toggle=False)

        # 3. Sleep for remainder of the period to maintain steady loop rate
        elapsed = time.time() - start_time
        time.sleep(max(0, period - elapsed))


def main():
    joystick_controller = JoystickController()
    px4_controller = PixhawkController()
    telemetry_state = TelemetryState()
    vision_state = VisionState()
    control_state = ControlState()

    control_thread = threading.Thread(
        target=control_loop,
        args=(px4_controller, joystick_controller, control_state, telemetry_state),
    )
    joystick_thread = threading.Thread(
        target=joystick_loop, args=(joystick_controller,), daemon=False
    )
    joystick_thread.start()
    control_thread.start()

    server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))
    add_ServerServicer_to_server(RosGrpcServicer(telemetry_state, vision_state), server)
    server.add_insecure_port("[::]:50051")
    server.start()

    print("Server running. Press Ctrl+C to stop.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
