import time


from rov26backend.generated.server_pb2 import telemetryRequest, telemetryResponse
from rov26backend.generated.server_pb2_grpc import ServerServicer
from rov26backend.models.telemetry_state import TelemetryState
from rov26backend.models.vision_state import VisionState
import logging

logger = logging.getLogger("ROV.gRPC")


class RosGrpcServicer(ServerServicer):
    def __init__(self, telemetry_state: TelemetryState, vision_state: VisionState):
        self.telemetry_state = telemetry_state
        self.vision_state = vision_state

    def getTelemetry(self, request: telemetryRequest, context):
        """This runs in a gRPC worker thread"""
        try:
            while context.is_active():
                latest_tel = self.telemetry_state.get_latest()
                latest_vis = self.vision_state.get_latest()
                latest_data = latest_tel | latest_vis
                # if qr_side_msg is None or qr_side_msg.data not in ["A", "B", "C", "D"]:
                #     qr = "NOT_FOUND"
                # else:
                #     qr = qr_side_msg.data
                logger.debug(latest_data)

                response = telemetryResponse(**latest_data)

                yield response

                time.sleep(0.1)
        except Exception as e:
            logger.warn(f"Error servicing client: {e}")
