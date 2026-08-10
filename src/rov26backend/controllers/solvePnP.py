from pymavlink import mavutil
import logging
import serial.tools.list_ports
import sys

logger = logging.getLogger("ROV.px4")


class solvePnP:
    