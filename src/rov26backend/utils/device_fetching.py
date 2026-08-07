import sys
import logging

if sys.platform == "win32":
    import wmi
else:
    import pyudev

logger = logging.getLogger("ROV.devi")


def get_webcam_device_idx(target_serial):
    if sys.platform == "win32":
        return _get_webcam_windows(target_serial)
    else:
        return _get_webcam_linux(target_serial)


def _get_webcam_windows(target_serial):
    c = wmi.WMI()

    wql = "SELECT * FROM Win32_PnPEntity WHERE (PNPClass='Camera' OR PNPClass='Image') AND PNPDeviceID LIKE 'USB%'"
    cameras = c.query(wql)

    cameras = sorted(cameras, key=lambda x: x.PNPDeviceID)

    for index, cam in enumerate(cameras):
        logger.info(f"{index} {cam}")
        if cam.PNPDeviceID:
            hardware_serial = cam.PNPDeviceID.split("\\")[-1]

            if hardware_serial.lower() == target_serial.lower():
                return index

    return None


def _get_webcam_linux(target_serial):
    context = pyudev.Context()

    # Webcams reside in the 'video4linux' subsystem
    for device in context.list_devices(subsystem="video4linux"):
        # Get the device attributes safely
        serial = device.get("ID_SERIAL")

        # Check if IDs match (comparing as lowercase strings)
        if serial == target_serial:
            # OPTIONAL: Filter out metadata/index nodes.
            # Many cameras create two nodes (e.g., video0 and video1).
            # Usually, the 'capture' device is the one you want.
            capabilities = device.get("ID_V4L_CAPABILITIES", "")
            if ":capture:" in capabilities:
                return int(device.device_node[10:])

    return None
