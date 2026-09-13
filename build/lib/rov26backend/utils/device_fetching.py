import sys
import logging

if sys.platform != "win32":
    import pyudev

logger = logging.getLogger("ROV.devi")


def get_webcam_device_idx(target_serial):
    if sys.platform == "win32":
        return _get_webcam_windows(target_serial)
    else:
        return _get_webcam_linux(target_serial)


def _get_webcam_windows(target_serial):
    import subprocess
    
    # We use PowerShell to natively run the WMI query without needing the pywin32 library
    cmd = [
        "powershell",
        "-NoProfile",
        "-Command",
        'Get-WmiObject -Query "SELECT PNPDeviceID FROM Win32_PnPEntity WHERE (PNPClass=\'Camera\' OR PNPClass=\'Image\') AND PNPDeviceID LIKE \'USB%\'" | Select-Object -ExpandProperty PNPDeviceID'
    ]

    try:
        # CREATE_NO_WINDOW (0x08000000) prevents a black console window from flashing on screen
        output = subprocess.check_output(cmd, text=True, creationflags=0x08000000)
        
        # Clean up the output and sort it exactly like the previous wmi implementation
        device_ids = [line.strip() for line in output.splitlines() if line.strip()]
        device_ids = sorted(device_ids)

        for index, pnp_id in enumerate(device_ids):
            logger.info(f"{index} {pnp_id}")
            
            hardware_serial = pnp_id.split("\\")[-1]

            if hardware_serial.lower() == target_serial.lower():
                return index

    except Exception as e:
        logger.error(f"Failed to fetch Windows webcams via PowerShell: {e}")

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

if __name__ == "__main__":
    _get_webcam_windows("dsdsds")