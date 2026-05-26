import os

from .models import PortInfo


def list_serial_ports() -> list[PortInfo]:
    ports = []
    try:
        import serial.tools.list_ports
    except ImportError:
        pass
    else:
        ports.extend(
            PortInfo(device=p.device, description=p.description or "", hwid=p.hwid or "")
            for p in serial.tools.list_ports.comports()
        )
    return ports


def port_exists(device: str) -> bool:
    try:
        import serial.tools.list_ports
    except ImportError:
        return False
    return any(p.device == device for p in serial.tools.list_ports.comports())


def _mock_ports() -> list[PortInfo]:
    return [PortInfo(device="MOCK", description="Mock ESP32 (FRONTEND_MOCK=1)", hwid="USB\\VID_1A86&PID_7523")]
