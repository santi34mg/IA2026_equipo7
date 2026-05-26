"""
Interactive WiFi setup for the ESP32 plant monitor over USB serial.

Workflow:
  1. Open the ESP32's serial port (auto-detected, or pass it explicitly).
  2. Confirm the ESP32 is alive (waits for a sensor row or boot log line).
  3. Prompt for SSID and password (password is hidden).
  4. Send WIFI:<ssid>:<password> and wait for a deterministic response.
  5. Report success (with the assigned IP) or a human-readable failure.

Usage:
    python configure_wifi.py            # auto-detect COM port
    python configure_wifi.py COM7       # use a specific port
"""

import datetime
import getpass
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found. Install it with:  pip install pyserial")
    sys.exit(1)


DEFAULT_BAUD = 115200
LIVENESS_TIMEOUT_S = 6.0
RESPONSE_TIMEOUT_S = 25.0


def detect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Connect the ESP32 and try again.")
    if len(ports) == 1:
        return ports[0].device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    choice = input("Select port number: ").strip()
    return ports[int(choice)].device


def read_line(ser: serial.Serial) -> str | None:
    raw = ser.readline()
    if not raw:
        return None
    try:
        return raw.decode("utf-8", errors="replace").rstrip("\r\n")
    except Exception:
        return None


def is_log_line(line: str) -> bool:
    return len(line) > 1 and line[0] in "IWED" and " (" in line[:20]


def looks_like_sensor_row(line: str) -> bool:
    parts = line.split(",")
    if len(parts) < 2:
        return False
    try:
        float(parts[0])
        return True
    except ValueError:
        return False


def check_alive(ser: serial.Serial, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if is_log_line(line) or looks_like_sensor_row(line):
            return True
    return False


def send_credentials(ser: serial.Serial, ssid: str, password: str) -> None:
    payload = f"WIFI:{ssid}:{password}\n".encode("utf-8")
    ser.write(payload)
    ser.flush()


HUMAN_REASONS = {
    "auth": 'Authentication failed (wrong password?)',
    "not_found": 'Network not found',
    "timeout": 'Timed out trying to associate',
    "other": 'Connection failed (other error)',
}


def wait_for_result(ser: serial.Serial, ssid: str, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if not line.startswith("[wifi_cfg] "):
            continue

        body = line[len("[wifi_cfg] "):]
        if body.startswith("CONNECT_OK"):
            ip = ""
            joined_ssid = ""
            for token in body.split():
                if token.startswith("ip="):
                    ip = token[len("ip="):]
                elif token.startswith("ssid="):
                    joined_ssid = token[len("ssid="):]
            if joined_ssid:
                print(f'\n[OK] Connected to "{joined_ssid}" - IP {ip}')
            else:
                print(f"\n[OK] Connected - IP {ip}")
            print(f"     Next step: python record_csv_wifi.py {ip}")
            return 0

        if body.startswith("CONNECT_FAIL"):
            reason = "other"
            for token in body.split():
                if token.startswith("reason="):
                    reason = token[len("reason="):]
                    break
            msg = HUMAN_REASONS.get(reason, f"Connection failed ({reason})")
            if reason == "not_found":
                msg = f'Network "{ssid}" not found'
            print(f"\n[FAIL] {msg}")
            return 2

        if body.startswith("PARSE_ERROR"):
            print("\n[FAIL] ESP32 could not parse credentials")
            return 3

    print(f"\n[FAIL] No response from ESP32 within {int(timeout_s)}s")
    return 4


def main() -> int:
    args = sys.argv[1:]
    port = args[0] if args else detect_port()

    print(f"Opening {port} at {DEFAULT_BAUD} baud")
    try:
        ser = serial.Serial(port, DEFAULT_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1

    with ser:
        print(f"Checking the ESP32 is alive (up to {LIVENESS_TIMEOUT_S:.0f}s)...")
        if not check_alive(ser, LIVENESS_TIMEOUT_S):
            print(f"[FAIL] No data from ESP32 on {port} - is it powered and flashed?")
            return 1
        print("ESP32 is alive.\n")

        ssid = input("SSID: ").strip()
        if not ssid:
            print("[FAIL] SSID cannot be empty")
            return 1
        password = getpass.getpass("Password: ")

        print(f"\nSending credentials and waiting up to {RESPONSE_TIMEOUT_S:.0f}s for result...")
        sent_at = datetime.datetime.now().strftime("%H:%M:%S")
        send_credentials(ser, ssid, password)
        print(f"[{sent_at}] sent WIFI:{ssid}:****")

        return wait_for_result(ser, ssid, RESPONSE_TIMEOUT_S)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(130)
