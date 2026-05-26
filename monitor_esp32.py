"""
Live serial monitor for the ESP32 plant monitor.

Prints every line the ESP32 emits over USB, with a timestamp and a short tag
classifying each line so you can spot WiFi events, errors and sensor rows at a
glance.

Usage:
    python monitor_esp32.py                 # auto-detect COM port
    python monitor_esp32.py COM7            # use a specific port
    python monitor_esp32.py --no-rows       # hide sensor rows (just logs/events)
    python monitor_esp32.py COM7 --no-rows  # combined
"""

import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found. Install:  pip install pyserial")
    sys.exit(1)


DEFAULT_BAUD = 115200


def _esp32_score(p) -> int:
    text = ((p.description or "") + " " + (p.hwid or "")).upper()
    score = 0
    if "CP210" in text:        score += 10
    if "CH340" in text:        score += 10
    if "CH341" in text:        score += 8
    if "SILICON LABS" in text: score += 8
    if "ESPRESSIF" in text:    score += 10
    if "10C4" in text and "EA60" in text: score += 5
    if "1A86" in text and "7523" in text: score += 5
    if "303A" in text:         score += 5
    return score


def detect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Connect the ESP32 and try again.")
    if len(ports) == 1:
        return ports[0].device
    ports.sort(key=_esp32_score, reverse=True)
    # If the best-scoring port stands out, use it without prompting.
    if _esp32_score(ports[0]) > 0:
        return ports[0].device
    print("Available ports (no clear ESP32 match):")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    choice = input("Select port number: ").strip()
    return ports[int(choice)].device


def classify(line: str) -> str:
    """Return a 4-char tag for visual grouping."""
    if line.startswith("[wifi_cfg]"):
        return "WIFI"
    if "Got IP" in line or line.startswith("I (") and "Online:" in line:
        return "NET "
    if line.startswith("E ("):
        return "ERR "
    if line.startswith("W ("):
        return "WARN"
    if line.startswith("I ("):
        return "INFO"
    if line.startswith("D ("):
        return "DBG "
    # Sensor row heuristic: 6+ comma-separated fields, first is a number.
    parts = line.split(",")
    if len(parts) >= 6:
        try:
            float(parts[0])
            return "ROW "
        except ValueError:
            pass
    return "    "


def main() -> None:
    args = sys.argv[1:]
    show_rows = True
    port = None
    for a in args:
        if a == "--no-rows":
            show_rows = False
        elif a.startswith("--"):
            print(f"Unknown flag: {a}")
            sys.exit(1)
        else:
            port = a

    if port is None:
        try:
            port = detect_port()
        except RuntimeError as e:
            print(str(e))
            sys.exit(1)

    print(f"Opening {port} at {DEFAULT_BAUD} baud")
    if not show_rows:
        print("(sensor rows hidden — pass without --no-rows to see them)")
    print("Press Ctrl+C to stop.\n")

    start = time.monotonic()
    try:
        with serial.Serial(port, DEFAULT_BAUD, timeout=1) as ser:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                except Exception:
                    continue
                if not line:
                    continue

                tag = classify(line)
                if tag == "ROW " and not show_rows:
                    continue

                elapsed = time.monotonic() - start
                print(f"[{elapsed:7.2f}s] [{tag}] {line}")
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
