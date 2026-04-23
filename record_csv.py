"""
Reads sensor CSV rows from the ESP32 serial port and saves them to a
timestamped file: data_plant_<YYYYMMDD_HHMMSS>.csv

Usage:
    python record_csv.py              # auto-detects first available COM port
    python record_csv.py COM4         # explicit port
    python record_csv.py COM4 115200  # explicit port + baud rate
"""

import sys
import datetime
import pathlib

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found. Install it with:  pip install pyserial")
    sys.exit(1)


DEFAULT_BAUD = 115200


def detect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Connect the ESP32 and try again.")
    if len(ports) == 1:
        return ports[0].device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} – {p.description}")
    choice = input("Select port number: ").strip()
    return ports[int(choice)].device


def make_csv_path() -> pathlib.Path:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return pathlib.Path(f"data_plant_{ts}.csv")


def record(port: str, baud: int, out_path: pathlib.Path) -> None:
    print(f"Opening {port} at {baud} baud")
    print(f"Writing to {out_path}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(port, baud, timeout=2) as ser, open(out_path, "w", newline="", encoding="utf-8") as csv_file:
        header_written = False
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue

            # Skip ESP-IDF log lines (start with I/W/E/D followed by ' (')
            if len(line) > 1 and line[0] in "IWED" and " (" in line[:20]:
                continue

            # Skip blank lines
            if not line:
                continue

            # The first non-log line should be the CSV header
            if not header_written and line.startswith("timestamp_epoch"):
                csv_file.write(line + "\n")
                csv_file.flush()
                print(line)
                header_written = True
                continue

            if header_written:
                csv_file.write(line + "\n")
                csv_file.flush()
                print(line)


def main() -> None:
    args = sys.argv[1:]
    port = args[0] if len(args) >= 1 else detect_port()
    baud = int(args[1]) if len(args) >= 2 else DEFAULT_BAUD
    out_path = make_csv_path()

    try:
        record(port, baud, out_path)
    except KeyboardInterrupt:
        print(f"\nStopped. Data saved to {out_path}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
