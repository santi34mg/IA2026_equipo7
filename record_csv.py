"""
Reads sensor CSV rows from the ESP32 serial port and saves them to a
timestamped file: data_plant_<YYYYMMDD_HHMMSS>.csv

Each times it gets a row, it gets the current time to save it into it

Usage:
    python record_csv.py            # auto-detects port, no estado column
    python record_csv.py estado      # auto-detects port, appends ,estado column with value "riego"
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


def record(port: str, baud: int, out_path: pathlib.Path, estado: str | None = None) -> None:
    print(f"Opening {port} at {baud} baud")
    print(f"Writing to {out_path}")
    if estado:
        print(f"Estado: {estado}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(port, baud, timeout=2) as ser, open(out_path, "w", newline="", encoding="utf-8") as csv_file:
        header_written = False
        row_count = 0
        t0 = None
        uptime0 = None
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
                print(f"[esp32] {line}")
                continue

            # Skip blank lines
            if not line:
                continue

            # The first non-log line is treated as the CSV header
            if not header_written:
                header = f"timestamp,{line},estado\n" if estado else f"timestamp,{line}\n"
                csv_file.write(header)
                csv_file.flush()
                print(f"[header] {line}")
                header_written = True
                continue

            if header_written:
                # Reemplaza el uptime del ESP32 por un datetime real
                parts = line.split(",")
                try:
                    uptime = float(parts[0])
                    if t0 is None:
                        t0 = datetime.datetime.now()
                        uptime0 = uptime
                    ts = t0 + datetime.timedelta(seconds=(uptime - uptime0))
                    ts_str = ts.strftime("%Y-%m-%d %H:%M:%S")
                    parts[0] = ts_str
                    line = ",".join(parts)
                except (ValueError, IndexError):
                    pass

                row = f"{line},{estado}\n" if estado else f"{line}\n"
                csv_file.write(row)
                csv_file.flush()
                row_count += 1
                print(f"[row {row_count:>4}] {line}")


def main() -> None:
    args = sys.argv[1:]
    port = detect_port()
    baud = DEFAULT_BAUD
    estado = args[0] if len(args) >= 1 else None
    out_path = make_csv_path()

    try:
        record(port, baud, out_path, estado)
    except KeyboardInterrupt:
        print(f"\nStopped. Data saved to {out_path}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()