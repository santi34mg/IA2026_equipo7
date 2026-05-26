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

from common.parser import ParserState, parse_serial_line

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
        # El ESP32 no manda header de texto, lo hardcodeamos
        columns = "timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity"
        header = f"{columns},estado\n" if estado else f"{columns}\n"
        csv_file.write(header)
        csv_file.flush()
        print(f"[header] {columns}")

        row_count = 0
        state = ParserState()
        while True:
            raw = ser.readline()
            if not raw:
                continue

            row = parse_serial_line(raw, state, estado)
            if row is None:
                # Log ESP-IDF lines to console so operator can see them
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                    if line and line[0] in "IWED" and " (" in line[:20]:
                        print(f"[esp32] {line}")
                except Exception:
                    pass
                continue

            csv_file.write(row + "\n")
            csv_file.flush()
            row_count += 1
            print(f"[row {row_count:>4}] {row}")


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
