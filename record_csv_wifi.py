"""
Reads sensor CSV rows from the ESP32 over WiFi (TCP) and saves them to a
timestamped file: data_plant_<YYYYMMDD_HHMMSS>.csv

The ESP32 must already be connected to WiFi. To configure it the first time
or to switch networks, run configure_wifi.py over USB serial first.

Usage:
    python record_csv_wifi.py                          # auto-discover via UDP
    python record_csv_wifi.py Decaido                  # auto-discover, estado="Decaido"
    python record_csv_wifi.py 192.168.1.42             # direct IP, no estado
    python record_csv_wifi.py 192.168.1.42 Decaido     # direct IP, estado="Decaido"
"""

import datetime
import pathlib
import re
import socket
import sys


DEFAULT_TCP_PORT = 8080
DISCOVERY_PORT = 8081
CONNECT_TIMEOUT_S = 10
DISCOVERY_TIMEOUT_S = 10.0

IP_REGEX = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")


def make_csv_path() -> pathlib.Path:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return pathlib.Path(f"data_plant_{ts}.csv")


def discover(timeout_s: float = DISCOVERY_TIMEOUT_S) -> tuple[str, int]:
    """Listen for the ESP32's UDP broadcast and return (ip, tcp_port)."""
    print(f"[discovery] Listening for ESP32 on UDP {DISCOVERY_PORT} (timeout {timeout_s:.0f}s)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
    except OSError as e:
        raise RuntimeError(f"Could not bind UDP {DISCOVERY_PORT}: {e}") from e

    sock.settimeout(timeout_s)
    try:
        while True:
            try:
                data, addr = sock.recvfrom(128)
            except socket.timeout:
                raise RuntimeError(
                    "Could not find ESP32 on the LAN. "
                    "Is it powered and connected? Try configure_wifi.py over USB."
                )
            payload = data.decode("utf-8", errors="replace").strip()
            if not payload.startswith("PLANT_MONITOR:"):
                continue
            parts = payload.split(":")
            if len(parts) < 3:
                continue
            ip = parts[1].strip()
            try:
                port = int(parts[2].strip())
            except ValueError:
                continue
            print(f"[discovery] Found ESP32 at {ip}:{port}")
            return ip, port
    finally:
        sock.close()


def record(host: str, port: int, out_path: pathlib.Path, estado: str | None = None) -> None:
    print(f"Connecting to {host}:{port}")
    print(f"Writing to {out_path}")
    if estado:
        print(f"Estado: {estado}")
    print("Press Ctrl+C to stop.\n")

    sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT_S)
    sock.settimeout(None)
    try:
        stream = sock.makefile("r", encoding="utf-8", errors="replace", newline="\n")
        with open(out_path, "w", newline="", encoding="utf-8") as csv_file:
            # El ESP32 no manda header de texto sobre TCP, lo hardcodeamos
            columns = "timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity"
            header = f"{columns},estado\n" if estado else f"{columns}\n"
            csv_file.write(header)
            csv_file.flush()
            print(f"[header] {columns}")

            row_count = 0
            t0 = None
            uptime0 = None
            for raw in stream:
                line = raw.rstrip("\r\n")

                # Skip ESP-IDF log lines (start with I/W/E/D followed by ' (')
                if len(line) > 1 and line[0] in "IWED" and " (" in line[:20]:
                    print(f"[esp32] {line}")
                    continue

                # Skip blank lines and any [wifi_cfg] chatter that might leak
                if not line or line.startswith("[wifi_cfg]"):
                    continue

                # Reemplaza el uptime/epoch del ESP32 por un datetime real
                parts = line.split(",")
                try:
                    uptime = float(parts[0])
                    if t0 is None:
                        t0 = datetime.datetime.now()
                        uptime0 = uptime
                    ts = t0 + datetime.timedelta(seconds=(uptime - uptime0))
                    parts[0] = ts.strftime("%Y-%m-%d %H:%M:%S")
                    line = ",".join(parts)
                except (ValueError, IndexError):
                    pass

                row = f"{line},{estado}\n" if estado else f"{line}\n"
                csv_file.write(row)
                csv_file.flush()
                row_count += 1
                print(f"[row {row_count:>4}] {line}")
    finally:
        try:
            sock.close()
        except OSError:
            pass


def parse_args(argv: list[str]) -> tuple[str | None, str | None]:
    """Returns (ip_or_None, estado_or_None) following the positional rules."""
    if not argv:
        return None, None
    if IP_REGEX.match(argv[0]):
        ip = argv[0]
        estado = argv[1] if len(argv) >= 2 else None
        return ip, estado
    # First arg isn't an IP, treat it as estado and discover
    return None, argv[0]


def main() -> None:
    ip, estado = parse_args(sys.argv[1:])
    port = DEFAULT_TCP_PORT

    try:
        if ip is None:
            ip, port = discover()
        out_path = make_csv_path()
        record(ip, port, out_path, estado)
    except KeyboardInterrupt:
        print("\nStopped.")
    except RuntimeError as e:
        print(str(e))
        sys.exit(1)
    except (socket.timeout, TimeoutError):
        print(f"Timed out connecting to {ip}:{port}. Is the ESP32 on the network?")
        sys.exit(1)
    except (ConnectionRefusedError, ConnectionResetError, OSError) as e:
        print(f"Connection error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
