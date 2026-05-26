import asyncio
import os
import socket
import sys
from collections.abc import Awaitable, Callable
from pathlib import Path

# common/ lives two levels up from frontend/app/
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from common.parser import ParserState, parse_serial_line

BAUD = 115200
CSV_HEADER = "timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity,estado\n"
TCP_CONNECT_TIMEOUT_S = 10.0


async def record_to_csv(
    port: str,
    out_path: Path,
    estado: str,
    on_row: Callable[[str, int], Awaitable[None]],
    cancel_event: asyncio.Event,
    first_data_timeout_s: float = 30.0,
) -> int:
    """Read serial data from the ESP32 and write rows to out_path.

    Blocks until cancel_event is set or a serial error occurs.
    Returns total rows written. The CSV file is always flushed/closed.
    """
    import serial

    loop = asyncio.get_running_loop()
    row_queue: asyncio.Queue[str | None] = asyncio.Queue()
    row_count = 0

    def _reader_thread() -> None:
        """Blocking serial reader — runs in a thread via asyncio.to_thread."""
        with serial.Serial(port, BAUD, timeout=2) as ser:
            parser_state = ParserState()
            got_first_row = False
            while not cancel_event.is_set():
                raw = ser.readline()
                if not raw:
                    continue
                row = parse_serial_line(raw, parser_state, estado)
                if row is None:
                    # Echo ESP-IDF log lines to console for debugging
                    try:
                        text = raw.decode("utf-8", errors="replace").rstrip()
                        if text and text[0] in "IWED" and " (" in text[:20]:
                            print(f"[esp32] {text}")
                    except Exception:
                        pass
                    continue
                got_first_row = True
                loop.call_soon_threadsafe(row_queue.put_nowait, row)
        loop.call_soon_threadsafe(row_queue.put_nowait, None)  # sentinel

    with open(out_path, "w", newline="", encoding="utf-8") as csv_file:
        csv_file.write(CSV_HEADER)
        csv_file.flush()

        reader = asyncio.create_task(asyncio.to_thread(_reader_thread))

        try:
            while True:
                try:
                    row = await asyncio.wait_for(row_queue.get(), timeout=2.0)
                except asyncio.TimeoutError:
                    if cancel_event.is_set():
                        break
                    continue

                if row is None:  # thread exited
                    break

                csv_file.write(row + "\n")
                csv_file.flush()
                row_count += 1
                await on_row(row, row_count)
        finally:
            cancel_event.set()  # ensure thread exits even on exception
            try:
                await asyncio.wait_for(reader, timeout=5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass

    return row_count


async def record_to_csv_tcp(
    host: str,
    tcp_port: int,
    out_path: Path,
    estado: str,
    on_row: Callable[[str, int], Awaitable[None]],
    cancel_event: asyncio.Event,
) -> int:
    """Read sensor rows from the ESP32 over TCP and write them to out_path.

    Mirrors record_to_csv but uses a TCP socket instead of a serial port.
    """
    loop = asyncio.get_running_loop()
    row_queue: asyncio.Queue[str | None] = asyncio.Queue()
    row_count = 0

    def _reader_thread() -> None:
        try:
            sock = socket.create_connection((host, tcp_port), timeout=TCP_CONNECT_TIMEOUT_S)
        except OSError as exc:
            loop.call_soon_threadsafe(row_queue.put_nowait, None)
            print(f"[recorder] TCP connect failed: {exc}")
            return
        try:
            sock.settimeout(2.0)
            stream = sock.makefile("rb")
            parser_state = ParserState()
            while not cancel_event.is_set():
                try:
                    raw = stream.readline()
                except (socket.timeout, TimeoutError):
                    continue
                if not raw:
                    break
                row = parse_serial_line(raw, parser_state, estado)
                if row is None:
                    try:
                        text = raw.decode("utf-8", errors="replace").rstrip()
                        if text and text[0] in "IWED" and " (" in text[:20]:
                            print(f"[esp32] {text}")
                    except Exception:
                        pass
                    continue
                loop.call_soon_threadsafe(row_queue.put_nowait, row)
        finally:
            try:
                sock.close()
            except OSError:
                pass
            loop.call_soon_threadsafe(row_queue.put_nowait, None)

    with open(out_path, "w", newline="", encoding="utf-8") as csv_file:
        csv_file.write(CSV_HEADER)
        csv_file.flush()

        reader = asyncio.create_task(asyncio.to_thread(_reader_thread))

        try:
            while True:
                try:
                    row = await asyncio.wait_for(row_queue.get(), timeout=2.0)
                except asyncio.TimeoutError:
                    if cancel_event.is_set():
                        break
                    continue

                if row is None:
                    break

                csv_file.write(row + "\n")
                csv_file.flush()
                row_count += 1
                await on_row(row, row_count)
        finally:
            cancel_event.set()
            try:
                await asyncio.wait_for(reader, timeout=5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass

    return row_count


async def record_to_csv_tcp_mock(
    host: str,
    tcp_port: int,
    out_path: Path,
    estado: str,
    on_row: Callable[[str, int], Awaitable[None]],
    cancel_event: asyncio.Event,
) -> int:
    """Mock TCP recorder — same shape as record_to_csv_mock but ignores host/port."""
    return await record_to_csv_mock(host, out_path, estado, on_row, cancel_event)


async def record_to_csv_mock(
    port: str,
    out_path: Path,
    estado: str,
    on_row: Callable[[str, int], Awaitable[None]],
    cancel_event: asyncio.Event,
    first_data_timeout_s: float = 30.0,
) -> int:
    """Mock recorder that replays sample data from a fixture file."""
    fixture = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "sample_serial.log"
    row_count = 0
    parser_state = ParserState()

    with open(out_path, "w", newline="", encoding="utf-8") as csv_file:
        csv_file.write(CSV_HEADER)
        csv_file.flush()

        if fixture.exists():
            lines = fixture.read_bytes().splitlines(keepends=True)
        else:
            # Generate synthetic rows if fixture is missing
            # Uptime in seconds (firmware sends seconds, not ms)
            lines = [
                f"{i * 2.5:.1f},{20 + i * 0.1:.2f},55.00,{21 + i * 0.05:.2f},{2000 + i},{1800 - i}\n".encode()
                for i in range(60)
            ]

        for raw in lines:
            if cancel_event.is_set():
                break
            await asyncio.sleep(0.25)
            row = parse_serial_line(raw, parser_state, estado)
            if row is None:
                continue
            csv_file.write(row + "\n")
            csv_file.flush()
            row_count += 1
            await on_row(row, row_count)

    return row_count
