"""
Serial line parser for the ESP32 plant-sensor firmware.

Extracted from record_csv.py so the CLI recorder and the FastAPI web app
share the same parsing logic and cannot drift.
"""

import datetime
from dataclasses import dataclass, field


@dataclass
class ParserState:
    t0: datetime.datetime | None = field(default=None)
    uptime0: float | None = field(default=None)


def parse_serial_line(raw: bytes, state: ParserState, estado: str | None) -> str | None:
    """Decode and parse one line from the ESP32 serial port.

    Returns a CSV row string (no trailing newline) or None if the line
    should be skipped (ESP-IDF log line, blank, decode error, no numeric
    first column).

    Replaces uptime seconds in column 0 with a real wall-clock timestamp
    anchored to the first data row received. Also accepts firmware output
    that already emits a Unix epoch timestamp in column 0.

    The firmware appends a `predicted` column (Decaida/Estable/Ideal) to each
    row; this function passes it through as-is. Appends ,<estado> when estado
    is not None.
    """
    try:
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
    except Exception:
        return None

    # Skip ESP-IDF log lines: I (12345) tag: ...
    if len(line) > 1 and line[0] in "IWED" and " (" in line[:20]:
        return None

    if not line:
        return None

    parts = line.split(",")
    try:
        uptime = float(parts[0])
        if uptime >= 1_000_000_000:
            parts[0] = datetime.datetime.fromtimestamp(uptime).strftime("%Y-%m-%d %H:%M:%S")
        else:
            if state.t0 is None:
                state.t0 = datetime.datetime.now()
                state.uptime0 = uptime
            ts = state.t0 + datetime.timedelta(seconds=(uptime - state.uptime0))
            parts[0] = ts.strftime("%Y-%m-%d %H:%M:%S")
        line = ",".join(parts)
    except (ValueError, IndexError):
        return None

    return f"{line},{estado}" if estado is not None else line
