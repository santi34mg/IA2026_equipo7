# ESP32 Plant Measurement — Web UI

A FastAPI + plain-browser UI that lets you:
1. Scan and select a serial port (auto-detects the ESP32).
2. Choose the plant state label (`Decaida`, `Estable`, `Ideal`, or custom).
3. Click **Run measurement** → flashes the ESP32, then records sensor rows to a CSV you specify.

## One-time setup

### 1. Populate `firmware/` (one developer with ESP-IDF runs this)

```bash
cd embedded && idf.py build && cd ..
cp embedded/build/bootloader/bootloader.bin             firmware/
cp embedded/build/partition_table/partition-table.bin   firmware/
cp embedded/build/embedded.bin                          firmware/
```

Commit the binaries — they don't need to be rebuilt unless the firmware changes.

### 2. Install dependencies

```bash
# From repo root:
.venv/bin/pip install -r frontend/requirements.txt
```

### 3. (Linux only) Allow serial port access

```bash
sudo usermod -aG dialout $USER
# then logout and log back in (or: newgrp dialout in the current shell)
```

### 4. Run the server

```bash
# From repo root:
.venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

Open http://localhost:8000.

## Mock mode (no ESP32 needed)

```bash
FRONTEND_MOCK=1 .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

In mock mode:
- Port list returns a fake `MOCK` port.
- Flashing emits synthetic progress lines over ~5 s.
- Recording replays `frontend/tests/fixtures/sample_serial.log` (or generates synthetic rows if the file is missing).

Useful for UI development and testing on a laptop without hardware.

## Architecture

```
frontend/app/main.py      — FastAPI app, REST + WebSocket routes
frontend/app/session.py   — one-session-at-a-time state machine
frontend/app/flasher.py   — async esptool subprocess runner
frontend/app/recorder.py  — serial → CSV writer (runs in a thread)
frontend/app/firmware.py  — firmware/*.bin validation
frontend/app/ports.py     — serial port discovery
frontend/app/ws_bus.py    — WebSocket broadcast to all tabs
common/parser.py          — shared line parser (also used by record_csv.py)
```

## API reference

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/ports` | List serial ports |
| GET | `/api/firmware` | Check firmware binaries |
| POST | `/api/validate-path` | Validate a CSV output path |
| POST | `/api/session` | Start flash + record |
| GET | `/api/session` | Get current session state |
| POST | `/api/session/stop` | Stop recording |
| WS | `/ws/session` | Live events (flash log, row data, state changes) |

## CSV format

```
timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity,estado
2026-05-19 14:23:01,23.45,55.0,22.10,2153,1766,Ideal
```

One row every ~2.5 s (firmware sample period). Compatible with `eda/main.ipynb`.
