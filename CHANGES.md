# Changes & How to Run

## What changed

### New: `common/parser.py`
Shared serial-line parser extracted from `record_csv.py`. Both the CLI recorder and the web app import from here so the parsing logic stays in sync.

### Modified: `record_csv.py`
Now imports `parse_serial_line` from `common.parser` instead of having the logic inline. Behavior is identical.

### New: `frontend/` — FastAPI web app

```
frontend/
├── requirements.txt          # pyserial, esptool, fastapi, uvicorn, pydantic
├── app/
│   ├── main.py               # FastAPI app + all routes
│   ├── models.py             # request/response schemas
│   ├── session.py            # flash→record state machine (one session at a time)
│   ├── flasher.py            # runs esptool as a subprocess, streams output live
│   ├── recorder.py           # reads serial port, writes CSV
│   ├── firmware.py           # validates firmware/*.bin files
│   ├── ports.py              # lists available serial ports
│   └── ws_bus.py             # WebSocket broadcast to all open browser tabs
└── static/
    ├── index.html            # single-page UI
    ├── style.css
    └── app.js                # vanilla JS state machine
```

### New: `firmware/`
Directory where the three pre-built ESP32 binaries must be placed before flashing. See [One-time firmware setup](#one-time-firmware-setup) below.

---

## One-time setup

### 1. Install Python dependencies

```bash
.venv/bin/pip install -r frontend/requirements.txt
```

### 2. Serial port permission (Linux only)

```bash
sudo usermod -aG dialout $USER
# then log out and log back in (or run: newgrp dialout)
```

### 3. One-time firmware setup

One developer with ESP-IDF v6.0 installed needs to build and copy the binaries once. After that, commit them so everyone else can use them directly.

```bash
cd embedded
idf.py build
cd ..

cp embedded/build/bootloader/bootloader.bin             firmware/
cp embedded/build/partition_table/partition-table.bin   firmware/
cp embedded/build/embedded.bin                          firmware/

git add firmware/*.bin && git commit -m "Add pre-built firmware binaries"
```

---

## How to run

### Normal mode (with ESP32 + firmware binaries)

```bash
PYTHONPATH=. .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

Open **http://localhost:8000** in your browser.

1. Select the serial port (click **Refresh** if the ESP32 is not listed).
2. Choose the plant state: `Decaida`, `Estable`, `Ideal`, or type a custom label.
3. Enter the full path for the output CSV (e.g. `/home/user/recordings/run1.csv`). Click **Validate** to check the path.
4. Click **Run measurement** — the ESP32 is flashed, then recording starts automatically.
5. Click **Stop** when done. The CSV path and row count are shown on screen.

### Mock mode (no ESP32 needed)

Useful for development and UI testing. Simulates the full flash → record flow using synthetic data.

```bash
PYTHONPATH=. FRONTEND_MOCK=1 .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

### CLI recorder (unchanged)

The original command-line recorder still works:

```bash
# no label
PYTHONPATH=. .venv/bin/python record_csv.py

# with a label
PYTHONPATH=. .venv/bin/python record_csv.py Ideal
```

---

## Output CSV format

```
timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity,estado
2026-05-19 14:23:01,23.45,55.0,22.10,2153,1766,Ideal
```

One row every ~2.5 seconds. Compatible with `eda/main.ipynb`.
