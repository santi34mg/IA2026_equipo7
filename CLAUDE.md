# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 plant sensor monitoring system. Three components:
1. **Embedded firmware** (C++/ESP-IDF) — reads DHT, KS soil, and light sensors every ~2.5 s
2. **Web frontend** (Python FastAPI) — flashes firmware and records sensor data via USB or WiFi
3. **EDA notebook** (Jupyter) — analyzes collected CSV data

Pre-built firmware binaries live in `firmware/` and are committed to git so the frontend can be used without an ESP-IDF toolchain.

## Commands

### Frontend (web app)

```bash
# Install dependencies (one time)
.venv/bin/pip install -r frontend/requirements.txt

# Run (requires connected ESP32)
PYTHONPATH=. .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000

# Run without hardware (mock mode)
FRONTEND_MOCK=1 PYTHONPATH=. .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

Linux users need serial port access: `sudo usermod -aG dialout $USER`

### CLI tools

```bash
# Record sensor data over USB
PYTHONPATH=. .venv/bin/python tools/record_csv.py [estado]

# Configure WiFi credentials over USB
python tools/configure_wifi.py [port]

# Live serial monitor
python tools/monitor_esp32.py

# Record over WiFi
python tools/record_csv_wifi.py
```

### EDA notebook

```bash
cd eda && jupyter notebook main.ipynb
```

### Firmware rebuild (requires ESP-IDF v6.0)

```bash
cd embedded && idf.py build && cd ..
cp embedded/build/bootloader/bootloader.bin firmware/
cp embedded/build/partition-table/partition-table.bin firmware/
cp embedded/build/embedded.bin firmware/
git add firmware/*.bin && git commit
```

## Architecture

### CSV row format

```
timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity,predicted,estado
2026-05-26 14:23:01,23.45,55.0,22.10,2153,1766,Ideal,Ideal
```

`predicted` = inference result from the ESP32's embedded DecisionTree.  
`estado` = human label assigned at session start.

### Analysis page (`frontend/app/analysis.py`, `/analysis`)

Serves EDA plots, model comparison metrics, and live multi-model predictions.
Requires `eda/models/` to be populated by running `eda/modelo.ipynb` end-to-end (generates `*.joblib`, `metrics.json`, `confusion_*.png`).

- If `eda/models/` is missing, the app starts in degraded mode: `/analysis` shows a banner but `/` (recording) still works.
- **Regenerate models** whenever `eda/datos*.csv` files change: re-run `modelo.ipynb` from top to bottom.
- `frontend/requirements.txt` now includes the ML stack (pandas, numpy, scikit-learn, matplotlib, seaborn, joblib). Run `pip install -r frontend/requirements.txt` after pulling if deps changed.
- sklearn joblib files are version-sensitive; if sklearn major version changes, regenerate `eda/models/`.

### Shared parser (`common/parser.py`)

Both the CLI recorder and the web app import this. It filters ESP-IDF log lines, converts ESP32 uptime seconds to wall-clock timestamps, and appends the `estado` column. Any change to parsing logic must go here.

### Frontend session state machine (`frontend/app/session.py`)

One session at a time. Transitions: `idle → flashing → recording → idle`. The WebSocket bus (`ws_bus.py`) broadcasts state changes and live rows to all open browser tabs.

### Firmware flashing (`frontend/app/flasher.py`)

Shells out to `esptool` at 460800 baud. Requires the three pre-built binaries in `firmware/` to be present (checked by `firmware.py` at startup).

### WiFi discovery (`frontend/app/discovery.py`, `embedded/main/discovery.cpp`)

ESP32 broadcasts a UDP packet on the LAN; the frontend listens and surfaces the IP. Manual IP entry is the fallback.

### Serial recording (`frontend/app/recorder.py`)

Runs the serial reader in a thread to avoid blocking the async event loop. Rows are written to a CSV file and also broadcast over WebSocket.

## Key paths

| Path | Purpose |
|------|---------|
| `frontend/app/main.py` | FastAPI app, all routes |
| `frontend/app/session.py` | Session state machine |
| `common/parser.py` | Shared serial line parser |
| `firmware/*.bin` | Pre-built ESP32 binaries |
| `embedded/main/` | ESP-IDF firmware source |
| `eda/main.ipynb` | Descriptive EDA notebook |
| `eda/modelo.ipynb` | Model training + export notebook |
| `eda/datos[1-9].csv` | Training recordings (used by notebooks + frontend) |
| `data/raw/` | Full original recordings (not loaded by code) |
| `tools/` | CLI scripts: record_csv.py, record_csv_wifi.py, configure_wifi.py, monitor_esp32.py |
| `eda/models/` | Trained pipelines (.joblib), metrics.json, confusion PNGs — regenerate by running modelo.ipynb |
| `frontend/app/analysis.py` | AnalysisService: loads models, renders EDA plots, predict_all |
| `frontend/static/analysis.html` | Analysis page |
| `docs/` | Academic report, changelog, assignment handout |
