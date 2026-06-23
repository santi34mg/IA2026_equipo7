# WiFi Guide

How to get the ESP32 plant monitor online and stream sensor data over the network.

---

## TL;DR

```powershell
# One-time WiFi setup over USB (asks for SSID + password):
python configure_wifi.py

# Capture data wirelessly (auto-discovers the ESP32 on the LAN):
python record_csv_wifi.py

# Add an `estado` label to every row:
python record_csv_wifi.py Decaido
```

To change WiFi networks later, just re-run `configure_wifi.py`.

**Prefer the web UI?** Start the FastAPI app (see [CHANGES.md](CHANGES.md)), open http://localhost:8000, flip the **Source** toggle to **WiFi**, pick an estado, click **Suggest** then **Run measurement**. The frontend handles discovery and writes the CSV using exactly the filename shown in the field — same flow as the CLI script but driven from the browser.

---

## The three ways the ESP32 gets online

At every boot, the firmware tries to connect in this order. The first one that succeeds wins.

1. **NVS credentials** — the SSID and password last set via `configure_wifi.py` (saved in flash, survives reboots). This is the highest priority because it represents the most recent explicit user choice.
2. **Hardcoded fallback list** — compiled into the firmware at [`embedded/main/wifi_known_networks.h`](embedded/main/wifi_known_networks.h). Tried in array order. Useful when you move the device between locations whose networks you know in advance.
3. **Idle** — if neither works, the ESP32 stays online for sensors + serial but doesn't try to reach the network. Connect over USB and run `configure_wifi.py` to give it credentials.

Whichever path wins, the firmware logs:
```
I (xxxxx) app_main: Online: ssid="MyHome" ip=192.168.1.42
```

---

## Configuring WiFi over USB

Run with the ESP32 plugged into USB:

```powershell
python configure_wifi.py            # auto-detects the COM port
python configure_wifi.py COM7       # or specify it explicitly
```

What it does:
1. Opens the serial port and waits up to 6 s for the ESP32 to say something — confirms it's alive.
2. Prompts you for `SSID:` and `Password:` (password is hidden via `getpass`).
3. Sends the credentials to the ESP32, which saves them to NVS and tries to connect.
4. Waits up to 25 s for a deterministic response.

Success looks like:
```
[OK] Connected to "MyHome" - IP 192.168.1.42
     Next step: python record_csv_wifi.py 192.168.1.42
```

Failure looks like one of:
```
[FAIL] Authentication failed (wrong password?)
[FAIL] Network "MyHome" not found
[FAIL] Timed out trying to associate
[FAIL] No response from ESP32 within 25s
```

If `[FAIL] No data from ESP32 on COMx`, the script never saw any output from the device — usually means the firmware isn't flashed, the cable is data-less (charge-only), or the wrong COM port was picked.

---

## Adding a hardcoded fallback network

Edit [`embedded/main/wifi_known_networks.h`](embedded/main/wifi_known_networks.h) — add or remove entries in the `kKnownNetworks[]` array:

```cpp
constexpr WifiKnownNetwork kKnownNetworks[] = {
    {"HomeWiFi",       "homepassword"},
    {"University-2.4", "unipassword"},
    {"Phone Hotspot",  "phonepassword"},
    // add more here
};
```

Then rebuild and reflash:
```powershell
cd embedded
idf.py build
"..\flash esp32.bat"
```

**Order matters** — the ESP32 tries entries top-to-bottom. Put the network you expect to be on most often first.

**NVS overrides this list** — credentials set via `configure_wifi.py` take priority on the next boot. So if you configured WiFi over USB once, that network is tried before the hardcoded list. To force the hardcoded list back into play, just set NVS to a network that doesn't exist (or wait until you're physically out of NVS range — the list is tried as fallback).

**Privacy note** — this header contains plaintext passwords. If the repo is public or shared, consider adding `embedded/main/wifi_known_networks.h` to `.gitignore` and committing a `.example.h` template instead.

---

## Capturing data wirelessly

```powershell
python record_csv_wifi.py                          # auto-discover, no estado
python record_csv_wifi.py Decaido                  # auto-discover, estado="Decaido"
python record_csv_wifi.py 192.168.1.42             # direct IP, no estado
python record_csv_wifi.py 192.168.1.42 Decaido     # direct IP, estado="Decaido"
```

**The IP-vs-estado rule**: the first argument is treated as an IP if it matches `^\d{1,3}(\.\d{1,3}){3}$`. Otherwise it's an estado label and the script falls back to auto-discovery. So `Decaido`, `Ideal`, `Riego` etc. always work as labels.

**Discovery output** looks like:
```
[discovery] Listening for ESP32 on UDP 8081 (timeout 10s)...
[discovery] Found ESP32 at 192.168.1.42:8080
```

Then the regular capture stream starts:
```
[header] timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity
[row    1] 2026-05-19 23:14:08,23.50,68.00,22.10,1840,2105
[row    2] 2026-05-19 23:14:11,23.50,68.00,22.10,1842,2100
...
```

**Output file**: `data_plant_<YYYYMMDD_HHMMSS>.csv` in the current directory, same format as the serial-mode script.

**To stop**: press `Ctrl+C`. The file is flushed after every row, so killing the script never loses data.

---

## Discovery details

- The ESP32 sends a UDP broadcast packet every 3 seconds while it's connected to WiFi.
- Destination: `255.255.255.255:8081`.
- Payload: `PLANT_MONITOR:<ip>:<tcp_port>\n` (e.g. `PLANT_MONITOR:192.168.1.42:8080\n`).
- `record_csv_wifi.py` binds UDP `0.0.0.0:8081` and waits up to 10 s for a matching packet.

If discovery fails:
- **LAN may block UDP broadcast** (some guest networks do). Workaround: pass the IP explicitly. You can find it in the serial logs after `Online: ssid="..." ip=...`, or look it up in your router's DHCP table.
- **Windows Firewall** may prompt the first time the script binds UDP 8081. Accept the prompt; subsequent runs won't ask.
- **Two devices** broadcasting on the same LAN: the script picks the first packet it sees. If you have multiple plant monitors, pass the IP directly to choose one.

---

## Serial protocol reference

Sent from PC → ESP32 over the USB serial line:
```
WIFI:<ssid>:<password>\n
```

Replies from ESP32 → PC, one line each, every config line starts with `[wifi_cfg] `:

| Token                                          | Meaning                              |
|------------------------------------------------|--------------------------------------|
| `[wifi_cfg] CONNECT_OK ssid=<name> ip=<a.b.c.d>` | Associated and got DHCP IP           |
| `[wifi_cfg] CONNECT_FAIL reason=auth`           | Auth failure (wrong password)        |
| `[wifi_cfg] CONNECT_FAIL reason=not_found`      | SSID not visible in scan             |
| `[wifi_cfg] CONNECT_FAIL reason=timeout`        | No association within window         |
| `[wifi_cfg] CONNECT_FAIL reason=other`          | Other / unmapped disconnect reason   |
| `[wifi_cfg] PARSE_ERROR`                        | Command was malformed                |

Notes:
- Passwords containing `:` are fine — the parser splits on the first `:` after `WIFI:`, and everything after that is password.
- Any line *not* starting with `WIFI:` is ignored by the firmware, so the protocol coexists with normal serial output (sensor rows, ESP-IDF logs).

---

## Troubleshooting

**`configure_wifi.py` says "No data from ESP32 on COMx"**
- Wrong COM port — try passing it explicitly: `python configure_wifi.py COM7`.
- Cable is charge-only (data lines missing). Try a different USB cable.
- Firmware not flashed. Rebuild and reflash.

**`configure_wifi.py` loops `[FAIL] Authentication failed (wrong password?)`**
- Double-check the password. The serial logs (visible during the run) will show `reason=auth`.
- If your password contains special characters, make sure your terminal didn't mangle them.

**`record_csv_wifi.py` says "Could not find ESP32 on the LAN"**
- The ESP32 isn't connected to WiFi yet — run `configure_wifi.py` first.
- The LAN blocks UDP broadcast. Pass the IP directly: `python record_csv_wifi.py 192.168.x.y`.
- Windows Firewall is blocking — check Notification Center for a prompt.

**TCP connects, then immediately drops**
- Another client already opened the connection. The ESP32 accepts one TCP client at a time; the new connection kicks the old one off. Don't run two `record_csv_wifi.py` instances at once.

**Bad password from the hardcoded list keeps trying forever**
- It shouldn't — each fallback attempt has a 15 s timeout. If you see it stuck, check the firmware logs over serial; the per-attempt result is logged as `SSID "..." failed (auth|not_found|other)`.

**Want a stable IP instead of relying on discovery?**
- Set a DHCP reservation in your router for the ESP32's MAC. Then you can always do `python record_csv_wifi.py <fixed-ip>` and skip the UDP step entirely.

---

## File map (where things live)

- [`embedded/main/wifi_manager.cpp`](embedded/main/wifi_manager.cpp) / [`.h`](embedded/main/wifi_manager.h) — WiFi connection logic, NVS storage, event handling.
- [`embedded/main/wifi_known_networks.h`](embedded/main/wifi_known_networks.h) — the hardcoded fallback list (edit this file).
- [`embedded/main/tcp_server.cpp`](embedded/main/tcp_server.cpp) — port 8080 listener, one-client-at-a-time row broadcaster.
- [`embedded/main/discovery.cpp`](embedded/main/discovery.cpp) — UDP 8081 announcer.
- [`embedded/main/main.cpp`](embedded/main/main.cpp) — wires everything together at boot, runs the `WIFI:` serial parser.
- [`configure_wifi.py`](configure_wifi.py) — interactive USB setup.
- [`record_csv_wifi.py`](record_csv_wifi.py) — TCP capture client with UDP auto-discovery.
- [`record_csv.py`](record_csv.py) — serial-only capture (unchanged from before WiFi).
