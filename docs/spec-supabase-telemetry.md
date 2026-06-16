# Spec: ESP32 → Supabase telemetry with a Vercel-hosted frontend

Status: Draft · Date: 2026-06-16 · Owner: equipo7

## 1. Goal

Replace the current LAN-only data path with a cloud-backed one:

1. The **ESP32** pushes every sensor reading directly to **Supabase** over WiFi (HTTP).
2. The **frontend** reads readings by querying **Supabase directly** (no Python
   backend in the data path).
3. The **frontend is hosted on Vercel** as a static / serverless deployment.

This makes readings available from anywhere, removes the requirement that a
client sit on the same WiFi as the device, and decouples the UI from the
firmware.

## 2. Current state (baseline)

- **Firmware (`embedded/main/`)** acts as a *TCP server*: it `listen()`s on a
  local port and `tcp_server_broadcast()`s each CSV row to connected clients
  (`tcp_server.cpp`, `storage.cpp:157`). It also UDP-broadcasts a discovery
  beacon (`discovery.h`). It has **no outbound HTTP client** and never contacts
  Supabase today.
- Reading row format (`storage_csv::build_row`):
  `timestamp_epoch,dht11_temp_c,dht11_humidity_pct,ks0033_temp_c,light_raw,moisture_raw,predicted`
- `predicted` is one of `Decaida | Estable | Ideal` (`classifier.h`).
- WiFi is already managed (`wifi_manager.cpp`) and SNTP time is available
  (`time_service.*`), so epoch timestamps exist on-device.
- **Frontend (`frontend/`)** is a FastAPI + vanilla-JS app that flashes the
  device, discovers it on the LAN, opens the TCP stream, and serves an analysis
  UI over a WebSocket bus. Today it depends on local hardware access.
- **Supabase** project `qvxxffjrdnjbnblbpcrp` (IA2026_equipo7) already exists and
  already has a `public.sensor_readings` table (see §4). The Supabase functions
  in the repo (`telegram-webhook`, `scheduler`) belong to the chatbot, not this
  pipeline.

## 3. Target architecture

```
  ┌──────────┐   HTTPS POST        ┌─────────────────────────┐
  │  ESP32   │ ──── reading ─────▶ │ Supabase                │
  │ (WiFi)   │  (anon or device    │  - PostgREST / RPC      │
  └──────────┘   key, TLS)         │  - sensor_readings table│
                                    │  - RLS policies         │
  ┌──────────────┐  HTTPS GET       └─────────────────────────┘
  │ Frontend     │ ◀── query ────────────────▲
  │ (Vercel)     │   (anon key, RLS read-only)│
  │ browser JS / │                            │
  │ supabase-js  │  Realtime (optional) ──────┘
  └──────────────┘
```

- The ESP32 is now an **HTTP client**, not a TCP server. Writes go to Supabase.
- The browser talks to Supabase **directly** via `supabase-js` (PostgREST for
  queries, optionally Realtime for live updates). No app server sits between the
  browser and the database in the read path.
- Vercel hosts the static frontend (and any serverless API routes used only to
  hide secrets, see §6).

## 4. Data contract

### 4.1 Table

Reuse the existing `public.sensor_readings`. Current columns:

| column          | type          | notes                          |
|-----------------|---------------|--------------------------------|
| `id`            | bigint        | PK, identity                   |
| `temperatura`   | real          | °C                             |
| `luz`           | integer       | light raw ADC                  |
| `humedad_suelo` | integer       | soil moisture raw ADC          |
| `label`         | text          | classifier output, nullable    |
| `created_at`    | timestamptz   | default `now()`                |

**Mismatch to resolve (decision needed — see §8):** the firmware row carries
more fields than the table stores. Recommended mapping for v1:

- `temperatura`   ← `dht11_temp_c` (or `ks0033_temp_c` — pick one canonical temp)
- `luz`           ← `light_raw`
- `humedad_suelo` ← `moisture_raw`
- `label`         ← `predicted`
- `created_at`    ← server default (`now()`), OR add a `device_ts` column to
  preserve the device epoch from `time_service`.

If we want to keep all signals, extend the table:

```sql
ALTER TABLE public.sensor_readings
  ADD COLUMN dht11_humidity_pct real,
  ADD COLUMN ks0033_temp_c      real,
  ADD COLUMN device_id          text,
  ADD COLUMN device_ts          timestamptz;
```

### 4.2 Write payload (ESP32 → Supabase)

PostgREST insert, `POST /rest/v1/sensor_readings`:

```http
POST /rest/v1/sensor_readings HTTP/1.1
Host: qvxxffjrdnjbnblbpcrp.supabase.co
apikey: <SUPABASE_KEY>
Authorization: Bearer <SUPABASE_KEY>
Content-Type: application/json
Prefer: return=minimal

{"temperatura": 24.3, "luz": 1820, "humedad_suelo": 1450, "label": "Ideal"}
```

Expect `201 Created`.

### 4.3 Read query (frontend → Supabase)

```js
const { data } = await supabase
  .from('sensor_readings')
  .select('*')
  .order('created_at', { ascending: false })
  .limit(200);
```

Optionally subscribe to `INSERT` via Supabase Realtime to live-update the chart.

## 5. ESP32 firmware changes (`embedded/main/`)

1. **Add an HTTP-client module** (`supabase_client.cpp/.h`) using
   `esp_http_client` with TLS:
   - `supabase_client_post_reading(const SensorData&, PlantState, int64_t ts)`
   - Builds the JSON body (reuse field extraction logic from
     `storage_csv::build_row`), sets `apikey` + `Authorization` headers.
   - Server cert: embed the Supabase root CA (bundle via
     `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE`) — do **not** disable cert verification.
2. **Wire it into the sampling loop** where `append_row` /
   `tcp_server_broadcast` are called (`storage.cpp:157`). Post on each reading.
3. **Resilience:**
   - Retry with backoff on failure; cap attempts.
   - Keep writing to SPIFFS CSV as the local buffer.
   - Optional v2: queue unsent rows and flush on reconnect (offline buffering).
   - Do not block the sensor task on the network — post from a worker
     task/queue.
4. **Config / secrets:** Supabase URL + key via `menuconfig`/`sdkconfig` (or
   `wifi_known_networks.h`-style header that is git-ignored). Never commit the
   key.
5. **TCP server / discovery:** keep for now (local debugging) or remove once the
   cloud path is trusted — decision in §8.

## 6. Frontend changes (`frontend/`) + Vercel

The current FastAPI app mixes two responsibilities: (a) hardware control
(flash/discover/record) and (b) data display. Only (b) moves to Vercel.

1. **New read path:** browser uses `@supabase/supabase-js` with the **anon
   key** and the project URL to query `sensor_readings` directly. Replace the
   WebSocket/TCP feed in `static/analysis.js` with Supabase queries + (optional)
   Realtime subscription.
2. **Build target for Vercel:**
   - Simplest: a static site (`static/` → `index.html`, `analysis.html`, JS,
     CSS) deployed as a Vercel static project. The Supabase URL + anon key are
     public-by-design (anon key is meant for browsers, protected by RLS).
   - If any secret must stay server-side, add a Vercel **serverless function**
     (`/api/*`) as a thin proxy; otherwise none is needed.
3. **What does NOT go to Vercel:** flashing and LAN discovery
   (`flasher.py`, `discovery.py`, `firmware.py`, serial/TCP code) require local
   hardware and stay as a local-only tool, or are dropped from the hosted UI.
4. **Env/config:** `NEXT_PUBLIC_SUPABASE_URL` / `..._ANON_KEY` style env vars in
   Vercel project settings (or injected into the static build).

## 7. Security (REQUIRED — currently failing)

The linked project has **RLS disabled on all tables**, including
`sensor_readings`. With the anon key exposed in the browser, the database is
fully open to read *and write* by anyone. Before going live:

1. Enable RLS:
   ```sql
   ALTER TABLE public.sensor_readings ENABLE ROW LEVEL SECURITY;
   ```
2. Add policies:
   - **Frontend (anon):** read-only.
     ```sql
     CREATE POLICY "anon read readings" ON public.sensor_readings
       FOR SELECT TO anon USING (true);
     ```
   - **Device writes:** do **not** let `anon` insert. Options, preferred first:
     - Insert via an **Edge Function** the ESP32 calls, which holds the
       `service_role` key server-side and validates a device shared-secret.
     - Or a dedicated device API key / signed token with an INSERT-only policy.
3. Keys: anon key in the browser is fine *only* with RLS as above. The
   `service_role` key must never reach the device firmware or the browser.

(See `get_advisors` `rls_disabled` advisory — do not enable RLS without adding
policies first, or all access breaks.)

## 8. Open decisions

1. **Canonical temperature:** `dht11_temp_c` vs `ks0033_temp_c` for
   `temperatura`, or store both (extend schema)?
2. **Timestamp source:** server `now()` vs device epoch (`device_ts`)?
3. **Device write auth:** direct PostgREST insert with a restricted key vs Edge
   Function proxy (recommended for not exposing write access).
4. **Keep or remove** the on-device TCP server + UDP discovery after cutover.
5. **Frontend stack on Vercel:** keep vanilla static `static/` as-is, or rebuild
   as a small Next/Vite app for `supabase-js` + env handling.
6. **Multi-device:** add a `device_id` column now or defer?

## 9. Out of scope

- Chatbot stack (`supabase/functions/telegram-webhook`, `scheduler`, RAG).
- ML model retraining / the classifier itself.
- Historical backfill of CSVs already on devices.
