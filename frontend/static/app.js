"use strict";

// ---------- DOM refs ----------
const sourceRadios  = document.querySelectorAll('input[name="source"]');
const portCard      = document.getElementById("portCard");
const wifiInfoCard  = document.getElementById("wifiInfoCard");
const discoveryStatus = document.getElementById("discoveryStatus");
const portSelect    = document.getElementById("portSelect");
const refreshPorts  = document.getElementById("refreshPorts");
const manualIp      = document.getElementById("manualIp");
const estadoSelect  = document.getElementById("estadoSelect");
const estadoCustom  = document.getElementById("estadoCustom");
const outPath       = document.getElementById("outPath");
const suggestPath   = document.getElementById("suggestPath");
const validatePath  = document.getElementById("validatePath");
const pathStatus    = document.getElementById("pathStatus");
const overwriteWarn = document.getElementById("overwriteWarn");
const runBtn        = document.getElementById("runBtn");
const stopBtn       = document.getElementById("stopBtn");
const statusBadge   = document.getElementById("statusBadge");
const firmwareBanner= document.getElementById("firmwareBanner");
const firmwareMissing=document.getElementById("firmwareMissing");
const flashSection  = document.getElementById("flashSection");
const flashLog      = document.getElementById("flashLog");
const recordSection = document.getElementById("recordSection");
const rowCount      = document.getElementById("rowCount");
const elapsed       = document.getElementById("elapsed");
const lastRow       = document.getElementById("lastRow");
const savedPath     = document.getElementById("savedPath");

// ---------- State ----------
// idle | starting | flashing | discovering | recording | stopping | done | error
let uiState = "idle";
let pendingOverwrite = false;
let firmwareOk = false;
let elapsedTimer = null;
let recordingStart = null;
let flashBuffer = [];
let flashFlushTimer = null;
let ws = null;
let wsReconnectTimer = null;

function getSource() {
  for (const r of sourceRadios) if (r.checked) return r.value;
  return "usb";
}

// ---------- State machine ----------
function setState(s) {
  uiState = s;
  const labels = { idle:"Idle", starting:"Starting…", flashing:"Flashing…",
                   discovering:"Discovering…", recording:"Recording",
                   stopping:"Stopping…", done:"Done", error:"Error" };
  statusBadge.textContent = labels[s] || s;
  statusBadge.className = `badge badge-${s}`;

  const isActive = ["starting","flashing","discovering","recording","stopping"].includes(s);
  stopBtn.disabled = !["flashing","discovering","recording"].includes(s);
  portSelect.disabled = isActive;
  estadoSelect.disabled = isActive;
  estadoCustom.disabled = isActive;
  outPath.disabled = isActive;
  refreshPorts.disabled = isActive;
  suggestPath.disabled = isActive;
  validatePath.disabled = isActive;
  if (manualIp) manualIp.disabled = isActive;
  for (const r of sourceRadios) r.disabled = isActive;

  // Flash log only relevant in USB mode
  const showFlash = getSource() === "usb" && ["flashing","recording","stopping","done","error"].includes(s);
  flashSection.style.display = showFlash ? "" : "none";
  recordSection.style.display = ["recording","stopping","done","error"].includes(s) ? "" : "none";

  if (s === "flashing") {
    flashLog.textContent = "";
    flashBuffer = [];
  }
  if (s === "discovering") {
    discoveryStatus.textContent = "Listening for ESP32 on UDP 8081…";
    discoveryStatus.className = "path-status";
  }
  if (s === "recording") {
    recordingStart = Date.now();
    savedPath.style.display = "none";
    startElapsed();
  }
  if (["done","error","idle"].includes(s)) {
    stopElapsed();
  }

  checkRunReady();
}

// ---------- Elapsed timer ----------
function startElapsed() {
  stopElapsed();
  elapsedTimer = setInterval(() => {
    const s = Math.floor((Date.now() - recordingStart) / 1000);
    elapsed.textContent = s + "s";
  }, 1000);
}
function stopElapsed() {
  if (elapsedTimer) { clearInterval(elapsedTimer); elapsedTimer = null; }
}

// ---------- Flash log (batched DOM writes) ----------
function appendFlashLine(line) {
  flashBuffer.push(line);
  if (!flashFlushTimer) {
    flashFlushTimer = setTimeout(() => {
      flashLog.textContent += flashBuffer.join("\n") + "\n";
      flashLog.scrollTop = flashLog.scrollHeight;
      flashBuffer = [];
      flashFlushTimer = null;
    }, 100);
  }
}

// ---------- WebSocket ----------
function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws/session`);

  ws.onopen = () => {
    if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; }
    // Resync: get current session state
    fetch("/api/session").then(r => r.status === 204 ? null : r.json()).then(data => {
      if (data && data.state && data.state !== uiState) {
        // Reflect source from server so the radio matches an in-progress session
        if (data.source) {
          for (const r of sourceRadios) r.checked = (r.value === data.source);
          applySourceVisibility();
        }
        setState(data.state);
        rowCount.textContent = data.row_count || 0;
        if (data.last_row) lastRow.textContent = data.last_row;
        if (data.discovered_ip) {
          discoveryStatus.textContent = `✓ Found ESP32 at ${data.discovered_ip}`;
          discoveryStatus.className = "path-status path-ok";
        }
      }
    }).catch(() => {});
  };

  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    handleWSEvent(msg);
  };

  ws.onclose = () => {
    wsReconnectTimer = setTimeout(connectWS, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function handleWSEvent(msg) {
  const { type, data } = msg;
  switch (type) {
    case "state":
      setState(data.state);
      if (data.row_count !== undefined) rowCount.textContent = data.row_count;
      break;
    case "flash_log":
      appendFlashLine(data.line);
      break;
    case "flash_done":
      if (data.exit_code !== 0) {
        appendFlashLine(`\n[ERROR] esptool exited with code ${data.exit_code}`);
      }
      break;
    case "discovered":
      discoveryStatus.textContent = `✓ Found ESP32 at ${data.ip}:${data.tcp_port}`;
      discoveryStatus.className = "path-status path-ok";
      break;
    case "record_row":
      rowCount.textContent = data.count;
      lastRow.textContent = data.row;
      break;
    case "stopped":
      setState("done");
      savedPath.textContent = `Saved: ${data.out_path}  (${data.row_count} rows)`;
      savedPath.style.display = "";
      break;
    case "error":
      setState("error");
      if (getSource() === "wifi" && data.reason) {
        discoveryStatus.textContent = `✗ ${data.reason}`;
        discoveryStatus.className = "path-status path-err";
      } else {
        appendFlashLine(`\n[ERROR] ${data.reason}`);
      }
      if (data.reason && data.reason.includes("dialout")) {
        appendFlashLine("Tip: run  sudo usermod -aG dialout $USER  then log out and back in.");
      }
      break;
  }
}

// ---------- Source toggle ----------
function applySourceVisibility() {
  const src = getSource();
  if (src === "usb") {
    portCard.style.display = "";
    wifiInfoCard.style.display = "none";
  } else {
    portCard.style.display = "none";
    wifiInfoCard.style.display = "";
    discoveryStatus.textContent = "";
    discoveryStatus.className = "path-status";
  }
  checkRunReady();
}
for (const r of sourceRadios) r.addEventListener("change", applySourceVisibility);

// ---------- Port list ----------
// Score each serial port by how ESP32-like it looks. The highest scorer is
// auto-selected so the user usually doesn't have to touch the dropdown.
function esp32Score(p) {
  const text = ((p.description || "") + " " + (p.hwid || "")).toUpperCase();
  let score = 0;
  // USB-to-UART bridge chips commonly soldered onto ESP32 dev boards
  if (text.includes("CP210"))         score += 10;  // Silicon Labs CP210x
  if (text.includes("SILICON LABS"))  score += 8;
  if (text.includes("CH340"))         score += 10;  // WCH CH340/CH341
  if (text.includes("CH341"))         score += 8;
  if (text.includes("ESPRESSIF"))     score += 10;  // native USB on S2/S3/C3
  // USB VID:PID matches in the hwid string
  if (text.includes("10C4") && text.includes("EA60")) score += 5;  // CP210x
  if (text.includes("1A86") && text.includes("7523")) score += 5;  // CH340
  if (text.includes("303A"))                          score += 5;  // Espressif
  // Generic fallbacks
  if (text.includes("USB-SERIAL"))    score += 3;
  if (text.includes("USB TO UART"))   score += 3;
  if (text.includes("USB SERIAL"))    score += 2;
  return score;
}

function pickBestPort(ports) {
  if (ports.length === 0) return null;
  if (ports.length === 1) return ports[0];
  let best = ports[0];
  let bestScore = esp32Score(best);
  for (let i = 1; i < ports.length; i++) {
    const s = esp32Score(ports[i]);
    if (s > bestScore) { best = ports[i]; bestScore = s; }
  }
  return best;
}

async function loadPorts() {
  portSelect.innerHTML = '<option value="">— scanning… —</option>';
  try {
    const data = await fetch("/api/ports").then(r => r.json());
    if (data.ports.length === 0) {
      portSelect.innerHTML = '<option value="">No ports found</option>';
    } else {
      const best = pickBestPort(data.ports);
      portSelect.innerHTML = data.ports
        .map(p => {
          const sel = (best && p.device === best.device) ? " selected" : "";
          return `<option value="${p.device}"${sel}>${p.device} — ${p.description}</option>`;
        })
        .join("");
    }
    checkRunReady();
  } catch {
    portSelect.innerHTML = '<option value="">Error loading ports</option>';
  }
}

// ---------- Firmware check ----------
async function checkFirmware() {
  try {
    const data = await fetch("/api/firmware").then(r => r.json());
    firmwareOk = data.ok;
    firmwareBanner.style.display = data.ok ? "none" : "";
    if (!data.ok) firmwareMissing.textContent = " Missing: " + data.missing.join(", ");
  } catch {
    firmwareOk = false;
  }
  checkRunReady();
}

// ---------- Run enable check ----------
function checkRunReady() {
  const notActive = !["starting","flashing","discovering","recording","stopping"].includes(uiState);
  const src = getSource();
  let ok = notActive;
  if (src === "usb") {
    // USB needs flashable firmware + selected port
    ok = ok && firmwareOk && !!portSelect.value;
  }
  // WiFi only requires not-active; estado/path validation happens on click
  runBtn.disabled = !ok;
}

// ---------- Estado custom ----------
estadoSelect.addEventListener("change", () => {
  estadoCustom.style.display = estadoSelect.value === "__other__" ? "" : "none";
});

function getEstado() {
  return estadoSelect.value === "__other__" ? estadoCustom.value.trim() : estadoSelect.value;
}

// ---------- Suggest path ----------
// The server picks the directory based on its OS:
//   Windows  → project root (this repo's directory)
//   Linux/Mac → /tmp/proyectoIA/  (created on demand)
suggestPath.addEventListener("click", async () => {
  const estado = getEstado() || "noestado";
  const src = getSource();
  try {
    const params = new URLSearchParams({ source: src, estado });
    const data = await fetch(`/api/suggest-path?${params}`).then(r => r.json());
    outPath.value = data.path;
  } catch {
    // Fallback to a bare filename in the cwd if the endpoint is unreachable
    const now = new Date();
    const pad = n => String(n).padStart(2, "0");
    const ts = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    outPath.value = `data_plant_${src}_${ts}_${estado}.csv`;
  }
  pathStatus.textContent = "";
  overwriteWarn.style.display = "none";
  pendingOverwrite = false;
});

// ---------- Validate path ----------
validatePath.addEventListener("click", async () => {
  const p = outPath.value.trim();
  if (!p) { pathStatus.textContent = "Enter a path first."; pathStatus.className = "path-status path-err"; return; }
  try {
    const data = await fetch("/api/validate-path", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({ path: p }),
    }).then(r => r.json());

    if (!data.ok) {
      pathStatus.textContent = "✗ " + (data.reason || "Invalid path.");
      pathStatus.className = "path-status path-err";
      overwriteWarn.style.display = "none";
    } else if (data.exists) {
      pathStatus.textContent = "✓ Path is valid (file already exists).";
      pathStatus.className = "path-status path-err";
      overwriteWarn.style.display = "";
    } else {
      pathStatus.textContent = "✓ Path is valid.";
      pathStatus.className = "path-status path-ok";
      overwriteWarn.style.display = "none";
    }
  } catch {
    pathStatus.textContent = "Could not validate path.";
    pathStatus.className = "path-status path-err";
  }
});

// ---------- Run ----------
runBtn.addEventListener("click", async () => {
  const source = getSource();
  const port = portSelect.value;
  const estado = getEstado();
  const path = outPath.value.trim();

  if (source === "usb" && !port)   { alert("Select a serial port first."); return; }
  if (!estado) { alert("Enter a plant state label."); return; }
  if (!path)   { alert("Enter an output CSV path (or click Suggest)."); return; }

  pendingOverwrite = false;
  overwriteWarn.style.display = "none";

  await doRun(source, port, estado, path, false);
});

async function doRun(source, port, estado, path, overwrite) {
  setState("starting");
  try {
    const body = { source, estado, out_path: path, overwrite };
    if (source === "usb") body.port = port;
    if (source === "wifi") {
      const ip = (manualIp?.value || "").trim();
      if (ip) body.ip = ip;
    }

    const resp = await fetch("/api/session", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body),
    });
    if (resp.status === 409) {
      const respBody = await resp.json();
      if (!overwrite && respBody.detail && respBody.detail.includes("already exists")) {
        setState("idle");
        overwriteWarn.style.display = "";
        // next Run click will overwrite
        runBtn.addEventListener("click", async () => {
          const src2 = getSource();
          const p2 = portSelect.value, e2 = getEstado(), pa2 = outPath.value.trim();
          if ((src2 === "wifi" || p2) && e2 && pa2) {
            overwriteWarn.style.display = "none";
            await doRun(src2, p2, e2, pa2, true);
          }
        }, { once: true });
        return;
      }
      setState("idle");
      alert("A session is already in progress.");
      return;
    }
    if (!resp.ok) {
      const respBody = await resp.json().catch(() => ({}));
      setState("error");
      const reason = respBody.detail || resp.statusText;
      if (source === "wifi") {
        discoveryStatus.textContent = `✗ ${reason}`;
        discoveryStatus.className = "path-status path-err";
      } else {
        appendFlashLine(`[ERROR] ${reason}`);
      }
      return;
    }
    // Success — WS will push state transitions from here
  } catch (err) {
    setState("error");
    appendFlashLine(`[ERROR] ${err.message}`);
  }
}

// ---------- Stop ----------
stopBtn.addEventListener("click", async () => {
  setState("stopping");
  await fetch("/api/session/stop", { method: "POST" }).catch(() => {});
});

// ---------- Refresh ports ----------
refreshPorts.addEventListener("click", loadPorts);

// ---------- Init ----------
(async () => {
  applySourceVisibility();
  connectWS();
  await Promise.all([loadPorts(), checkFirmware()]);
})();
