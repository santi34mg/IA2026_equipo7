"use strict";

// ---------- DOM refs ----------
const portSelect    = document.getElementById("portSelect");
const refreshPorts  = document.getElementById("refreshPorts");
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
// idle | starting | flashing | recording | stopping | done | error
let uiState = "idle";
let pendingOverwrite = false;
let firmwareOk = false;
let elapsedTimer = null;
let recordingStart = null;
let flashBuffer = [];
let flashFlushTimer = null;
let ws = null;
let wsReconnectTimer = null;

// ---------- State machine ----------
function setState(s) {
  uiState = s;
  const labels = { idle:"Idle", starting:"Starting…", flashing:"Flashing…",
                   recording:"Recording", stopping:"Stopping…", done:"Done", error:"Error" };
  statusBadge.textContent = labels[s] || s;
  statusBadge.className = `badge badge-${s}`;

  const isActive = ["starting","flashing","recording","stopping"].includes(s);
  runBtn.disabled = isActive || !firmwareOk;
  stopBtn.disabled = !["flashing","recording"].includes(s);
  portSelect.disabled = isActive;
  estadoSelect.disabled = isActive;
  estadoCustom.disabled = isActive;
  outPath.disabled = isActive;
  refreshPorts.disabled = isActive;
  suggestPath.disabled = isActive;
  validatePath.disabled = isActive;

  flashSection.style.display = ["flashing","recording","stopping","done","error"].includes(s) ? "" : "none";
  recordSection.style.display = ["recording","stopping","done","error"].includes(s) ? "" : "none";

  if (s === "flashing") {
    flashLog.textContent = "";
    flashBuffer = [];
  }
  if (s === "recording") {
    recordingStart = Date.now();
    savedPath.style.display = "none";
    startElapsed();
  }
  if (["done","error","idle"].includes(s)) {
    stopElapsed();
  }
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
        setState(data.state);
        rowCount.textContent = data.row_count || 0;
        if (data.last_row) lastRow.textContent = data.last_row;
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
      appendFlashLine(`\n[ERROR] ${data.reason}`);
      if (data.reason && data.reason.includes("dialout")) {
        appendFlashLine("Tip: run  sudo usermod -aG dialout $USER  then log out and back in.");
      }
      break;
  }
}

// ---------- Port list ----------
async function loadPorts() {
  portSelect.innerHTML = '<option value="">— scanning… —</option>';
  try {
    const data = await fetch("/api/ports").then(r => r.json());
    portSelect.innerHTML = data.ports.length === 0
      ? '<option value="">No ports found</option>'
      : data.ports.map(p => `<option value="${p.device}">${p.device} — ${p.description}</option>`).join("");
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
  const notActive = !["starting","flashing","recording","stopping"].includes(uiState);
  runBtn.disabled = !notActive || !firmwareOk || !portSelect.value;
}

// ---------- Estado custom ----------
estadoSelect.addEventListener("change", () => {
  estadoCustom.style.display = estadoSelect.value === "__other__" ? "" : "none";
});

function getEstado() {
  return estadoSelect.value === "__other__" ? estadoCustom.value.trim() : estadoSelect.value;
}

// ---------- Suggest path ----------
suggestPath.addEventListener("click", () => {
  const now = new Date();
  const pad = n => String(n).padStart(2, "0");
  const ts = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
  const estado = getEstado() || "noestado";
  outPath.value = `/tmp/data_plant_${ts}_${estado}.csv`;
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
  const port = portSelect.value;
  const estado = getEstado();
  const path = outPath.value.trim();

  if (!port)   { alert("Select a serial port first."); return; }
  if (!estado) { alert("Enter a plant state label."); return; }
  if (!path)   { alert("Enter an output CSV path."); return; }

  pendingOverwrite = false;
  overwriteWarn.style.display = "none";

  await doRun(port, estado, path, false);
});

async function doRun(port, estado, path, overwrite) {
  setState("starting");
  try {
    const resp = await fetch("/api/session", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({ port, estado, out_path: path, overwrite }),
    });
    if (resp.status === 409) {
      const body = await resp.json();
      if (!overwrite && body.detail && body.detail.includes("already exists")) {
        setState("idle");
        overwriteWarn.style.display = "";
        // next Run click will overwrite
        runBtn.addEventListener("click", async () => {
          const p2 = portSelect.value, e2 = getEstado(), pa2 = outPath.value.trim();
          if (p2 && e2 && pa2) {
            overwriteWarn.style.display = "none";
            await doRun(p2, e2, pa2, true);
          }
        }, { once: true });
        return;
      }
      setState("idle");
      alert("A session is already in progress.");
      return;
    }
    if (!resp.ok) {
      const body = await resp.json().catch(() => ({}));
      setState("error");
      appendFlashLine(`[ERROR] ${body.detail || resp.statusText}`);
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
  connectWS();
  await Promise.all([loadPorts(), checkFirmware()]);
})();
