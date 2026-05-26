"use strict";

// ---------- DOM refs ----------
const noModelsBanner  = document.getElementById("noModelsBanner");
const edaSpinner      = document.getElementById("edaSpinner");
const edaGrid         = document.getElementById("edaGrid");
const metricsLoading  = document.getElementById("metricsLoading");
const metricsTable    = document.getElementById("metricsTable");
const confusionGrid   = document.getElementById("confusionGrid");
const liveStatus      = document.getElementById("liveStatus");
const liveGrid        = document.getElementById("liveGrid");

// ---------- EDA plots ----------
async function loadEdaPlots() {
  try {
    const data = await fetch("/api/eda/plots").then(r => r.json());
    const plots = data.plots || [];
    if (plots.length === 0) {
      edaSpinner.textContent = "No hay datos disponibles. ¿Están los archivos datos*.csv en eda/?";
      return;
    }
    edaSpinner.style.display = "none";

    // Render all plots simultaneously
    plots.forEach(({ id, label }) => {
      const wrap = document.createElement("div");
      wrap.className = "eda-item";

      const title = document.createElement("p");
      title.className = "eda-label";
      title.textContent = label;

      const img = document.createElement("img");
      img.alt = label;
      img.className = "eda-plot-img";
      img.src = `/api/eda/plot/${encodeURIComponent(id)}?t=${Date.now()}`;
      img.onerror = () => { wrap.style.display = "none"; };

      wrap.appendChild(title);
      wrap.appendChild(img);
      edaGrid.appendChild(wrap);
    });
  } catch {
    edaSpinner.textContent = "Error cargando gráficos.";
  }
}

// ---------- Metrics table ----------
function fmt(v, decimals = 4) {
  if (v === null || v === undefined) return "—";
  return typeof v === "number" ? v.toFixed(decimals) : String(v);
}

function buildMetricsTable(metrics) {
  const models = metrics.models || {};
  const names  = Object.keys(models);
  if (names.length === 0) { metricsLoading.textContent = "Sin datos de métricas."; return; }

  const cols = [
    { key: "name",             label: "Modelo" },
    { key: "accuracy",         label: "Accuracy" },
    { key: "macro_f1",         label: "Macro-F1" },
    { key: "f1_Decaida",       label: "F1 Decaida" },
    { key: "f1_Estable",       label: "F1 Estable" },
    { key: "f1_Ideal",         label: "F1 Ideal" },
    { key: "cv",               label: "CV macro-F1" },
    { key: "roc_auc",          label: "ROC AUC" },
    { key: "exported_kb",      label: "KB" },
    { key: "selected",         label: "✓" },
  ];

  const head = `<thead><tr>${cols.map(c => `<th>${c.label}</th>`).join("")}</tr></thead>`;
  const rows = names.map(name => {
    const m = models[name];
    const cells = {
      name:        `<td class="model-name${m.is_selected ? " selected-model" : ""}">${name}</td>`,
      accuracy:    `<td>${fmt(m.accuracy)}</td>`,
      macro_f1:    `<td>${fmt(m.macro_f1)}</td>`,
      f1_Decaida:  `<td>${fmt(m.f1_per_class?.Decaida)}</td>`,
      f1_Estable:  `<td>${fmt(m.f1_per_class?.Estable)}</td>`,
      f1_Ideal:    `<td>${fmt(m.f1_per_class?.Ideal)}</td>`,
      cv:          `<td>${fmt(m.cv_macro_f1_mean)} ± ${fmt(m.cv_macro_f1_std, 4)}</td>`,
      roc_auc:     `<td>${fmt(m.roc_auc)}</td>`,
      exported_kb: `<td>${m.exported_kb != null ? fmt(m.exported_kb, 2) : "—"}</td>`,
      selected:    `<td>${m.is_selected ? "★" : ""}</td>`,
    };
    return `<tr>${cols.map(c => cells[c.key]).join("")}</tr>`;
  }).join("");

  metricsTable.innerHTML = head + `<tbody>${rows}</tbody>`;
  metricsTable.style.display = "";
  metricsLoading.style.display = "none";
}

async function loadModels() {
  try {
    const resp = await fetch("/api/models");
    if (resp.status === 503) {
      noModelsBanner.style.display = "";
      metricsLoading.textContent = "Modelos no encontrados.";
      return;
    }
    const metrics = await resp.json();
    buildMetricsTable(metrics);
    loadConfusionImages(Object.keys(metrics.models || {}));
  } catch {
    metricsLoading.textContent = "Error cargando métricas.";
  }
}

function loadConfusionImages(modelNames) {
  confusionGrid.innerHTML = "";
  if (modelNames.length === 0) return;
  modelNames.forEach(name => {
    const wrap = document.createElement("div");
    wrap.className = "confusion-item";
    const img = document.createElement("img");
    img.src = `/api/eda/confusion/${encodeURIComponent(name)}`;
    img.alt = name;
    img.className = "confusion-img";
    img.onerror = () => { wrap.style.display = "none"; };
    const label = document.createElement("p");
    label.className = "confusion-label";
    label.textContent = name;
    wrap.appendChild(img);
    wrap.appendChild(label);
    confusionGrid.appendChild(wrap);
  });
}

// ---------- Live multi-predict via WebSocket ----------
let ws = null;
let wsReconnectTimer = null;

function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws/session`);

  ws.onopen = () => {
    if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; }
    fetch("/api/session")
      .then(r => r.status === 204 ? null : r.json())
      .then(data => {
        if (data && data.state === "recording") {
          setLiveActive(true);
        }
      }).catch(() => {});
  };

  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    handleWSEvent(msg);
  };

  ws.onclose = () => { wsReconnectTimer = setTimeout(connectWS, 2000); };
  ws.onerror = () => { ws.close(); };
}

function handleWSEvent(msg) {
  const { type, data } = msg;
  if (type === "state") {
    setLiveActive(data.state === "recording");
  } else if (type === "stopped" || type === "error") {
    setLiveActive(false);
    liveStatus.textContent = "Sesión finalizada.";
  } else if (type === "multi_predict") {
    updateLiveGrid(data);
  }
}

function setLiveActive(active) {
  liveGrid.style.display = active ? "" : "none";
  liveStatus.style.display = active ? "none" : "";
  if (active && liveGrid.children.length === 0) {
    liveStatus.textContent = "Esperando primera fila…";
    liveStatus.style.display = "";
  }
  if (!active) {
    liveStatus.style.display = "";
  }
}

function updateLiveGrid(preds) {
  liveStatus.style.display = "none";
  liveGrid.style.display = "";

  const names = Object.keys(preds);
  // Build or update cells
  names.forEach(name => {
    let cell = document.getElementById(`live-cell-${CSS.escape(name)}`);
    if (!cell) {
      cell = document.createElement("div");
      cell.className = "live-cell";
      cell.id = `live-cell-${CSS.escape(name)}`;
      const label = document.createElement("span");
      label.className = "live-model-name";
      label.textContent = name;
      const badge = document.createElement("span");
      badge.className = "prediction-badge";
      badge.id = `live-badge-${CSS.escape(name)}`;
      cell.appendChild(label);
      cell.appendChild(badge);
      liveGrid.appendChild(cell);
    }
    const badge = document.getElementById(`live-badge-${CSS.escape(name)}`);
    if (badge) {
      const pred = preds[name] || "—";
      badge.textContent = pred;
      badge.className = "prediction-badge prediction-" + (pred.toLowerCase());
    }
  });
}

// ---------- Init ----------
(async () => {
  await Promise.all([loadEdaPlots(), loadModels()]);
  connectWS();
})();
