"use strict";

// Live Supabase dashboard for the hosted home page (Olivia).
// Polls sensor_readings every ~15s and redraws the three metric charts.
// Falls back to a "disconnected" state when the newest reading is too old.

const READINGS_TABLE = "sensor_readings";
const CHART_MAX_POINTS = 60;
const POLL_MS = 15000;
const STALE_MS = 45000; // última lectura más vieja que esto → desconectada

const statusEmoji = document.getElementById("statusEmoji");
const statusMsg = document.getElementById("statusMsg");
const lvLabel = document.getElementById("lvLabel");
const lvTime = document.getElementById("lvTime");
const lvTemp = document.getElementById("lvTemp");
const lvLuz = document.getElementById("lvLuz");
const lvHum = document.getElementById("lvHum");
const chartTemp = document.getElementById("chartTemp");
const chartLuz = document.getElementById("chartLuz");
const chartHum = document.getElementById("chartHum");
const pulseDot = document.getElementById("pulseDot");
const liveLabel = document.getElementById("liveLabel");

const MOODS = {
  Ideal:   { emoji: "🌟", msg: "¡Estoy radiante! Mis condiciones son perfectas." },
  Estable: { emoji: "🌱", msg: "Estoy bien, aunque podría estar mejor." },
  Decaida: { emoji: "😟", msg: "No me siento bien… necesito atención." },
};

let hasData = false;

// Generic line+fill sparkline for one metric.
function drawChart(canvas, values, stroke, fill) {
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  const vals = values.filter(Number.isFinite);
  if (vals.length < 2) return;

  let min = Math.min(...vals), max = Math.max(...vals);
  if (min === max) { min -= 1; max += 1; }
  const range = max - min;
  const pad = 14;
  const px = (i) => (i / (vals.length - 1)) * w;
  const py = (v) => h - pad - ((v - min) / range) * (h - 2 * pad);

  ctx.beginPath();
  ctx.moveTo(px(0), py(vals[0]));
  vals.forEach((v, i) => ctx.lineTo(px(i), py(v)));
  ctx.lineTo(w, h);
  ctx.lineTo(0, h);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();

  ctx.beginPath();
  vals.forEach((v, i) => (i ? ctx.lineTo(px(i), py(v)) : ctx.moveTo(px(i), py(v))));
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2.5;
  ctx.lineJoin = "round";
  ctx.stroke();

  const lx = px(vals.length - 1), ly = py(vals[vals.length - 1]);
  ctx.beginPath();
  ctx.arc(lx, ly, 3.5, 0, Math.PI * 2);
  ctx.fillStyle = stroke;
  ctx.fill();
}

function setLive(connected) {
  if (pulseDot) pulseDot.classList.toggle("stale", !connected);
  if (liveLabel) liveLabel.textContent = connected ? "En vivo · actualiza cada 15s" : "Sin conexión";
}

function renderValues(row) {
  if (lvTemp) lvTemp.textContent = row.temperatura != null ? Number(row.temperatura).toFixed(1) : "—";
  if (lvLuz)  lvLuz.textContent  = row.luz != null ? row.luz : "—";
  if (lvHum)  lvHum.textContent  = row.humedad_suelo != null ? row.humedad_suelo : "—";

  const ageMs = row.created_at ? (Date.now() - new Date(row.created_at).getTime()) : Infinity;
  const stale = ageMs > STALE_MS;
  setLive(!stale);

  if (stale) {
    if (lvLabel) {
      lvLabel.textContent = "Sin conexión";
      lvLabel.className = "prediction-badge status-label";
    }
    if (statusEmoji) statusEmoji.textContent = "🥀";
    if (statusMsg) statusMsg.textContent = "Hace rato que no recibo lecturas nuevas… ¿estoy conectada?";
  } else {
    const lbl = row.label || "—";
    if (lvLabel) {
      lvLabel.textContent = lbl;
      lvLabel.className = "prediction-badge status-label prediction-" + String(lbl).toLowerCase();
    }
    const mood = MOODS[lbl];
    if (statusEmoji) statusEmoji.textContent = mood ? mood.emoji : "🌿";
    if (statusMsg) statusMsg.textContent = mood ? mood.msg : "Esperando estado…";
  }

  if (lvTime && row.created_at) {
    lvTime.textContent = "Última lectura · " + new Date(row.created_at).toLocaleTimeString();
  }
}

async function refresh(client) {
  try {
    const { data, error } = await client
      .from(READINGS_TABLE)
      .select("*")
      .order("created_at", { ascending: false })
      .limit(CHART_MAX_POINTS);
    if (error || !Array.isArray(data) || !data.length) {
      if (statusMsg && !hasData) statusMsg.textContent = "Sin lecturas todavía. Esperando al ESP32…";
      setLive(false);
      return;
    }
    hasData = true;
    const rows = data.slice().reverse(); // oldest → newest

    drawChart(chartTemp, rows.map((r) => Number(r.temperatura)),   "#8c3a2c", "rgba(140, 58, 44, 0.10)");
    drawChart(chartLuz,  rows.map((r) => Number(r.luz)),           "#a07828", "rgba(160, 120, 40, 0.12)");
    drawChart(chartHum,  rows.map((r) => Number(r.humedad_suelo)), "#527f60", "rgba(82, 127, 96, 0.12)");

    renderValues(rows[rows.length - 1]);
  } catch {
    /* keep last good state */
  }
}

(async () => {
  const client = await window.initSupabase();
  if (!client) {
    if (statusMsg) statusMsg.textContent = "No se pudo conectar a Supabase.";
    setLive(false);
    return;
  }
  await refresh(client);
  setInterval(() => refresh(client), POLL_MS);
})();
