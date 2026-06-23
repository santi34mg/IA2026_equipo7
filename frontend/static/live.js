"use strict";

// Live Supabase dashboard for the hosted home page. Reads sensor_readings
// directly (via supabase.js) and updates in real time through Realtime.

const READINGS_TABLE = "sensor_readings";
const CHART_MAX_POINTS = 40;
const chartTemps = [];

const liveStatus = document.getElementById("liveStatus");
const liveValues = document.getElementById("liveValues");
const lvTemp = document.getElementById("lvTemp");
const lvLuz = document.getElementById("lvLuz");
const lvHum = document.getElementById("lvHum");
const lvLabel = document.getElementById("lvLabel");
const lvTime = document.getElementById("lvTime");
const liveChart = document.getElementById("liveChart");

function drawChart() {
  if (!liveChart) return;
  const ctx = liveChart.getContext("2d");
  const w = liveChart.width, h = liveChart.height;
  ctx.clearRect(0, 0, w, h);
  const vals = chartTemps.filter((v) => Number.isFinite(v));
  if (vals.length < 2) return;
  const min = Math.min(...vals), max = Math.max(...vals);
  const range = (max - min) || 1;
  const pad = 8;
  ctx.beginPath();
  vals.forEach((v, i) => {
    const x = (i / (vals.length - 1)) * w;
    const y = h - pad - ((v - min) / range) * (h - 2 * pad);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.strokeStyle = "#27ae60";
  ctx.lineWidth = 2;
  ctx.stroke();
}

function renderReading(row) {
  liveValues.style.display = "";
  liveStatus.style.display = "none";
  if (lvTemp) lvTemp.textContent = row.temperatura != null ? Number(row.temperatura).toFixed(2) : "—";
  if (lvLuz)  lvLuz.textContent  = row.luz != null ? row.luz : "—";
  if (lvHum)  lvHum.textContent  = row.humedad_suelo != null ? row.humedad_suelo : "—";
  if (lvLabel) {
    const lbl = row.label || "—";
    lvLabel.textContent = lbl;
    lvLabel.className = "prediction-badge prediction-" + String(lbl).toLowerCase();
  }
  if (lvTime && row.created_at) {
    lvTime.textContent = "Última lectura: " + new Date(row.created_at).toLocaleTimeString();
  }
  const t = Number(row.temperatura);
  if (Number.isFinite(t)) {
    chartTemps.push(t);
    if (chartTemps.length > CHART_MAX_POINTS) chartTemps.shift();
    drawChart();
  }
}

(async () => {
  const client = await window.initSupabase();
  if (!client) {
    liveStatus.textContent = "No se pudo conectar a Supabase.";
    return;
  }

  // Seed with the most recent readings (oldest→newest for the chart).
  try {
    const { data, error } = await client
      .from(READINGS_TABLE)
      .select("*")
      .order("created_at", { ascending: false })
      .limit(CHART_MAX_POINTS);
    if (!error && Array.isArray(data) && data.length) {
      data.slice().reverse().forEach(renderReading);
    } else {
      liveStatus.textContent = "Sin lecturas todavía. Esperando al ESP32…";
    }
  } catch {
    liveStatus.textContent = "Sin lecturas todavía. Esperando al ESP32…";
  }

  // Live updates on every new row inserted by the device.
  client
    .channel("live-home")
    .on(
      "postgres_changes",
      { event: "INSERT", schema: "public", table: READINGS_TABLE },
      (payload) => renderReading(payload.new),
    )
    .subscribe();
})();
