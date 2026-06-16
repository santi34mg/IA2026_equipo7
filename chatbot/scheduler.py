"""Mensajes proactivos: reporte de sensores cada 30 min y alerta de riego."""
import logging
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from personality import PLANT_NAME, SENSOR_REPORT_TEMPLATE, WATERING_ALERT

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
EDA_DIR = PROJECT_ROOT / "eda"
MODELS_DIR = EDA_DIR / "models"

LABEL_NAMES = ["Decaida", "Estable", "Ideal"]
FEATURE_COLS = ["temperatura", "luz", "humedad_suelo"]
COL_NAMES = ["temperatura", "humedad", "luz", "humedad_suelo", "label"]
COL_NAMES_TS = ["timestamp"] + COL_NAMES

CSV_CONFIG = {
    "datos1.csv": dict(header=None, names=COL_NAMES_TS),
    "datos2.csv": dict(header=None, names=COL_NAMES),
    "datos3.csv": dict(header=0,    names=COL_NAMES),
    "datos4.csv": dict(header=None, names=COL_NAMES),
    "datos5.csv": dict(header=None, names=COL_NAMES),
    "datos6.csv": dict(header=None, names=COL_NAMES_TS),
    "datos7.csv": dict(header=None, names=COL_NAMES_TS),
    "datos8.csv": dict(header=None, names=COL_NAMES_TS),
    "datos9.csv": dict(header=None, names=COL_NAMES_TS),
}

_model = None

def _load_model():
    global _model
    if _model is not None:
        return _model
    try:
        import joblib
        model_path = MODELS_DIR / "DecisionTree_d4.joblib"
        if model_path.exists():
            _model = joblib.load(model_path)
            logger.info("Scheduler: modelo cargado desde %s", model_path)
        else:
            logger.warning("Scheduler: DecisionTree_d4.joblib no encontrado")
    except Exception as e:
        logger.warning("Scheduler: error cargando modelo: %s", e)
    return _model


def get_latest_reading() -> Optional[dict]:
    """Lee la última fila disponible de los CSV históricos."""
    last_row = None
    last_file = None

    for name, kwargs in CSV_CONFIG.items():
        path = EDA_DIR / name
        if not path.exists():
            continue
        try:
            df = pd.read_csv(path, **kwargs)
            df = df.drop(columns=["humedad", "timestamp"], errors="ignore")
            df = df[df["label"].isin(LABEL_NAMES)]
            for col in FEATURE_COLS:
                df[col] = pd.to_numeric(df[col], errors="coerce")
            df = df.dropna(subset=FEATURE_COLS)
            if not df.empty:
                last_row = df.iloc[-1]
                last_file = name
        except Exception as e:
            logger.warning("Scheduler: error leyendo %s: %s", name, e)

    if last_row is None:
        return None

    row = last_row
    features = [float(row["temperatura"]), float(row["luz"]), float(row["humedad_suelo"])]

    model = _load_model()
    if model is not None:
        try:
            X = np.array([features], dtype=np.float32)
            idx = int(model.predict(X)[0])
            estado = LABEL_NAMES[idx] if 0 <= idx < len(LABEL_NAMES) else str(idx)
        except Exception as e:
            logger.warning("Scheduler: error en predicción: %s", e)
            estado = str(row.get("label", "Desconocido"))
    else:
        estado = str(row.get("label", "Desconocido"))

    return {
        "temperatura": features[0],
        "luz": int(features[1]),
        "humedad_suelo": int(features[2]),
        "estado": estado,
        "fuente": last_file,
    }


def format_sensor_message(reading: dict) -> str:
    return SENSOR_REPORT_TEMPLATE.format(
        temperatura=reading["temperatura"],
        luz=reading["luz"],
        humedad_suelo=reading["humedad_suelo"],
        estado=reading["estado"],
        timestamp=datetime.now().strftime("%d/%m/%Y %H:%M"),
    )


async def sensor_report_job(context) -> None:
    """Job que se ejecuta cada 30 minutos para enviar reporte de sensores."""
    chat_ids: set = context.bot_data.get("chat_ids", set())
    if not chat_ids:
        return

    reading = get_latest_reading()
    if reading is None:
        logger.warning("Scheduler: no hay datos de sensores disponibles")
        return

    message = format_sensor_message(reading)

    for chat_id in chat_ids:
        try:
            await context.bot.send_message(
                chat_id=chat_id,
                text=message,
                parse_mode="Markdown",
            )
            if reading["estado"] == "Decaida":
                await context.bot.send_message(
                    chat_id=chat_id,
                    text=WATERING_ALERT,
                )
        except Exception as e:
            logger.warning("Scheduler: error enviando a chat_id %s: %s", chat_id, e)


def register_jobs(application) -> None:
    """Registra el job periódico de 30 minutos."""
    application.job_queue.run_repeating(
        sensor_report_job,
        interval=1800,
        first=10,
        name="sensor_report",
    )
    logger.info("Scheduler: job de sensores registrado (cada 30 min)")
