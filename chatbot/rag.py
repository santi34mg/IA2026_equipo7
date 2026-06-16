"""RAG: indexa datos CSV históricos + conocimiento científico y recupera chunks relevantes."""
import logging
import pickle
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
EDA_DIR = PROJECT_ROOT / "eda"
MODELS_DIR = EDA_DIR / "models"
SCIENCE_DOC = Path(__file__).parent / "science_knowledge.md"
CACHE_FILE = Path(__file__).parent / ".rag_cache.pkl"

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


def _load_dataframe() -> pd.DataFrame:
    parts = []
    for name, kwargs in CSV_CONFIG.items():
        path = EDA_DIR / name
        if path.exists():
            df = pd.read_csv(path, **kwargs)
            parts.append(df)
    if not parts:
        raise FileNotFoundError("No CSV files found in eda/")
    df = pd.concat(parts, ignore_index=True)
    df = df.drop(columns=["humedad", "timestamp"], errors="ignore")
    df = df[df["label"].isin(LABEL_NAMES)]
    for col in FEATURE_COLS:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.dropna(subset=FEATURE_COLS)


def _build_csv_chunks(df: pd.DataFrame, window: int = 20) -> list[str]:
    chunks = []

    # Overall stats
    for label in LABEL_NAMES:
        sub = df[df["label"] == label]
        if sub.empty:
            continue
        chunk = (
            f"Estadísticas históricas — Estado {label} ({len(sub)} muestras):\n"
            f"  Temperatura: media={sub['temperatura'].mean():.1f}°C, "
            f"min={sub['temperatura'].min():.1f}°C, max={sub['temperatura'].max():.1f}°C\n"
            f"  Luz: media={sub['luz'].mean():.0f}, "
            f"min={sub['luz'].min():.0f}, max={sub['luz'].max():.0f}\n"
            f"  Humedad suelo: media={sub['humedad_suelo'].mean():.0f}, "
            f"min={sub['humedad_suelo'].min():.0f}, max={sub['humedad_suelo'].max():.0f}"
        )
        chunks.append(chunk)

    # Windowed summaries of the most recent data
    recent = df.tail(200)
    for i in range(0, len(recent), window):
        batch = recent.iloc[i : i + window]
        label_counts = batch["label"].value_counts().to_dict()
        dominant = max(label_counts, key=label_counts.get)
        chunk = (
            f"Mediciones recientes (ventana {i // window + 1}):\n"
            f"  Temperatura: {batch['temperatura'].mean():.1f}°C\n"
            f"  Luz: {batch['luz'].mean():.0f}\n"
            f"  Humedad suelo: {batch['humedad_suelo'].mean():.0f}\n"
            f"  Estado predominante: {dominant} ({label_counts})"
        )
        chunks.append(chunk)

    return chunks


def _build_science_chunks() -> list[str]:
    if not SCIENCE_DOC.exists():
        return []
    text = SCIENCE_DOC.read_text(encoding="utf-8")
    paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]
    return paragraphs


def _load_model_metrics_chunk() -> Optional[str]:
    metrics_path = MODELS_DIR / "metrics.json"
    if not metrics_path.exists():
        return None
    import json
    with open(metrics_path) as f:
        metrics = json.load(f)
    lines = ["Métricas del modelo predictivo de Flora:"]
    for name, m in metrics.get("models", {}).items():
        acc = m.get("accuracy") or m.get("test_accuracy", "N/A")
        f1 = m.get("macro_f1") or m.get("test_macro_f1", "N/A")
        selected = " ← SELECCIONADO" if m.get("is_selected") else ""
        lines.append(f"  {name}: accuracy={acc}, macro_f1={f1}{selected}")
    return "\n".join(lines)


class RAGIndex:
    def __init__(self, gemini_client) -> None:
        self._client = gemini_client
        self._chunks: list[str] = []
        self._embeddings: Optional[np.ndarray] = None

    def build(self) -> None:
        if self._try_load_cache():
            logger.info("RAG: cache cargado (%d chunks)", len(self._chunks))
            return

        logger.info("RAG: construyendo índice desde cero...")
        chunks: list[str] = []

        try:
            df = _load_dataframe()
            chunks.extend(_build_csv_chunks(df))
            logger.info("RAG: %d chunks de CSV", len(chunks))
        except Exception as e:
            logger.warning("RAG: no se pudieron cargar CSVs: %s", e)

        science_chunks = _build_science_chunks()
        chunks.extend(science_chunks)
        logger.info("RAG: +%d chunks científicos", len(science_chunks))

        metrics_chunk = _load_model_metrics_chunk()
        if metrics_chunk:
            chunks.append(metrics_chunk)

        self._chunks = chunks
        logger.info("RAG: embeddizando %d chunks...", len(self._chunks))

        embeddings = []
        for i, chunk in enumerate(self._chunks):
            try:
                emb = self._client.embed_document(chunk)
                embeddings.append(emb)
            except Exception as e:
                logger.warning("RAG: error embeddizando chunk %d: %s", i, e)
                embeddings.append([0.0] * 768)

        self._embeddings = np.array(embeddings, dtype=np.float32)
        self._save_cache()
        logger.info("RAG: índice construido con %d chunks", len(self._chunks))

    def retrieve(self, query: str, k: int = 4) -> list[str]:
        if self._embeddings is None or len(self._chunks) == 0:
            return []
        try:
            q_emb = np.array(self._client.embed(query), dtype=np.float32)
        except Exception as e:
            logger.warning("RAG: error embeddizando query: %s", e)
            return self._chunks[:k]

        norms_docs = np.linalg.norm(self._embeddings, axis=1, keepdims=True)
        norm_q = np.linalg.norm(q_emb)
        if norm_q == 0:
            return self._chunks[:k]

        sims = (self._embeddings @ q_emb) / (norms_docs.squeeze() * norm_q + 1e-9)
        top_k = np.argsort(sims)[::-1][:k]
        return [self._chunks[i] for i in top_k]

    def _try_load_cache(self) -> bool:
        if not CACHE_FILE.exists():
            return False
        try:
            with open(CACHE_FILE, "rb") as f:
                data = pickle.load(f)
            self._chunks = data["chunks"]
            self._embeddings = data["embeddings"]
            return True
        except Exception as e:
            logger.warning("RAG: no se pudo cargar cache: %s", e)
            return False

    def _save_cache(self) -> None:
        try:
            with open(CACHE_FILE, "wb") as f:
                pickle.dump({"chunks": self._chunks, "embeddings": self._embeddings}, f)
        except Exception as e:
            logger.warning("RAG: no se pudo guardar cache: %s", e)

    def invalidate_cache(self) -> None:
        if CACHE_FILE.exists():
            CACHE_FILE.unlink()
        self._chunks = []
        self._embeddings = None
