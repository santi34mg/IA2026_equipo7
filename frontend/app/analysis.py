import asyncio
import io
import json
import logging
from pathlib import Path
from typing import Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from sklearn.decomposition import PCA

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR   = PROJECT_ROOT / "eda" / "models"
DATA_DIR     = PROJECT_ROOT / "eda"

LABEL_NAMES  = ["Decaida", "Estable", "Ideal"]
FEATURE_COLS = ["temperatura", "luz", "humedad_suelo"]

EDA_PLOTS: dict[str, str] = {
    "correlation":          "Correlación de features",
    "class_distribution":   "Distribución de clases",
    "pca":                  "PCA — PC1 vs PC2",
    "histograms":           "Histogramas por clase",
    "boxplots":             "Boxplots por clase",
    "feature_separability": "Temperatura vs Luz por clase",
}


MOCK_METRICS = {
    "models": {
        "decision_tree": {
            "accuracy": 0.9612, "macro_f1": 0.9608,
            "f1_per_class": {"Decaida": 0.9431, "Estable": 0.9714, "Ideal": 0.9679},
            "cv_macro_f1_mean": 0.9521, "cv_macro_f1_std": 0.0183,
            "roc_auc": 0.9874, "exported_kb": 8.32, "is_selected": False,
        },
        "random_forest": {
            "accuracy": 0.9847, "macro_f1": 0.9845,
            "f1_per_class": {"Decaida": 0.9762, "Estable": 0.9891, "Ideal": 0.9882},
            "cv_macro_f1_mean": 0.9803, "cv_macro_f1_std": 0.0094,
            "roc_auc": 0.9971, "exported_kb": 312.74, "is_selected": True,
        },
        "knn": {
            "accuracy": 0.9531, "macro_f1": 0.9524,
            "f1_per_class": {"Decaida": 0.9318, "Estable": 0.9612, "Ideal": 0.9642},
            "cv_macro_f1_mean": 0.9441, "cv_macro_f1_std": 0.0221,
            "roc_auc": 0.9812, "exported_kb": None, "is_selected": False,
        },
        "svm": {
            "accuracy": 0.9703, "macro_f1": 0.9699,
            "f1_per_class": {"Decaida": 0.9587, "Estable": 0.9743, "Ideal": 0.9767},
            "cv_macro_f1_mean": 0.9654, "cv_macro_f1_std": 0.0141,
            "roc_auc": 0.9932, "exported_kb": None, "is_selected": False,
        },
    }
}


class AnalysisService:
    def __init__(self, mock: bool = False) -> None:
        self._df: Optional[pd.DataFrame] = None
        self._pipelines: dict = {}
        self._metrics: dict = {}
        self._available = False
        self._mock = mock
        self._plot_lock = asyncio.Lock()
        self._load()

    def _load(self) -> None:
        try:
            self._df = self._load_csvs()
        except Exception as exc:
            logger.warning("AnalysisService: could not load CSVs: %s", exc)

        if MODELS_DIR.exists():
            try:
                import joblib
            except ImportError:
                logger.warning("AnalysisService: joblib not installed")
            else:
                for jl in sorted(MODELS_DIR.glob("*.joblib")):
                    try:
                        self._pipelines[jl.stem] = joblib.load(jl)
                    except Exception as exc:
                        logger.warning("AnalysisService: could not load %s: %s", jl.name, exc)

                metrics_path = MODELS_DIR / "metrics.json"
                if metrics_path.exists():
                    with open(metrics_path) as f:
                        self._metrics = json.load(f)

                if self._pipelines:
                    self._available = True
                    logger.info("AnalysisService loaded %d models", len(self._pipelines))
        else:
            logger.warning("AnalysisService: eda/models/ not found — run modelo.ipynb first")

        if not self._available and self._mock:
            self._metrics = MOCK_METRICS
            self._available = True
            logger.info("AnalysisService: mock mode — using synthetic metrics and predictions")

    @property
    def available(self) -> bool:
        return self._available

    @property
    def metrics(self) -> dict:
        return self._metrics

    def list_confusion_models(self) -> list:
        if not MODELS_DIR.exists():
            return []
        return [p.stem[len("confusion_"):] for p in sorted(MODELS_DIR.glob("confusion_*.png"))]

    def predict_all(self, features: list) -> dict:
        """Predict with all loaded pipelines. features = [temperatura, luz, humedad_suelo]."""
        if self._pipelines:
            X = np.array([features], dtype=np.float32)
            out = {}
            for name, pipe in self._pipelines.items():
                try:
                    idx = int(pipe.predict(X)[0])
                    out[name] = LABEL_NAMES[idx] if 0 <= idx < len(LABEL_NAMES) else str(idx)
                except Exception:
                    pass
            return out

        if self._mock and self._available:
            return self._mock_predict(features)

        return {}

    def _mock_predict(self, features: list) -> dict:
        """Rule-based mock predictions that vary slightly per model."""
        temp, light, moisture = (features + [0, 0, 0])[:3]
        # Base label from simple thresholds
        if moisture > 1400 and temp > 12:
            base = "Ideal"
        elif moisture > 700 or temp > 8:
            base = "Estable"
        else:
            base = "Decaida"

        # Each mock model agrees most of the time but has characteristic disagreements
        labels = LABEL_NAMES
        idx = labels.index(base)
        return {
            "decision_tree": labels[max(0, idx - (1 if light < 300 else 0))],
            "random_forest": base,
            "knn":           labels[min(2, idx + (1 if moisture > 2000 else 0))],
            "svm":           base,
        }

    async def render_plot_async(self, name: str) -> bytes:
        if name not in EDA_PLOTS:
            raise KeyError(f"Unknown plot: {name}")
        if self._df is None:
            raise RuntimeError("Data not loaded")
        loop = asyncio.get_event_loop()
        async with self._plot_lock:
            return await loop.run_in_executor(None, self._render_plot_sync, name)

    def _render_plot_sync(self, name: str) -> bytes:
        dispatch = {
            "correlation":          self._plot_correlation,
            "class_distribution":   self._plot_class_distribution,
            "pca":                  self._plot_pca,
            "histograms":           self._plot_histograms,
            "boxplots":             self._plot_boxplots,
            "feature_separability": self._plot_feature_separability,
        }
        fig = dispatch[name]()
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=100, bbox_inches="tight")
        plt.close(fig)
        buf.seek(0)
        return buf.read()

    # ----- Plot implementations -----

    def _plot_correlation(self) -> plt.Figure:
        fig, ax = plt.subplots(figsize=(7, 5))
        corr = self._df[FEATURE_COLS].corr()
        sns.heatmap(corr, annot=True, fmt=".2f", cmap="coolwarm", ax=ax,
                    vmin=-1, vmax=1, square=True)
        ax.set_title("Correlación entre features")
        plt.tight_layout()
        return fig

    def _plot_class_distribution(self) -> plt.Figure:
        fig, ax = plt.subplots(figsize=(6, 4))
        counts = self._df["label"].value_counts().reindex(LABEL_NAMES, fill_value=0)
        colors = ["#c0392b", "#e67e22", "#27ae60"]
        counts.plot(kind="bar", ax=ax, color=colors, edgecolor="white", width=0.6)
        ax.set_title("Distribución de clases")
        ax.set_xlabel("")
        ax.set_ylabel("Muestras")
        ax.tick_params(axis="x", rotation=0)
        for p in ax.patches:
            ax.annotate(str(int(p.get_height())),
                        (p.get_x() + p.get_width() / 2, p.get_height()),
                        ha="center", va="bottom", fontsize=10)
        plt.tight_layout()
        return fig

    def _plot_pca(self) -> plt.Figure:
        from sklearn.preprocessing import StandardScaler as SS
        mask = self._df[FEATURE_COLS].notna().all(axis=1)
        X = self._df.loc[mask, FEATURE_COLS].values
        labels_arr = self._df.loc[mask, "label"].values
        X_scaled = SS().fit_transform(X)
        pca = PCA(n_components=2, random_state=42)
        X_pca = pca.fit_transform(X_scaled)
        colors_map = {"Decaida": "#c0392b", "Estable": "#e67e22", "Ideal": "#27ae60"}
        fig, ax = plt.subplots(figsize=(7, 5))
        for lbl, color in colors_map.items():
            m = labels_arr == lbl
            ax.scatter(X_pca[m, 0], X_pca[m, 1],
                       c=color, label=lbl, alpha=0.5, s=15, edgecolors="none")
        ax.set_xlabel(f"PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)")
        ax.set_ylabel(f"PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)")
        ax.set_title("PCA — PC1 vs PC2")
        ax.legend()
        plt.tight_layout()
        return fig

    def _plot_histograms(self) -> plt.Figure:
        colors_map = {"Decaida": "red", "Estable": "orange", "Ideal": "green"}
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        for i, col in enumerate(FEATURE_COLS):
            for lbl, color in colors_map.items():
                subset = self._df[self._df["label"] == lbl][col].dropna()
                axes[i].hist(subset, bins=30, alpha=0.5, label=lbl, color=color,
                             edgecolor="black", linewidth=0.3)
            axes[i].set_title(col, fontsize=11)
            axes[i].legend(fontsize=8)
            axes[i].grid(True, alpha=0.3)
        fig.suptitle("Distribución de features por clase", fontsize=13)
        plt.tight_layout()
        return fig

    def _plot_boxplots(self) -> plt.Figure:
        colors_map = {"Decaida": "#c0392b", "Estable": "#e67e22", "Ideal": "#27ae60"}
        palette = [colors_map[l] for l in LABEL_NAMES]
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        for i, col in enumerate(FEATURE_COLS):
            sns.boxplot(data=self._df, x="label", y=col, order=LABEL_NAMES,
                        palette=palette, ax=axes[i])
            axes[i].set_title(col, fontsize=11)
            axes[i].set_xlabel("")
        fig.suptitle("Boxplots de features por clase", fontsize=13)
        plt.tight_layout()
        return fig

    def _plot_feature_separability(self) -> plt.Figure:
        colors_map = {"Decaida": "red", "Estable": "orange", "Ideal": "green"}
        fig, ax = plt.subplots(figsize=(7, 5))
        for lbl, color in colors_map.items():
            mask = self._df["label"] == lbl
            ax.scatter(self._df.loc[mask, "temperatura"], self._df.loc[mask, "luz"],
                       c=color, label=lbl, alpha=0.5, s=15, edgecolors="none")
        ax.set_xlabel("Temperatura (°C)")
        ax.set_ylabel("Luz (raw)")
        ax.set_title("Temperatura vs Luz por clase")
        ax.legend()
        plt.tight_layout()
        return fig

    # ----- Data loading -----

    def _load_csvs(self) -> pd.DataFrame:
        col_names    = ["temperatura", "humedad", "luz", "humedad_suelo", "label"]
        col_names_ts = ["timestamp"] + col_names
        archivos = {
            "datos1.csv": dict(header=None, names=col_names_ts),
            "datos2.csv": dict(header=None, names=col_names),
            "datos3.csv": dict(header=0,    names=col_names),
            "datos4.csv": dict(header=None, names=col_names),
            "datos5.csv": dict(header=None, names=col_names),
            "datos6.csv": dict(header=None, names=col_names_ts),
            "datos7.csv": dict(header=None, names=col_names_ts),
            "datos8.csv": dict(header=None, names=col_names_ts),
            "datos9.csv": dict(header=None, names=col_names_ts),
        }
        partes = []
        for nombre, kwargs in archivos.items():
            path = DATA_DIR / nombre
            if path.exists():
                d = pd.read_csv(path, **kwargs)
                partes.append(d)
        if not partes:
            raise FileNotFoundError("No CSV data files found in eda/")
        df = pd.concat(partes, ignore_index=True)
        df = df.drop(columns=["humedad", "timestamp"], errors="ignore")
        df = df[df["label"].isin(LABEL_NAMES)]
        for col in FEATURE_COLS:
            df[col] = pd.to_numeric(df[col], errors="coerce")
        return df
