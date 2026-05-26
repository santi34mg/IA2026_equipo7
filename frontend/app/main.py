import os
import platform
from datetime import datetime
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .analysis import MODELS_DIR, AnalysisService
from .firmware import firmware_status, validate_firmware
from .models import (
    FirmwareResponse,
    PortsResponse,
    SessionView,
    StartRequest,
    ValidatePathRequest,
    ValidatePathResponse,
)
from .ports import _mock_ports, list_serial_ports, port_exists
from .session import SessionManager
from .ws_bus import WSBus

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"
PROJECT_ROOT = Path(__file__).resolve().parents[2]
LINUX_DATA_DIR = Path("/tmp/proyectoIA")
MOCK = os.environ.get("FRONTEND_MOCK", "").lower() in ("1", "true", "yes")


def _suggested_directory() -> Path:
    """Where Suggest should drop new recordings, by OS."""
    if platform.system() == "Windows":
        return PROJECT_ROOT
    # Linux / Mac: /tmp/proyectoIA — create on demand
    try:
        LINUX_DATA_DIR.mkdir(parents=True, exist_ok=True)
    except OSError:
        return PROJECT_ROOT  # fallback if /tmp is unwriteable
    return LINUX_DATA_DIR


def create_app() -> FastAPI:
    app = FastAPI(title="ESP32 Plant Measurement")
    app.state.sessions = SessionManager()
    app.state.bus = WSBus()
    app.state.analysis = AnalysisService()

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    @app.get("/", include_in_schema=False)
    async def index() -> FileResponse:
        return FileResponse(str(STATIC_DIR / "index.html"))

    @app.get("/api/ports", response_model=PortsResponse)
    async def get_ports() -> PortsResponse:
        ports = _mock_ports() if MOCK else list_serial_ports()
        return PortsResponse(ports=ports)

    @app.get("/api/firmware", response_model=FirmwareResponse)
    async def get_firmware() -> FirmwareResponse:
        if MOCK:
            from .models import FirmwareFile
            return FirmwareResponse(
                ok=True,
                missing=[],
                files=[FirmwareFile(offset=o, name=n, size=0) for o, n in [(0x1000, "bootloader.bin"), (0x8000, "partition-table.bin"), (0x10000, "embedded.bin")]],
            )
        return firmware_status()

    @app.get("/api/suggest-path")
    async def suggest_path(source: str = "usb", estado: str = "noestado") -> dict:
        # Sanitise estado so it can't break out of the filename
        safe_estado = "".join(c for c in estado if c.isalnum() or c in "-_") or "noestado"
        safe_source = source if source in ("usb", "wifi") else "usb"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"data_plant_{safe_source}_{ts}_{safe_estado}.csv"
        directory = _suggested_directory()
        return {"path": str(directory / filename)}

    @app.post("/api/validate-path", response_model=ValidatePathResponse)
    async def validate_path(req: ValidatePathRequest) -> ValidatePathResponse:
        p = Path(req.path)
        parent = p.parent
        exists = p.exists()
        parent_ok = parent.exists() and os.access(parent, os.W_OK)
        if not parent.exists():
            return ValidatePathResponse(ok=False, exists=exists, parent_writeable=False, reason="Parent directory does not exist.")
        if not parent_ok:
            return ValidatePathResponse(ok=False, exists=exists, parent_writeable=False, reason="Parent directory is not writeable.")
        return ValidatePathResponse(ok=True, exists=exists, parent_writeable=True)

    @app.get("/analysis", include_in_schema=False)
    async def analysis_page() -> FileResponse:
        return FileResponse(str(STATIC_DIR / "analysis.html"))

    @app.get("/api/models")
    async def get_models() -> JSONResponse:
        svc: AnalysisService = app.state.analysis
        if not svc.available:
            return JSONResponse(
                status_code=503,
                content={"detail": "Models not loaded. Run eda/modelo.ipynb first."},
            )
        return JSONResponse(svc.metrics)

    @app.get("/api/eda/plots")
    async def list_eda_plots() -> JSONResponse:
        from .analysis import EDA_PLOTS
        return JSONResponse({"plots": [{"id": k, "label": v} for k, v in EDA_PLOTS.items()]})

    @app.get("/api/eda/plot/{name}")
    async def get_eda_plot(name: str) -> Response:
        svc: AnalysisService = app.state.analysis
        if svc._df is None:
            return JSONResponse(
                status_code=503,
                content={"detail": "Data not loaded. Check that eda/datos*.csv files exist."},
            )
        try:
            png = await svc.render_plot_async(name)
        except KeyError:
            raise HTTPException(status_code=404, detail=f"Unknown plot: {name}")
        return Response(content=png, media_type="image/png")

    @app.get("/api/eda/confusion/{model_name}")
    async def get_confusion(model_name: str) -> FileResponse:
        png_path = MODELS_DIR / f"confusion_{model_name}.png"
        if not png_path.exists():
            raise HTTPException(status_code=404, detail=f"Confusion matrix not found for model: {model_name}")
        return FileResponse(str(png_path), media_type="image/png")

    @app.post("/api/session", response_model=SessionView, status_code=201)
    async def start_session(req: StartRequest) -> SessionView:
        # Port + firmware checks only apply to USB mode (WiFi skips the flash step entirely).
        if req.source == "usb":
            if not req.port:
                raise HTTPException(status_code=400, detail="USB mode requires a serial port.")
            if not MOCK and not port_exists(req.port):
                raise HTTPException(status_code=400, detail=f"Port not found: {req.port}")
            if not MOCK:
                try:
                    validate_firmware()
                except FileNotFoundError as exc:
                    raise HTTPException(status_code=503, detail=str(exc))
        session = await app.state.sessions.start(req, app.state.bus, mock=MOCK,
                                                  analysis=app.state.analysis)
        return session.to_view()

    @app.get("/api/session")
    async def get_session(response: Response) -> SessionView | None:
        s = app.state.sessions.current()
        if s is None:
            response.status_code = 204
            return None
        return s.to_view()

    @app.post("/api/session/stop", response_model=SessionView)
    async def stop_session() -> SessionView:
        s = await app.state.sessions.stop()
        if s is None:
            raise HTTPException(status_code=404, detail="No active session.")
        return s.to_view()

    @app.websocket("/ws/session")
    async def ws_session(ws: WebSocket) -> None:
        bus: WSBus = app.state.bus
        await bus.connect(ws)
        # Send current session state immediately on connect so fresh tabs sync up
        s = app.state.sessions.current()
        if s is not None:
            await ws.send_json({"type": "state", "session_id": s.id, "data": {"state": s.state, "row_count": s.row_count}})
        try:
            while True:
                await ws.receive_text()  # keep connection alive; client sends nothing
        except WebSocketDisconnect:
            pass
        finally:
            await bus.disconnect(ws)

    return app


app = create_app()

if __name__ == "__main__":
    uvicorn.run("frontend.app.main:app", host="127.0.0.1", port=8000, reload=False)
