import asyncio
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from fastapi import HTTPException

from .models import SessionView, StartRequest

if TYPE_CHECKING:
    from .ws_bus import WSBus


@dataclass
class Session:
    id: str
    state: Literal["flashing", "discovering", "recording", "stopping", "done", "error"]
    source: Literal["usb", "wifi"]
    port: str | None
    estado: str
    out_path: Path
    started_at: datetime
    row_count: int = 0
    last_row: str | None = None
    error: str | None = None
    discovered_ip: str | None = None
    manual_ip: str | None = None  # set if user typed an IP to skip discovery
    cancel_event: asyncio.Event = field(default_factory=asyncio.Event)
    task: asyncio.Task | None = None
    analysis: object = None  # AnalysisService, optional

    def to_view(self) -> SessionView:
        return SessionView(
            id=self.id,
            state=self.state,
            source=self.source,
            port=self.port,
            discovered_ip=self.discovered_ip,
            estado=self.estado,
            out_path=str(self.out_path),
            started_at=self.started_at,
            row_count=self.row_count,
            last_row=self.last_row,
            error=self.error,
        )


_ACTIVE_STATES = {"flashing", "discovering", "recording", "stopping"}


class SessionManager:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._current: Session | None = None

    def current(self) -> Session | None:
        return self._current

    async def start(self, req: StartRequest, bus: "WSBus", mock: bool = False, analysis=None) -> Session:
        async with self._lock:
            if self._current and self._current.state in _ACTIVE_STATES:
                raise HTTPException(status_code=409, detail="A session is already in progress.")

            if req.source == "usb" and not req.port:
                raise HTTPException(status_code=400, detail="USB mode requires a serial port.")

            out = Path(req.out_path)
            if not out.parent.exists():
                raise HTTPException(status_code=400, detail=f"Directory does not exist: {out.parent}")
            if not _is_writeable(out.parent):
                raise HTTPException(status_code=400, detail=f"Directory is not writeable: {out.parent}")
            if out.exists() and not req.overwrite:
                raise HTTPException(status_code=409, detail="File already exists. Set overwrite=true to replace it.")

            initial_state: Literal["flashing", "discovering"] = "flashing" if req.source == "usb" else "discovering"

            manual_ip = (req.ip or "").strip() or None

            session = Session(
                id=uuid.uuid4().hex,
                state=initial_state,
                source=req.source,
                port=req.port,
                estado=req.estado,
                out_path=out,
                started_at=datetime.now(),
                manual_ip=manual_ip,
                analysis=analysis,
            )
            self._current = session

        session.task = asyncio.create_task(self._run(session, bus, mock))
        return session

    async def stop(self) -> Session | None:
        async with self._lock:
            s = self._current
            if s is None or s.state not in _ACTIVE_STATES:
                return s
            s.state = "stopping"
            s.cancel_event.set()
        return s

    async def _run(self, session: Session, bus: "WSBus", mock: bool) -> None:
        try:
            if session.source == "usb":
                await self._run_usb(session, bus, mock)
            else:
                await self._run_wifi(session, bus, mock)
        except Exception as exc:
            await self._fail(session, bus, str(exc))

    async def _run_usb(self, session: Session, bus: "WSBus", mock: bool) -> None:
        from .firmware import validate_firmware
        from .flasher import flash_esp32, flash_esp32_mock
        from .recorder import record_to_csv, record_to_csv_mock

        flash_fn = flash_esp32_mock if mock else flash_esp32
        record_fn = record_to_csv_mock if mock else record_to_csv

        # -- Flash --
        await bus.broadcast({"type": "state", "session_id": session.id, "data": {"state": "flashing"}})

        if mock:
            layout = []
        else:
            try:
                layout = validate_firmware()
            except FileNotFoundError as exc:
                await self._fail(session, bus, str(exc))
                return

        async def on_flash_line(line: str) -> None:
            await bus.broadcast({"type": "flash_log", "session_id": session.id, "data": {"line": line}})

        exit_code = await flash_fn(session.port, layout, on_flash_line, session.cancel_event)
        await bus.broadcast({"type": "flash_done", "session_id": session.id, "data": {"exit_code": exit_code}})

        if exit_code != 0 or session.cancel_event.is_set():
            await self._fail(session, bus, f"esptool exited with code {exit_code}")
            return

        # -- Record --
        session.state = "recording"
        await bus.broadcast({"type": "state", "session_id": session.id, "data": {"state": "recording"}})

        async def on_row(row: str, count: int) -> None:
            session.last_row = row
            session.row_count = count
            primary, preds = _compute_predictions(session, row)
            await bus.broadcast({
                "type": "record_row",
                "session_id": session.id,
                "data": {"row": row, "count": count, "predicted": primary},
            })
            if preds:
                asyncio.ensure_future(bus.broadcast({
                    "type": "multi_predict",
                    "session_id": session.id,
                    "data": preds,
                }))

        await record_fn(
            session.port, session.out_path, session.estado,
            on_row, session.cancel_event,
        )

        session.state = "done"
        await bus.broadcast({
            "type": "stopped",
            "session_id": session.id,
            "data": {"out_path": str(session.out_path), "row_count": session.row_count},
        })

    async def _run_wifi(self, session: Session, bus: "WSBus", mock: bool) -> None:
        from .discovery import Discovered, discover_esp32, discover_esp32_mock
        from .recorder import record_to_csv_tcp, record_to_csv_tcp_mock

        discover_fn = discover_esp32_mock if mock else discover_esp32
        record_fn = record_to_csv_tcp_mock if mock else record_to_csv_tcp

        if session.manual_ip:
            # User typed an IP — skip discovery entirely. Belt-and-suspenders for
            # networks where both UDP broadcast AND the TCP subnet sweep fail.
            discovered = Discovered(ip=session.manual_ip, tcp_port=8080)
            session.discovered_ip = discovered.ip
            await bus.broadcast({
                "type": "discovered",
                "session_id": session.id,
                "data": {"ip": discovered.ip, "tcp_port": discovered.tcp_port},
            })
        else:
            # -- Discovery (two-phase: UDP broadcast → TCP subnet sweep) --
            await bus.broadcast({"type": "state", "session_id": session.id, "data": {"state": "discovering"}})

            try:
                discovered = await discover_fn(cancel_event=session.cancel_event)
            except TimeoutError:
                await self._fail(session, bus, "Could not find ESP32 on the LAN. Is it powered and connected to WiFi? You can also type the IP manually in the WiFi card.")
                return
            except asyncio.CancelledError:
                session.state = "done"
                await bus.broadcast({"type": "stopped", "session_id": session.id, "data": {"out_path": str(session.out_path), "row_count": 0}})
                return
            except OSError as exc:
                await self._fail(session, bus, f"Discovery socket error: {exc}")
                return

            if session.cancel_event.is_set():
                session.state = "done"
                await bus.broadcast({"type": "stopped", "session_id": session.id, "data": {"out_path": str(session.out_path), "row_count": 0}})
                return

            session.discovered_ip = discovered.ip
            await bus.broadcast({
                "type": "discovered",
                "session_id": session.id,
                "data": {"ip": discovered.ip, "tcp_port": discovered.tcp_port},
            })

        # -- Record --
        session.state = "recording"
        await bus.broadcast({"type": "state", "session_id": session.id, "data": {"state": "recording"}})

        async def on_row(row: str, count: int) -> None:
            session.last_row = row
            session.row_count = count
            primary, preds = _compute_predictions(session, row)
            await bus.broadcast({
                "type": "record_row",
                "session_id": session.id,
                "data": {"row": row, "count": count, "predicted": primary},
            })
            if preds:
                asyncio.ensure_future(bus.broadcast({
                    "type": "multi_predict",
                    "session_id": session.id,
                    "data": preds,
                }))

        await record_fn(
            discovered.ip, discovered.tcp_port, session.out_path, session.estado,
            on_row, session.cancel_event,
        )

        session.state = "done"
        await bus.broadcast({
            "type": "stopped",
            "session_id": session.id,
            "data": {"out_path": str(session.out_path), "row_count": session.row_count},
        })

    async def _fail(self, session: Session, bus: "WSBus", reason: str) -> None:
        session.state = "error"
        session.error = reason
        await bus.broadcast({"type": "error", "session_id": session.id, "data": {"reason": reason}})


def _is_writeable(path: Path) -> bool:
    import os
    return os.access(path, os.W_OK)


_PRIMARY_MODEL = "LogisticRegression"


def _compute_predictions(session: "Session", row: str) -> tuple[str | None, dict]:
    """Run all loaded models on the row and return (LogisticRegression, all_preds).

    Row format: timestamp,dht_temp,dht_hum,ks_temp,light,soil,estado
    Features: [dht_temp(col1), light(col4), soil(col5)]
    """
    if session.analysis is None or not session.analysis.available:
        return None, {}
    try:
        parts = row.split(",")
        if len(parts) < 6:
            return None, {}
        temp     = float(parts[1])
        light    = float(parts[4])
        moisture = float(parts[5])
    except (ValueError, IndexError):
        return None, {}

    preds = session.analysis.predict_all([temp, light, moisture]) or {}
    return preds.get(_PRIMARY_MODEL), preds
