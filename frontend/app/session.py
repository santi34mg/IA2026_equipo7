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
    state: Literal["flashing", "recording", "stopping", "done", "error"]
    port: str
    estado: str
    out_path: Path
    started_at: datetime
    row_count: int = 0
    last_row: str | None = None
    error: str | None = None
    cancel_event: asyncio.Event = field(default_factory=asyncio.Event)
    task: asyncio.Task | None = None

    def to_view(self) -> SessionView:
        return SessionView(
            id=self.id,
            state=self.state,
            port=self.port,
            estado=self.estado,
            out_path=str(self.out_path),
            started_at=self.started_at,
            row_count=self.row_count,
            last_row=self.last_row,
            error=self.error,
        )


_ACTIVE_STATES = {"flashing", "recording", "stopping"}


class SessionManager:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._current: Session | None = None

    def current(self) -> Session | None:
        return self._current

    async def start(self, req: StartRequest, bus: "WSBus", mock: bool = False) -> Session:
        async with self._lock:
            if self._current and self._current.state in _ACTIVE_STATES:
                raise HTTPException(status_code=409, detail="A session is already in progress.")

            out = Path(req.out_path)
            if not out.parent.exists():
                raise HTTPException(status_code=400, detail=f"Directory does not exist: {out.parent}")
            if not _is_writeable(out.parent):
                raise HTTPException(status_code=400, detail=f"Directory is not writeable: {out.parent}")
            if out.exists() and not req.overwrite:
                raise HTTPException(status_code=409, detail="File already exists. Set overwrite=true to replace it.")

            session = Session(
                id=uuid.uuid4().hex,
                state="flashing",
                port=req.port,
                estado=req.estado,
                out_path=out,
                started_at=datetime.now(),
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
        import os
        from .firmware import validate_firmware
        from .flasher import flash_esp32, flash_esp32_mock
        from .recorder import record_to_csv, record_to_csv_mock

        flash_fn = flash_esp32_mock if mock else flash_esp32
        record_fn = record_to_csv_mock if mock else record_to_csv

        try:
            # -- Firmware pre-flight (skipped in mock mode) --
            if mock:
                layout = []
            else:
                try:
                    layout = validate_firmware()
                except FileNotFoundError as exc:
                    await self._fail(session, bus, str(exc))
                    return

            # -- Flash --
            await bus.broadcast({"type": "state", "session_id": session.id, "data": {"state": "flashing"}})

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
                await bus.broadcast({
                    "type": "record_row",
                    "session_id": session.id,
                    "data": {"row": row, "count": count},
                })

            await record_fn(
                session.port, session.out_path, session.estado,
                on_row, session.cancel_event,
            )

            # -- Done --
            session.state = "done"
            await bus.broadcast({
                "type": "stopped",
                "session_id": session.id,
                "data": {"out_path": str(session.out_path), "row_count": session.row_count},
            })

        except Exception as exc:
            await self._fail(session, bus, str(exc))

    async def _fail(self, session: Session, bus: "WSBus", reason: str) -> None:
        session.state = "error"
        session.error = reason
        await bus.broadcast({"type": "error", "session_id": session.id, "data": {"reason": reason}})


def _is_writeable(path: Path) -> bool:
    import os
    return os.access(path, os.W_OK)
