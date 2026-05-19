import asyncio
from typing import Any

from fastapi import WebSocket


class WSBus:
    def __init__(self) -> None:
        self._subs: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._subs.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        self._subs.discard(ws)

    async def broadcast(self, event: dict[str, Any]) -> None:
        dead: set[WebSocket] = set()
        for ws in list(self._subs):
            try:
                await ws.send_json(event)
            except Exception:
                dead.add(ws)
        self._subs -= dead
