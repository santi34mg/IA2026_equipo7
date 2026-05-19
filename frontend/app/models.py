from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field


class PortInfo(BaseModel):
    device: str
    description: str
    hwid: str


class PortsResponse(BaseModel):
    ports: list[PortInfo]


class FirmwareFile(BaseModel):
    offset: int
    name: str
    size: int


class FirmwareResponse(BaseModel):
    ok: bool
    missing: list[str]
    files: list[FirmwareFile]


class ValidatePathRequest(BaseModel):
    path: str


class ValidatePathResponse(BaseModel):
    ok: bool
    exists: bool
    parent_writeable: bool
    reason: str | None = None


class StartRequest(BaseModel):
    port: str
    estado: str = Field(min_length=1, max_length=64)
    out_path: str
    overwrite: bool = False


class SessionView(BaseModel):
    id: str
    state: Literal["flashing", "recording", "stopping", "done", "error"]
    port: str
    estado: str
    out_path: str
    started_at: datetime
    row_count: int
    last_row: str | None
    error: str | None
