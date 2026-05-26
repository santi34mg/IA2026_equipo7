import asyncio
import os
import sys
from collections.abc import Awaitable, Callable
from pathlib import Path


async def flash_esp32(
    port: str,
    layout: list[tuple[int, Path]],
    on_line: Callable[[str], Awaitable[None]],
    cancel_event: asyncio.Event,
) -> int:
    """Flash the ESP32 using esptool. Returns the process exit code.

    Replicates the exact argv from 'flash esp32.bat' but cross-platform:
    chip=esp32, baud=460800, flash-mode=dio, flash-size=2MB, flash-freq=40m,
    --before default-reset --after hard-reset.
    """
    addr_file_args: list[str] = []
    for offset, path in layout:
        addr_file_args += [hex(offset), str(path)]

    argv = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32",
        "-p", port,
        "-b", "460800",
        "--before", "default-reset",
        "--after", "hard-reset",
        "write-flash",
        "--flash-mode", "dio",
        "--flash-size", "2MB",
        "--flash-freq", "40m",
        *addr_file_args,
    ]

    proc = await asyncio.create_subprocess_exec(
        *argv,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )

    assert proc.stdout is not None

    async def _read_output() -> None:
        async for raw_line in proc.stdout:  # type: ignore[union-attr]
            line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            await on_line(line)

    read_task = asyncio.create_task(_read_output())

    async def _watch_cancel() -> None:
        await cancel_event.wait()
        try:
            proc.terminate()
        except ProcessLookupError:
            pass
        await asyncio.sleep(3)
        try:
            proc.kill()
        except ProcessLookupError:
            pass

    cancel_task = asyncio.create_task(_watch_cancel())

    try:
        await proc.wait()
    finally:
        read_task.cancel()
        cancel_task.cancel()
        try:
            await read_task
        except asyncio.CancelledError:
            pass
        try:
            await cancel_task
        except asyncio.CancelledError:
            pass

    return proc.returncode or 0


async def flash_esp32_mock(
    port: str,
    layout: list[tuple[int, Path]],
    on_line: Callable[[str], Awaitable[None]],
    cancel_event: asyncio.Event,
) -> int:
    for i in range(1, 21):
        if cancel_event.is_set():
            return 1
        pct = i * 5
        await on_line(f"Writing at 0x00010000... ({pct} %)")
        await asyncio.sleep(0.25)
    await on_line("Hash of data verified.")
    await on_line("Leaving...")
    await on_line("Hard resetting via RTS pin...")
    return 0
