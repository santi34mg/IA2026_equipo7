"""UDP discovery of the ESP32 plant monitor on the local network.

Two-phase strategy:

  Phase A — UDP broadcast (fast, ~4 s)
      The ESP32 (firmware v2+) broadcasts a packet of the form
          PLANT_MONITOR:<ip>:<tcp_port>\\n
      to 255.255.255.255:8081 every ~3 seconds while it's online. The laptop
      binds UDP 8081 and waits for one. Works on any LAN that forwards
      broadcast traffic.

  Phase B — TCP subnet sweep (fallback, ~3-6 s)
      Many networks block UDP broadcast between clients (mobile hotspots,
      guest WiFi, enterprise networks). On those, Phase A times out — but
      direct TCP between clients usually still works. We sweep every IP in
      our /24 subnet, try connecting to port 8080, and read for a sensor row
      (firmware emits one every ~2.5 s). Whoever speaks CSV is the ESP.

Both phases honor cancel_event so the Stop button aborts cleanly.
"""

import asyncio
import socket
from collections.abc import Callable
from dataclasses import dataclass

DISCOVERY_PORT = 8081
ESP_TCP_PORT = 8080
DEFAULT_TIMEOUT_S = 10.0
UDP_PHASE_TIMEOUT_S = 4.0
SWEEP_CONCURRENCY = 64
SWEEP_CONNECT_TIMEOUT_S = 0.6
SWEEP_READ_TIMEOUT_S = 3.2  # must exceed the firmware's 2.5 s sample period


@dataclass
class Discovered:
    ip: str
    tcp_port: int


# ─────────────────────────── Phase A: UDP broadcast ───────────────────────────

async def _discover_via_udp(timeout_s: float, cancel_event: asyncio.Event | None) -> Discovered:
    loop = asyncio.get_running_loop()
    result_queue: asyncio.Queue[Discovered] = asyncio.Queue(maxsize=1)
    stop_flag = {"stop": False}

    def _blocking_listen() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", DISCOVERY_PORT))
        except OSError:
            sock.close()
            return
        sock.settimeout(0.5)
        try:
            while not stop_flag["stop"]:
                try:
                    data, _ = sock.recvfrom(128)
                except socket.timeout:
                    continue
                try:
                    payload = data.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                if not payload.startswith("PLANT_MONITOR:"):
                    continue
                parts = payload.split(":")
                if len(parts) < 3:
                    continue
                try:
                    discovered = Discovered(ip=parts[1].strip(), tcp_port=int(parts[2].strip()))
                except ValueError:
                    continue
                loop.call_soon_threadsafe(result_queue.put_nowait, discovered)
                return
        finally:
            sock.close()

    listener = asyncio.create_task(asyncio.to_thread(_blocking_listen))
    try:
        if cancel_event is not None:
            cancel_task = asyncio.create_task(cancel_event.wait())
            get_task = asyncio.create_task(result_queue.get())
            try:
                done, _ = await asyncio.wait(
                    {cancel_task, get_task},
                    timeout=timeout_s,
                    return_when=asyncio.FIRST_COMPLETED,
                )
            finally:
                for t in (cancel_task, get_task):
                    if not t.done():
                        t.cancel()
            if cancel_task in done:
                raise asyncio.CancelledError("Discovery cancelled")
            if get_task in done:
                return get_task.result()
            raise TimeoutError("No ESP32 announce received")
        return await asyncio.wait_for(result_queue.get(), timeout=timeout_s)
    finally:
        stop_flag["stop"] = True
        try:
            await asyncio.wait_for(listener, timeout=2.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            pass


# ────────────────────────── Phase B: TCP subnet sweep ──────────────────────────

def _get_local_ip() -> str | None:
    """Returns the laptop's IP on the interface used to reach the internet, or None."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No packet actually sent — this just picks the right interface.
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def _looks_like_sensor_row(text: str) -> bool:
    """Any line of 6+ comma-separated fields whose first field is numeric."""
    for line in text.splitlines():
        parts = line.split(",")
        if len(parts) < 6:
            continue
        try:
            float(parts[0])
            return True
        except ValueError:
            continue
    return False


async def _probe_tcp(ip: str, port: int) -> str | None:
    """Try TCP connect + read. Returns ip if it looks like an ESP, else None."""
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(ip, port),
            timeout=SWEEP_CONNECT_TIMEOUT_S,
        )
    except (OSError, asyncio.TimeoutError):
        return None

    try:
        try:
            data = await asyncio.wait_for(reader.read(512), timeout=SWEEP_READ_TIMEOUT_S)
        except asyncio.TimeoutError:
            return None
        if not data:
            return None
        text = data.decode("utf-8", errors="replace")
        return ip if _looks_like_sensor_row(text) else None
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


async def _discover_via_subnet_sweep(
    timeout_s: float,
    cancel_event: asyncio.Event | None,
) -> Discovered:
    local_ip = _get_local_ip()
    if not local_ip:
        raise TimeoutError("Could not determine local IP for subnet sweep")
    prefix = local_ip.rsplit(".", 1)[0]
    candidates = [f"{prefix}.{i}" for i in range(1, 255) if f"{prefix}.{i}" != local_ip]

    sem = asyncio.Semaphore(SWEEP_CONCURRENCY)

    async def bounded_probe(ip: str) -> str | None:
        async with sem:
            return await _probe_tcp(ip, ESP_TCP_PORT)

    tasks = {asyncio.create_task(bounded_probe(ip)): ip for ip in candidates}
    cancel_task = asyncio.create_task(cancel_event.wait()) if cancel_event else None

    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    found: str | None = None
    cancelled = False
    try:
        while tasks and found is None and not cancelled:
            remaining = deadline - loop.time()
            if remaining <= 0:
                break
            wait_set: set[asyncio.Task] = set(tasks.keys())
            if cancel_task is not None:
                wait_set.add(cancel_task)
            done, _ = await asyncio.wait(
                wait_set,
                timeout=remaining,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if not done:
                break
            if cancel_task is not None and cancel_task in done:
                cancelled = True
                break
            for d in done:
                if d in tasks:
                    tasks.pop(d)
                    try:
                        result = d.result()
                    except Exception:
                        result = None
                    if result:
                        found = result
                        break
    finally:
        for t in tasks:
            if not t.done():
                t.cancel()
        if cancel_task is not None and not cancel_task.done():
            cancel_task.cancel()

    if cancelled:
        raise asyncio.CancelledError("Sweep cancelled")
    if not found:
        raise TimeoutError("No ESP32 found via subnet sweep")
    return Discovered(ip=found, tcp_port=ESP_TCP_PORT)


# ─────────────────────────── Public entry points ───────────────────────────

async def discover_esp32(
    timeout_s: float = DEFAULT_TIMEOUT_S,
    cancel_event: asyncio.Event | None = None,
    on_phase: Callable[[str], None] | None = None,
) -> Discovered:
    """Two-phase discovery — broadcast first, subnet sweep as fallback."""
    if on_phase:
        on_phase("broadcast")
    udp_budget = min(UDP_PHASE_TIMEOUT_S, timeout_s)
    try:
        return await _discover_via_udp(timeout_s=udp_budget, cancel_event=cancel_event)
    except TimeoutError:
        pass

    if cancel_event is not None and cancel_event.is_set():
        raise asyncio.CancelledError("Discovery cancelled before sweep")

    if on_phase:
        on_phase("sweep")
    sweep_budget = max(timeout_s - udp_budget, 3.0)
    return await _discover_via_subnet_sweep(timeout_s=sweep_budget, cancel_event=cancel_event)


async def discover_esp32_mock(
    timeout_s: float = DEFAULT_TIMEOUT_S,
    cancel_event: asyncio.Event | None = None,
    on_phase: Callable[[str], None] | None = None,
) -> Discovered:
    """Mock discovery — pretends to find an ESP32 after ~1.5s."""
    if on_phase:
        on_phase("broadcast")
    delay = min(1.5, timeout_s)
    try:
        if cancel_event is not None:
            await asyncio.wait_for(cancel_event.wait(), timeout=delay)
            raise asyncio.CancelledError("Discovery cancelled")
        else:
            await asyncio.sleep(delay)
    except asyncio.TimeoutError:
        pass
    return Discovered(ip="192.168.99.99", tcp_port=8080)
