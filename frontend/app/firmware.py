from pathlib import Path

from .models import FirmwareFile, FirmwareResponse

FIRMWARE_DIR = Path(__file__).resolve().parents[2] / "firmware"

# Offsets match embedded/partitions.csv and flash esp32.bat
FLASH_LAYOUT: list[tuple[int, str]] = [
    (0x1000,  "bootloader.bin"),
    (0x8000,  "partition-table.bin"),
    (0x10000, "embedded.bin"),
]


def validate_firmware() -> list[tuple[int, Path]]:
    """Return list of (offset, absolute_path) for each required binary.

    Raises FileNotFoundError listing all missing files if any are absent.
    """
    result: list[tuple[int, Path]] = []
    missing: list[str] = []
    for offset, name in FLASH_LAYOUT:
        p = FIRMWARE_DIR / name
        if p.exists():
            result.append((offset, p))
        else:
            missing.append(name)
    if missing:
        raise FileNotFoundError(f"Missing firmware files: {', '.join(missing)}")
    return result


def firmware_status() -> FirmwareResponse:
    missing: list[str] = []
    files: list[FirmwareFile] = []
    for offset, name in FLASH_LAYOUT:
        p = FIRMWARE_DIR / name
        if p.exists():
            files.append(FirmwareFile(offset=offset, name=name, size=p.stat().st_size))
        else:
            missing.append(name)
    return FirmwareResponse(ok=len(missing) == 0, missing=missing, files=files)
