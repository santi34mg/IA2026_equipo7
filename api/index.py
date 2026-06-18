import sys
from pathlib import Path

# Make `frontend/` importable as a package root
sys.path.insert(0, str(Path(__file__).parent.parent / "frontend"))

from app.main import app  # noqa: E402 — path must be set first
