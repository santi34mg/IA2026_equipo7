#!/usr/bin/env bash
set -e

FRONTEND_MOCK=1 PYTHONPATH=. .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
