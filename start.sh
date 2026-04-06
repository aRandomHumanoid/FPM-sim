#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON_BIN:-python3}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8000}"
AUTO_PORT="${AUTO_PORT:-1}"
MAX_PORT_SEARCH="${MAX_PORT_SEARCH:-25}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Error: $PYTHON_BIN not found. Install Python 3.10+ or set PYTHON_BIN." >&2
  exit 1
fi

if [[ ! -x ".venv/bin/python" ]]; then
  echo "[start] Creating virtual environment..."
  "$PYTHON_BIN" -m venv .venv
fi

VENV_PY=".venv/bin/python"

if ! "$VENV_PY" -m pip --version >/dev/null 2>&1; then
  echo "[start] pip missing; attempting bootstrap..."
  if ! "$VENV_PY" -m ensurepip --upgrade >/dev/null 2>&1; then
    if command -v curl >/dev/null 2>&1; then
      TMP_GET_PIP="/tmp/get-pip.py"
      curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "$TMP_GET_PIP"
      "$VENV_PY" "$TMP_GET_PIP"
    else
      echo "Error: pip bootstrap failed and curl is unavailable." >&2
      echo "Install curl or recreate the environment with a Python build that includes ensurepip." >&2
      exit 1
    fi
  fi
fi

if [[ "${SKIP_INSTALL:-0}" != "1" ]]; then
  echo "[start] Installing/updating dependencies..."
  "$VENV_PY" -m pip install --disable-pip-version-check -r requirements.txt
fi

is_port_free() {
  local host="$1"
  local port="$2"
  "$VENV_PY" - "$host" "$port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.bind((host, port))
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
}

if ! is_port_free "$HOST" "$PORT"; then
  if [[ "$AUTO_PORT" != "1" ]]; then
    echo "Error: $HOST:$PORT is already in use." >&2
    echo "Set a different port, for example: PORT=8010 ./start.sh" >&2
    exit 1
  fi

  original_port="$PORT"
  found_port=""
  for ((i=1; i<=MAX_PORT_SEARCH; i++)); do
    candidate="$((PORT + i))"
    if is_port_free "$HOST" "$candidate"; then
      found_port="$candidate"
      break
    fi
  done

  if [[ -z "$found_port" ]]; then
    echo "Error: $HOST:$PORT is in use, and no free port was found in the next $MAX_PORT_SEARCH ports." >&2
    echo "Try: PORT=8010 ./start.sh" >&2
    exit 1
  fi

  PORT="$found_port"
  echo "[start] Port $HOST:$original_port is busy; using $HOST:$PORT instead."
fi

echo "[start] Launching simulator at http://$HOST:$PORT"
echo "[start] Serial endpoint info: http://$HOST:$PORT/api/serial"

exec "$VENV_PY" -m uvicorn app.main:app --host "$HOST" --port "$PORT" --reload
