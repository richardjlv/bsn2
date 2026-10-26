#!/usr/bin/env bash
# Usar depois de compilar e carregar o workspace ROS 2 correspondente.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$SCRIPT_DIR/tools/collect_coverage.py" "$@"
