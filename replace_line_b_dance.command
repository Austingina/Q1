#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
TOOL_DEPS="${LINE_B_TOOL_DEPS:-$ROOT/infra/.line_b_tool_deps}"

if [[ -x "$ROOT/infra/.venv/bin/python" ]]; then
  PYTHON="$ROOT/infra/.venv/bin/python"
elif [[ -n "${LINE_B_PYTHON:-}" ]]; then
  PYTHON="$LINE_B_PYTHON"
elif command -v python3.13 >/dev/null 2>&1; then
  PYTHON="$(command -v python3.13)"
else
  PYTHON="$(command -v python3)"
fi

if [[ $# -eq 0 ]]; then
  if command -v osascript >/dev/null 2>&1; then
    SOURCE="$(osascript -e 'POSIX path of (choose folder with prompt "选择完整舞蹈包文件夹（必须包含 policy.onnx、deploy.yaml、motion.npz）")')"
  else
    echo "请输入新的完整舞蹈包目录或 ZIP 路径："
    IFS= read -r SOURCE
  fi
  set -- "$SOURCE"
fi

export PYTHONPATH="$ROOT:$TOOL_DEPS${PYTHONPATH:+:$PYTHONPATH}"
if ! "$PYTHON" -c 'import mujoco, numpy, onnxruntime, yaml' >/dev/null 2>&1; then
  echo "首次运行：正在安装舞蹈包校验与 MuJoCo 验收依赖…"
  mkdir -p "$TOOL_DEPS"
  "$PYTHON" -m pip install --disable-pip-version-check --target "$TOOL_DEPS" \
    -r "$ROOT/scripts/line_b_dance_tool_requirements.txt"
fi

exec "$PYTHON" "$ROOT/scripts/replace_line_b_dance.py" "$@"
