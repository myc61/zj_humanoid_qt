#!/bin/bash

set -e

# 脚本所在目录与项目根目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 运行时稳定性：避免外部 Qt 路径污染；Linux 下优先 xcb
unset QT_PLUGIN_PATH
unset QML2_IMPORT_PATH
export PYTHONFAULTHANDLER="${PYTHONFAULTHANDLER:-1}"
if [[ -z "${QT_QPA_PLATFORM:-}" ]]; then
	if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
		export QT_QPA_PLATFORM="wayland"
	elif [[ -n "${DISPLAY:-}" ]]; then
		export QT_QPA_PLATFORM="xcb"
	fi
fi
export QT_OPENGL="${QT_OPENGL:-software}"

# 默认固定使用系统 Python，避免被已激活的虚拟环境污染。
# 如需指定解释器，可通过环境变量 TOOLCHAIN_PYTHON 覆盖。
PY_BIN="${TOOLCHAIN_PYTHON:-/usr/bin/python3}"
if [[ ! -x "$PY_BIN" ]]; then
	echo "[ERR] Python 解释器不存在或不可执行: $PY_BIN" >&2
	exit 1
fi

echo "[INFO] 使用 Python: $PY_BIN"

# 运行应用程序（通过启动引导脚本，避免 src/types 覆盖标准库 types）
"$PY_BIN" "$SCRIPT_DIR/launch.py" "$@"