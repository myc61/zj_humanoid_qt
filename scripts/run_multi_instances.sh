#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ "$#" -lt 1 ]]; then
  echo "用法: $0 机器人A [机器人B 机器人C ...]"
  echo "示例: $0 robot_66 robot_67 robot_68"
  exit 1
fi

if command -v humanoid-robot-delivery-toolchain >/dev/null 2>&1; then
  APP_CMD=(humanoid-robot-delivery-toolchain)
else
  APP_CMD=(python3 "$SCRIPT_DIR/launch.py")
fi

for instance in "$@"; do
  nohup "${APP_CMD[@]}" --instance-name "$instance" >/tmp/humanoid_toolchain_${instance}.log 2>&1 &
  echo "[OK] 已启动实例: $instance (PID=$!)"
done

echo "[INFO] 查看进程: ps -ef | grep humanoid-robot-delivery-toolchain | grep -v grep"
