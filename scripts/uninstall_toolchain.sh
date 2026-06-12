#!/usr/bin/env bash
set -euo pipefail

APP_ID="humanoid-robot-delivery-toolchain"
PURGE=0

if [[ "${1:-}" == "--purge" ]]; then
  PURGE=1
fi

echo "[INFO] 开始卸载 $APP_ID"

if command -v dpkg-query >/dev/null 2>&1 && dpkg-query -W -f='${Status}' "$APP_ID" 2>/dev/null | grep -q "installed"; then
  if [[ "$PURGE" -eq 1 ]]; then
    echo "[STEP] apt purge $APP_ID"
    sudo apt purge -y "$APP_ID"
  else
    echo "[STEP] apt remove $APP_ID"
    sudo apt remove -y "$APP_ID"
  fi
  sudo apt -f install -y || true
  echo "[OK] DEB 安装已卸载"
else
  echo "[INFO] 未检测到 DEB 安装（$APP_ID）"
fi

# AppImage 通常是免安装，这里只清理常见用户侧快捷方式（如有）
USER_DESKTOP_FILE="$HOME/.local/share/applications/$APP_ID.desktop"
if [[ -f "$USER_DESKTOP_FILE" ]]; then
  rm -f "$USER_DESKTOP_FILE"
  echo "[OK] 已删除用户快捷方式: $USER_DESKTOP_FILE"
fi

if [[ "$PURGE" -eq 1 ]]; then
  # 仅清理本项目构建时的打包虚拟环境，不会影响系统或其他项目 pip 包
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
  PACK_VENV="$PROJECT_ROOT/.pack-venv"
  if [[ -d "$PACK_VENV" ]]; then
    rm -rf "$PACK_VENV"
    echo "[OK] 已删除打包虚拟环境: $PACK_VENV"
  fi
fi

echo "[DONE] 卸载完成"
echo "[NOTE] 不会卸载系统 Python 或你测试环境里的 pip 包。"
