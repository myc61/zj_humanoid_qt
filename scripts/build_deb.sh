#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
APP_ID="humanoid-robot-delivery-toolchain"
PY_ENTRY="$PROJECT_ROOT/scripts/launch.py"
VENV_DIR="$PROJECT_ROOT/.pack-venv"
DIST_DIR="$PROJECT_ROOT/dist"
BUILD_DIR="$PROJECT_ROOT/build"
DEB_BUILD_ROOT="$BUILD_DIR/deb"
EDITION="internal"
FEATURE_CONFIG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --edition)
      EDITION="${2:-}"
      shift 2
      ;;
    --feature-config)
      FEATURE_CONFIG="${2:-}"
      shift 2
      ;;
    *)
      echo "[ERR] 未知参数: $1"
      echo "用法: $0 [--edition internal|customer] [--feature-config path/to/config.yaml]"
      exit 1
      ;;
  esac
done

if [[ "$EDITION" != "internal" && "$EDITION" != "customer" ]]; then
  echo "[ERR] edition 仅支持 internal 或 customer，当前: $EDITION"
  exit 1
fi

if [[ -z "$FEATURE_CONFIG" ]]; then
  FEATURE_CONFIG="$PROJECT_ROOT/config/feature_profiles/$EDITION.yaml"
fi
if [[ ! -f "$FEATURE_CONFIG" ]]; then
  echo "[ERR] 未找到功能配置文件: $FEATURE_CONFIG"
  exit 1
fi

if [[ ! -f "$PY_ENTRY" ]]; then
  echo "[ERR] 未找到入口文件: $PY_ENTRY"
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERR] 未找到 python3"
  exit 1
fi
if ! command -v dpkg-deb >/dev/null 2>&1; then
  echo "[ERR] 未找到 dpkg-deb，请先安装 dpkg-dev"
  exit 1
fi

VERSION="$(PROJECT_ROOT="$PROJECT_ROOT" python3 - <<'PY'
import os
import pathlib
try:
    import tomllib
except Exception:
    import tomli as tomllib
p = pathlib.Path(os.environ['PROJECT_ROOT']) / 'pyproject.toml'
data = tomllib.loads(p.read_text(encoding='utf-8'))
print(data.get('tool', {}).get('poetry', {}).get('version', '0.1.0'))
PY
)"

ARCH_RAW="$(uname -m)"
case "$ARCH_RAW" in
  x86_64) DEB_ARCH="amd64" ;;
  aarch64|arm64) DEB_ARCH="arm64" ;;
  *)
    echo "[ERR] 不支持的架构: $ARCH_RAW"
    exit 1
    ;;
esac

mkdir -p "$DIST_DIR" "$BUILD_DIR"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[INFO] 创建打包虚拟环境: $VENV_DIR"
  python3 -m venv "$VENV_DIR"
fi

copy_optional_runtime_tools() {
  local bundle_dir="$1"
  local xterm_bin=""
  xterm_bin="$(command -v xterm || true)"
  if [[ -n "$xterm_bin" && -x "$xterm_bin" ]]; then
    mkdir -p "$bundle_dir/tools/bin"
    cp "$xterm_bin" "$bundle_dir/tools/bin/xterm"
    chmod +x "$bundle_dir/tools/bin/xterm"
    echo "[INFO] 已打包运行时工具: xterm -> $bundle_dir/tools/bin/xterm"
  else
    echo "[WARN] 当前构建机未安装 xterm，DEB 内不会内置 xterm；安装时将依赖系统 xterm 包"
  fi
}

install_runtime_launcher() {
  local bundle_dir="$1"
  local app_bin="$bundle_dir/$APP_ID"
  local real_bin="$bundle_dir/$APP_ID.bin"

  if [[ ! -x "$app_bin" ]]; then
    echo "[ERR] 未找到运行入口: $app_bin"
    exit 1
  fi

  mv "$app_bin" "$real_bin"

  cat > "$app_bin" <<'EOF'
#!/usr/bin/env bash
set -e

SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_BIN="$SELF_DIR/humanoid-robot-delivery-toolchain.bin"
LOG_ROOT="${XDG_STATE_HOME:-$HOME/.local/state}/humanoid-robot-delivery-toolchain"
LOG_FILE="$LOG_ROOT/launcher.log"
XTERM_BIN="$SELF_DIR/tools/bin/xterm"

mkdir -p "$LOG_ROOT"

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

echo "[$(date '+%F %T')] launching $APP_BIN" >> "$LOG_FILE"
status=0
set +e
"$APP_BIN" "$@" >> "$LOG_FILE" 2>&1
status=$?
set -e
if [[ $status -ne 0 ]]; then
  msg="程序启动失败，退出码: $status\n日志: $LOG_FILE"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --width=520 --title="Humanoid Robot Delivery Toolchain" --text="$msg" || true
  elif [[ -x "$XTERM_BIN" ]]; then
    "$XTERM_BIN" -hold -e bash -lc "echo -e '$msg'; echo; tail -n 200 '$LOG_FILE'" || true
  fi
fi
exit $status
EOF

  chmod +x "$app_bin"
  echo "[INFO] 已安装运行包装器: $app_bin"
}

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip setuptools wheel
python -m pip install -r "$PROJECT_ROOT/requirements.txt" pyinstaller

TYPES_DIR="$PROJECT_ROOT/src/types"
TYPES_BAK_DIR="$PROJECT_ROOT/src/__types_build_backup__"
restore_types_dir() {
  if [[ -d "$TYPES_BAK_DIR" ]]; then
    mv "$TYPES_BAK_DIR" "$TYPES_DIR"
  fi
}
trap restore_types_dir EXIT
if [[ -d "$TYPES_DIR" ]]; then
  rm -rf "$TYPES_BAK_DIR"
  mv "$TYPES_DIR" "$TYPES_BAK_DIR"
fi

echo "[INFO] 构建 PyInstaller onedir 包..."
pyinstaller \
  --noconfirm \
  --clean \
  --windowed \
  --paths "$PROJECT_ROOT/src" \
  --add-data "$PROJECT_ROOT/check/jetpack_check.sh:check" \
  --add-data "$PROJECT_ROOT/config/wa2_ls_joint_data.yaml:config" \
  --add-data "$PROJECT_ROOT/scripts/docker_multi_deploy.sh:scripts" \
  --hidden-import app \
  --hidden-import matplotlib \
  --hidden-import matplotlib.pyplot \
  --hidden-import matplotlib.backends.backend_qtagg \
  --collect-data matplotlib \
  --collect-binaries matplotlib \
  --collect-submodules matplotlib \
  --collect-submodules mpl_toolkits \
  --name "$APP_ID" \
  --distpath "$DIST_DIR" \
  --workpath "$BUILD_DIR/pyinstaller" \
  --specpath "$BUILD_DIR/spec" \
  "$PY_ENTRY"

restore_types_dir
trap - EXIT

BUNDLE_DIR="$DIST_DIR/$APP_ID"
if [[ ! -d "$BUNDLE_DIR" ]]; then
  echo "[ERR] PyInstaller 输出不存在: $BUNDLE_DIR"
  exit 1
fi

cp "$FEATURE_CONFIG" "$BUNDLE_DIR/feature_config.yaml"
copy_optional_runtime_tools "$BUNDLE_DIR"
install_runtime_launcher "$BUNDLE_DIR"

PKG_NAME="${APP_ID}-${EDITION}"
PKG_DIR="$DEB_BUILD_ROOT/${PKG_NAME}_${VERSION}_${DEB_ARCH}"
rm -rf "$PKG_DIR"
mkdir -p \
  "$PKG_DIR/DEBIAN" \
  "$PKG_DIR/opt/$APP_ID" \
  "$PKG_DIR/usr/bin" \
  "$PKG_DIR/usr/share/applications" \
  "$PKG_DIR/usr/share/icons/hicolor/scalable/apps"

cp -a "$BUNDLE_DIR/." "$PKG_DIR/opt/$APP_ID/"
cp "$PROJECT_ROOT/packaging/$APP_ID.desktop" "$PKG_DIR/usr/share/applications/$APP_ID.desktop"
cp "$PROJECT_ROOT/packaging/$APP_ID.svg" "$PKG_DIR/usr/share/icons/hicolor/scalable/apps/$APP_ID.svg"

cat > "$PKG_DIR/usr/bin/$APP_ID" <<'EOF'
#!/usr/bin/env bash
set -e
exec /opt/humanoid-robot-delivery-toolchain/humanoid-robot-delivery-toolchain "$@"
EOF
chmod +x "$PKG_DIR/usr/bin/$APP_ID"

cat > "$PKG_DIR/DEBIAN/control" <<EOF
Package: $PKG_NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $DEB_ARCH
Maintainer: mayichao
Depends: libgl1, libxkbcommon-x11-0, libegl1, libdbus-1-3, libfontconfig1, libxcb-cursor0, xterm
Description: Humanoid robot delivery toolchain GUI
 PyQt6 desktop tool for ROSBridge/SSH robot delivery operations.
EOF

dpkg-deb --build "$PKG_DIR"
DEB_PATH="$DIST_DIR/${PKG_NAME}_${VERSION}_${DEB_ARCH}.deb"
mv "$PKG_DIR.deb" "$DEB_PATH"

echo "[OK] DEB 已生成: $DEB_PATH"
echo "[INFO] edition: $EDITION"
echo "[INFO] feature_config: $FEATURE_CONFIG"
echo "[NOTE] 该包已内置 Python 依赖，无需目标机器 pip install"
