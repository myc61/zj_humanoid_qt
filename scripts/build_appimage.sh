#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
APP_ID="humanoid-robot-delivery-toolchain"
PY_ENTRY="$PROJECT_ROOT/scripts/launch.py"
VENV_DIR="$PROJECT_ROOT/.pack-venv"
DIST_DIR="$PROJECT_ROOT/dist"
BUILD_DIR="$PROJECT_ROOT/build"
APPDIR="$DIST_DIR/AppDir"
TOOLS_DIR="$PROJECT_ROOT/tools"
EDITION="internal"
FEATURE_CONFIG=""
KEEP_INTERMEDIATES=0

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
    --keep-intermediates)
      KEEP_INTERMEDIATES=1
      shift
      ;;
    *)
      echo "[ERR] 未知参数: $1"
      echo "用法: $0 [--edition internal|customer] [--feature-config path/to/config.yaml] [--keep-intermediates]"
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
  x86_64) APPIMAGE_ARCH="x86_64" ;;
  aarch64|arm64) APPIMAGE_ARCH="aarch64" ;;
  *)
    echo "[ERR] 不支持的架构: $ARCH_RAW"
    exit 1
    ;;
esac

mkdir -p "$DIST_DIR" "$BUILD_DIR" "$TOOLS_DIR"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[INFO] 创建打包虚拟环境: $VENV_DIR"
  python3 -m venv "$VENV_DIR"
fi

copy_path_preserve_layout() {
  local src="$1"
  local dest_root="$2"
  if [[ -L "$src" || -f "$src" ]]; then
    mkdir -p "$dest_root"
    cp -a --parents "$src" "$dest_root"
    return
  fi

  if [[ -d "$src" ]]; then
    case "$src" in
      /|/etc|/usr|/usr/bin|/usr/share|/usr/share/doc|/usr/share/man|/usr/share/icons|/usr/share/pixmaps|/etc/X11)
        return
        ;;
    esac
    mkdir -p "$dest_root$src"
  fi
}

copy_binary_linked_libs() {
  local binary_path="$1"
  local dest_root="$2"
  ldd "$binary_path" | while IFS= read -r line; do
    local lib_path=""
    lib_path="$(awk '
      /=> \/.*/ {print $3; exit}
      /^\// {print $1; exit}
    ' <<< "$line")"
    if [[ -n "$lib_path" && -e "$lib_path" ]]; then
      copy_path_preserve_layout "$lib_path" "$dest_root"
    fi
  done
}

copy_package_contents() {
  local package_name="$1"
  local dest_root="$2"
  if [[ -z "$package_name" ]]; then
    return
  fi
  if ! dpkg -L "$package_name" >/dev/null 2>&1; then
    return
  fi
  while IFS= read -r pkg_path; do
    [[ -n "$pkg_path" ]] || continue
    [[ "$pkg_path" == "/." ]] && continue
    copy_path_preserve_layout "$pkg_path" "$dest_root"
  done < <(dpkg -L "$package_name")
}

bundle_xterm_runtime() {
  local bundle_dir="$1"
  local xterm_bin=""
  local xterm_root="$bundle_dir/tools/xterm-root"
  local wrapper_path="$bundle_dir/tools/bin/xterm"
  xterm_bin="$(command -v xterm || true)"

  if [[ -z "$xterm_bin" || ! -x "$xterm_bin" ]]; then
    echo "[WARN] 当前构建机未安装 xterm，AppImage 内不会内置 xterm；目标机将继续使用系统终端回退"
    return
  fi
  if ! command -v dpkg >/dev/null 2>&1; then
    echo "[WARN] 当前构建机缺少 dpkg，无法按包内容打包 xterm；目标机将继续使用系统终端回退"
    return
  fi

  rm -rf "$xterm_root"
  mkdir -p "$xterm_root" "$bundle_dir/tools/bin"

  copy_package_contents xterm "$xterm_root"
  copy_package_contents libutempter0 "$xterm_root"
  copy_binary_linked_libs "$xterm_bin" "$xterm_root"

  cat > "$wrapper_path" <<'EOF'
#!/usr/bin/env bash
set -e
SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
XTERM_ROOT="$(cd "$SELF_DIR/.." && pwd)/xterm-root"

append_path_if_exists() {
  local target="$1"
  if [[ -d "$target" ]]; then
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
      export LD_LIBRARY_PATH="$target:$LD_LIBRARY_PATH"
    else
      export LD_LIBRARY_PATH="$target"
    fi
  fi
}

append_path_if_exists "$XTERM_ROOT/lib"
append_path_if_exists "$XTERM_ROOT/lib64"
append_path_if_exists "$XTERM_ROOT/usr/lib"
append_path_if_exists "$XTERM_ROOT/lib/x86_64-linux-gnu"
append_path_if_exists "$XTERM_ROOT/usr/lib/x86_64-linux-gnu"
append_path_if_exists "$XTERM_ROOT/lib/aarch64-linux-gnu"
append_path_if_exists "$XTERM_ROOT/usr/lib/aarch64-linux-gnu"

if [[ -d "$XTERM_ROOT/etc/X11/app-defaults" ]]; then
  export XAPPLRESDIR="$XTERM_ROOT/etc/X11/app-defaults"
fi

exec "$XTERM_ROOT/usr/bin/xterm" "$@"
EOF
  chmod +x "$wrapper_path"
  echo "[INFO] 已将 xterm 整包运行时打入 AppImage: $wrapper_path"
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
bundle_xterm_runtime "$BUNDLE_DIR"
install_runtime_launcher "$BUNDLE_DIR"

rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/lib/$APP_ID" "$APPDIR/usr/share/applications" "$APPDIR/usr/share/icons/hicolor/scalable/apps"
cp -a "$BUNDLE_DIR/." "$APPDIR/usr/lib/$APP_ID/"

cp "$PROJECT_ROOT/packaging/$APP_ID.desktop" "$APPDIR/$APP_ID.desktop"
cp "$PROJECT_ROOT/packaging/$APP_ID.desktop" "$APPDIR/usr/share/applications/$APP_ID.desktop"
cp "$PROJECT_ROOT/packaging/$APP_ID.svg" "$APPDIR/$APP_ID.svg"
cp "$PROJECT_ROOT/packaging/$APP_ID.svg" "$APPDIR/usr/share/icons/hicolor/scalable/apps/$APP_ID.svg"
ln -sf "$APP_ID.svg" "$APPDIR/.DirIcon"

cat > "$APPDIR/AppRun" <<'EOF'
#!/usr/bin/env bash
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "$HERE/usr/lib/humanoid-robot-delivery-toolchain/humanoid-robot-delivery-toolchain" "$@"
EOF
chmod +x "$APPDIR/AppRun"

APPIMAGETOOL="$TOOLS_DIR/appimagetool-$APPIMAGE_ARCH.AppImage"
RUNTIME_FILE="$TOOLS_DIR/runtime-$APPIMAGE_ARCH"

download_file() {
  local url="$1"
  local out="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -fL --retry 3 --retry-delay 2 "$url" -o "$out"
  elif command -v wget >/dev/null 2>&1; then
    wget --tries=3 -O "$out" "$url"
  else
    echo "[ERR] 需要 curl 或 wget 下载文件: $url"
    exit 1
  fi
}

if [[ ! -f "$APPIMAGETOOL" ]]; then
  echo "[INFO] 下载 appimagetool..."
  URL="https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-$APPIMAGE_ARCH.AppImage"
  download_file "$URL" "$APPIMAGETOOL"
  chmod +x "$APPIMAGETOOL"
fi

if [[ ! -s "$RUNTIME_FILE" ]]; then
  echo "[INFO] 下载 AppImage runtime..."
  RUNTIME_URL="https://github.com/AppImage/type2-runtime/releases/download/continuous/runtime-$APPIMAGE_ARCH"
  download_file "$RUNTIME_URL" "$RUNTIME_FILE"
fi

OUTPUT="$DIST_DIR/${APP_ID}-${EDITION}-${VERSION}-${APPIMAGE_ARCH}.AppImage"
OUTPUT_TMP="$OUTPUT.tmp"
rm -f "$OUTPUT_TMP"
ARCH="$APPIMAGE_ARCH" "$APPIMAGETOOL" --runtime-file "$RUNTIME_FILE" "$APPDIR" "$OUTPUT_TMP"
mv -f "$OUTPUT_TMP" "$OUTPUT"
SHA256_FILE="$OUTPUT.sha256"
sha256sum "$OUTPUT" > "$SHA256_FILE"

if [[ "$KEEP_INTERMEDIATES" != "1" ]]; then
  rm -rf "$BUNDLE_DIR" "$APPDIR"
fi

echo "[OK] AppImage 已生成: $OUTPUT"
echo "[INFO] SHA256 校验文件: $SHA256_FILE"
echo "[INFO] edition: $EDITION"
echo "[INFO] feature_config: $FEATURE_CONFIG"
if [[ "$KEEP_INTERMEDIATES" == "1" ]]; then
  echo "[INFO] 已保留中间产物: $BUNDLE_DIR, $APPDIR"
else
  echo "[INFO] 已清理中间产物，仅保留最终 AppImage"
fi
echo "[NOTE] 该包已内置 Python 依赖，无需目标机器 pip install"
