#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

REQUIRED_PIP_PACKAGES=(
  "PyQt6>=6.5"
  "roslibpy>=1.6"
  "paramiko>=2.11"
  "requests>=2.28"
  "PyYAML>=6.0"
  "pyinstaller"
)

# Ubuntu/Debian runtime packages for build and Qt startup.
REQUIRED_APT_PACKAGES=(
  python3
  python3-venv
  python3-pip
  dpkg-dev
  curl
  libxcb-cursor0
  libxkbcommon-x11-0
  libegl1
  libgl1
  libopengl0
  libxcb-icccm4
  libxcb-image0
  libxcb-keysyms1
  libxcb-randr0
  libxcb-render-util0
  libxcb-shape0
  libxcb-xfixes0
  libxrender1
  libxi6
  libxkbcommon0
  libdbus-1-3
)

INSTALLED_APT_PACKAGES=()
NEWLY_INSTALLED_APT_PACKAGES=()
MISSING_APT_PACKAGES=()
FAILED_APT_PACKAGES=()

PYTHON_OK="no"
VENV_OK="no"
PIP_OK="no"
PIP_INSTALL_OK="no"

APT_UPDATED="no"

log_info() {
  echo "[INFO] $*"
}

log_warn() {
  echo "[WARN] $*"
}

log_err() {
  echo "[ERR] $*" >&2
}

can_use_apt() {
  command -v apt-get >/dev/null 2>&1
}

run_apt() {
  if [[ $(id -u) -eq 0 ]]; then
    apt-get "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo apt-get "$@"
  else
    return 1
  fi
}

is_pkg_installed() {
  dpkg -s "$1" >/dev/null 2>&1
}

ensure_apt_update_once() {
  if [[ "$APT_UPDATED" == "no" ]]; then
    log_info "正在执行 apt-get update ..."
    run_apt update
    APT_UPDATED="yes"
  fi
}

install_apt_package() {
  local pkg="$1"
  ensure_apt_update_once
  log_info "安装系统依赖: $pkg"
  if run_apt install -y "$pkg"; then
    NEWLY_INSTALLED_APT_PACKAGES+=("$pkg")
    return 0
  fi
  return 1
}

check_python_tools() {
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_OK="yes"
  fi

  if [[ "$PYTHON_OK" == "yes" ]] && python3 -m venv --help >/dev/null 2>&1; then
    VENV_OK="yes"
  fi

  if [[ "$PYTHON_OK" == "yes" ]] && python3 -m pip --version >/dev/null 2>&1; then
    PIP_OK="yes"
  fi
}

install_pip_dependencies() {
  log_info "升级 pip/setuptools/wheel"
  python3 -m pip install --upgrade pip setuptools wheel

  log_info "安装 Python 依赖: 固定包列表 + pyinstaller"
  python3 -m pip install "${REQUIRED_PIP_PACKAGES[@]}"
}

print_list() {
  local title="$1"
  shift
  local items=("$@")
  echo "$title"
  if [[ ${#items[@]} -eq 0 ]]; then
    echo "  - (none)"
    return
  fi
  local it
  for it in "${items[@]}"; do
    echo "  - $it"
  done
}

main() {
  log_info "项目根目录: $PROJECT_ROOT"

  if ! can_use_apt; then
    log_err "未找到 apt-get，本脚本当前仅支持 Ubuntu/Debian。"
    exit 1
  fi

  if [[ $(id -u) -ne 0 ]] && ! command -v sudo >/dev/null 2>&1; then
    log_err "需要 root 或 sudo 权限来安装系统依赖。"
    exit 1
  fi

  log_info "步骤 1/3: 检查并安装系统依赖"
  local pkg
  for pkg in "${REQUIRED_APT_PACKAGES[@]}"; do
    if is_pkg_installed "$pkg"; then
      INSTALLED_APT_PACKAGES+=("$pkg")
      continue
    fi

    MISSING_APT_PACKAGES+=("$pkg")
    if is_pkg_installed "$pkg"; then
      INSTALLED_APT_PACKAGES+=("$pkg")
      continue
    fi
    if install_apt_package "$pkg"; then
      INSTALLED_APT_PACKAGES+=("$pkg")
    else
      FAILED_APT_PACKAGES+=("$pkg")
    fi
  done

  log_info "步骤 2/3: 校验 python3/venv/pip"
  check_python_tools

  if [[ "$PYTHON_OK" != "yes" || "$VENV_OK" != "yes" || "$PIP_OK" != "yes" ]]; then
    log_err "Python 工具链不完整，无法继续安装 pip 依赖。"
  else
    log_info "步骤 3/3: 安装 pip 依赖"
    if install_pip_dependencies; then
      PIP_INSTALL_OK="yes"
    else
      PIP_INSTALL_OK="no"
    fi
  fi

  echo
  echo "========== 环境检查报告 =========="
  echo "Python3 可用       : $PYTHON_OK"
  echo "venv 可用          : $VENV_OK"
  echo "pip 可用           : $PIP_OK"
  echo "pip 依赖安装成功   : $PIP_INSTALL_OK"
  echo
  print_list "已安装/可用的系统包:" "${INSTALLED_APT_PACKAGES[@]}"
  echo
  print_list "本次新安装的系统包:" "${NEWLY_INSTALLED_APT_PACKAGES[@]}"
  echo
  print_list "检测到缺失的系统包:" "${MISSING_APT_PACKAGES[@]}"
  echo
  print_list "安装失败的系统包:" "${FAILED_APT_PACKAGES[@]}"
  echo "================================="

  if [[ ${#FAILED_APT_PACKAGES[@]} -gt 0 ]]; then
    exit 2
  fi
  if [[ "$PYTHON_OK" != "yes" || "$VENV_OK" != "yes" || "$PIP_OK" != "yes" || "$PIP_INSTALL_OK" != "yes" ]]; then
    exit 3
  fi
}

main "$@"