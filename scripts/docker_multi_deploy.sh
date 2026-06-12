#!/usr/bin/env bash
# docker_multi_deploy.sh - 多镜像Docker部署管理脚本

# 定义服务与默认镜像
declare -A SERVICES=(
  ["mani"]="manipulation:latest"
  ["mani22"]="manipulation2204e:latest"
  ["sen22"]="sensor_22CUDA:latest"
  ["sen20"]="sensor_20CUDA:latest"
  ["navbrain"]="navbrain_ros:latest"
  ["nviz"]="nviz:latest"
  ["envi"]="environment_ros1_260116:latest"
  ["ros"]="ros_core:latest"
  ["demos"]="demos:latest"
)

CONTAINER_PREFIX="robot"
VERSION_DIR="./versions"

init() {
  mkdir -p "$VERSION_DIR"
}

_deploy_core() {
  local service="$1"
  local version="${2:-latest}"
  local image="${SERVICES[$service]:-}"

  if [[ -z "$image" ]]; then
    echo "❌ 未知服务: $service"
    echo "可用服务: ${!SERVICES[*]}"
    return 1
  fi

  local container_name="${CONTAINER_PREFIX}-${service}"
  local image_name="${image%%:*}"

  echo "📦 部署服务: $service"
  echo "  镜像: $image_name:$version"
  echo "  容器: $container_name"

  local image_file="${image_name}_${version}.tar"
  if [[ -f "$image_file" ]]; then
    echo "加载镜像文件: $image_file"
    docker load -i "$image_file" || return 1
  else
    echo "⚠️ 镜像文件不存在: $image_file，尝试从仓库拉取"
    docker pull "${image_name}:${version}" || return 1
  fi

  docker stop "$container_name" >/dev/null 2>&1 || true
  docker rm "$container_name" >/dev/null 2>&1 || true

  local run_args=(
    --name "$container_name"
    --network host
    --privileged
    -v /dev:/dev
  )

  case "$service" in
    "sen22"|"sen20"|"navbrain"|"nviz")
      run_args+=(--runtime nvidia)
      ;;
  esac

  docker run -d "${run_args[@]}" "${image_name}:${version}" || return 1

  if [[ -f "${VERSION_DIR}/${service}.txt" ]]; then
    cp "${VERSION_DIR}/${service}.txt" "${VERSION_DIR}/${service}.txt.prev" || true
  fi
  echo "$version" > "${VERSION_DIR}/${service}.txt"
  echo "✅ $service 部署完成"
}

deploy() {
  init
  _deploy_core "$1" "${2:-latest}"
}

deploy_all() {
  init
  local version="${1:-latest}"
  echo "🚀 批量部署所有服务 (版本: $version)"
  local service
  for service in "${!SERVICES[@]}"; do
    _deploy_core "$service" "$version"
    sleep 2
  done
}

status() {
  init
  echo "📊 所有机器人容器状态"
  echo "========================"

  echo "🟢 运行中容器:"
  docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}" | grep -E "${CONTAINER_PREFIX}-|NAMES" || true

  echo
  echo "📋 所有容器:"
  docker ps -a --format "table {{.Names}}\t{{.Image}}\t{{.Status}}" | grep -E "${CONTAINER_PREFIX}-|NAMES" || true

  echo
  echo "📌 已部署版本记录:"
  local version_file
  shopt -s nullglob
  for version_file in "$VERSION_DIR"/*.txt; do
    local base
    base="$(basename "$version_file")"
    [[ "$base" == *.prev ]] && continue
    local service version
    service="${base%.txt}"
    version="$(cat "$version_file" 2>/dev/null || echo "unknown")"
    echo "  $service: $version"
  done
  shopt -u nullglob
}

rollback() {
  init
  local service="$1"
  local prev_file="${VERSION_DIR}/${service}.txt.prev"

  if [[ ! -f "$prev_file" ]]; then
    echo "❌ $service 没有可回滚版本记录: $prev_file"
    return 1
  fi

  local prev_version
  prev_version="$(cat "$prev_file")"
  if [[ -z "$prev_version" ]]; then
    echo "❌ $service 上一个版本为空"
    return 1
  fi

  echo "↩️ 回滚 $service 到 $prev_version"
  _deploy_core "$service" "$prev_version"
}

images() {
  echo "📦 本地镜像列表"
  docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}\t{{.Size}}"
}

stop_service() {
  local service="$1"
  local container_name="${CONTAINER_PREFIX}-${service}"
  echo "🛑 停止服务: $service"
  docker stop "$container_name" >/dev/null 2>&1 || true
  docker rm "$container_name" >/dev/null 2>&1 || true
  echo "✅ 已停止"
}

logs() {
  local service="$1"
  local lines="${2:-50}"
  local container_name="${CONTAINER_PREFIX}-${service}"
  echo "📝 查看 $service 日志 (最后 $lines 行)"
  docker logs --tail "$lines" "$container_name" 2>&1
}

help() {
  echo "Docker多镜像管理脚本"
  echo "===================="
  echo "可用服务: ${!SERVICES[*]}"
  echo
  echo "命令用法:"
  echo "  status                     - 查看所有容器状态"
  echo "  images                     - 查看本地镜像"
  echo "  deploy <service> [version] - 部署指定服务"
  echo "  deploy-all [version]       - 部署所有服务"
  echo "  stop <service>             - 停止服务"
  echo "  logs <service> [lines]     - 查看服务日志"
  echo "  rollback <service>         - 回滚服务"
  echo "  help                       - 显示帮助"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  case "${1:-}" in
    status) status ;;
    images) images ;;
    deploy) deploy "$2" "$3" ;;
    deploy-all) deploy_all "$2" ;;
    stop) stop_service "$2" ;;
    logs) logs "$2" "$3" ;;
    rollback) rollback "$2" ;;
    help|"") help ;;
    *) echo "未知命令: $1"; help ;;
  esac
fi
