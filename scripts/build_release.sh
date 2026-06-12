#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
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

cd "$PROJECT_ROOT"

echo "[STEP] 1/2 构建 AppImage"
if [[ -n "$FEATURE_CONFIG" ]]; then
	"$SCRIPT_DIR/build_appimage.sh" --edition "$EDITION" --feature-config "$FEATURE_CONFIG"
else
	"$SCRIPT_DIR/build_appimage.sh" --edition "$EDITION"
fi

echo "[STEP] 2/2 构建 DEB"
if [[ -n "$FEATURE_CONFIG" ]]; then
	"$SCRIPT_DIR/build_deb.sh" --edition "$EDITION" --feature-config "$FEATURE_CONFIG"
else
	"$SCRIPT_DIR/build_deb.sh" --edition "$EDITION"
fi

echo "[OK] 发布包已完成（AppImage + DEB），产物位于 dist/"
