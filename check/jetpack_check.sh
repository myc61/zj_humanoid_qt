
#!/usr/bin/env bash

set -e

echo "=== Jetson L4T / JetPack Consistency Check ==="
echo

# -----------------------------
# 1. L4T -> JetPack 映射表
# -----------------------------
declare -A L4T_TO_JETPACK=(
  ["36.4.4"]="JetPack 6.2.1"
  ["36.4.3"]="JetPack 6.2"
  ["36.4"]="JetPack 6.1"
  ["36.3"]="JetPack 6.0"
  ["36.2"]="JetPack 6.0 DP"
  ["35.6.2"]="JetPack 5.1.5"
  ["35.6.1"]="JetPack 5.1.5"
  ["35.6.0"]="JetPack 5.1.4"
  ["35.5.0"]="JetPack 5.1.3"
  ["35.4.1"]="JetPack 5.1.2"
  ["35.3.1"]="JetPack 5.1.1"
  ["35.2.1"]="JetPack 5.1"
  ["35.1"]="JetPack 5.0.2"
  ["34.1.1"]="JetPack 5.0.1 DP"
  ["34.1"]="JetPack 5.0 DP"
)

# -----------------------------
# 2. 获取 Bootloader L4T
# -----------------------------
BOOT_L4T_RAW=$(sudo nvbootctrl dump-slots-info | grep "Current version" | awk '{print $3}')
BOOT_L4T="R${BOOT_L4T_RAW}"

BOOT_JETPACK="${L4T_TO_JETPACK[$BOOT_L4T_RAW]:-Unknown}"

# -----------------------------
# 3. 获取 Rootfs L4T
# -----------------------------
NV_RELEASE_LINE=$(head -n 1 /etc/nv_tegra_release)

ROOT_MAJOR=$(echo "$NV_RELEASE_LINE" | sed -n 's/^# R\([0-9]\+\).*/\1/p')
ROOT_MINOR=$(echo "$NV_RELEASE_LINE" | sed -n 's/.*REVISION: \([0-9]\+\.[0-9]\+\).*/\1/p')

ROOTFS_L4T_RAW="${ROOT_MAJOR}.${ROOT_MINOR}"
ROOTFS_L4T="R${ROOTFS_L4T_RAW}"
ROOTFS_JETPACK="${L4T_TO_JETPACK[$ROOTFS_L4T_RAW]:-Unknown}"

# -----------------------------
# 4. 输出结果
# -----------------------------
echo "Bootloader L4T     : ${BOOT_L4T}"
echo "Bootloader JetPack : ${BOOT_JETPACK}"
echo
echo "Rootfs L4T         : ${ROOTFS_L4T}"
echo "Rootfs JetPack     : ${ROOTFS_JETPACK}"
echo

# -----------------------------
# 5. 一致性判断
# -----------------------------
if [[ "$BOOT_L4T_RAW" == "$ROOTFS_L4T_RAW" ]]; then
  echo "Result             : ✅ MATCH"
else
  echo "Result             : ❌ MISMATCH"
fi

echo
echo "============================================"

