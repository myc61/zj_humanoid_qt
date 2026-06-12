
#!/usr/bin/env bash

set -euo pipefail

# 定义错误退出函数，保留完整错误信息
error_exit() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $1" >&2
    exit "${2:-1}"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ======================== 第一步：检查66本机（本地）的版本信息 ========================
echo -e "\n=== Local Host Check (192.168.217.66) ==="
echo -e "💡 接下来会提示输入【66本机的sudo密码】（执行SDK版本检查）\n"

# 输出本地（66）基本信息
echo -e "[Local] Hostname: $(hostname)"
echo -e "[Local] IP Address: $(hostname -I | awk '{print $1}')"
echo -e "\n[Local] Ubuntu Info:"
if command -v lsb_release >/dev/null 2>&1; then
    lsb_release -a
else
    if [[ -r /etc/os-release ]]; then
        . /etc/os-release
        echo "Ubuntu Version     : ${PRETTY_NAME:-Unknown}"
    else
        echo "Ubuntu Version     : Unknown (lsb_release not found)"
    fi
fi

# 检查并输出66本机的SDK版本（需输入66的sudo密码）
echo -e "\n[Local] SDK Version:"
SDK_DIR="$HOME/CodeFiles/sdk/zjhrobotsdkreleaseproject"
if [[ ! -d "$SDK_DIR" ]]; then
    error_exit "SDK 目录不存在: $SDK_DIR" 5
fi
cd "$SDK_DIR" || error_exit "无法进入 SDK 目录: $SDK_DIR" 5
sudo ./runcmd.sh version || error_exit "执行 runcmd.sh version 失败（密码错误或权限不足）" 6

# 检查 R3/R2 版本并输出66本机的上下肢信息
echo -e "\n[Local] Software Version Check:"
if [ -d /opt/ros/noetic/share/pico_runtime ]; then
    echo "[Local] 当前软件版本: R3"
    echo -e "\n[Local] Upper-limb Version (R3 via rosservice):"
    # 验证 rosservice 命令是否存在
    if ! command -v rosservice >/dev/null 2>&1; then
        error_exit "rosservice 命令未找到（ROS 环境加载失败）" 127
    fi
    # 调用上肢版本服务，保留完整输出
    rosservice call /zj_humanoid/upperlimb/version || echo "[Local WARNING]: 上肢版本服务调用失败（服务可能未启动）"
else
    echo "[Local] 当前软件版本: R2"
    echo -e "\n[Local] Upper-limb Files (R2):"
    UPLIMB_DIR="$HOME/sport_ctrl_ros/lib/uplimb/"
    if [[ ! -d "$UPLIMB_DIR" ]]; then
        error_exit "上肢库目录不存在: $UPLIMB_DIR" 7
    fi
    cd "$UPLIMB_DIR" || error_exit "无法进入上肢库目录: $UPLIMB_DIR" 7
    ls -1
    cnt=$(ls -1 | wc -l)
    echo "[Local] 文件数: $cnt"
    if [ "$cnt" -eq 2 ]; then
        echo "[Local] 对应老版本，无法查看具体版本"
    elif [ "$cnt" -eq 5 ]; then
        ver=$(ls -1 | grep -E '\.so\.0\.6\.0$|\.so\.1\.0\.0$' | head -n1)
        if [ -n "$ver" ]; then
            echo "[Local] 检测到库版本: $ver"
        else
            echo "✅[Local] 未检测到 so 版本后缀 (so.0.6.0 / so.1.0.0)"
        fi
    else
        echo "[Local] 文件数不为 2 或 5，实际: $cnt"
    fi
fi

# 输出66本机的下肢版本信息
# echo -e "\n[Local] Lower-limb Version:"
# LOWERLIMB_CMD="cd $HOME/catkin_ws_amp_show && source devel/setup.bash && rosservice call /zj_humanoid/lowerlimb/version"
# bash -lc "$LOWERLIMB_CMD" || echo "[Local WARNING]: 下肢版本服务调用失败（服务可能未启动）"
# ======================== 关键修改：下肢版本根据R2/R3切换指令 ========================
# 输出66本机的下肢版本信息
echo -e "\n[Local] Lower-limb Version:"
# 判断R2/R3版本，执行对应指令
if [ -d /opt/ros/noetic/share/pico_runtime ]; then
    # R3版本：先加载catkin环境，再执行rostopic echo（-n 1限制只输出1次）
    echo "[Local] 当前为R3版本，执行指令：cd catkin_ws_amp_show && source devel/setup.bash && rostopic echo /zj_humanoid/lowerlimb/version -n 1"
    LOWERLIMB_R3_CMD="cd $HOME/catkin_ws_amp_show && source devel/setup.bash && rostopic echo /zj_humanoid/lowerlimb/version -n 1"
    bash -lc "$LOWERLIMB_R3_CMD" || echo "[Local WARNING]: R3下肢版本话题获取失败（话题未发布/节点未启动/环境加载失败）"
else
    # R2版本：执行cd + source + rosservice call
    echo "[Local] 当前为R2版本，执行指令：cd catkin_ws_amp_show && source devel/setup.bash && rosservice call /zj_humanoid/lowerlimb/versions"
    LOWERLIMB_CMD="cd $HOME/catkin_ws_amp_show && source devel/setup.bash && rosservice call /zj_humanoid/lowerlimb/versions"
    bash -lc "$LOWERLIMB_CMD" || echo "[Local WARNING]: R2下肢版本服务调用失败（服务未启动/环境加载失败）"
fi


# 输出66本机的系统信息
echo -e "\n[Local] System Info:"
LOGICAL_CORES=$(grep -c "^processor" /proc/cpuinfo 2>/dev/null || echo "Unknown")
# 准确获取物理核心数（去重physical id + core id）
PHYSICAL_CORES=$(grep -E "^physical id|^core id" /proc/cpuinfo 2>/dev/null | paste -d' ' - - | sort -u | wc -l || echo "Unknown")
# 输出核心数（同时展示逻辑+物理，更清晰）
echo "[Local] CPU Logical Cores (总核心数): ${LOGICAL_CORES}"
echo "[Local] CPU Physical Cores (物理核心数): ${PHYSICAL_CORES}"
# ========== 替换结束 ==========
echo "[Local] Memory Info: $(free -h | grep Mem | awk '{print "Total: "$2", Used: "$3", Free: "$4}')"
echo -e "============================================="

# ======================== 第二步：准备远程执行脚本（文件化，避免逐行解析） ========================
# 临时远程脚本内容
REMOTE_SCRIPT_CONTENT=$(cat <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

# 远程错误处理函数
remote_error() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [Remote ERROR]: $1" >&2
    exit "${2:-1}"
}

# 核心执行逻辑
main() {
    echo -e "\n=== Jetson L4T / JetPack Consistency Check (192.168.217.100) ==="
    TARGET="$HOME/jetpack_check.sh"

    # 检查jetpack_check.sh是否存在
    if [[ ! -f "${TARGET}" ]]; then
        remote_error "100上未找到jetpack_check.sh，路径：${TARGET}" 10
    fi

    # 赋予执行权限
    chmod +x "${TARGET}" || remote_error "无法给jetpack_check.sh添加执行权限" 11

    # 执行并捕获输出（手动输入sudo密码）
    echo -e "🔍 即将执行jetpack_check.sh，请输入【100的sudo密码】（输入不回显）："
    set +e
    CHECK_OUTPUT="$(${TARGET} 2>&1)"
    CHECK_STATUS=$?
    set -e

    # 打印执行日志
    echo -e "\n📝 jetpack_check.sh 执行输出："
    echo "${CHECK_OUTPUT}"
    echo -e "============================================="

    # 容错处理
    if [[ ${CHECK_STATUS} -ne 0 ]]; then
        if echo "${CHECK_OUTPUT}" | grep -q "bad array subscript"; then
            echo "⚠️  jetpack_check.sh 执行时出现数组下标警告，但核心功能已完成"
        elif echo "${CHECK_OUTPUT}" | grep -q "Sorry, try again"; then
            remote_error "sudo密码错误！请输入100上naviai用户的正确sudo密码" 12
        else
            remote_error "jetpack_check.sh 执行失败，退出码：${CHECK_STATUS}" ${CHECK_STATUS}
        fi
    fi

    # 输出Ubuntu版本
    echo -e "\n=== Local Ubuntu Info (192.168.217.100) ==="
    if command -v lsb_release >/dev/null 2>&1; then
        lsb_release -a
    else
        if [[ -r /etc/os-release ]]; then
            . /etc/os-release
            echo "Ubuntu Version     : ${PRETTY_NAME:-Unknown}"
        else
            echo "Ubuntu Version     : Unknown (lsb_release not found)"
        fi
    fi

    echo -e "\n[Remote] All checks completed successfully!"
}

# 执行主函数
main
EOF
)

# 保存临时远程脚本到66本机
REMOTE_SCRIPT_FILE="${SCRIPT_DIR}/remote_exec_check.sh"
echo "${REMOTE_SCRIPT_CONTENT}" > "${REMOTE_SCRIPT_FILE}"
chmod +x "${REMOTE_SCRIPT_FILE}"

# ======================== 第三步：传输文件到100（jetpack_check.sh + 远程执行脚本） ========================
# 远程配置
REMOTE_USER="naviai"          
REMOTE_HOST="192.168.217.100" 
JETPACK_FILE="${SCRIPT_DIR}/jetpack_check.sh"
REMOTE_JETPACK="~/jetpack_check.sh"
REMOTE_EXEC_SCRIPT="~/remote_exec_check.sh"

# 检查jetpack_check.sh是否存在
if [[ ! -f "${JETPACK_FILE}" ]]; then
    error_exit "66上未找到jetpack_check.sh，路径：${JETPACK_FILE}" 8
fi

# 传输jetpack_check.sh
echo -e "\n=== Transferring jetpack_check.sh to ${REMOTE_HOST} ==="
echo -e "💡 接下来会提示输入【100的登录密码】（传输文件）\n"
set +e
scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 "${JETPACK_FILE}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_JETPACK}"
if [[ $? -ne 0 ]]; then
    error_exit "传输jetpack_check.sh到100失败（密码错误/网络不通）" 9
fi

# 传输远程执行脚本
scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 "${REMOTE_SCRIPT_FILE}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_EXEC_SCRIPT}"
if [[ $? -ne 0 ]]; then
    error_exit "传输远程执行脚本到100失败" 13
fi
set -e

echo "✅ 所有文件已传输到100的naviai家目录"

# ======================== 第四步：远程执行脚本（强制伪终端，手动输入密码） ========================
echo -e "\n=== Remote Host Check (${REMOTE_USER}@${REMOTE_HOST}) ==="
echo -e "💡 接下来会：1. 提示输入【100的登录密码】 2. 执行脚本时提示输入【100的sudo密码】\n"
set +e
# 强制伪终端执行远程脚本
ssh -t -t -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 "${REMOTE_USER}@${REMOTE_HOST}" "bash ${REMOTE_EXEC_SCRIPT}"
REMOTE_STATUS=$?
set -e

# ======================== 第五步：清理临时文件 ========================
# 删除66本机的临时远程脚本
rm -f "${REMOTE_SCRIPT_FILE}"
# 可选：删除100上的临时脚本（执行完后清理）
ssh -o StrictHostKeyChecking=accept-new "${REMOTE_USER}@${REMOTE_HOST}" "rm -f ${REMOTE_EXEC_SCRIPT}" >/dev/null 2>&1

# ======================== 结果判断 ========================
if [[ ${REMOTE_STATUS} -eq 0 ]]; then
    echo -e "\n=== Check Summary ==="
    echo "✅ 所有检查完成："
    echo "  - 66本机（本地）：SDK、上下肢版本、系统信息已检查"
    echo "  - 100远程（目标）：jetpack_check.sh已执行，JetPack、Ubuntu版本已检查"
else
    error_exit "远程主机（100）检查失败 (退出码=${REMOTE_STATUS})" "${REMOTE_STATUS}"
fi

exit 0