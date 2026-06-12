# humanoid-robot-delivery-toolchain

人形机器人交付工具链，面向 ROS1 现场交付、工厂测试和运维排障场景。

项目当前以 PyQt6 GUI 为主，结合 ROSBridge、SSH(小脑) 和 SSH(大脑) 提供以下能力：

- 连接管理与状态确认
- 文件传输与远程文件编辑
- 关节监控、示教、MoveJ 测试与动作序列执行
- 工厂测试流程化执行与报告导出
- 回放工具（movej_by_path / ServoJ）
- 运维测试、日志排查与现场兜底诊断
- AppImage / DEB / Windows EXE 打包发布

## 1. 项目特点

- 支持 Ubuntu 和 Windows 两端使用
- 连接面支持 ROSBridge、SSH(小脑)、SSH(大脑) 三链路协同
- UI 组织方式贴近交付流程，而不是单纯按代码模块划分
- 大部分现场操作可以直接在 GUI 内完成，不需要反复切终端
- 支持客户版 / 内部版 / 自定义功能开关配置

## 2. 当前界面结构

当前主界面页签如下：

1. 连接
2. 状态监控
3. 功能栏
4. 关节监控
5. 关节控制
6. 导航
7. 测试Demo
8. 工厂测试
9. 回放工具
10. 视觉监控
11. 位姿估计
12. 运维测试
13. 日志排查
14. 命令
15. 帮助

说明：

- 容器操作入口当前已隐藏，不再作为主界面页签对外展示
- 关节控制中的 MoveL 测试入口当前已隐藏，底层逻辑仍保留
- 回放工具页已经合并了原“惠阳数采”和“ServoJ工具”

## 3. 核心功能

### 3.1 连接

- 连接 ROSBridge
- 连接 SSH(小脑)
- 连接 SSH(大脑)
- 顶部统一显示连接质量和三链路状态

默认现场网络参数：

- 大脑：192.168.217.100
- 小脑：192.168.217.66
- ROSBridge：9090

### 3.2 状态监控

- 电池电量
- 机器人型号 / 嵌入式版本 / 软件版本 / 上肢版本
- 机器人状态
- Orin / Pico 状态与网络延迟
- 版本检测结果
- 外设检测结果
- 系统信息窗口

### 3.3 功能栏

- 上传文件 / 下载文件
- 上传文件夹 / 下载文件夹
- 重启嵌入式
- 重启中间件
- 拉取运控日志
- 拉取嵌入式日志
- 中间件升级包部署
- 远程文件编辑

中间件升级支持：

- 选择本地 .firmware 包
- 选择 robot_type
- 选择部署目标（大脑 / 小脑）
- 选择是否强制重新安装（--force）

### 3.4 关节监控

- 支持 WA2 / WA2_LS / WA1 / I2
- 关节位置监控
- TCP 速度监控
- TCP 位置监控
- 支持关节录制和绘图

### 3.5 关节控制

- 进入示教模式 / 退出示教模式
- 归位
- 解除保护
- ROSBridge / SSH 通道切换
- MoveJ 测试
- 动作序列编辑、执行、导入、导出
- 控制 Plot

MoveJ 测试当前规则：

- 只支持直接输入 [] 数组
- 不再提供逐关节输入框
- 支持按机型和部位自动校验长度
- 支持读取当前姿态回填

### 3.6 导航

- 开始建图
- 建图后处理
- 重启感知容器

### 3.7 测试Demo

- 运行 Demo
- 刷新测试用例
- 查看输出结果
- 远端目录可填写（默认 test-tools/Humanode）

### 3.8 工厂测试

工厂测试页按步骤组织，当前流程为：

1. BrainSense 与中间件控制
2. 手指关节运动
3. 手指压力检测
4. Force XYZ 检测
5. 语音播报与倾听
6. 相机图像检测
7. robot_info 校验
8. 上肢运动检测

其中步骤 4 当前逻辑为：

- 只读取左右手 wrist force sensor 的 force.x / force.y / force.z
- 支持“采集 Force 基线”
- 支持“开始 Force 检测”与“停止 Force 检测”
- 当前显示值按 1Hz 刷新，避免界面闪动过快
- delta = current - baseline 自动实时刷新
- 完成本项时记录监测期间左右手 x / y / z 各自偏差的历史最大值

工厂测试报告当前会导出：

- txt
- json
- html
- pdf

报告目录位于：

- logs/factory_reports/

### 3.9 回放工具

回放工具页当前包含两个折叠区：

- movej_by_path
- ServoJ 工具

movej_by_path 用于：

- 进入全身示教
- 采样记录
- 回放
- 导出 NPZ

ServoJ 工具用于：

- 进入全身示教后自动录制 joint_states
- 退出示教后停止录制
- 导出固定频率 NPZ
- 回放当前录制

### 3.10 视觉监控

- 检测当前帧
- 多帧采样
- 预览图像
- 查看六路话题频率

### 3.11 位姿估计

- 定时拉取远端 PNG
- 分别查看可乐 / 水结果图
- 观察图像与识别结果变化

### 3.12 运维测试

当前运维测试页已集成：

- Audio 功能
- Hand 压力监听与手指控制
- BrainSense 与中间件控制
- UpperLimb 功能
- Robot 快照检测
- Sensor 检测

### 3.13 日志排查

日志排查页当前支持三种模式：

- 错误驱动式
- 指定时段
- 就近时间

模式差异：

- 错误驱动式：需要连接 SSH(大脑)，因为会读取应用层 jsonl 并按失败事件切时间窗
- 指定时段：不强制要求连接 SSH(大脑)
- 就近时间：不强制要求连接 SSH(大脑)

如果指定时段 / 就近时间模式下未连接大脑：

- 会跳过应用层 jsonl 提取
- 仍然继续提取小脑中间件日志、Ubuntu 系统日志和嵌入式日志

嵌入式日志提取规则：

- 优先匹配 /var/www/html/log/sdk/<日期目录>/ 下的压缩日志包
- 错误驱动式按失败事件时间点匹配最近的嵌入式压缩包
- 指定时段 / 就近时间模式会匹配该时间范围内的一个或多个嵌入式压缩包
- 如果没有匹配到压缩包，会回退提取 /home/nav01/CodeFiles/sdk/zjhrobotsdkreleaseproject/LogFile

日志排查结果会保存到：

- logs/log_audit_test/

单次日志排查任务通常会保存：

- application_events.jsonl
- middleware.log
- ubuntu_system.log
- summary.txt
- 匹配到的嵌入式压缩日志，或回退提取的 LogFile 目录

### 3.14 命令

- rosservice call
- rostopic echo
- 停止持续输出
- 刷新话题 / 服务列表
- Tab 自动补全
- 双击列表自动填充命令模板

## 4. 技术栈

- Python 3.10+
- PyQt6
- roslibpy
- paramiko
- requests
- PyYAML
- matplotlib
- reportlab

依赖定义见：

- [pyproject.toml](pyproject.toml)
- [requirements.txt](requirements.txt)

## 5. 运行环境要求

### 5.1 本机

- Python 3.10 或更高版本
- 可访问机器人网络
- Ubuntu 或 Windows

### 5.2 机器人侧

- ROS1 环境可用
- rosbridge_server 可用
- SSH 服务开启

示例：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

## 6. 开发环境启动

### Ubuntu

```bash
cd /path/to/humanoid-robot-delivery-toolchain
chmod +x ./scripts/run_ubuntu.sh
./scripts/run_ubuntu.sh
```

可选：

```bash
TOOLCHAIN_PYTHON=/usr/bin/python3 ./scripts/run_ubuntu.sh
```

### Windows

```powershell
cd C:\path\to\humanoid-robot-delivery-toolchain
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\scripts\setup_windows.ps1
.\scripts\run_windows.ps1
```

## 7. 打包发布

### 7.1 一键打包 AppImage + DEB

```bash
cd /path/to/humanoid-robot-delivery-toolchain
./scripts/build_release.sh --edition internal
./scripts/build_release.sh --edition customer
./scripts/build_release.sh --edition customer --feature-config ./config/feature_profiles/huiyang.yaml
```

### 7.2 AppImage

```bash
./scripts/build_appimage.sh --edition internal
./scripts/build_appimage.sh --edition customer
./scripts/build_appimage.sh --edition customer --feature-config ./config/feature_profiles/huiyang.yaml
```

特点：

- 自动创建 .pack-venv
- 使用 PyInstaller onedir 打包
- 内置运行包装器
- 支持将 xterm 运行时打入 AppImage
- 启动失败时写入 launcher.log 并尝试弹窗或 xterm 回显

### 7.3 DEB

```bash
./scripts/build_deb.sh --edition internal
./scripts/build_deb.sh --edition customer
./scripts/build_deb.sh --edition customer --feature-config ./config/feature_profiles/huiyang.yaml
```

安装：

```bash
sudo dpkg -i dist/humanoid-robot-delivery-toolchain-*.deb
sudo apt -f install -y
```

### 7.4 Windows EXE

```powershell
.\scripts\build_windows_exe.ps1 -Edition internal
.\scripts\build_windows_exe.ps1 -Edition customer
.\scripts\build_windows_exe.ps1 -Edition customer -FeatureConfig .\config\feature_profiles\huiyang.yaml
```

### 7.5 功能配置文件

默认功能配置：

- config/feature_profiles/internal.yaml
- config/feature_profiles/customer.yaml
- config/feature_profiles/huiyang.yaml

## 8. 目录说明

主要目录如下：

- src/: GUI 主程序与业务逻辑
- scripts/: 启动、打包、部署辅助脚本
- config/: 配置文件、功能开关、轨迹 YAML
- tests/: 测试代码
- logs/: 运行日志、工厂报告、日志排查结果
- Humanode_Naviai-main/: Demo 与相关工程
- packaging/: 桌面入口等打包资源

## 9. 当前实现说明

和旧版本相比，当前项目有几个需要特别注意的点：

- 主窗口核心逻辑大量集中在 src/ui/main_window.py
- 工厂测试步骤 4 已从“六维力全量显示”收敛为“Force XYZ 实时检测”
- 日志排查的 range / near 模式不再强制依赖 SSH(大脑)
- BrainSense 启动走系统 ssh -C -Y 的 X11 forwarding 方案
- 打包脚本已经纳入 xterm 运行时与 launcher 日志兜底

## 10. 适用场景

这个工具主要面向：

- 工厂测试人员
- 现场交付人员
- 运维排障人员
- 机器人控制与中间件联调人员

如果你的目标是：

- 快速确认机器人链路是否正常
- 现场执行上肢、语音、相机、压力、Force 检测
- 抓取并整理交付期日志
- 为客户打包一份可直接运行的图形工具

这个项目就是当前的主工具链。