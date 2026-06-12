FEATURE_FLAG_SPECS = [
    {
        "key": "connection_tab",
        "label": "连接页",
        "group": "页面",
        "description": "控制连接页签显示。",
        "default": True,
    },
    {
        "key": "status_monitor_tab",
        "label": "状态监控页",
        "group": "页面",
        "description": "控制状态监控页签显示。",
        "default": True,
    },
    {
        "key": "function_tab",
        "label": "功能栏页",
        "group": "页面",
        "description": "控制功能栏页签显示。",
        "default": True,
    },
    {
        "key": "joint_monitor_tab",
        "label": "关节监控页",
        "group": "页面",
        "description": "控制关节监控页签显示。",
        "default": True,
    },
    {
        "key": "small_brain_upload_file",
        "label": "小脑上传文件",
        "group": "功能栏",
        "description": "控制功能栏中的小脑上传文件操作。",
        "default": True,
    },
    {
        "key": "small_brain_download_file",
        "label": "小脑下载文件",
        "group": "功能栏",
        "description": "控制功能栏中的小脑下载文件操作。",
        "default": True,
    },
    {
        "key": "small_brain_upload_folder",
        "label": "小脑上传文件夹",
        "group": "功能栏",
        "description": "控制功能栏中的小脑上传文件夹操作。",
        "default": True,
    },
    {
        "key": "small_brain_download_folder",
        "label": "小脑下载文件夹",
        "group": "功能栏",
        "description": "控制功能栏中的小脑下载文件夹操作。",
        "default": True,
    },
    {
        "key": "small_brain_restart_embedded",
        "label": "重启嵌入式",
        "group": "功能栏",
        "description": "控制小脑嵌入式重启按钮。",
        "default": True,
    },
    {
        "key": "small_brain_restart_middleware",
        "label": "重启中间件",
        "group": "功能栏",
        "description": "控制小脑中间件重启按钮。",
        "default": True,
    },
    {
        "key": "small_brain_pull_middleware_log",
        "label": "拉取运控日志",
        "group": "功能栏",
        "description": "控制小脑运控日志拉取按钮。",
        "default": True,
    },
    {
        "key": "small_brain_pull_embedded_logs",
        "label": "拉取嵌入式日志",
        "group": "功能栏",
        "description": "控制小脑嵌入式日志拉取按钮。",
        "default": True,
    },
    {
        "key": "middleware_upgrade",
        "label": "中间件升级",
        "group": "功能栏",
        "description": "控制中间件升级包浏览与部署。",
        "default": True,
    },
    {
        "key": "remote_file_read",
        "label": "远程文件读取",
        "group": "功能栏",
        "description": "控制远程文件目录浏览、打开与重新加载。",
        "default": True,
    },
    {
        "key": "remote_file_write",
        "label": "远程文件写入",
        "group": "功能栏",
        "description": "控制远程文件保存、编辑与注释修改。",
        "default": True,
    },
    {
        "key": "joint_control_tab",
        "label": "关节控制页",
        "group": "页面",
        "description": "控制关节控制页签显示。",
        "default": True,
    },
    {
        "key": "navigation_tab",
        "label": "导航页",
        "group": "页面",
        "description": "控制导航页签显示。",
        "default": True,
    },
    {
        "key": "container_operations_tab",
        "label": "容器操作页",
        "group": "页面",
        "description": "控制容器操作页签显示。",
        "default": True,
    },
    {
        "key": "humanode_demo_tab",
        "label": "测试Demo页",
        "group": "页面",
        "description": "控制 Humanode 测试 Demo 页签显示。",
        "default": True,
    },
    {
        "key": "factory_test_tab",
        "label": "工厂测试页",
        "group": "页面",
        "description": "控制工厂测试页签显示。",
        "default": True,
    },
    {
        "key": "huiyang_tab",
        "label": "惠阳数采页",
        "group": "页面",
        "description": "控制惠阳数采页签显示。",
        "default": True,
    },
    {
        "key": "servoj_tool_tab",
        "label": "ServoJ工具页",
        "group": "页面",
        "description": "控制 ServoJ 工具页签显示。",
        "default": True,
    },
    {
        "key": "vision_monitor_tab",
        "label": "视觉监控页",
        "group": "页面",
        "description": "控制视觉监控页签显示。",
        "default": True,
    },
    {
        "key": "pose_estimation_tab",
        "label": "位姿估计页",
        "group": "页面",
        "description": "控制位姿估计页签显示。",
        "default": True,
    },
    {
        "key": "maintenance_tab",
        "label": "运维测试页",
        "group": "页面",
        "description": "控制运维测试页签显示。",
        "default": True,
    },
    {
        "key": "log_audit_tab",
        "label": "日志排查页",
        "group": "页面",
        "description": "控制日志排查页签显示。",
        "default": True,
    },
    {
        "key": "command_tab",
        "label": "命令页",
        "group": "页面",
        "description": "控制命令页签显示。",
        "default": True,
    },
    {
        "key": "help_tab",
        "label": "帮助页",
        "group": "页面",
        "description": "控制帮助页签显示。",
        "default": True,
    },
]

DEFAULT_FEATURE_FLAGS = {spec["key"]: bool(spec.get("default", True)) for spec in FEATURE_FLAG_SPECS}

FEATURE_FLAG_LABELS = {spec["key"]: spec["label"] for spec in FEATURE_FLAG_SPECS}

LEGACY_FEATURE_GROUPS = {
    "connection": ["connection_tab"],
    "status_monitor": ["status_monitor_tab"],
    "function": ["function_tab"],
    "joint_monitor": ["joint_monitor_tab"],
    "small_brain_file_transfer": [
        "small_brain_upload_file",
        "small_brain_download_file",
        "small_brain_upload_folder",
        "small_brain_download_folder",
    ],
    "remote_file_edit": [
        "remote_file_read",
        "remote_file_write",
    ],
    "joint_control": ["joint_control_tab"],
    "navigation": ["navigation_tab"],
    "container_operations": ["container_operations_tab"],
    "factory_test": ["factory_test_tab"],
    "huiyang": ["huiyang_tab"],
    "servoj_tool": ["servoj_tool_tab"],
    "vision_monitor": ["vision_monitor_tab"],
    "pose_estimation": ["pose_estimation_tab"],
    "maintenance": ["maintenance_tab"],
    "log_audit": ["log_audit_tab"],
    "command": ["command_tab"],
    "help": ["help_tab"],
}


def normalize_feature_flags(raw_flags) -> dict:
    merged = dict(DEFAULT_FEATURE_FLAGS)
    if not isinstance(raw_flags, dict):
        return merged

    for legacy_key, mapped_keys in LEGACY_FEATURE_GROUPS.items():
        if legacy_key not in raw_flags:
            continue
        legacy_value = bool(raw_flags.get(legacy_key))
        for mapped_key in mapped_keys:
            if mapped_key not in raw_flags:
                merged[mapped_key] = legacy_value

    for key, default_value in DEFAULT_FEATURE_FLAGS.items():
        merged[key] = bool(raw_flags.get(key, default_value))
    return merged