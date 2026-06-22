import ast
import json
import threading
import importlib
import math
import html
import copy
import os
import stat
import shlex
import re
import subprocess
import sys
import time
import base64
import hashlib
import yaml
import traceback
import tarfile
import tempfile
import shutil
import roslibpy
from datetime import datetime, timedelta
from PyQt6.QtCore import pyqtSignal, Qt, QTimer, QStringListModel, QRunnable, QThreadPool
from PyQt6.QtGui import QImage, QPixmap, QKeySequence, QTextCursor, QTextDocument, QShortcut
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QPlainTextEdit,
    QFileDialog, QListWidget, QInputDialog, QTabWidget, QGridLayout, QComboBox, QScrollArea, QCheckBox,
    QDialog, QFormLayout, QDialogButtonBox, QMessageBox, QToolButton, QSplitter, QCompleter, QListWidgetItem, QStackedWidget,
    QTextEdit, QAbstractItemView, QDoubleSpinBox, QSpinBox
)

from ros.connection import BridgeClient
from core.feature_flags import DEFAULT_FEATURE_FLAGS, FEATURE_FLAG_LABELS, FEATURE_FLAG_SPECS, normalize_feature_flags
from ssh.sftp_client import SshSftpClient
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.pdfbase import pdfmetrics
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle, Image as RLImage


APP_DISPLAY_VERSION = "v1.0.0"
ANSI_ESCAPE_RE = re.compile(r"\x1B(?:\[[0-?]*[ -/]*[@-~]|[@-_])")
FACTORY_BRAINSENSE_REMOTE_DIR = "test-tools/BrainSense/scripts"
FACTORY_MONITORED_TOPICS = (
    "/zj_humanoid/chassis/agv_imu",
    "/zj_humanoid/chassis/charge_state",
    "/zj_humanoid/chassis/collision_state",
    "/zj_humanoid/chassis/odom_info",
    "/zj_humanoid/chassis/stop_state",
    "/zj_humanoid/hand/finger_pressures/left",
    "/zj_humanoid/hand/finger_pressures/right",
    "/zj_humanoid/hand/joint_states",
    "/zj_humanoid/audio/listen_state",
    "/zj_humanoid/navigation/mpc_trajectory",
    "/zj_humanoid/navigation/navigation/status",
    "/zj_humanoid/navigation/navigation_code",
    "/zj_humanoid/robot/battery_info",
    "/zj_humanoid/robot/joint_motor/errors",
    "/zj_humanoid/robot/monitor_status",
    "/zj_humanoid/robot/orin_states/errors",
    "/zj_humanoid/robot/orin_states/resource",
    "/zj_humanoid/robot/pico_states/errors",
    "/zj_humanoid/robot/pico_states/resource",
    "/zj_humanoid/robot/robot_state",
    "/zj_humanoid/robot/work_status_from_start",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/camera_info",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressed",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/theora",
    "/zj_humanoid/sensor/realsense_head/color/camera_info",
    "/zj_humanoid/sensor/realsense_head/color/image_raw",
    "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed",
    "/zj_humanoid/sensor/realsense_head/color/image_raw/theora",
    "/zj_humanoid/sensor/realsense_head/depth/camera_info",
    "/zj_humanoid/sensor/realsense_head/depth/image_rect_raw",
    "/zj_humanoid/sensor/realsense_head/depth/image_rect_raw/compressed",
    "/zj_humanoid/sensor/realsense_head/depth/image_rect_raw/compressedDepth",
    "/zj_humanoid/sensor/realsense_head/depth/image_rect_raw/theora",
    "/zj_humanoid/sensor/realsense_head/realsense2_camera_manager_head/bond",
    "/zj_humanoid/upperlimb/jacobian/left_arm",
    "/zj_humanoid/upperlimb/jacobian/right_arm",
    "/zj_humanoid/upperlimb/joint_states",
    "/zj_humanoid/upperlimb/motion/status",
    "/zj_humanoid/upperlimb/occupancy_state",
    "/zj_humanoid/upperlimb/tcp_pose/left_arm",
    "/zj_humanoid/upperlimb/tcp_pose/right_arm",
    "/zj_humanoid/upperlimb/tcp_speed/dual_arm",
    "/zj_humanoid/upperlimb/teach_mode/replay/status",
    "/zj_humanoid/upperlimb/teach_mode/start_record/status",
    "/zj_humanoid/upperlimb/uplimb_state",
)


class _FnRunnable(QRunnable):
    def __init__(self, fn):
        super().__init__()
        self._fn = fn

    def run(self):
        self._fn()


class _LogWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._force_close = False

    def force_close(self):
        self._force_close = True
        self.close()

    def closeEvent(self, event):
        if self._force_close:
            event.accept()
            return
        self.hide()
        event.ignore()


class MainWindow(QWidget):
    log_signal = pyqtSignal(str)
    maintenance_show_signal = pyqtSignal()
    maintenance_item_signal = pyqtSignal(str)
    maintenance_clear_signal = pyqtSignal()
    maintenance_hand_pressure_signal = pyqtSignal(str, object)
    maintenance_pressure_state_signal = pyqtSignal(bool, str)
    battery_signal = pyqtSignal(str)
    joints_signal = pyqtSignal(str)
    joints_map_signal = pyqtSignal(dict)
    tcp_pose_monitor_signal = pyqtSignal(dict)
    tcp_speed_monitor_signal = pyqtSignal(dict)
    connect_btn_signal = pyqtSignal(str, bool, str)
    command_btn_signal = pyqtSignal(bool)
    status_signal = pyqtSignal(str, str)
    robot_orin_status_signal = pyqtSignal(str)
    robot_pico_status_signal = pyqtSignal(str)
    robot_orin_latency_signal = pyqtSignal(str)
    robot_pico_latency_signal = pyqtSignal(str)
    robot_state_info_signal = pyqtSignal(str)
    joint_monitor_health_signal = pyqtSignal(str, str)
    robot_status_health_signal = pyqtSignal(str)
    safety_lock_view_signal = pyqtSignal(str, str)
    robot_basic_info_signal = pyqtSignal(dict)
    robot_system_info_signal = pyqtSignal(dict)
    version_item_signal = pyqtSignal(str)
    version_clear_signal = pyqtSignal()
    peripheral_item_signal = pyqtSignal(str)
    peripheral_clear_signal = pyqtSignal()
    container_item_signal = pyqtSignal(str)
    container_clear_signal = pyqtSignal()
    huiyang_log_item_signal = pyqtSignal(str)
    servoj_tool_log_item_signal = pyqtSignal(str)
    servoj_tool_refresh_signal = pyqtSignal()
    huiyang_info_signal = pyqtSignal()
    huiyang_status_signal = pyqtSignal(str)
    huiyang_controls_signal = pyqtSignal(bool, bool)
    humanode_demo_item_signal = pyqtSignal(str)
    humanode_demo_clear_signal = pyqtSignal()
    humanode_demo_session_state_signal = pyqtSignal(bool, str)
    factory_test_item_signal = pyqtSignal(str)
    factory_test_clear_signal = pyqtSignal()
    factory_force_values_signal = pyqtSignal(str, object)
    factory_force_live_signal = pyqtSignal(str, object)
    factory_force_monitor_state_signal = pyqtSignal(bool, str)
    factory_container_status_signal = pyqtSignal(str, str)
    factory_topic_service_audit_signal = pyqtSignal(dict)
    factory_camera_state_signal = pyqtSignal(bool, str)
    container_runtime_list_signal = pyqtSignal(object)
    container_runtime_file_opened_signal = pyqtSignal(str, str, str)
    container_runtime_file_saved_signal = pyqtSignal(str, str)
    container_runtime_failed_signal = pyqtSignal(str)
    vision_image_signal = pyqtSignal(object, str)
    vision_topic_rate_signal = pyqtSignal(str, str)
    pose_image_signal = pyqtSignal(str, object, str)
    factory_camera_image_signal = pyqtSignal(str, object, str)
    motion_exec_state_signal = pyqtSignal(bool)
    remote_edit_opened_signal = pyqtSignal(str, str, str)
    remote_edit_saved_signal = pyqtSignal(str, str)
    remote_edit_failed_signal = pyqtSignal(str)
    remote_edit_files_signal = pyqtSignal(list, str)
    remote_edit_dirs_signal = pyqtSignal(list, str)
    ros_lists_loaded_signal = pyqtSignal(list, list)
    log_audit_item_signal = pyqtSignal(str)
    log_audit_clear_signal = pyqtSignal()
    small_brain_sudo_prompt_signal = pyqtSignal(object)

    def __init__(self, instance_name: str = "", edition: str = "internal", feature_flags: dict | None = None, feature_profile_name: str = "", feature_config_path: str = ""):
        super().__init__()
        self.instance_name = (instance_name or "").strip()
        self.edition = self._normalize_edition(edition)
        self.feature_flags = self._normalize_feature_flags(feature_flags)
        self.feature_profile_name = str(feature_profile_name or "").strip()
        self.feature_config_path = str(feature_config_path or "").strip()
        self._instance_tag = self._sanitize_instance_tag(self.instance_name)
        title = f"Humanoid Robot Delivery Toolchain {APP_DISPLAY_VERSION}"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"
        if self.feature_profile_name:
            title = f"{title} [{self.feature_profile_name}]"
        self.setWindowTitle(title)
        self.resize(1100, 720)

        self.ros = None
        self.ros_monitor = None
        self.ros_status_monitor = None
        self.ssh = SshSftpClient()      # 小脑
        self.ssh_big = SshSftpClient()  # 大脑
        self._ssh_small_password = ""

        # 需在 _build_ui 前初始化（_build_ui 内会调用 _set_motion_editing）
        self._motion_rows = []
        self._motion_row_seq = 0
        self._motion_editing = False
        self._motion_sequences = []
        self._motion_sequence_seq = 0
        self._motion_current_sequence_idx = -1
        self._suspend_motion_persist = False
        self._motion_exec_running = False
        self._motion_exec_stop_event = threading.Event()
        self._remote_edit_target = ""
        self._remote_edit_path = ""
        self._remote_edit_supported_exts = {
            ".txt", ".md", ".rst", ".log", ".csv", ".tsv",
            ".json", ".yaml", ".yml", ".toml", ".ini", ".cfg", ".conf", ".env", ".xml",
            ".sh", ".bash", ".zsh", ".fish", ".ps1", ".bat", ".cmd",
            ".py", ".pyi", ".cpp", ".cc", ".cxx", ".c", ".h", ".hpp", ".hh", ".hxx",
            ".java", ".kt", ".go", ".rs", ".js", ".ts", ".jsx", ".tsx", ".vue",
            ".css", ".scss", ".sass", ".less", ".html", ".htm", ".sql",
            ".proto", ".service", ".msg", ".srv", ".action", ".launch",
        }
        self._remote_edit_supported_names = {
            "Dockerfile", "Containerfile", "Makefile", "CMakeLists.txt",
            "README", "README.md", "LICENSE", "NOTICE", "requirements.txt",
        }
        self._remote_edit_dir_history = ["/tmp"]
        self._remote_edit_file_history = ["test.txt"]
        self._joint_ctrl_transport_mode = "ros"
        self._container_runtime_entries = []
        self._container_runtime_details = {}
        self._container_edit_container = ""
        self._container_edit_path = ""
        self._middleware_deploy_local_path = ""
        self._mpc_deploy_local_path = ""
        self._mpc_image_local_path = ""
        self._mpc_docker_offline_local_path = ""
        self._humanode_demo_local_dir = ""
        # 惠阳数采
        self._huiyang_recording = False
        self._huiyang_frames: list = []       # [{t: float, joints: list[float]}]
        self._huiyang_initial_frame: list = []
        self._huiyang_record_start_ts: float = 0.0
        self._huiyang_record_timer = None
        self._huiyang_record_lock = threading.Lock()
        self._huiyang_playback_running = False
        self._huiyang_remote_recording = False
        self._huiyang_remote_record_path = ""
        self._huiyang_remote_record_pid_path = ""
        self._huiyang_remote_record_log_path = ""
        self._humanode_demo_session = None
        self._humanode_demo_session_case_label = ""
        self._factory_flow_steps = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self._factory_flow_state = {step: {"done": False, "time": "", "note": ""} for step in self._factory_flow_steps}
        self._factory_pressure_capture_active = False
        self._factory_pressure_max = {
            "left": [None, None, None, None, None, None],
            "right": [None, None, None, None, None, None],
        }
        self._factory_camera_subscribers = {}
        self._factory_camera_last_image_path = {"head": "", "chest": ""}
        self._factory_camera_last_frame_ts = {"head": 0.0, "chest": 0.0}
        self._factory_force_subscribers = {}
        self._factory_force_polling_active = False
        self._factory_force_ssh_session = None
        self._factory_force_last_emit_ts = {"left": 0.0, "right": 0.0}
        self._factory_force_delta_max = {"left": [None, None, None], "right": [None, None, None]}
        self._factory_force_topics = {
            "left": "/zj_humanoid/hand/wrist_force_sensor/left",
            "right": "/zj_humanoid/hand/wrist_force_sensor/right",
        }
        self._factory_topic_service_audit = {"topics": [], "summary": "-", "topic_count": 0}
        self._factory_report_records = []
        self._maintenance_pressure_topics = {
            "left": "/zj_humanoid/hand/finger_pressures/left",
            "right": "/zj_humanoid/hand/finger_pressures/right",
        }
        self._maintenance_robot_topics = [
            "/zj_humanoid/robot/battery_info",
            "/zj_humanoid/robot/joint_motor/errors",
            "/zj_humanoid/robot/monitor_status",
            "/zj_humanoid/robot/orin_states/errors",
            "/zj_humanoid/robot/orin_states/resource",
            "/zj_humanoid/robot/pico_states/errors",
            "/zj_humanoid/robot/pico_states/resource",
            "/zj_humanoid/robot/robot_state",
            "/zj_humanoid/robot/work_status_from_start",
        ]
        self._maintenance_robot_services = [
            ("Orin WiFi 列表", "/zj_humanoid/robot/orin_states/wifi_list"),
            ("Pico WiFi 列表", "/zj_humanoid/robot/pico_states/wifi_list"),
        ]
        self._maintenance_pressure_active = False
        self._maintenance_pressure_ssh_session = None
        self._maintenance_finger_presets = self._load_maintenance_finger_presets()
        self._maintenance_wa1_joint_limit_cache = None
        self._maintenance_wa1_limit_margin = 0.05
        self._maintenance_wa1_single_joint_running = False
        self._maintenance_wa1_single_joint_stop_requested = False
        self._maintenance_wa1_single_joint_remote_pid = ""
        self._storage_root_cache = None
        self.robot_model = "WA2"
        self._cmd_topics = []
        self._cmd_services = []
        self._cmd_completion_entries = []
        self._service_req_template_cache = {}
        self._version_refresh_lock = threading.Lock()
        self._version_refreshing = False

        self._alive = True

        self._build_ui()
        self._apply_edition_mode()

        self._install_exception_logger()
        QTimer.singleShot(0, self._show_log_window)

        self.log_signal.connect(self._append_log, Qt.ConnectionType.QueuedConnection)
        self.battery_signal.connect(self.battery_conn_value.setText)
        self.battery_signal.connect(self._set_robot_battery_text)
        self.joints_signal.connect(self.joints_value.setText)
        self.joints_map_signal.connect(self._update_joint_fields)
        self.tcp_pose_monitor_signal.connect(self._update_tcp_pose_view)
        self.tcp_speed_monitor_signal.connect(self._update_tcp_speed_view)
        self.connect_btn_signal.connect(self._set_connect_button_state)
        self.command_btn_signal.connect(self._set_command_button_state)
        self.status_signal.connect(self._set_status_text)
        self.robot_orin_status_signal.connect(self.robot_orin_status_value.setText)
        self.robot_pico_status_signal.connect(self.robot_pico_status_value.setText)
        self.robot_orin_latency_signal.connect(self.robot_orin_latency_value.setText)
        self.robot_pico_latency_signal.connect(self.robot_pico_latency_value.setText)
        self.robot_state_info_signal.connect(self.robot_state_info_value.setText)
        self.joint_monitor_health_signal.connect(self._set_joint_monitor_health)
        self.robot_status_health_signal.connect(self._set_robot_status_health)
        self.safety_lock_view_signal.connect(self._set_safety_lock_view)
        self.robot_basic_info_signal.connect(self._set_robot_basic_info)
        self.robot_system_info_signal.connect(self._set_robot_system_info)
        self.version_item_signal.connect(self.version_result_list.appendPlainText)
        self.version_clear_signal.connect(self.version_result_list.clear)
        self.peripheral_item_signal.connect(self.peripheral_result_list.appendPlainText)
        self.peripheral_clear_signal.connect(self.peripheral_result_list.clear)
        self.container_item_signal.connect(self.container_output.appendPlainText, Qt.ConnectionType.QueuedConnection)
        self.container_clear_signal.connect(self.container_output.clear, Qt.ConnectionType.QueuedConnection)
        self.huiyang_log_item_signal.connect(self._huiyang_append_log, Qt.ConnectionType.QueuedConnection)
        self.servoj_tool_log_item_signal.connect(self._servoj_tool_append_log, Qt.ConnectionType.QueuedConnection)
        self.servoj_tool_refresh_signal.connect(self._servoj_tool_update_status, Qt.ConnectionType.QueuedConnection)
        self.huiyang_info_signal.connect(self._huiyang_update_info, Qt.ConnectionType.QueuedConnection)
        self.huiyang_status_signal.connect(
            lambda s: self.huiyang_status_label.setText(f"状态: {s}") if hasattr(self, "huiyang_status_label") else None,
            Qt.ConnectionType.QueuedConnection,
        )
        self.huiyang_controls_signal.connect(self._set_huiyang_controls, Qt.ConnectionType.QueuedConnection)
        self.maintenance_show_signal.connect(self._show_maintenance_result_window)
        self.maintenance_item_signal.connect(self.maintenance_output.appendPlainText, Qt.ConnectionType.QueuedConnection)
        self.maintenance_clear_signal.connect(self.maintenance_output.clear, Qt.ConnectionType.QueuedConnection)
        self.maintenance_hand_pressure_signal.connect(self._update_maintenance_hand_pressure)
        self.maintenance_pressure_state_signal.connect(self._set_maintenance_pressure_state)
        self.humanode_demo_item_signal.connect(self._append_humanode_demo_output, Qt.ConnectionType.QueuedConnection)
        self.humanode_demo_clear_signal.connect(self.humanode_demo_output.clear, Qt.ConnectionType.QueuedConnection)
        self.humanode_demo_session_state_signal.connect(self._set_humanode_demo_session_state)
        self.factory_test_item_signal.connect(self._append_factory_test_output, Qt.ConnectionType.QueuedConnection)
        self.factory_test_clear_signal.connect(self.factory_test_output.clear, Qt.ConnectionType.QueuedConnection)
        self.factory_force_values_signal.connect(self._factory_set_force_values, Qt.ConnectionType.QueuedConnection)
        self.factory_force_live_signal.connect(self._factory_update_force_live_side, Qt.ConnectionType.QueuedConnection)
        self.factory_force_monitor_state_signal.connect(self._factory_set_force_monitor_state, Qt.ConnectionType.QueuedConnection)
        self.factory_container_status_signal.connect(self._factory_set_container_status, Qt.ConnectionType.QueuedConnection)
        self.factory_topic_service_audit_signal.connect(self._factory_set_topic_service_audit, Qt.ConnectionType.QueuedConnection)
        self.factory_camera_state_signal.connect(self._factory_set_camera_state, Qt.ConnectionType.QueuedConnection)
        self.container_runtime_list_signal.connect(self._on_container_runtime_list_loaded)
        self.container_runtime_file_opened_signal.connect(self._on_container_runtime_file_opened)
        self.container_runtime_file_saved_signal.connect(self._on_container_runtime_file_saved)
        self.container_runtime_failed_signal.connect(self._on_container_runtime_failed)
        self.vision_image_signal.connect(self._set_vision_image, Qt.ConnectionType.QueuedConnection)
        self.vision_topic_rate_signal.connect(self._set_vision_topic_rate, Qt.ConnectionType.QueuedConnection)
        self.pose_image_signal.connect(self._set_pose_image, Qt.ConnectionType.QueuedConnection)
        self.factory_camera_image_signal.connect(self._set_factory_camera_image, Qt.ConnectionType.QueuedConnection)
        self.motion_exec_state_signal.connect(self._set_motion_exec_running)
        self.remote_edit_opened_signal.connect(self._on_remote_edit_opened)
        self.remote_edit_saved_signal.connect(self._on_remote_edit_saved)
        self.remote_edit_failed_signal.connect(self._on_remote_edit_failed)
        self.remote_edit_files_signal.connect(self._on_remote_edit_files_loaded)
        self.remote_edit_dirs_signal.connect(self._on_remote_edit_dirs_loaded)
        self.ros_lists_loaded_signal.connect(self._on_ros_lists_loaded, Qt.ConnectionType.QueuedConnection)
        self.log_audit_item_signal.connect(self.log_audit_output.appendPlainText, Qt.ConnectionType.QueuedConnection)
        self.log_audit_clear_signal.connect(self.log_audit_output.clear, Qt.ConnectionType.QueuedConnection)
        self.small_brain_sudo_prompt_signal.connect(self._on_request_small_brain_sudo_password, Qt.ConnectionType.QueuedConnection)

        self._monitor_started = False
        self._ssh_connecting = False
        self._ssh_big_connecting = False
        self._ros_connecting = False
        self._ros_op_seq = 0
        self._ros_op_lock = threading.Lock()
        self._cmd_echo_topic = None
        self._vision_active = False
        self._vision_continuous = False
        self._vision_topic = None
        self._vision_rate_refreshing = False
        self._pose_polling = False
        self._pose_busy = False
        self._pose_remote_coke = ""
        self._pose_remote_water = ""
        self._joint_recording = False
        self._joint_record_start_ts = 0.0
        self._joint_record_samples = []
        self._joint_record_lock = threading.Lock()
        self._servoj_tool_recording = False
        self._servoj_tool_start_ts = 0.0
        self._servoj_tool_samples = []
        self._servoj_tool_lock = threading.Lock()
        self._servoj_tool_topic = None
        self._servoj_tool_teach_active = False
        self._servoj_tool_playback_running = False
        self._servoj_tool_remote_recording = False
        self._servoj_tool_remote_record_path = ""
        self._servoj_tool_remote_record_pid_path = ""
        self._servoj_tool_remote_record_log_path = ""
        self._servoj_tool_remote_record_script_path = ""
        self._joint_live_plot = None
        self._latest_joint_state_map = {}
        self._latest_joint_state_ts = 0.0
        self._joint_state_fresh_timeout_sec = 1.5
        self._joint_monitor_fresh_timeout_sec = 4.0
        self._latest_joint_state_lock = threading.Lock()
        self._joint_monitor_topic = "/zj_humanoid/upperlimb/joint_states"
        self._tcp_pose_left_monitor_topic = "/zj_humanoid/upperlimb/tcp_pose/left_arm"
        self._tcp_pose_right_monitor_topic = "/zj_humanoid/upperlimb/tcp_pose/right_arm"
        self._tcp_speed_monitor_topic = "/zj_humanoid/upperlimb/tcp_speed/dual_arm"
        self._joint_monitor_client_lock = threading.Lock()
        self._joint_monitor_recovering = False
        self._tcp_pose_monitor_cache = {"left": {}, "right": {}}
        self._joint_monitor_last_recover_ts = 0.0
        self._joint_monitor_wait_first_sample_until = 0.0
        self._robot_status_topics = [
            "/zj_humanoid/robot/battery_info",
            "/zj_humanoid/robot/orin_states/errors",
            "/zj_humanoid/robot/orin_states/resource",
            "/zj_humanoid/robot/pico_states/errors",
            "/zj_humanoid/robot/pico_states/resource",
            "/zj_humanoid/robot/robot_state",
        ]
        self._robot_status_monitor_client_lock = threading.Lock()
        self._robot_status_monitor_recovering = False
        self._robot_status_last_update_ts = 0.0
        self._robot_status_last_recover_ts = 0.0
        self._robot_status_fresh_timeout_sec = 5.0
        self._safety_lock_service_candidates = [
            "/zj_humanoid/upperlimb/safetyLock",
            "/zj_humanoid/upperlimb/safety_lock",
            "/zj_humanoid/upperlimb/getSafetyLock",
            "/zj_humanoid/upperlimb/get_safety_lock",
        ]
        self._movej_thread_pool = QThreadPool.globalInstance()
        self._joint_ctrl_trace_lock = threading.Lock()
        self._joint_ctrl_trace_seq = []
        self._joint_ctrl_trace_single = []
        self._plot_dialog_refs = []
        self._pose_last_hash = {"coke": None, "water": None}
        self._pose_timer = QTimer(self)
        self._pose_timer.setInterval(1000)
        self._pose_timer.timeout.connect(self._poll_pose_images)
        self._safety_lock_timer = QTimer(self)
        self._safety_lock_timer.setInterval(2000)
        self._safety_lock_timer.timeout.connect(self._poll_safety_lock_state)
        self._safety_lock_timer.start()
        self._joint_monitor_watchdog_timer = QTimer(self)
        self._joint_monitor_watchdog_timer.setInterval(1000)
        self._joint_monitor_watchdog_timer.timeout.connect(self._check_joint_monitor_health)
        self._joint_monitor_watchdog_timer.start()
        self._robot_status_watchdog_timer = QTimer(self)
        self._robot_status_watchdog_timer.setInterval(2000)
        self._robot_status_watchdog_timer.timeout.connect(self._check_robot_status_monitor_health)
        self._robot_status_watchdog_timer.start()
        self._safety_lock_last_state = None
        self._safety_lock_polling = False
        self._refresh_robot_status_view()

    def _install_exception_logger(self):
        # 将未捕获异常打印到运行日志
        import sys

        def _emit_exception(prefix: str, exc_type, exc_value, exc_tb):
            try:
                lines = traceback.format_exception(exc_type, exc_value, exc_tb)
                text = "".join(lines).strip()
                self.log_signal.emit(f"{prefix}\n{text}")
            except Exception:
                pass

        def _sys_hook(exc_type, exc_value, exc_tb):
            _emit_exception("[ERR] 未捕获异常", exc_type, exc_value, exc_tb)

        sys.excepthook = _sys_hook

        if hasattr(threading, "excepthook"):
            def _thread_hook(args):
                _emit_exception("[ERR] 线程未捕获异常", args.exc_type, args.exc_value, args.exc_traceback)
            threading.excepthook = _thread_hook

    def _next_ros_op_seq(self) -> int:
        with self._ros_op_lock:
            self._ros_op_seq += 1
            return self._ros_op_seq

    def _is_current_ros_op_seq(self, seq: int) -> bool:
        with self._ros_op_lock:
            return int(seq) == int(self._ros_op_seq)

    def _sanitize_instance_tag(self, text: str) -> str:
        if not text:
            return ""
        return re.sub(r"[^A-Za-z0-9._-]+", "_", text).strip("_")

    def _normalize_edition(self, text: str) -> str:
        return "customer" if str(text or "").strip().lower() == "customer" else "internal"

    def _normalize_feature_flags(self, raw_flags) -> dict:
        return normalize_feature_flags(raw_flags)

    def _feature_enabled(self, key: str) -> bool:
        return bool(self.feature_flags.get(key, DEFAULT_FEATURE_FLAGS.get(key, True)))

    def _feature_label(self, key: str) -> str:
        return FEATURE_FLAG_LABELS.get(key, key)

    def _hide_tab_if_present(self, widget):
        if not hasattr(self, "tabs") or widget is None:
            return
        idx = self.tabs.indexOf(widget)
        if idx >= 0:
            self.tabs.removeTab(idx)

    def _set_widgets_enabled(self, enabled: bool, *widgets):
        for widget in widgets:
            if widget is not None:
                widget.setEnabled(enabled)

    def _is_smallbrain_target(self, target_name: str = "") -> bool:
        name = str(target_name or (self.transfer_target.currentText() if hasattr(self, "transfer_target") else "小脑")).strip()
        return name != "大脑"

    def _ensure_feature_enabled(self, key: str) -> bool:
        if self._feature_enabled(key):
            return True
        self.log_signal.emit(f"[ERR] 当前功能配置已禁用{self._feature_label(key)}")
        return False

    def _ensure_smallbrain_feature(self, key: str) -> bool:
        if not self._is_smallbrain_target():
            return True
        return self._ensure_feature_enabled(key)

    def _refresh_feature_control_states(self):
        is_smallbrain = self._is_smallbrain_target()
        if hasattr(self, "btn_upload"):
            self.btn_upload.setEnabled((not is_smallbrain) or self._feature_enabled("small_brain_upload_file"))
        if hasattr(self, "btn_download"):
            self.btn_download.setEnabled((not is_smallbrain) or self._feature_enabled("small_brain_download_file"))
        if hasattr(self, "btn_upload_dir"):
            self.btn_upload_dir.setEnabled((not is_smallbrain) or self._feature_enabled("small_brain_upload_folder"))
        if hasattr(self, "btn_download_dir"):
            self.btn_download_dir.setEnabled((not is_smallbrain) or self._feature_enabled("small_brain_download_folder"))
        self._set_widgets_enabled(self._feature_enabled("small_brain_restart_embedded"), getattr(self, "btn_restart_embedded", None))
        self._set_widgets_enabled(self._feature_enabled("small_brain_restart_middleware"), getattr(self, "btn_restart_middleware", None))
        self._set_widgets_enabled(self._feature_enabled("small_brain_pull_middleware_log"), getattr(self, "btn_pull_middleware_log", None))
        self._set_widgets_enabled(self._feature_enabled("small_brain_pull_embedded_logs"), getattr(self, "btn_pull_embedded_logs", None))

        middleware_enabled = self._feature_enabled("middleware_upgrade")
        self._set_widgets_enabled(
            middleware_enabled,
            getattr(self, "middleware_deploy_pkg_label", None),
            getattr(self, "middleware_deploy_pkg_edit", None),
            getattr(self, "middleware_deploy_robot_type_label", None),
            getattr(self, "middleware_deploy_robot_type_combo", None),
            getattr(self, "middleware_deploy_target_label", None),
            getattr(self, "middleware_deploy_big_checkbox", None),
            getattr(self, "middleware_deploy_small_checkbox", None),
            getattr(self, "middleware_deploy_force_checkbox", None),
            getattr(self, "btn_browse_middleware_pkg", None),
            getattr(self, "btn_deploy_middleware_pkg", None),
            getattr(self, "mpc_deploy_pkg_label", None),
            getattr(self, "mpc_deploy_pkg_edit", None),
            getattr(self, "mpc_deploy_remote_dir_label", None),
            getattr(self, "mpc_deploy_remote_dir_edit", None),
            getattr(self, "mpc_deploy_auto_extract_checkbox", None),
            getattr(self, "btn_browse_mpc_pkg", None),
            getattr(self, "btn_deploy_mpc_pkg", None),
            getattr(self, "mpc_image_pkg_label", None),
            getattr(self, "mpc_image_pkg_edit", None),
            getattr(self, "mpc_docker_offline_pkg_label", None),
            getattr(self, "mpc_docker_offline_pkg_edit", None),
            getattr(self, "mpc_image_name_label", None),
            getattr(self, "mpc_image_name_edit", None),
            getattr(self, "mpc_container_name_label", None),
            getattr(self, "mpc_container_name_edit", None),
            getattr(self, "btn_browse_mpc_image_pkg", None),
            getattr(self, "btn_browse_mpc_docker_offline_pkg", None),
            getattr(self, "btn_load_mpc_image", None),
            getattr(self, "mpc_ros_master_label", None),
            getattr(self, "mpc_ros_master_uri_edit", None),
            getattr(self, "mpc_ros_ip_label", None),
            getattr(self, "mpc_ros_ip_edit", None),
            getattr(self, "btn_prepare_mpc_container", None),
            getattr(self, "btn_deploy_mpc_all", None),
        )

        remote_read_enabled = self._feature_enabled("remote_file_read")
        remote_write_enabled = self._feature_enabled("remote_file_write")
        self._set_widgets_enabled(
            remote_read_enabled,
            getattr(self, "remote_edit_dir_label", None),
            getattr(self, "remote_edit_dir_input", None),
            getattr(self, "btn_remote_edit_refresh_dirs", None),
            getattr(self, "btn_remote_edit_enter_dir", None),
            getattr(self, "btn_remote_edit_parent_dir", None),
            getattr(self, "remote_edit_file_label", None),
            getattr(self, "remote_edit_file_input", None),
            getattr(self, "btn_remote_edit_refresh_files", None),
            getattr(self, "btn_remote_edit_pick_file", None),
            getattr(self, "btn_remote_edit_open", None),
            getattr(self, "btn_remote_edit_reload", None),
            getattr(self, "remote_find_label", None),
            getattr(self, "remote_find_input", None),
            getattr(self, "btn_remote_find_prev", None),
            getattr(self, "btn_remote_find_next", None),
        )
        self._set_widgets_enabled(remote_write_enabled, getattr(self, "btn_remote_edit_save", None), getattr(self, "btn_remote_edit_undo", None), getattr(self, "btn_remote_toggle_comment", None))
        if hasattr(self, "remote_file_editor"):
            self.remote_file_editor.setReadOnly(not remote_write_enabled)
        if hasattr(self, "remote_edit_status"):
            if not remote_read_enabled:
                self.remote_edit_status.setText("当前功能配置已禁用远程文件读取功能")
            elif not remote_write_enabled:
                self.remote_edit_status.setText("当前功能配置为远程文件只读模式")
        huiyang_enabled = self._feature_enabled("huiyang_tab")
        servoj_enabled = self._feature_enabled("servoj_tool_tab")
        if hasattr(self, "replay_huiyang_section"):
            self.replay_huiyang_section.setVisible(huiyang_enabled)
        if hasattr(self, "replay_servoj_tool_section"):
            self.replay_servoj_tool_section.setVisible(servoj_enabled)

    def _on_transfer_target_changed(self, _target: str):
        self._refresh_feature_control_states()
        if self._feature_enabled("remote_file_read"):
            self.refresh_remote_edit_directories()

    def _apply_edition_mode(self):
        self._refresh_feature_control_states()
        if not self._feature_enabled("connection_tab"):
            self._hide_tab_if_present(getattr(self, "tab_conn", None))
        if not self._feature_enabled("status_monitor_tab"):
            self._hide_tab_if_present(getattr(self, "tab_robot_status", None))
        if not self._feature_enabled("function_tab"):
            self._hide_tab_if_present(getattr(self, "tab_transfer", None))
        if not self._feature_enabled("joint_monitor_tab"):
            self._hide_tab_if_present(getattr(self, "tab_ros", None))
        if not self._feature_enabled("joint_control_tab"):
            self._hide_tab_if_present(getattr(self, "tab_joint_ctrl", None))
        if not self._feature_enabled("navigation_tab"):
            self._hide_tab_if_present(getattr(self, "tab_nav", None))
        if not self._feature_enabled("container_operations_tab"):
            self._hide_tab_if_present(getattr(self, "tab_container", None))
        if not self._feature_enabled("humanode_demo_tab"):
            self._hide_tab_if_present(getattr(self, "tab_humanode_demo", None))
        if not self._feature_enabled("factory_test_tab"):
            self._hide_tab_if_present(getattr(self, "tab_factory_test", None))
        if not (self._feature_enabled("huiyang_tab") or self._feature_enabled("servoj_tool_tab")):
            self._hide_tab_if_present(getattr(self, "tab_replay_tools", None))
        if not self._feature_enabled("vision_monitor_tab"):
            self._hide_tab_if_present(getattr(self, "tab_vision", None))
        if not self._feature_enabled("pose_estimation_tab"):
            self._hide_tab_if_present(getattr(self, "tab_pose", None))
        if not self._feature_enabled("maintenance_tab"):
            self._hide_tab_if_present(getattr(self, "tab_maintenance", None))
        if not self._feature_enabled("log_audit_tab"):
            self._hide_tab_if_present(getattr(self, "tab_log_audit", None))
        if not self._feature_enabled("command_tab"):
            self._hide_tab_if_present(getattr(self, "tab_cmd", None))
        if not self._feature_enabled("help_tab"):
            self._hide_tab_if_present(getattr(self, "tab_help", None))

    def _show_feature_config_window(self):
        if not hasattr(self, "feature_config_window") or self.feature_config_window is None:
            return
        self._refresh_feature_config_window()
        self.feature_config_window.show()
        self.feature_config_window.raise_()
        self.feature_config_window.activateWindow()

    def _create_feature_config_window(self):
        title = "功能配置"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"
        self.feature_config_window = _LogWindow()
        self.feature_config_window.setWindowTitle(title)
        self.feature_config_window.resize(880, 520)
        layout = QVBoxLayout(self.feature_config_window)
        self.feature_config_text = QPlainTextEdit()
        self.feature_config_text.setReadOnly(True)
        layout.addWidget(self.feature_config_text)
        self._refresh_feature_config_window()

    def _refresh_feature_config_window(self):
        if not hasattr(self, "feature_config_text"):
            return
        lines = [
            f"配置名称: {self.feature_profile_name or '-'}",
            f"edition: {self.edition}",
            f"配置文件: {self.feature_config_path or '-'}",
            "",
        ]
        current_group = ""
        for spec in FEATURE_FLAG_SPECS:
            group = str(spec.get("group") or "其他")
            if group != current_group:
                if current_group:
                    lines.append("")
                current_group = group
                lines.append(f"[{group}]")
            key = str(spec.get("key") or "")
            lines.append(f"- {self._feature_label(key)}: {'启用' if self._feature_enabled(key) else '关闭'}")
            desc = str(spec.get("description") or "").strip()
            if desc:
                lines.append(f"  {desc}")
        self.feature_config_text.setPlainText("\n".join(lines))

    def _get_joint_names_by_model(self, model: str):
        if str(model).upper() == "WA1":
            return [
                "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R", "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                "Neck_Z", "Neck_Y", "Waist_Z", "Waist_Y", "Lifting_Z",
            ]
        if str(model).upper() == "I2":
            return [
                "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R", "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                "Neck_Z", "Neck_Y", "A_Waist",
            ]
        return [
            "Chest_Z_L", "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
            "Chest_Z_R", "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R", "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
            "Neck_Z", "Neck_Y", "Pitch_Y_B", "Pitch_Y_M", "Waist_Z", "Waist_Y",
        ]

    def _valid_arm_types_for_model(self, model: str):
        m = str(model or "WA2").upper()
        if m == "WA1":
            return {1, 2, 3, 4, 8, 16, 31}
        if m == "I2":
            return {1, 2, 3, 4, 8, 15}
        return {1, 2, 3, 4, 8, 15}

    def _wa1_full_body_names(self):
        return [
            "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
            "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R", "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
            "Neck_Z", "Neck_Y", "Waist_Z", "Waist_Y", "Lifting_Z",
        ]

    def _i2_full_body_names(self):
        return [
            "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
            "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R", "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
            "Neck_Z", "Neck_Y", "A_Waist",
        ]

    def _clear_layout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
            else:
                child_layout = item.layout()
                if child_layout is not None:
                    self._clear_layout(child_layout)

    def _build_collapsible_section(self, title: str, body: QWidget, expanded: bool = True):
        wrapper = QWidget()
        wrapper_layout = QVBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)
        wrapper_layout.setSpacing(4)

        toggle = QToolButton()
        toggle.setText(title)
        toggle.setCheckable(True)
        toggle.setChecked(bool(expanded))
        toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        toggle.setArrowType(Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow)
        toggle.setStyleSheet("font-weight: 700; text-align: left;")

        body.setVisible(bool(expanded))

        def _on_toggled(checked: bool):
            body.setVisible(bool(checked))
            toggle.setArrowType(Qt.ArrowType.DownArrow if checked else Qt.ArrowType.RightArrow)

        toggle.toggled.connect(_on_toggled)

        wrapper_layout.addWidget(toggle)
        wrapper_layout.addWidget(body)
        return wrapper, toggle

    def _load_maintenance_finger_presets(self) -> list[tuple[str, list[float]]]:
        candidate_paths = []
        meipass = getattr(sys, "_MEIPASS", "")
        if meipass:
            candidate_paths.append(os.path.join(meipass, "config", "finger_joints.yaml"))
        runtime_dir = os.path.dirname(os.path.abspath(sys.executable)) if getattr(sys, "frozen", False) else ""
        if runtime_dir:
            candidate_paths.append(os.path.join(runtime_dir, "config", "finger_joints.yaml"))
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        candidate_paths.extend([
            os.path.join(project_root, "config", "finger_joints.yaml"),
            os.path.join(project_root, "Humanode_Naviai-main", "utils", "finger_joints.yaml"),
        ])
        preset_path = next((path for path in candidate_paths if os.path.isfile(path)), "")
        if not preset_path:
            return []
        try:
            with open(preset_path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception:
            return []

        presets = []
        original = data.get("original", {}).get("Position")
        if isinstance(original, list) and original:
            presets.append(("original", [float(x) for x in original]))

        finger_dof = data.get("finger_dof") or {}
        if isinstance(finger_dof, dict):
            for key in sorted(finger_dof.keys(), key=lambda item: int(item) if str(item).isdigit() else str(item)):
                pos = (finger_dof.get(key) or {}).get("Position")
                if isinstance(pos, list) and pos:
                    presets.append((f"preset_{key}", [float(x) for x in pos]))
        return presets

    def _maintenance_finger_preset_values(self, preset_name: str, target: str) -> list[float]:
        for name, values in self._maintenance_finger_presets:
            if name != preset_name:
                continue
            target_name = str(target or "dual").strip().lower()
            if target_name == "left":
                return list(values[:6])
            if target_name == "right":
                return list(values[-6:])
            return list(values)
        return []

    def _finger_labels_for_side(self) -> list[str]:
        return ["Thumb", "Index", "Middle", "Ring", "Pinky", "empty"]

    def _emit_maintenance(self, text: str):
        self.maintenance_show_signal.emit()
        self.maintenance_item_signal.emit(str(text or ""))

    def _set_maintenance_pressure_state(self, active: bool, status_text: str):
        self._maintenance_pressure_active = bool(active)
        if hasattr(self, "maintenance_pressure_status"):
            self.maintenance_pressure_status.setText(str(status_text or ("监听中" if active else "未启动")))
        if hasattr(self, "btn_maintenance_pressure_start"):
            self.btn_maintenance_pressure_start.setEnabled(not active)
        if hasattr(self, "btn_maintenance_pressure_stop"):
            self.btn_maintenance_pressure_stop.setEnabled(active)

    def _clear_maintenance_pressure_display(self):
        for side in ("left", "right"):
            for widget in getattr(self, "maintenance_pressure_fields", {}).get(side, []):
                widget.setText("-")
            if hasattr(self, "maintenance_pressure_meta") and side in self.maintenance_pressure_meta:
                self.maintenance_pressure_meta[side].setText("seq=- time=-")

    def _update_maintenance_hand_pressure(self, side: str, payload):
        side_key = str(side or "").strip().lower()
        if side_key not in {"left", "right"} or not isinstance(payload, dict):
            return

        header = payload.get("header") or {}
        seq = header.get("seq") if isinstance(header, dict) else None
        stamp = header.get("stamp") if isinstance(header, dict) else {}
        stamp_text = "-"
        if isinstance(stamp, dict) and stamp.get("secs") is not None:
            stamp_text = f"{stamp.get('secs')}.{int(stamp.get('nsecs') or 0):09d}"
        if hasattr(self, "maintenance_pressure_meta") and side_key in self.maintenance_pressure_meta:
            self.maintenance_pressure_meta[side_key].setText(f"seq={seq if seq is not None else '-'} time={stamp_text}")

        values = payload.get("pressure")
        if not isinstance(values, list):
            return
        for idx, widget in enumerate(getattr(self, "maintenance_pressure_fields", {}).get(side_key, [])):
            value = values[idx] if idx < len(values) else None
            widget.setText("-" if value is None else f"{float(value):.2f}")

        if bool(getattr(self, "_factory_pressure_capture_active", False)):
            max_map = getattr(self, "_factory_pressure_max", {}).get(side_key, [])
            for idx in range(min(len(values), len(max_map))):
                val = values[idx]
                if val is None:
                    continue
                val_f = float(val)
                old = max_map[idx]
                if old is None or val_f > float(old):
                    max_map[idx] = val_f
            self._factory_refresh_pressure_max_text()

    def _maintenance_list_payload(self, service_name: str, values: list, fallback_keys: tuple[str, ...]) -> dict:
        fields = self._resolve_service_request_fields(service_name)
        for field in fields:
            field_name = str(field.get("name") or "").strip()
            if field_name:
                return {field_name: values}
        for key in fallback_keys:
            if key:
                return {key: values}
        raise RuntimeError(f"无法推断服务请求字段: {service_name}")

    def _maintenance_parse_float_list(self, text: str) -> list[float]:
        raw = str(text or "").strip()
        if not raw:
            raise ValueError("请输入手指关节值")
        try:
            parsed = ast.literal_eval(raw)
            if isinstance(parsed, (list, tuple)):
                return [float(x) for x in parsed]
        except Exception:
            pass
        parts = [item.strip() for item in raw.split(",") if item.strip()]
        if not parts:
            raise ValueError("请输入逗号分隔的手指关节值")
        return [float(item) for item in parts]

    def _maintenance_expected_joint_count(self, target: str) -> int:
        return 12 if str(target or "dual").strip().lower() == "dual" else 6

    def _maintenance_format_resp(self, resp) -> str:
        if isinstance(resp, dict):
            return json.dumps(resp, ensure_ascii=False, indent=2)
        return self._format_resp_text(resp)

    def _call_ros_service_async(self, label: str, service_name: str, request: dict | None = None, timeout: float = 8.0, emit_line=None):
        if not self._ensure_ros():
            return

        log = emit_line or self.log_signal.emit

        def worker():
            try:
                req = request or {}
                log(f"[REQ] {label}: {service_name} {json.dumps(req, ensure_ascii=False)}")
                resp = self.ros.request_service(service_name, req, timeout=timeout)
                log(f"[OK] {label}\n{self._maintenance_format_resp(resp)}")
            except Exception as e:
                log(f"[ERR] {label}失败: {e}")

        self._run_async(worker)

    def _on_ros_lists_loaded(self, display_topics: list, display_services: list):
        topics = [t for t in (display_topics or []) if isinstance(t, str) and t.strip()]
        services = [s for s in (display_services or []) if isinstance(s, str) and s.strip()]

        self.topic_list.clear()
        self.topic_list.addItems(topics)
        self._cmd_topics = list(topics)
        self._refresh_vision_topic_options(topics)

        self.service_list.clear()
        self.service_list.addItems(services)
        self._cmd_services = list(services)
        self._rebuild_command_completions()

        self.log_signal.emit("[OK] 话题/服务列表已刷新")

    def _maintenance_call_service_async(self, label: str, service_name: str, request: dict | None = None, timeout: float = 8.0):
        self._call_ros_service_async(label, service_name, request, timeout=timeout, emit_line=self._emit_maintenance)

    def call_maintenance_tts(self):
        text = self.maintenance_tts_text.text().strip() if hasattr(self, "maintenance_tts_text") else ""
        if not text:
            self._emit_maintenance("[WARN] 请输入 TTS 文本")
            return
        request = {"text": [text], "isPlay": bool(self.maintenance_tts_play_checkbox.isChecked())}
        self._maintenance_call_service_async("TTS", "/zj_humanoid/audio/tts_service", request)

    def refresh_maintenance_microphones(self):
        self._maintenance_call_service_async("麦克风设备列表", "/zj_humanoid/audio/microphone/get_devices_list")

    def refresh_maintenance_speakers(self):
        self._maintenance_call_service_async("扬声器设备列表", "/zj_humanoid/audio/speaker/get_devices_list")

    def refresh_maintenance_audio_version(self):
        self._maintenance_call_service_async("语音模块版本", "/zj_humanoid/audio/version")

    def refresh_maintenance_sensor_realsense_serials(self):
        if not self._ensure_ssh_big():
            return

        def worker():
            inner_cmd = "rs-enumerate-devices -S"
            compose_cmds = [
                f"cd ~/navi_project && docker-compose exec -T sensor bash -lc {shlex.quote(inner_cmd)}",
                f"cd ~/navi_project && docker compose exec -T sensor bash -lc {shlex.quote(inner_cmd)}",
            ]
            last_error = ""

            self._emit_maintenance("[INFO] 开始检测 Sensor 容器中的 Realsense 序列号")
            for index, cmd in enumerate(compose_cmds, start=1):
                self._emit_maintenance(f"[REQ] 尝试{index}: {cmd}")
                try:
                    out, err, exit_code = self._run_bash_with_exit_code(self.ssh_big, cmd)
                except Exception as e:
                    last_error = str(e)
                    self._emit_maintenance(f"[WARN] 尝试{index}执行异常: {e}")
                    continue

                out_text = (out or "").strip()
                err_text = (err or "").strip()
                if exit_code == 0:
                    text = out_text or err_text or "命令执行完成，但没有输出"
                    self._emit_maintenance(f"[OK] Realsense 序列号\n{text}")
                    return

                last_error = err_text or out_text or f"exit_code={exit_code}"
                self._emit_maintenance(f"[WARN] 尝试{index}失败: {last_error}")

            self._emit_maintenance(f"[ERR] 获取 Realsense 序列号失败: {last_error or '未知错误'}")

        self._run_async(worker)

    def start_maintenance_pressure_monitor(self):
        if self._maintenance_pressure_active:
            self._emit_maintenance("[INFO] 手部压力监听已在运行")
            return
        ros_ok = bool(self.ros and self.ros.check_connection())
        ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
        if not ros_ok and not ssh_ok:
            self._emit_maintenance("[ERR] 请先连接 ROSBridge 或 SSH(小脑)")
            return
        self.maintenance_pressure_state_signal.emit(False, "启动中...")

        def worker():
            try:
                if not ros_ok:
                    cmd = self._build_remote_ros_echo_poller_command("pressure", self._maintenance_pressure_topics, interval_sec=0.05)

                    def _on_line(line: str):
                        text = str(line or "").strip()
                        if not text:
                            return
                        try:
                            evt = json.loads(text)
                        except Exception:
                            return
                        kind = str(evt.get("kind") or "")
                        if kind == "sample":
                            msg = self._parse_ros_cli_yaml(evt.get("text") or "") or {}
                            if isinstance(msg, dict):
                                self.maintenance_hand_pressure_signal.emit(str(evt.get("side") or ""), msg)
                            return
                        if kind == "error":
                            side_name = str(evt.get("side") or "")
                            err_text = str(evt.get("error") or "unknown")
                            self._emit_maintenance(f"[WARN] 压力监听SSH读取失败({side_name}): {err_text}")

                    def _on_exit(_out: str, _err: str, _exit_code):
                        self._maintenance_pressure_ssh_session = None

                    self._maintenance_pressure_ssh_session = self.ssh.start_interactive_session(
                        cmd,
                        on_output=_on_line,
                        on_exit=_on_exit,
                    )
                    self._maintenance_pressure_active = True
                    self.maintenance_pressure_state_signal.emit(True, "双手压力监听中(SSH脚本)")
                    self._emit_maintenance("[OK] 已开始监听双手手部压力传感器(SSH脚本)")
                    return

                def _make_callback(side_name: str):
                    def _callback(msg: dict):
                        self.maintenance_hand_pressure_signal.emit(side_name, msg)
                    return _callback

                for side_name, topic in self._maintenance_pressure_topics.items():
                    self.ros.subscribe(topic, _make_callback(side_name))
                self.maintenance_pressure_state_signal.emit(True, "双手压力监听中")
                self._emit_maintenance("[OK] 已开始监听双手手部压力传感器")
            except Exception as e:
                try:
                    for topic in self._maintenance_pressure_topics.values():
                        self.ros.unsubscribe(topic)
                except Exception:
                    pass
                self.maintenance_pressure_state_signal.emit(False, f"启动失败: {e}")
                self._emit_maintenance(f"[ERR] 启动手部压力监听失败: {e}")

        self._run_async(worker)

    def stop_maintenance_pressure_monitor(self, emit_log: bool = True):
        if self._maintenance_pressure_ssh_session:
            try:
                self._maintenance_pressure_ssh_session.close()
            except Exception:
                pass
            self._maintenance_pressure_ssh_session = None
        ros_client = self.ros
        if ros_client:
            for topic in self._maintenance_pressure_topics.values():
                try:
                    ros_client.unsubscribe(topic)
                except Exception:
                    pass
        self.maintenance_pressure_state_signal.emit(False, "未启动")
        self._clear_maintenance_pressure_display()
        if emit_log:
            self._emit_maintenance("[INFO] 已停止双手压力监听")

    def apply_maintenance_finger_preset(self):
        preset_name = self.maintenance_hand_preset_combo.currentData() if hasattr(self, "maintenance_hand_preset_combo") else ""
        target = self.maintenance_hand_target_combo.currentData() if hasattr(self, "maintenance_hand_target_combo") else "dual"
        values = self._maintenance_finger_preset_values(preset_name, target)
        if not values:
            self._emit_maintenance("[WARN] 当前预置没有可用的关节值")
            return
        self.maintenance_hand_joint_values.setText(json.dumps(values, ensure_ascii=False))
        self._emit_maintenance(f"[INFO] 已载入手指预置: {preset_name}")

    def send_maintenance_hand_joints(self):
        if not self._ensure_ros():
            return
        target = self.maintenance_hand_target_combo.currentData() if hasattr(self, "maintenance_hand_target_combo") else "dual"
        service_name = f"/zj_humanoid/hand/joint_switch/{target}"
        try:
            joints = self._maintenance_parse_float_list(self.maintenance_hand_joint_values.text() if hasattr(self, "maintenance_hand_joint_values") else "")
        except Exception as e:
            self._emit_maintenance(f"[ERR] 解析手指关节值失败: {e}")
            return
        expected = self._maintenance_expected_joint_count(target)
        if len(joints) != expected:
            self._emit_maintenance(f"[ERR] {target} 目标需要 {expected} 个关节值，当前为 {len(joints)} 个")
            return
        try:
            payload = self._maintenance_list_payload(service_name, joints, ("joint", "joints", "position", "positions"))
        except Exception as e:
            self._emit_maintenance(f"[ERR] 构造手指控制请求失败: {e}")
            return
        self._maintenance_call_service_async(f"手指控制({target})", service_name, payload)

    def _maintenance_topic_once(self, topic_name: str, timeout: float = 2.5):
        if not (self.ros and self.ros.check_connection()):
            raise RuntimeError("ROSBridge 未连接")

        done = threading.Event()
        box = {"msg": None}
        tmp_topic = None

        def _callback(msg: dict):
            if done.is_set():
                return
            box["msg"] = msg
            done.set()

        try:
            topic_type = self.ros._get_topic_type(topic_name)
            if not topic_type:
                raise RuntimeError(f"无法获取话题类型: {topic_name}")
            tmp_topic = roslibpy.Topic(self.ros.ros, topic_name, topic_type)
            tmp_topic.subscribe(_callback)
            done.wait(timeout=timeout)
            return box.get("msg") if done.is_set() else None
        finally:
            try:
                if tmp_topic is not None:
                    tmp_topic.unsubscribe()
            except Exception:
                pass

    def refresh_maintenance_robot_snapshot(self):
        if not self._ensure_ros():
            return

        def worker():
            try:
                topic_names = set(self.ros.list_topics() or [])
                self._emit_maintenance("[INFO] 开始检测 Robot 话题与服务")
                for topic_name in self._maintenance_robot_topics:
                    if topic_name not in topic_names:
                        self._emit_maintenance(f"[WARN] 话题不存在: {topic_name}")
                        continue
                    self._emit_maintenance(f"[REQ] rostopic echo -n 1 {topic_name}")
                    msg = self._maintenance_topic_once(topic_name, timeout=2.5)
                    if msg is None:
                        self._emit_maintenance(f"[WARN] 话题一帧超时: {topic_name}")
                    else:
                        self._emit_maintenance(f"[OK] {topic_name}\n{self._maintenance_format_resp(msg)}")

                for label, service_name in self._maintenance_robot_services:
                    self._emit_maintenance(f"[REQ] {label}: {service_name} {{}}")
                    try:
                        resp = self.ros.request_service(service_name, {}, timeout=6.0)
                    except Exception as e:
                        self._emit_maintenance(f"[ERR] {label} 调用失败: {e}")
                        continue
                    self._emit_maintenance(f"[OK] {label}\n{self._maintenance_format_resp(resp)}")
            except Exception as e:
                self._emit_maintenance(f"[ERR] Robot 检测失败: {e}")

        self._run_async(worker)

    def _toggle_log_panel(self, checked: bool):
        if checked:
            self._show_log_window()
            return
        if hasattr(self, "log_window") and self.log_window is not None:
            self.log_window.hide()

    def _ensure_log_panel_visible(self):
        self._show_log_window()

    def _show_log_window(self):
        if not hasattr(self, "log_window") or self.log_window is None:
            return
        self.log_window.show()
        self.log_window.raise_()
        self.log_window.activateWindow()

    def _create_log_window(self):
        title = "调试日志"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"

        self.log_window = _LogWindow()
        self.log_window.setWindowTitle(title)
        self.log_window.resize(980, 420)

        layout = QVBoxLayout(self.log_window)
        header = QHBoxLayout()
        self.log_conn_quality_label = QLabel("连接质量: -")
        self.log_conn_quality_label.setStyleSheet("color: #bbb;")
        header.addWidget(self.log_conn_quality_label)
        header.addStretch(1)
        header.addWidget(self.btn_clear_log)
        layout.addLayout(header)
        layout.addWidget(self.output)

    def _show_system_info_window(self):
        if not hasattr(self, "system_info_window") or self.system_info_window is None:
            return
        self.system_info_window.show()
        self.system_info_window.raise_()
        self.system_info_window.activateWindow()

    def _create_system_info_window(self):
        title = "系统信息"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"

        self.system_info_window = _LogWindow()
        self.system_info_window.setWindowTitle(title)
        self.system_info_window.resize(920, 360)

        layout = QVBoxLayout(self.system_info_window)
        self.system_info_text = QPlainTextEdit()
        self.system_info_text.setReadOnly(True)
        self.system_info_text.setPlaceholderText("等待版本检查结果...")
        layout.addWidget(self.system_info_text)

    def _show_humanode_demo_result_window(self, case_label: str = ""):
        if not hasattr(self, "humanode_demo_result_window") or self.humanode_demo_result_window is None:
            return
        title = "测试用例结果"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"
        if case_label:
            title = f"{title} - {case_label}"
        self.humanode_demo_result_window.setWindowTitle(title)
        self.humanode_demo_result_window.show()
        self.humanode_demo_result_window.raise_()
        self.humanode_demo_result_window.activateWindow()

    def _create_humanode_demo_result_window(self):
        title = "测试用例结果"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"

        self.humanode_demo_result_window = _LogWindow()
        self.humanode_demo_result_window.setWindowTitle(title)
        self.humanode_demo_result_window.resize(1180, 760)

        layout = QVBoxLayout(self.humanode_demo_result_window)
        header = QHBoxLayout()
        header.addWidget(QLabel("测试用例输出"))
        header.addStretch(1)

        self.btn_clear_humanode_demo_result = QPushButton("清空结果")
        self.btn_clear_humanode_demo_result.clicked.connect(lambda: self.humanode_demo_clear_signal.emit())
        header.addWidget(self.btn_clear_humanode_demo_result)
        layout.addLayout(header)

        self.humanode_demo_result_text = QPlainTextEdit()
        self.humanode_demo_result_text.setReadOnly(True)
        self.humanode_demo_result_text.setPlaceholderText("点击测试用例按钮后，这里会弹窗显示完整输出。")
        layout.addWidget(self.humanode_demo_result_text)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("交互状态"))
        self.humanode_demo_session_status = QLineEdit("当前无交互会话")
        self.humanode_demo_session_status.setReadOnly(True)
        status_row.addWidget(self.humanode_demo_session_status)
        layout.addLayout(status_row)

        input_row = QHBoxLayout()
        input_row.addWidget(QLabel("发送输入"))
        self.humanode_demo_input_edit = QLineEdit()
        self.humanode_demo_input_edit.setPlaceholderText("当脚本提示 input/按回车时，在这里输入并发送。")
        self.humanode_demo_input_edit.returnPressed.connect(self.send_humanode_demo_input)
        input_row.addWidget(self.humanode_demo_input_edit, 1)

        self.btn_humanode_demo_send_input = QPushButton("发送")
        self.btn_humanode_demo_send_input.clicked.connect(self.send_humanode_demo_input)
        input_row.addWidget(self.btn_humanode_demo_send_input)

        self.btn_humanode_demo_send_enter = QPushButton("发送回车")
        self.btn_humanode_demo_send_enter.clicked.connect(self.send_humanode_demo_enter)
        input_row.addWidget(self.btn_humanode_demo_send_enter)

        self.btn_humanode_demo_interrupt = QPushButton("发送 Ctrl+C")
        self.btn_humanode_demo_interrupt.clicked.connect(self.interrupt_humanode_demo_session)
        input_row.addWidget(self.btn_humanode_demo_interrupt)

        self.btn_humanode_demo_close_session = QPushButton("结束会话")
        self.btn_humanode_demo_close_session.clicked.connect(self.close_humanode_demo_session)
        input_row.addWidget(self.btn_humanode_demo_close_session)
        layout.addLayout(input_row)

        self.humanode_demo_item_signal.connect(self._append_humanode_demo_output)
        self.humanode_demo_clear_signal.connect(self.humanode_demo_result_text.clear)
        self._set_humanode_demo_session_state(False, "当前无交互会话")

    def _show_maintenance_result_window(self):
        if not hasattr(self, "maintenance_result_window") or self.maintenance_result_window is None:
            return
        self.maintenance_result_window.show()
        self.maintenance_result_window.raise_()
        self.maintenance_result_window.activateWindow()

    def _create_maintenance_result_window(self):
        title = "运维测试结果"
        if self.instance_name:
            title = f"{title} [{self.instance_name}]"

        self.maintenance_result_window = _LogWindow()
        self.maintenance_result_window.setWindowTitle(title)
        self.maintenance_result_window.resize(1080, 720)

        layout = QVBoxLayout(self.maintenance_result_window)
        header = QHBoxLayout()
        header.addWidget(QLabel("运维测试输出"))
        header.addStretch(1)

        self.btn_clear_maintenance_result = QPushButton("清空结果")
        self.btn_clear_maintenance_result.clicked.connect(lambda: self.maintenance_clear_signal.emit())
        header.addWidget(self.btn_clear_maintenance_result)
        layout.addLayout(header)

        self.maintenance_output = QPlainTextEdit()
        self.maintenance_output.setReadOnly(True)
        self.maintenance_output.setPlaceholderText("执行 Audio/Hand 测试后，这里会显示完整输出。")
        layout.addWidget(self.maintenance_output)

    def _dispose_auxiliary_window(self, attr_name: str):
        window = getattr(self, attr_name, None)
        if window is None:
            return
        try:
            if hasattr(window, "force_close"):
                window.force_close()
            else:
                window.close()
        except Exception:
            pass
        try:
            window.hide()
        except Exception:
            pass
        try:
            window.deleteLater()
        except Exception:
            pass
        setattr(self, attr_name, None)

    def _dispose_auxiliary_windows(self):
        for attr_name in (
            "humanode_demo_result_window",
            "maintenance_result_window",
            "feature_config_window",
            "system_info_window",
            "log_window",
        ):
            self._dispose_auxiliary_window(attr_name)

        for dlg in list(getattr(self, "_plot_dialog_refs", [])):
            try:
                dlg.close()
            except Exception:
                pass
            try:
                dlg.deleteLater()
            except Exception:
                pass
        self._plot_dialog_refs.clear()

    def _find_logo_path(self):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        candidates = [
            os.path.join(repo_root, "assert", "logo.jpg"),
            os.path.join(repo_root, "assert", "logo.png"),
            os.path.join(repo_root, "assets", "logo.png"),
            os.path.join(repo_root, "assets", "logo.jpg"),
            os.path.join(repo_root, "assets", "logo.jpeg"),
            os.path.join(repo_root, "packaging", "humanoid-robot-delivery-toolchain.svg"),
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return ""

    def _refresh_logo_pixmap(self):
        if not hasattr(self, "logo_label"):
            return
        if not hasattr(self, "_logo_source_pixmap") or self._logo_source_pixmap.isNull():
            self.logo_label.setPixmap(QPixmap())
            self.logo_label.setText("浙江人形机器人创新中心有限公司")
            return

        max_h = max(48, self.logo_label.maximumHeight() - 4)
        max_w = max(320, self.width() - 32)
        scaled = self._logo_source_pixmap.scaled(
            max_w,
            max_h,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.logo_label.setText("")
        self.logo_label.setPixmap(scaled)

    def _rebuild_joint_grid(self):
        if not hasattr(self, "joint_grid_layout"):
            return
        self._clear_layout(self.joint_grid_layout)
        self.joint_fields = {}
        for idx, name in enumerate(self.joint_names):
            row = idx // 2
            col = (idx % 2) * 2
            self.joint_grid_layout.addWidget(QLabel(name), row, col)
            val = QLineEdit("-")
            val.setReadOnly(True)
            self.joint_fields[name] = val
            self.joint_grid_layout.addWidget(val, row, col + 1)

    def _rebuild_joint_ctrl_parts(self):
        if not hasattr(self, "joint_ctrl_part"):
            return
        current = self.joint_ctrl_part.currentData()
        model = str(getattr(self, "robot_model", "WA2")).upper()
        # 示教模式全身统一用 15；WA1 的升降保持独立 arm_type=16
        full_body_type = 15
        self.joint_ctrl_part.blockSignals(True)
        self.joint_ctrl_part.clear()
        self.joint_ctrl_part.addItem("左臂", 1)
        self.joint_ctrl_part.addItem("右臂", 2)
        self.joint_ctrl_part.addItem("双臂", 3)
        self.joint_ctrl_part.addItem("脖子", 4)
        self.joint_ctrl_part.addItem("腰部", 8)
        if model == "WA1":
            self.joint_ctrl_part.addItem("升降", 16)
        self.joint_ctrl_part.addItem("全身", full_body_type)
        self.joint_ctrl_part.blockSignals(False)
        if current is not None:
            idx = self.joint_ctrl_part.findData(current)
            if idx >= 0:
                self.joint_ctrl_part.setCurrentIndex(idx)

    def _movej_test_arm_type_items_for_model(self, model: str):
        m = str(model or "WA2").upper()
        full_body_type = 31 if m == "WA1" else 15
        items = [
            ("左臂", 1),
            ("右臂", 2),
            ("双臂", 3),
            ("脖子", 4),
            ("腰部", 8),
        ]
        if m == "WA1":
            items.append(("升降", 16))
        items.append(("全身", full_body_type))
        return items

    def _wa1_lifting_limits(self):
        return 0.0, 0.235

    def _is_wa1_lifting_arm_type(self, arm_type: int) -> bool:
        try:
            arm_type_i = int(arm_type)
        except Exception:
            return False
        return str(getattr(self, "robot_model", "WA2")).upper() == "WA1" and arm_type_i == 16

    def _validate_pose_for_arm_type(self, arm_type: int, pose: list):
        values = [float(x) for x in (pose or [])]
        if self._is_wa1_lifting_arm_type(arm_type):
            if len(values) != 1:
                raise RuntimeError("升降目标必须是单元素数组，例如 [0.12]")
            lower, upper = self._wa1_lifting_limits()
            value = float(values[0])
            if value < lower or value > upper:
                raise RuntimeError(f"升降目标超限: {value:.3f}，允许范围 [{lower:.3f}, {upper:.3f}]")
        return values

    def _prompt_wa1_lifting_value(self, title: str, initial_value: float = None):
        lower, upper = self._wa1_lifting_limits()
        default_value = lower if initial_value is None else float(initial_value)
        default_value = max(lower, min(upper, default_value))
        value, ok = QInputDialog.getDouble(
            self,
            title,
            f"请输入升降目标值，范围 [{lower:.3f}, {upper:.3f}]",
            default_value,
            lower,
            upper,
            3,
        )
        if not ok:
            return None
        return float(value)

    def _rebuild_movej_test_parts(self):
        if not hasattr(self, "movej_test_part"):
            return
        current = self.movej_test_part.currentData()
        items = self._movej_test_arm_type_items_for_model(getattr(self, "robot_model", "WA2"))
        self.movej_test_part.blockSignals(True)
        self.movej_test_part.clear()
        for text, data in items:
            self.movej_test_part.addItem(text, data)
        self.movej_test_part.blockSignals(False)
        if current is not None:
            idx = self.movej_test_part.findData(current)
            if idx >= 0:
                self.movej_test_part.setCurrentIndex(idx)
        self._rebuild_movej_test_joint_inputs()

    def _rebuild_movej_test_joint_inputs(self):
        if not hasattr(self, "movej_test_part"):
            return
        arm_type = 1
        if hasattr(self, "movej_test_part"):
            try:
                arm_type = int(self.movej_test_part.currentData())
            except Exception:
                arm_type = 1
        names = self._joint_names_for_arm_type(arm_type)
        if hasattr(self, "movej_test_list_edit"):
            example_values = ["0.0"] * max(1, len(names))
            if self._is_wa1_lifting_arm_type(arm_type):
                example_values = ["0.120"]
            self.movej_test_list_edit.setPlaceholderText(
                f"直接输入数组，例如: [{', '.join(example_values)}]"
            )
        if hasattr(self, "movej_test_hint"):
            hint = f"当前部位: {self._arm_type_text(arm_type)} | 关节数: {len(names)} | 仅支持[]输入"
            if self._is_wa1_lifting_arm_type(arm_type):
                lower, upper = self._wa1_lifting_limits()
                hint += f" | 升降范围 [{lower:.3f}, {upper:.3f}]"
            self.movej_test_hint.setText(hint)

    def _set_movej_test_list_text(self, values: list):
        if not hasattr(self, "movej_test_list_edit"):
            return
        try:
            text = json.dumps([float(v) for v in (values or [])], ensure_ascii=False)
        except Exception:
            text = "[]"
        self.movej_test_list_edit.setPlainText(text)

    def _parse_movej_test_list_text(self, raw_text: str, expected_count: int):
        text = (raw_text or "").strip()
        if not text:
            raise RuntimeError("未输入目标数组")

        parsed = None
        try:
            parsed = json.loads(text)
        except Exception:
            try:
                parsed = yaml.safe_load(text)
            except Exception as e:
                raise RuntimeError(f"数组解析失败: {e}")

        if not isinstance(parsed, list):
            raise RuntimeError("目标必须是数组，例如 [0.1, -0.2]")
        if len(parsed) != int(expected_count):
            raise RuntimeError(f"数组长度不匹配: 期望{expected_count}，实际{len(parsed)}")

        out = []
        for i, v in enumerate(parsed, start=1):
            try:
                out.append(float(v))
            except Exception:
                raise RuntimeError(f"数组第{i}项不是数字: {v}")
        return out

    def _fill_movej_test_from_current_pose(self):
        if not hasattr(self, "movej_test_part"):
            return
        try:
            arm_type = int(self.movej_test_part.currentData())
        except Exception:
            arm_type = 1
        names = self._joint_names_for_arm_type(arm_type)
        if not names:
            self.log_signal.emit("[ERR] 当前部位不支持MoveJ测试")
            return
        data = self._fetch_joint_state_once() or {}
        missing = []
        vals = []
        for n in names:
            if n not in data:
                missing.append(n)
                continue
            vals.append(float(data[n]))
        if missing:
            self._set_movej_test_list_text(vals)
            self.log_signal.emit(f"[WARN] 已部分填充，缺少关节数据: {', '.join(missing)}")
        else:
            self._set_movej_test_list_text(vals)
            self.log_signal.emit(f"[OK] 已填充当前姿态: {self._arm_type_text(arm_type)}")

    def _collect_movej_test_pose(self):
        try:
            arm_type = int(self.movej_test_part.currentData())
        except Exception:
            arm_type = 1
        names = self._joint_names_for_arm_type(arm_type)
        if not names:
            raise RuntimeError("当前部位不支持MoveJ测试")

        list_text = self.movej_test_list_edit.toPlainText() if hasattr(self, "movej_test_list_edit") else ""
        vals = self._parse_movej_test_list_text(list_text, len(names))
        vals = self._validate_pose_for_arm_type(arm_type, vals)
        return arm_type, names, vals

    def _execute_single_movej(self, arm_type: int, pose: list, source_tag: str):
        service_name = self._movej_single_service_for_arm_type(arm_type)
        if not service_name:
            self.log_signal.emit(f"[ERR] {source_tag} 未找到movej服务: arm_type={arm_type}")
            return
        cfg = self._prompt_movej_config()
        if not cfg:
            return
        v, acc, t, is_async = cfg
        pose = self._validate_pose_for_arm_type(arm_type, pose)
        req = {
            "joints": [float(x) for x in pose],
            "v": v,
            "acc": acc,
            "t": t,
            "is_async": is_async,
            "arm_type": int(arm_type),
        }
        names = self._joint_names_for_arm_type(int(arm_type))
        desired_map = {}
        for i, n in enumerate(names):
            if i < len(pose):
                desired_map[n] = float(pose[i])
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

        def _pose_reached(target_map: dict, actual_map: dict, tol: float = 0.08) -> bool:
            if not target_map or not actual_map:
                return False
            for jn, target in target_map.items():
                if jn not in actual_map:
                    return False
                try:
                    if abs(float(actual_map[jn]) - float(target)) > float(tol):
                        return False
                except Exception:
                    return False
            return True

        if not self._monitor_started:
            self.start_monitor()

        def worker():
            try:
                self._clear_joint_control_trace("single")
                trace_start = time.monotonic()
                sampler_stop = threading.Event()
                sampler = threading.Thread(
                    target=self._sample_control_trace,
                    args=("single", trace_start, max(1.0, float(t) if float(t) > 0 else 2.0), lambda _et: desired_map, sampler_stop, source_tag),
                    daemon=True,
                )
                sampler.start()
                self.log_signal.emit(f"[REQ] movej单步({transport_mode}): {source_tag} {self._arm_type_text(arm_type)} -> {service_name}, payload={json.dumps(req, ensure_ascii=False)}")
                if transport_mode == "ros":
                    if not (self.ros and self.ros.check_connection()):
                        raise RuntimeError("ROSBridge 未连接")
                    service_timeout = 20.0
                    try:
                        resp = self.ros.request_service(service_name, req, timeout=service_timeout)
                        self.log_signal.emit(f"[OK] movej单步成功: {self._format_resp_text(resp)}")
                    except TimeoutError:
                        verify_deadline = time.monotonic() + 2.5
                        reached = False
                        while time.monotonic() < verify_deadline:
                            if _pose_reached(desired_map, self._get_latest_joint_state_map()):
                                reached = True
                                break
                            time.sleep(0.2)
                        if reached:
                            self.log_signal.emit(
                                "[WARN] movej服务响应超时，但关节已到位，按成功处理(可能是服务端回包延迟)"
                            )
                        else:
                            raise
                else:
                    if not self._ensure_ssh():
                        return
                    cmd = f"rosservice call {service_name} {shlex.quote(json.dumps(req, ensure_ascii=False))}"
                    self.log_signal.emit(f"[REQ] {cmd}")
                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                    text = ((out or "") + "\n" + (err or "")).strip()
                    self.log_signal.emit(f"[OK] movej单步完成: {text or '调用完成'}")

                self._append_joint_control_trace_sample(
                    "single",
                    time.monotonic() - trace_start,
                    desired_map,
                    self._get_latest_joint_state_map(),
                    tag=source_tag,
                )
                self.log_signal.emit("[INFO] 已记录单步调试期望/实时轨迹，可点击 控制Plot 查看")
            except Exception as e:
                self.log_signal.emit(f"[ERR] movej单步失败: {e}")
            finally:
                try:
                    sampler_stop.set()
                except Exception:
                    pass
                try:
                    sampler.join(timeout=1.0)
                except Exception:
                    pass

        self._run_movej_async(worker)

    def run_movej_test(self):
        if not hasattr(self, "movej_test_part"):
            return
        try:
            arm_type, names, pose = self._collect_movej_test_pose()
            self.log_signal.emit(
                f"[INFO] MoveJ测试目标: {self._arm_type_text(arm_type)} joints={len(names)}"
            )
            self._execute_single_movej(arm_type, pose, "独立MoveJ测试")
        except Exception as e:
            self.log_signal.emit(f"[ERR] MoveJ测试参数错误: {e}")

    def _prompt_movel_config(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("movel配置")
        form = QFormLayout(dlg)

        v_edit = QLineEdit("0.2")
        acc_edit = QLineEdit("0.2")
        async_combo = QComboBox()
        async_combo.addItems(["同步", "异步"])

        form.addRow("速度 v", v_edit)
        form.addRow("加速度 acc", acc_edit)
        form.addRow("是否异步", async_combo)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        form.addRow(buttons)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return None

        try:
            v = float((v_edit.text() or "0.2").strip())
            acc = float((acc_edit.text() or "0.2").strip())
        except Exception:
            self.log_signal.emit("[ERR] movel v/acc 格式错误")
            return None

        is_async = async_combo.currentText() == "异步"
        return v, acc, is_async

    def _movel_single_service_for_arm_key(self, arm_key: str) -> str:
        mapping = {
            "left": "/zj_humanoid/upperlimb/movel/left_arm",
            "right": "/zj_humanoid/upperlimb/movel/right_arm",
        }
        return mapping.get(str(arm_key or "left").lower())

    def _tcp_pose_topic_for_arm_key(self, arm_key: str) -> str:
        if str(arm_key or "left").lower() == "right":
            return self._tcp_pose_right_monitor_topic
        return self._tcp_pose_left_monitor_topic

    def _tcp_pose_label_for_arm_key(self, arm_key: str) -> str:
        return "右臂" if str(arm_key or "left").lower() == "right" else "左臂"

    def _extract_tcp_position(self, pose_msg: dict):
        if not isinstance(pose_msg, dict):
            raise RuntimeError("TCP pose 数据为空")
        position = pose_msg.get("position") if isinstance(pose_msg.get("position"), dict) else None
        if not isinstance(position, dict):
            raise RuntimeError("TCP pose 缺少 position")
        try:
            return {
                "x": float(position.get("x", 0.0)),
                "y": float(position.get("y", 0.0)),
                "z": float(position.get("z", 0.0)),
            }
        except Exception as e:
            raise RuntimeError(f"TCP position 解析失败: {e}") from e

    def _build_geometry_pose_from_tcp_pose(self, pose_msg: dict):
        position = self._extract_tcp_position(pose_msg)
        quaternion = pose_msg.get("quaternion") if isinstance(pose_msg.get("quaternion"), dict) else {}
        try:
            orientation = {
                "x": float(quaternion.get("x", 0.0)),
                "y": float(quaternion.get("y", 0.0)),
                "z": float(quaternion.get("z", 0.0)),
                "w": float(quaternion.get("w", 1.0)),
            }
        except Exception as e:
            raise RuntimeError(f"TCP quaternion 解析失败: {e}") from e
        return {"position": position, "orientation": orientation}

    def _format_xyz_triplet(self, xyz: dict) -> str:
        if not isinstance(xyz, dict):
            return "-"
        try:
            return f"x={float(xyz.get('x', 0.0)):.4f}, y={float(xyz.get('y', 0.0)):.4f}, z={float(xyz.get('z', 0.0)):.4f}"
        except Exception:
            return "-"

    def _parse_movel_distance(self) -> float:
        text = self.movel_test_distance_edit.text().strip() if hasattr(self, "movel_test_distance_edit") else ""
        try:
            return float(text)
        except Exception as e:
            raise RuntimeError(f"距离格式错误: {e}") from e

    def _compute_movel_target_pose(self, current_pose: dict, axis: str, direction_sign: float, distance: float):
        target_pose = copy.deepcopy(current_pose)
        position = target_pose.get("position") if isinstance(target_pose.get("position"), dict) else None
        if not isinstance(position, dict):
            raise RuntimeError("目标姿态缺少 position")
        if axis not in ("x", "y", "z"):
            raise RuntimeError(f"不支持的方向轴: {axis}")
        position[axis] = float(position.get(axis, 0.0)) + float(direction_sign) * float(distance)
        return target_pose

    def _default_geometry_pose(self):
        return {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
        }

    def _set_movel_test_preview(self, current_pose: dict = None, target_pose: dict = None):
        if hasattr(self, "movel_test_current_xyz"):
            current_position = current_pose.get("position") if isinstance(current_pose, dict) else None
            self.movel_test_current_xyz.setText(self._format_xyz_triplet(current_position))
        if hasattr(self, "movel_test_target_xyz"):
            target_position = target_pose.get("position") if isinstance(target_pose, dict) else None
            self.movel_test_target_xyz.setText(self._format_xyz_triplet(target_position))

    def _reset_movel_test_preview(self):
        self._movel_test_current_pose = None
        self._set_movel_test_preview(None, None)

    def _update_movel_test_target_preview(self):
        current_pose = self._movel_test_current_pose if isinstance(self._movel_test_current_pose, dict) else None
        if not current_pose:
            self._set_movel_test_preview(None, None)
            return
        try:
            direction = self.movel_test_direction_combo.currentData() if hasattr(self, "movel_test_direction_combo") else None
            if not isinstance(direction, tuple) or len(direction) != 2:
                raise RuntimeError("未选择MoveL方向")
            axis, direction_sign = direction
            distance = self._parse_movel_distance()
            target_pose = self._compute_movel_target_pose(current_pose, axis, direction_sign, distance)
            self._set_movel_test_preview(current_pose, target_pose)
        except Exception:
            self._set_movel_test_preview(current_pose, None)

    def _fetch_tcp_pose_once(self, arm_key: str):
        key = str(arm_key or "left").lower()
        cached = self._tcp_pose_monitor_cache.get(key) if isinstance(getattr(self, "_tcp_pose_monitor_cache", None), dict) else None
        if isinstance(cached, dict) and isinstance(cached.get("position"), dict):
            return copy.deepcopy(cached)

        topic = self._tcp_pose_topic_for_arm_key(key)

        if self.ros and self.ros.check_connection():
            done = threading.Event()
            box = {"data": None}

            def cb(msg: dict):
                try:
                    if isinstance(msg, dict) and isinstance(msg.get("position"), dict):
                        box["data"] = copy.deepcopy(msg)
                except Exception:
                    box["data"] = None
                finally:
                    done.set()

            try:
                topic_type = self.ros._get_topic_type(topic)
                if topic_type:
                    tmp_topic = roslibpy.Topic(self.ros.ros, topic, topic_type)
                    tmp_topic.subscribe(cb)
                else:
                    tmp_topic = None
                done.wait(timeout=2.0)
            except Exception:
                pass
            finally:
                try:
                    if 'tmp_topic' in locals() and tmp_topic is not None:
                        tmp_topic.unsubscribe()
                except Exception:
                    pass

            if isinstance(box.get("data"), dict) and box["data"]:
                return box["data"]

        if self.ssh and self.ssh.ssh and self.ssh.sftp:
            try:
                out, err = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {topic}")
                text = (out or "") + "\n" + (err or "")
                msg = self._parse_ros_cli_yaml(text) or {}
                if isinstance(msg, dict) and isinstance(msg.get("position"), dict):
                    return msg
            except Exception:
                pass

        raise RuntimeError(f"未获取到{self._tcp_pose_label_for_arm_key(key)} TCP pose")

    def _fill_movel_test_from_current_pose(self):
        if not hasattr(self, "movel_test_arm_combo"):
            return
        try:
            arm_key = str(self.movel_test_arm_combo.currentData() or "left").lower()
            current_tcp_pose = self._fetch_tcp_pose_once(arm_key)
            geometry_pose = self._build_geometry_pose_from_tcp_pose(current_tcp_pose)
            self._movel_test_current_pose = geometry_pose
            self._update_movel_test_target_preview()
            self.log_signal.emit(f"[OK] 已读取当前TCP姿态: {self._tcp_pose_label_for_arm_key(arm_key)}")
        except Exception as e:
            self._reset_movel_test_preview()
            self.log_signal.emit(f"[ERR] 读取当前TCP失败: {e}")

    def run_movel_test(self):
        if not hasattr(self, "movel_test_arm_combo"):
            return
        try:
            arm_key = str(self.movel_test_arm_combo.currentData() or "left").lower()
            direction = self.movel_test_direction_combo.currentData() if hasattr(self, "movel_test_direction_combo") else None
            if not isinstance(direction, tuple) or len(direction) != 2:
                raise RuntimeError("未选择MoveL方向")
            axis, direction_sign = direction
            distance = self._parse_movel_distance()
            current_tcp_pose = self._fetch_tcp_pose_once(arm_key)
            current_pose = self._build_geometry_pose_from_tcp_pose(current_tcp_pose)
            target_pose = self._compute_movel_target_pose(current_pose, axis, direction_sign, distance)
            self._movel_test_current_pose = current_pose
            self._set_movel_test_preview(current_pose, target_pose)

            service_name = self._movel_single_service_for_arm_key(arm_key)
            if not service_name:
                raise RuntimeError(f"未找到MoveL服务: arm={arm_key}")
            cfg = self._prompt_movel_config()
            if not cfg:
                return
            v, acc, is_async = cfg
            req = {
                "pose": [copy.deepcopy(target_pose), self._default_geometry_pose()],
                "v": float(v),
                "acc": float(acc),
                "is_async": bool(is_async),
            }
            transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

            def worker():
                try:
                    self.log_signal.emit(
                        f"[REQ] movel单步({transport_mode}): {self._tcp_pose_label_for_arm_key(arm_key)} -> {service_name}, payload={json.dumps(req, ensure_ascii=False)}"
                    )
                    if transport_mode == "ros":
                        if not (self.ros and self.ros.check_connection()):
                            raise RuntimeError("ROSBridge 未连接")
                        resp = self.ros.request_service(service_name, req, timeout=20.0)
                        self.log_signal.emit(f"[OK] movel单步成功: {self._format_resp_text(resp)}")
                    else:
                        if not self._ensure_ssh():
                            return
                        cmd = f"rosservice call {service_name} {shlex.quote(json.dumps(req, ensure_ascii=False))}"
                        self.log_signal.emit(f"[REQ] {cmd}")
                        out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                        text = ((out or "") + "\n" + (err or "")).strip()
                        self.log_signal.emit(f"[OK] movel单步完成: {text or '调用完成'}")
                except Exception as e:
                    self.log_signal.emit(f"[ERR] movel单步失败: {e}")

            self._run_movej_async(worker)
        except Exception as e:
            self.log_signal.emit(f"[ERR] MoveL测试参数错误: {e}")

    def _set_robot_model(self, model: str):
        model_up = str(model).upper()
        if model_up == "WA1":
            new_model = "WA1"
        elif model_up.startswith("I"):
            new_model = "I2"
        elif model_up == "WA2_LS":
            new_model = "WA2_LS"
        else:
            new_model = "WA2"
        old_model = getattr(self, "robot_model", None)
        if old_model == new_model and self.joint_fields:
            return

        if hasattr(self, "motion_seq_selector"):
            self._persist_current_sequence_rows()

        self.robot_model = new_model

        if hasattr(self, "robot_model_combo"):
            self.robot_model_combo.blockSignals(True)
            self.robot_model_combo.setCurrentText(new_model)
            self.robot_model_combo.blockSignals(False)
        if hasattr(self, "robot_model_ctrl_combo"):
            self.robot_model_ctrl_combo.blockSignals(True)
            self.robot_model_ctrl_combo.setCurrentText(new_model)
            self.robot_model_ctrl_combo.blockSignals(False)
        if hasattr(self, "middleware_deploy_robot_type_combo"):
            self.middleware_deploy_robot_type_combo.blockSignals(True)
            self.middleware_deploy_robot_type_combo.setCurrentText(self._default_middleware_deploy_robot_type())
            self.middleware_deploy_robot_type_combo.blockSignals(False)
        if hasattr(self, "btn_factory_test_upperlimb"):
            if new_model == "WA2_LS":
                self.btn_factory_test_upperlimb.setText("8) 上传YAML并执行上肢回放")
            else:
                self.btn_factory_test_upperlimb.setText("8) 运行上肢测试")
        if hasattr(self, "btn_maintenance_upperlimb_test"):
            if new_model == "WA2_LS":
                self.btn_maintenance_upperlimb_test.setText("上传YAML并执行上肢回放")
            else:
                self.btn_maintenance_upperlimb_test.setText("执行上肢测试")
        if hasattr(self, "maintenance_wa1_single_joint_box"):
            self.maintenance_wa1_single_joint_box.setVisible(new_model == "WA1")
        if hasattr(self, "maintenance_wa1_joint_combo"):
            self._maintenance_refresh_wa1_joint_options()
        if hasattr(self, "maintenance_wa1_status_label"):
            self._maintenance_update_wa1_status_label()

        self.joint_names = self._get_joint_names_by_model(self.robot_model)
        self._rebuild_joint_grid()
        self._rebuild_joint_ctrl_parts()
        self._rebuild_movej_test_parts()

        if hasattr(self, "motion_seq_selector"):
            self._restore_motion_sequences_from_disk()
            self.log_signal.emit(f"[OK] 已切换型号: {new_model}，已加载该型号动作序列")
            self._update_motion_model_label()

        if hasattr(self, "humanode_demo_case_layout"):
            self.refresh_humanode_demo_case_buttons()
        if hasattr(self, "factory_camera_head_required"):
            self._factory_refresh_camera_labels()
        if hasattr(self, "factory_camera_head_topic_value"):
            self._factory_update_camera_topics_display()
        if hasattr(self, "huiyang_model_combo"):
            self._update_huiyang_mode_label()

    def _update_motion_model_label(self):
        if not hasattr(self, "motion_model_label"):
            return
        model = str(getattr(self, "robot_model", "WA2")).upper()
        self.motion_model_label.setText(f"当前序列型号: {model}")

    def _middleware_deploy_robot_type_options(self) -> list[str]:
        return ["WA2_LS", "WA1", "WA1_400L", "WA1_400K", "I2"]

    def _default_middleware_deploy_robot_type(self) -> str:
        model = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
        if model == "WA1":
            return "WA1"
        if model == "I2":
            return "I2"
        return "WA2_LS"

    def _set_joint_ctrl_transport_mode(self, mode: str):
        mode = "ssh" if str(mode).lower() == "ssh" else "ros"
        self._joint_ctrl_transport_mode = mode
        if hasattr(self, "btn_joint_ctrl_ros"):
            self.btn_joint_ctrl_ros.blockSignals(True)
            self.btn_joint_ctrl_ros.setChecked(mode == "ros")
            self.btn_joint_ctrl_ros.blockSignals(False)
            if mode == "ros":
                self.btn_joint_ctrl_ros.setStyleSheet("background-color: #1f6feb; color: #ffffff; border: 1px solid #58a6ff; font-weight: 700;")
            else:
                self.btn_joint_ctrl_ros.setStyleSheet("background-color: #30363d; color: #c9d1d9; border: 1px solid #6e7681;")
        if hasattr(self, "btn_joint_ctrl_ssh"):
            self.btn_joint_ctrl_ssh.blockSignals(True)
            self.btn_joint_ctrl_ssh.setChecked(mode == "ssh")
            self.btn_joint_ctrl_ssh.blockSignals(False)
            if mode == "ssh":
                self.btn_joint_ctrl_ssh.setStyleSheet("background-color: #d97706; color: #ffffff; border: 1px solid #f59e0b; font-weight: 700;")
            else:
                self.btn_joint_ctrl_ssh.setStyleSheet("background-color: #30363d; color: #c9d1d9; border: 1px solid #6e7681;")
        QTimer.singleShot(0, self._poll_safety_lock_state)

    def _build_ui(self):
        self.tabs = QTabWidget()

        # ---------- Tab 1: 连接 ----------
        tab_conn = QWidget()
        conn_layout = QVBoxLayout()

        self.ros_host = QLineEdit("192.168.217.100")
        self.ros_port = QLineEdit("9090")
        self.btn_ros = QPushButton("连接 ROSBridge")
        self.btn_ros.clicked.connect(self.connect_ros)
        self.btn_ros_disconnect = QPushButton("断开 ROSBridge")
        self.btn_ros_disconnect.clicked.connect(self.disconnect_ros)

        ros_line = QHBoxLayout()
        ros_line.addWidget(QLabel("ROS Host"))
        ros_line.addWidget(self.ros_host)
        ros_line.addWidget(QLabel("Port"))
        ros_line.addWidget(self.ros_port)
        ros_line.addWidget(self.btn_ros)
        ros_line.addWidget(self.btn_ros_disconnect)

        self.ssh_host = QLineEdit("192.168.217.66")
        self.ssh_port = QLineEdit("22")
        self.ssh_user = QLineEdit("nav01")
        self.ssh_pwd = QLineEdit(" ")
        self.ssh_pwd.setEchoMode(QLineEdit.EchoMode.Password)
        self.btn_ssh = QPushButton("连接 SSH(小脑)")
        self.btn_ssh.clicked.connect(self.connect_ssh)
        self.btn_ssh_disconnect = QPushButton("断开 SSH(小脑)")
        self.btn_ssh_disconnect.clicked.connect(self.disconnect_ssh)

        ssh_line = QHBoxLayout()
        ssh_line.addWidget(QLabel("小脑 SSH Host"))
        ssh_line.addWidget(self.ssh_host)
        ssh_line.addWidget(QLabel("Port"))
        ssh_line.addWidget(self.ssh_port)
        ssh_line.addWidget(QLabel("User"))
        ssh_line.addWidget(self.ssh_user)
        ssh_line.addWidget(QLabel("Password"))
        ssh_line.addWidget(self.ssh_pwd)
        ssh_line.addWidget(self.btn_ssh)
        ssh_line.addWidget(self.btn_ssh_disconnect)

        self.ssh_big_host = QLineEdit("192.168.217.100")
        self.ssh_big_port = QLineEdit("22")
        self.ssh_big_user = QLineEdit("naviai")
        self.ssh_big_pwd = QLineEdit("naviai@2024")
        self.ssh_big_pwd.setEchoMode(QLineEdit.EchoMode.Password)
        self.btn_ssh_big = QPushButton("连接 SSH(大脑)")
        self.btn_ssh_big.clicked.connect(self.connect_ssh_big)
        self.btn_ssh_big_disconnect = QPushButton("断开 SSH(大脑)")
        self.btn_ssh_big_disconnect.clicked.connect(self.disconnect_ssh_big)

        ssh_big_line = QHBoxLayout()
        ssh_big_line.addWidget(QLabel("大脑 SSH Host"))
        ssh_big_line.addWidget(self.ssh_big_host)
        ssh_big_line.addWidget(QLabel("Port"))
        ssh_big_line.addWidget(self.ssh_big_port)
        ssh_big_line.addWidget(QLabel("User"))
        ssh_big_line.addWidget(self.ssh_big_user)
        ssh_big_line.addWidget(QLabel("Password"))
        ssh_big_line.addWidget(self.ssh_big_pwd)
        ssh_big_line.addWidget(self.btn_ssh_big)
        ssh_big_line.addWidget(self.btn_ssh_big_disconnect)

        conn_layout.addLayout(ros_line)
        conn_layout.addLayout(ssh_line)
        conn_layout.addLayout(ssh_big_line)

        # 连接状态与电池信息
        self.ros_status_value = QLineEdit("未连接")
        self.ros_status_value.setReadOnly(True)
        self.ssh_status_value = QLineEdit("未连接")
        self.ssh_status_value.setReadOnly(True)
        self.ssh_big_status_value = QLineEdit("未连接")
        self.ssh_big_status_value.setReadOnly(True)
        self.battery_conn_value = QLineEdit("-")
        self.battery_conn_value.setReadOnly(True)

        status_line = QHBoxLayout()
        status_line.addWidget(QLabel("ROSBridge 状态"))
        status_line.addWidget(self.ros_status_value)
        status_line.addWidget(QLabel("SSH(小脑) 状态"))
        status_line.addWidget(self.ssh_status_value)
        status_line.addWidget(QLabel("SSH(大脑) 状态"))
        status_line.addWidget(self.ssh_big_status_value)

        battery_line = QHBoxLayout()
        battery_line.addWidget(QLabel("电池电量"))
        battery_line.addWidget(self.battery_conn_value)
        battery_line.addStretch(1)

        conn_layout.addLayout(status_line)
        conn_layout.addLayout(battery_line)
        tab_conn.setLayout(conn_layout)

        # ---------- Tab 2: 状态监控 ----------
        tab_robot_status = QWidget()
        robot_layout = QVBoxLayout()

        robot_grid = QGridLayout()
        self.robot_battery_value = QLineEdit("-")
        self.robot_battery_value.setReadOnly(True)
        self.robot_version_label = QLabel("机器人型号")
        self.robot_version_value = QLineEdit("-")
        self.robot_version_value.setReadOnly(True)
        self.hardware_version_label = QLabel("嵌入式版本")
        self.hardware_version_value = QLineEdit("-")
        self.hardware_version_value.setReadOnly(True)
        self.software_version_value = QLineEdit("-")
        self.software_version_value.setReadOnly(True)
        self.upperlimb_version_label = QLabel("上肢版本")
        self.upperlimb_version_value = QLineEdit("-")
        self.upperlimb_version_value.setReadOnly(True)
        self.lowerlimb_version_label = QLabel("下肢版本")
        self.lowerlimb_version_value = QLineEdit("-")
        self.lowerlimb_version_value.setReadOnly(True)
        self.robot_state_info_value = QLineEdit("-")
        self.robot_state_info_value.setReadOnly(True)
        self.robot_orin_status_value = QLineEdit("-")
        self.robot_orin_status_value.setReadOnly(True)
        self.robot_orin_latency_value = QLineEdit("-")
        self.robot_orin_latency_value.setReadOnly(True)
        self.robot_pico_status_value = QLineEdit("-")
        self.robot_pico_status_value.setReadOnly(True)
        self.robot_pico_latency_value = QLineEdit("-")
        self.robot_pico_latency_value.setReadOnly(True)

        robot_grid.addWidget(QLabel("电池电量"), 0, 0)
        robot_grid.addWidget(self.robot_battery_value, 0, 1)
        robot_grid.addWidget(self.robot_version_label, 1, 0)
        robot_grid.addWidget(self.robot_version_value, 1, 1)
        robot_grid.addWidget(self.hardware_version_label, 2, 0)
        robot_grid.addWidget(self.hardware_version_value, 2, 1)
        robot_grid.addWidget(QLabel("中间件版本"), 3, 0)
        robot_grid.addWidget(self.software_version_value, 3, 1)
        robot_grid.addWidget(self.upperlimb_version_label, 4, 0)
        robot_grid.addWidget(self.upperlimb_version_value, 4, 1)
        robot_grid.addWidget(self.lowerlimb_version_label, 5, 0)
        robot_grid.addWidget(self.lowerlimb_version_value, 5, 1)
        robot_grid.addWidget(QLabel("机器人状态"), 6, 0)
        robot_grid.addWidget(self.robot_state_info_value, 6, 1)
        robot_grid.addWidget(QLabel("Orin 监控"), 7, 0)
        robot_grid.addWidget(self.robot_orin_status_value, 7, 1)
        robot_grid.addWidget(QLabel("Orin 网络延迟"), 8, 0)
        robot_grid.addWidget(self.robot_orin_latency_value, 8, 1)
        robot_grid.addWidget(QLabel("Pico 监控"), 9, 0)
        robot_grid.addWidget(self.robot_pico_status_value, 9, 1)
        robot_grid.addWidget(QLabel("Pico 网络延迟"), 10, 0)
        robot_grid.addWidget(self.robot_pico_latency_value, 10, 1)

        self.btn_refresh_robot_status = QPushButton("刷新状态")
        self.btn_refresh_robot_status.clicked.connect(self.refresh_robot_status)
        self.btn_check_version_status = QPushButton("检查版本")
        self.btn_check_version_status.clicked.connect(self.check_version)
        self.btn_show_system_info = QPushButton("系统信息窗口")
        self.btn_show_system_info.clicked.connect(self._show_system_info_window)
        self.btn_check_peripheral_status = QPushButton("检查外设")
        self.btn_check_peripheral_status.clicked.connect(self.check_peripherals)

        self.version_result_list = QPlainTextEdit()
        self.version_result_list.setReadOnly(True)
        self.peripheral_result_list = QPlainTextEdit()
        self.peripheral_result_list.setReadOnly(True)

        robot_layout.addLayout(robot_grid)
        robot_layout.addWidget(QLabel("版本检测结果"))
        robot_layout.addWidget(self.version_result_list)
        robot_layout.addWidget(QLabel("外设检测结果"))
        robot_layout.addWidget(self.peripheral_result_list)
        btn_line = QHBoxLayout()
        btn_line.addWidget(self.btn_refresh_robot_status)
        btn_line.addWidget(self.btn_check_version_status)
        btn_line.addWidget(self.btn_show_system_info)
        btn_line.addWidget(self.btn_check_peripheral_status)
        btn_line.addStretch(1)
        robot_layout.addLayout(btn_line)
        robot_layout.addStretch(1)
        tab_robot_status.setLayout(robot_layout)
        self.lowerlimb_version_label.setVisible(False)
        self.lowerlimb_version_value.setVisible(False)

        # ---------- Tab 2: 功能栏 ----------
        tab_transfer = QWidget()
        transfer_layout = QVBoxLayout()

        self.btn_upload = QPushButton("上传文件")
        self.btn_upload.clicked.connect(self.upload_file)

        self.btn_download = QPushButton("下载文件")
        self.btn_download.clicked.connect(self.download_file)

        self.btn_upload_dir = QPushButton("上传文件夹")
        self.btn_upload_dir.clicked.connect(self.upload_folder)

        self.btn_download_dir = QPushButton("下载文件夹")
        self.btn_download_dir.clicked.connect(self.download_folder)

        self.btn_restart_embedded = QPushButton("重启嵌入式")
        self.btn_restart_embedded.clicked.connect(self.restart_embedded_service)
        self.btn_restart_middleware = QPushButton("重启中间件")
        self.btn_restart_middleware.clicked.connect(self.restart_middleware_service)
        self.btn_pull_middleware_log = QPushButton("拉取运控日志")
        self.btn_pull_middleware_log.clicked.connect(self.pull_middleware_log)
        self.btn_pull_embedded_logs = QPushButton("拉取嵌入式日志")
        self.btn_pull_embedded_logs.clicked.connect(self.pull_embedded_logs)
        self.middleware_deploy_pkg_edit = QLineEdit()
        self.middleware_deploy_pkg_edit.setPlaceholderText("选择本地 Middleware 升级包，例如 Middleware_v1.3.0_xxx.firmware")
        self.middleware_deploy_robot_type_combo = QComboBox()
        self.middleware_deploy_robot_type_combo.setEditable(True)
        self.middleware_deploy_robot_type_combo.addItems(self._middleware_deploy_robot_type_options())
        self.middleware_deploy_robot_type_combo.setCurrentText(self._default_middleware_deploy_robot_type())
        self.middleware_deploy_big_checkbox = QCheckBox("大脑")
        self.middleware_deploy_big_checkbox.setChecked(True)
        self.middleware_deploy_small_checkbox = QCheckBox("小脑")
        self.middleware_deploy_small_checkbox.setChecked(True)
        self.middleware_deploy_force_checkbox = QCheckBox("强制重新安装(--force)")
        self.btn_browse_middleware_pkg = QPushButton("浏览升级包")
        self.btn_browse_middleware_pkg.clicked.connect(self.browse_middleware_package)
        self.btn_deploy_middleware_pkg = QPushButton("部署中间件升级包")
        self.btn_deploy_middleware_pkg.clicked.connect(self.deploy_middleware_package)
        self.mpc_deploy_pkg_edit = QLineEdit()
        self.mpc_deploy_pkg_edit.setPlaceholderText("选择本地 MPC 包，例如 mpc-delivery-install-v0.0.20.zip")
        self.mpc_deploy_remote_dir_edit = QLineEdit("/home/nav01/tele-workspace/tele-delivery")
        self.mpc_deploy_remote_dir_edit.setPlaceholderText("小脑部署目录")
        self.mpc_deploy_auto_extract_checkbox = QCheckBox("上传后自动解压")
        self.mpc_deploy_auto_extract_checkbox.setChecked(True)
        self.btn_browse_mpc_pkg = QPushButton("浏览MPC包")
        self.btn_browse_mpc_pkg.clicked.connect(self.browse_mpc_package)
        self.btn_deploy_mpc_pkg = QPushButton("部署MPC到小脑")
        self.btn_deploy_mpc_pkg.clicked.connect(self.deploy_mpc_package)
        self.mpc_image_pkg_edit = QLineEdit()
        self.mpc_image_pkg_edit.setPlaceholderText("选择镜像包，例如 tele-image-delivery-v1.tar")
        self.mpc_docker_offline_pkg_edit = QLineEdit()
        self.mpc_docker_offline_pkg_edit.setPlaceholderText("可选：手动选择离线 Docker 包，例如 docker-28.2.1.tgz")
        self.mpc_image_name_edit = QLineEdit("tele-image:delivery-v1")
        self.mpc_image_name_edit.setPlaceholderText("镜像名:tag")
        self.mpc_container_name_edit = QLineEdit("tele-delivery-container")
        self.mpc_container_name_edit.setPlaceholderText("容器名")
        self.mpc_ros_master_uri_edit = QLineEdit("http://192.168.217.1:11311")
        self.mpc_ros_ip_edit = QLineEdit("192.168.217.66")
        self.btn_browse_mpc_image_pkg = QPushButton("浏览镜像包")
        self.btn_browse_mpc_image_pkg.clicked.connect(self.browse_mpc_image_package)
        self.btn_browse_mpc_docker_offline_pkg = QPushButton("浏览离线Docker包")
        self.btn_browse_mpc_docker_offline_pkg.clicked.connect(self.browse_mpc_docker_offline_package)
        self.btn_load_mpc_image = QPushButton("导入MPC镜像")
        self.btn_load_mpc_image.clicked.connect(self.load_mpc_image_package)
        self.btn_prepare_mpc_container = QPushButton("创建/启动MPC容器")
        self.btn_prepare_mpc_container.clicked.connect(self.prepare_mpc_container)
        self.btn_deploy_mpc_all = QPushButton("MPC全部部署")
        self.btn_deploy_mpc_all.clicked.connect(self.deploy_mpc_all)
        self.btn_show_feature_config = QPushButton("功能配置")
        self.btn_show_feature_config.clicked.connect(self._show_feature_config_window)

        self.transfer_target = QComboBox()
        self.transfer_target.addItems(["小脑", "大脑"])
        self.transfer_target.currentTextChanged.connect(self._on_transfer_target_changed)

        target_line = QHBoxLayout()
        target_line.addWidget(QLabel("传输目标"))
        target_line.addWidget(self.transfer_target)
        target_line.addWidget(self.btn_show_feature_config)
        target_line.addStretch(1)

        line1 = QHBoxLayout()
        line1.addWidget(self.btn_upload)
        line1.addWidget(self.btn_download)

        line2 = QHBoxLayout()
        line2.addWidget(self.btn_upload_dir)
        line2.addWidget(self.btn_download_dir)

        line3 = QHBoxLayout()
        line3.addWidget(self.btn_restart_embedded)
        line3.addWidget(self.btn_restart_middleware)
        line3.addStretch(1)

        line4 = QHBoxLayout()
        line4.addWidget(self.btn_pull_middleware_log)
        line4.addWidget(self.btn_pull_embedded_logs)
        line4.addStretch(1)

        line5 = QHBoxLayout()
        self.middleware_deploy_pkg_label = QLabel("升级包")
        line5.addWidget(self.middleware_deploy_pkg_label)
        line5.addWidget(self.middleware_deploy_pkg_edit)
        self.middleware_deploy_robot_type_label = QLabel("robot_type")
        line5.addWidget(self.middleware_deploy_robot_type_label)
        line5.addWidget(self.middleware_deploy_robot_type_combo)
        self.middleware_deploy_target_label = QLabel("部署目标")
        line5.addWidget(self.middleware_deploy_target_label)
        line5.addWidget(self.middleware_deploy_big_checkbox)
        line5.addWidget(self.middleware_deploy_small_checkbox)
        line5.addWidget(self.middleware_deploy_force_checkbox)
        line5.addWidget(self.btn_browse_middleware_pkg)
        line5.addWidget(self.btn_deploy_middleware_pkg)

        line6 = QHBoxLayout()
        self.mpc_deploy_pkg_label = QLabel("MPC包")
        line6.addWidget(self.mpc_deploy_pkg_label)
        line6.addWidget(self.mpc_deploy_pkg_edit)
        self.mpc_deploy_remote_dir_label = QLabel("远端目录")
        line6.addWidget(self.mpc_deploy_remote_dir_label)
        line6.addWidget(self.mpc_deploy_remote_dir_edit)
        line6.addWidget(self.mpc_deploy_auto_extract_checkbox)
        line6.addWidget(self.btn_browse_mpc_pkg)
        line6.addWidget(self.btn_deploy_mpc_pkg)

        line7 = QHBoxLayout()
        self.mpc_image_pkg_label = QLabel("镜像包")
        line7.addWidget(self.mpc_image_pkg_label)
        line7.addWidget(self.mpc_image_pkg_edit)
        self.mpc_image_name_label = QLabel("镜像名")
        line7.addWidget(self.mpc_image_name_label)
        line7.addWidget(self.mpc_image_name_edit)
        self.mpc_container_name_label = QLabel("容器名")
        line7.addWidget(self.mpc_container_name_label)
        line7.addWidget(self.mpc_container_name_edit)
        line7.addWidget(self.btn_browse_mpc_image_pkg)
        line7.addWidget(self.btn_load_mpc_image)

        line8 = QHBoxLayout()
        self.mpc_ros_master_label = QLabel("ROS_MASTER_URI")
        line8.addWidget(self.mpc_ros_master_label)
        line8.addWidget(self.mpc_ros_master_uri_edit)
        self.mpc_ros_ip_label = QLabel("ROS_IP")
        line8.addWidget(self.mpc_ros_ip_label)
        line8.addWidget(self.mpc_ros_ip_edit)
        line8.addWidget(self.btn_prepare_mpc_container)
        line8.addWidget(self.btn_deploy_mpc_all)
        line8.addStretch(1)

        line9 = QHBoxLayout()
        self.mpc_docker_offline_pkg_label = QLabel("离线Docker包")
        line9.addWidget(self.mpc_docker_offline_pkg_label)
        line9.addWidget(self.mpc_docker_offline_pkg_edit)
        line9.addWidget(self.btn_browse_mpc_docker_offline_pkg)
        line9.addStretch(1)

        self.remote_edit_dir_input = QComboBox()
        self.remote_edit_dir_input.setEditable(True)
        self.remote_edit_dir_input.setMinimumWidth(300)
        self.remote_edit_dir_input.addItems(self._remote_edit_dir_history)
        self.remote_edit_dir_input.setCurrentText("/tmp")
        self.remote_edit_dir_input.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.remote_edit_dir_input.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContentsOnFirstShow)
        self.remote_edit_dir_input.setToolTip("目录可手动输入，也可从下拉中选择")
        self.remote_edit_dir_input.setPlaceholderText("远程目录，例如 /tmp")
        self.remote_edit_dir_input.currentTextChanged.connect(lambda _t: self.refresh_remote_edit_files())

        self.remote_edit_file_input = QComboBox()
        self.remote_edit_file_input.setEditable(True)
        self.remote_edit_file_input.setMinimumWidth(260)
        self.remote_edit_file_input.addItems(self._remote_edit_file_history)
        self.remote_edit_file_input.setCurrentText("test.txt")
        self.remote_edit_file_input.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.remote_edit_file_input.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContentsOnFirstShow)
        self.remote_edit_file_input.setToolTip("文件名可手动输入，也可从下拉中选择")
        self.remote_edit_file_input.setPlaceholderText("文件名，例如 test.txt")

        self.btn_remote_edit_refresh_dirs = QPushButton("刷新目录")
        self.btn_remote_edit_refresh_dirs.clicked.connect(self.refresh_remote_edit_directories)
        self.btn_remote_edit_enter_dir = QPushButton("进入目录")
        self.btn_remote_edit_enter_dir.clicked.connect(self.enter_remote_edit_directory)
        self.btn_remote_edit_parent_dir = QPushButton("上一级")
        self.btn_remote_edit_parent_dir.clicked.connect(self.go_remote_edit_parent_directory)
        self.btn_remote_edit_refresh_files = QPushButton("刷新文件")
        self.btn_remote_edit_refresh_files.clicked.connect(self.refresh_remote_edit_files)
        self.btn_remote_edit_pick_file = QPushButton("浏览选择...")
        self.btn_remote_edit_pick_file.clicked.connect(self.pick_remote_file_dialog)
        self.btn_remote_edit_open = QPushButton("打开远程文件")
        self.btn_remote_edit_open.clicked.connect(self.open_remote_file_for_edit)
        self.btn_remote_edit_reload = QPushButton("重新加载")
        self.btn_remote_edit_reload.clicked.connect(self.reload_remote_file_for_edit)
        self.btn_remote_edit_save = QPushButton("保存远程文件")
        self.btn_remote_edit_save.clicked.connect(self.save_remote_file_from_editor)
        self.btn_remote_edit_undo = QPushButton("撤销")
        self.btn_remote_edit_undo.clicked.connect(self.undo_remote_file_edit)

        edit_dir_line = QHBoxLayout()
        self.remote_edit_dir_label = QLabel("远程目录")
        edit_dir_line.addWidget(self.remote_edit_dir_label)
        edit_dir_line.addWidget(self.remote_edit_dir_input)
        edit_dir_line.addWidget(self.btn_remote_edit_enter_dir)
        edit_dir_line.addWidget(self.btn_remote_edit_parent_dir)
        edit_dir_line.addWidget(self.btn_remote_edit_refresh_dirs)

        edit_file_line = QHBoxLayout()
        self.remote_edit_file_label = QLabel("文件")
        edit_file_line.addWidget(self.remote_edit_file_label)
        edit_file_line.addWidget(self.remote_edit_file_input)
        edit_file_line.addWidget(self.btn_remote_edit_refresh_files)
        edit_file_line.addWidget(self.btn_remote_edit_pick_file)

        edit_btn_line = QHBoxLayout()
        edit_btn_line.addWidget(self.btn_remote_edit_open)
        edit_btn_line.addWidget(self.btn_remote_edit_reload)
        edit_btn_line.addWidget(self.btn_remote_edit_save)
        edit_btn_line.addWidget(self.btn_remote_edit_undo)
        edit_btn_line.addStretch(1)

        self.remote_find_input = QLineEdit()
        self.remote_find_input.setPlaceholderText("搜索文本")
        self.btn_remote_find_prev = QPushButton("上一个")
        self.btn_remote_find_prev.clicked.connect(lambda: self.remote_find_text(backward=True))
        self.btn_remote_find_next = QPushButton("下一个")
        self.btn_remote_find_next.clicked.connect(lambda: self.remote_find_text(backward=False))
        self.btn_remote_toggle_comment = QPushButton("注释/反注释")
        self.btn_remote_toggle_comment.clicked.connect(self.toggle_remote_comment_selection)

        edit_find_line = QHBoxLayout()
        self.remote_find_label = QLabel("查找")
        edit_find_line.addWidget(self.remote_find_label)
        edit_find_line.addWidget(self.remote_find_input)
        edit_find_line.addWidget(self.btn_remote_find_prev)
        edit_find_line.addWidget(self.btn_remote_find_next)
        edit_find_line.addWidget(self.btn_remote_toggle_comment)

        self.remote_edit_status = QLineEdit("未打开远程文件")
        self.remote_edit_status.setReadOnly(True)
        self.remote_file_editor = QPlainTextEdit()
        self.remote_file_editor.setPlaceholderText("打开远程文件后可直接编辑；支持撤销（Ctrl+Z）")
        self.remote_file_editor.setMinimumHeight(180)
        self.remote_file_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.remote_file_editor.document().setModified(False)
        self.remote_file_editor.undoAvailable.connect(self.btn_remote_edit_undo.setEnabled)
        self.remote_find_input.returnPressed.connect(lambda: self.remote_find_text(backward=False))
        self._remote_shortcut_find = QShortcut(QKeySequence("Ctrl+F"), self.remote_file_editor)
        self._remote_shortcut_find.activated.connect(self.focus_remote_find_input)
        self._remote_shortcut_find_next = QShortcut(QKeySequence("F3"), self.remote_file_editor)
        self._remote_shortcut_find_next.activated.connect(lambda: self.remote_find_text(backward=False))
        self._remote_shortcut_find_prev = QShortcut(QKeySequence("Shift+F3"), self.remote_file_editor)
        self._remote_shortcut_find_prev.activated.connect(lambda: self.remote_find_text(backward=True))
        self._remote_shortcut_toggle_comment = QShortcut(QKeySequence("Ctrl+/"), self.remote_file_editor)
        self._remote_shortcut_toggle_comment.activated.connect(self.toggle_remote_comment_selection)
        self.btn_remote_edit_save.setEnabled(False)
        self.btn_remote_edit_reload.setEnabled(False)
        self.btn_remote_edit_undo.setEnabled(False)
        QTimer.singleShot(0, self.refresh_remote_edit_directories)

        transfer_layout.addLayout(target_line)
        transfer_layout.addLayout(line1)
        transfer_layout.addLayout(line2)
        transfer_layout.addLayout(line3)
        transfer_layout.addLayout(line4)
        transfer_layout.addLayout(line5)
        # 隐藏 MPC 部署功能入口；底层逻辑暂时保留，后续如需恢复可重新挂回这四行布局。
        # transfer_layout.addLayout(line6)
        # transfer_layout.addLayout(line7)
        # transfer_layout.addLayout(line8)
        # transfer_layout.addLayout(line9)
        self.remote_edit_section_label = QLabel("远程文件编辑（小脑/大脑）")
        transfer_layout.addWidget(self.remote_edit_section_label)
        transfer_layout.addLayout(edit_dir_line)
        transfer_layout.addLayout(edit_file_line)
        transfer_layout.addLayout(edit_btn_line)
        transfer_layout.addLayout(edit_find_line)
        transfer_layout.addWidget(self.remote_edit_status)
        transfer_layout.addWidget(self.remote_file_editor)
        tab_transfer.setLayout(transfer_layout)

        # ---------- Tab 3: 关节监控 ----------
        tab_ros = QWidget()
        ros_layout = QVBoxLayout()

        self.joints_value = QLineEdit("-")
        self.joints_value.setReadOnly(True)
        self.btn_start_monitor = QPushButton("开始关节监控")
        self.btn_start_monitor.clicked.connect(self.start_monitor)
        self.btn_joint_record = QPushButton("开始录制")
        self.btn_joint_record.clicked.connect(self.toggle_joint_recording)
        self.btn_joint_record_stop = QPushButton("结束录制")
        self.btn_joint_record_stop.clicked.connect(self.stop_joint_recording)
        self.btn_joint_record_stop.setEnabled(False)
        self.btn_joint_plot = QPushButton("绘图")
        self.btn_joint_plot.clicked.connect(self.plot_recorded_joint_state)
        self.joint_record_status_value = QLineEdit("未录制")
        self.joint_record_status_value.setReadOnly(True)
        self.robot_model_combo = QComboBox()
        self.robot_model_combo.addItems(["WA2", "WA2_LS", "WA1", "I2"])
        self.robot_model_combo.setCurrentText(self.robot_model)
        self.robot_model_combo.currentTextChanged.connect(self._set_robot_model)

        monitor_header = QHBoxLayout()
        monitor_header.addWidget(self.btn_start_monitor)
        monitor_header.addWidget(self.btn_joint_record)
        monitor_header.addWidget(self.btn_joint_record_stop)
        monitor_header.addWidget(self.btn_joint_plot)
        monitor_header.addWidget(QLabel("型号"))
        monitor_header.addWidget(self.robot_model_combo)
        monitor_header.addWidget(QLabel("录制状态"))
        monitor_header.addWidget(self.joint_record_status_value)
        monitor_header.addStretch(1)

        monitor_status_line = QHBoxLayout()
        self.joint_monitor_status_value = QLineEdit("未启动")
        self.joint_monitor_status_value.setReadOnly(True)
        self.joint_monitor_last_update_value = QLineEdit("-")
        self.joint_monitor_last_update_value.setReadOnly(True)
        self.robot_status_health_value = QLineEdit("未启动")
        self.robot_status_health_value.setReadOnly(True)
        monitor_status_line.addWidget(QLabel("关节监听"))
        monitor_status_line.addWidget(self.joint_monitor_status_value)
        monitor_status_line.addWidget(QLabel("最后更新"))
        monitor_status_line.addWidget(self.joint_monitor_last_update_value)
        monitor_status_line.addWidget(QLabel("robot状态监听"))
        monitor_status_line.addWidget(self.robot_status_health_value)
        monitor_status_line.addStretch(1)

        monitor_view_line = QHBoxLayout()
        self.joint_monitor_view_combo = QComboBox()
        self.joint_monitor_view_combo.addItem("关节角度")
        self.joint_monitor_view_combo.addItem("tcp速度")
        self.joint_monitor_view_combo.addItem("tcp位置")
        monitor_view_line.addWidget(QLabel("显示内容"))
        monitor_view_line.addWidget(self.joint_monitor_view_combo)
        monitor_view_line.addStretch(1)

        self.joint_names = self._get_joint_names_by_model(self.robot_model)
        self.joint_fields = {}

        self.joint_grid_layout = QGridLayout()
        self._rebuild_joint_grid()

        joint_position_panel = QWidget()
        joint_position_layout = QVBoxLayout(joint_position_panel)
        joint_position_layout.setContentsMargins(0, 0, 0, 0)
        joint_position_layout.setSpacing(6)
        joint_position_layout.addWidget(QLabel("关节位置 (/zj_humanoid/upperlimb/joint_states)"))
        joint_position_layout.addLayout(self.joint_grid_layout)

        tcp_pose_panel = QWidget()
        tcp_pose_layout = QVBoxLayout(tcp_pose_panel)
        tcp_pose_layout.setContentsMargins(0, 0, 0, 0)
        tcp_pose_layout.setSpacing(6)
        tcp_pose_layout.addWidget(QLabel("左臂末端位置 (/zj_humanoid/upperlimb/tcp_pose/left_arm)"))
        self.tcp_pose_left_value = QPlainTextEdit()
        self.tcp_pose_left_value.setReadOnly(True)
        self.tcp_pose_left_value.setPlaceholderText("等待左臂末端位姿数据...")
        tcp_pose_layout.addWidget(self.tcp_pose_left_value)
        tcp_pose_layout.addWidget(QLabel("右臂末端位置 (/zj_humanoid/upperlimb/tcp_pose/right_arm)"))
        self.tcp_pose_right_value = QPlainTextEdit()
        self.tcp_pose_right_value.setReadOnly(True)
        self.tcp_pose_right_value.setPlaceholderText("等待右臂末端位姿数据...")
        tcp_pose_layout.addWidget(self.tcp_pose_right_value)

        tcp_speed_panel = QWidget()
        tcp_speed_layout = QVBoxLayout(tcp_speed_panel)
        tcp_speed_layout.setContentsMargins(0, 0, 0, 0)
        tcp_speed_layout.setSpacing(6)
        tcp_speed_layout.addWidget(QLabel("末端速度 (/zj_humanoid/upperlimb/tcp_speed/dual_arm)"))
        self.tcp_speed_value = QPlainTextEdit()
        self.tcp_speed_value.setReadOnly(True)
        self.tcp_speed_value.setPlaceholderText("等待双臂 TCP 速度数据...")
        tcp_speed_layout.addWidget(self.tcp_speed_value)

        self.joint_monitor_stack = QStackedWidget()
        self.joint_monitor_stack.addWidget(joint_position_panel)
        self.joint_monitor_stack.addWidget(tcp_speed_panel)
        self.joint_monitor_stack.addWidget(tcp_pose_panel)
        self.joint_monitor_view_combo.currentIndexChanged.connect(self.joint_monitor_stack.setCurrentIndex)

        ros_layout.addLayout(monitor_header)
        ros_layout.addLayout(monitor_status_line)
        ros_layout.addLayout(monitor_view_line)
        ros_layout.addWidget(self.joint_monitor_stack)
        tab_ros.setLayout(ros_layout)
        self._reset_tcp_monitor_views()

        # ---------- Tab 4: 关节控制 ----------
        tab_joint_ctrl = QWidget()
        joint_ctrl_layout = QVBoxLayout()

        self.joint_ctrl_part = QComboBox()
        self._rebuild_joint_ctrl_parts()

        self.btn_teach_enter = QPushButton("进入示教模式")
        self.btn_teach_enter.clicked.connect(self.enter_teach_mode)
        self.btn_teach_exit = QPushButton("退出示教模式")
        self.btn_teach_exit.clicked.connect(self.exit_teach_mode)
        self.btn_go_home = QPushButton("归位")
        self.btn_go_home.clicked.connect(self.go_home)
        self.btn_unlock_upperlimb = QPushButton("解除保护")
        self.btn_unlock_upperlimb.clicked.connect(self.unlock_upperlimb)
        self.btn_joint_ctrl_ros = QPushButton("ROSBridge")
        self.btn_joint_ctrl_ros.setCheckable(True)
        self.btn_joint_ctrl_ros.clicked.connect(lambda: self._set_joint_ctrl_transport_mode("ros"))
        self.btn_joint_ctrl_ssh = QPushButton("SSH")
        self.btn_joint_ctrl_ssh.setCheckable(True)
        self.btn_joint_ctrl_ssh.clicked.connect(lambda: self._set_joint_ctrl_transport_mode("ssh"))
        self._set_joint_ctrl_transport_mode(self._joint_ctrl_transport_mode)
        self.robot_model_ctrl_combo = QComboBox()
        self.robot_model_ctrl_combo.addItems(["WA2", "WA2_LS", "WA1", "I2"])
        self.robot_model_ctrl_combo.setCurrentText(self.robot_model)
        self.robot_model_ctrl_combo.currentTextChanged.connect(self._set_robot_model)

        teach_line = QHBoxLayout()
        teach_line.addWidget(QLabel("型号"))
        teach_line.addWidget(self.robot_model_ctrl_combo)
        teach_line.addWidget(QLabel("通道"))
        teach_line.addWidget(self.btn_joint_ctrl_ros)
        teach_line.addWidget(self.btn_joint_ctrl_ssh)
        teach_line.addWidget(QLabel("部位"))
        teach_line.addWidget(self.joint_ctrl_part)
        teach_line.addWidget(self.btn_teach_enter)
        teach_line.addWidget(self.btn_teach_exit)
        teach_line.addWidget(self.btn_go_home)
        teach_line.addWidget(self.btn_unlock_upperlimb)
        self.safety_lock_status_label = QLabel("安全保护: -")
        self.safety_lock_status_label.setStyleSheet("color: #c9d1d9; font-weight: 600;")
        teach_line.addWidget(self.safety_lock_status_label)
        teach_line.addStretch(1)

        movej_test_header = QHBoxLayout()
        self.movej_test_part = QComboBox()
        self.movej_test_part.setMinimumWidth(120)
        self.movej_test_part.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.movej_test_part.currentIndexChanged.connect(self._rebuild_movej_test_joint_inputs)
        self.btn_movej_test_fill = QPushButton("读取当前姿态")
        self.btn_movej_test_fill.clicked.connect(self._fill_movej_test_from_current_pose)
        self.btn_movej_test_clear = QPushButton("清空目标")
        self.btn_movej_test_clear.clicked.connect(lambda: self.movej_test_list_edit.clear())
        self.btn_movej_test_send = QPushButton("执行MoveJ测试")
        self.btn_movej_test_send.clicked.connect(self.run_movej_test)

        movej_test_header.addWidget(QLabel("部位"))
        movej_test_header.addWidget(self.movej_test_part)
        movej_test_header.addSpacing(8)
        movej_test_header.addWidget(self.btn_movej_test_fill)
        movej_test_header.addWidget(self.btn_movej_test_clear)
        movej_test_header.addWidget(self.btn_movej_test_send)
        movej_test_header.addStretch(1)

        self.movej_test_list_edit = QPlainTextEdit()
        self.movej_test_list_edit.setFixedHeight(56)
        self.movej_test_list_edit.setPlaceholderText("直接输入数组，例如: [0.0, 0.0]")

        movej_test_panel = QWidget()
        movej_test_panel_layout = QVBoxLayout(movej_test_panel)
        movej_test_panel_layout.setContentsMargins(8, 8, 8, 8)
        movej_test_panel_layout.setSpacing(6)
        movej_test_panel_layout.addLayout(movej_test_header)
        movej_test_panel_layout.addWidget(QLabel("目标数组 []（仅支持此输入方式）"))
        movej_test_panel_layout.addWidget(self.movej_test_list_edit)
        movej_test_panel.setStyleSheet("QWidget { border: 1px solid #30363d; border-radius: 6px; }")
        self._rebuild_movej_test_parts()

        movel_test_header = QHBoxLayout()
        self.movel_test_arm_combo = QComboBox()
        self.movel_test_arm_combo.addItem("左臂", "left")
        self.movel_test_arm_combo.addItem("右臂", "right")
        self.movel_test_direction_combo = QComboBox()
        self.movel_test_direction_combo.addItem("前 (x+)", ("x", 1.0))
        self.movel_test_direction_combo.addItem("后 (x-)", ("x", -1.0))
        self.movel_test_direction_combo.addItem("左 (y+)", ("y", 1.0))
        self.movel_test_direction_combo.addItem("右 (y-)", ("y", -1.0))
        self.movel_test_direction_combo.addItem("上 (z+)", ("z", 1.0))
        self.movel_test_direction_combo.addItem("下 (z-)", ("z", -1.0))
        self.movel_test_distance_edit = QLineEdit("0.05")
        self.movel_test_distance_edit.setFixedWidth(80)
        self.btn_movel_test_fill = QPushButton("读取当前TCP")
        self.btn_movel_test_fill.clicked.connect(self._fill_movel_test_from_current_pose)
        self.btn_movel_test_send = QPushButton("执行MoveL测试")
        self.btn_movel_test_send.clicked.connect(self.run_movel_test)

        movel_test_header.addWidget(QLabel("手臂"))
        movel_test_header.addWidget(self.movel_test_arm_combo)
        movel_test_header.addWidget(QLabel("方向"))
        movel_test_header.addWidget(self.movel_test_direction_combo)
        movel_test_header.addWidget(QLabel("距离(m)"))
        movel_test_header.addWidget(self.movel_test_distance_edit)
        movel_test_header.addWidget(self.btn_movel_test_fill)
        movel_test_header.addWidget(self.btn_movel_test_send)
        movel_test_header.addStretch(1)

        self.movel_test_current_xyz = QLineEdit()
        self.movel_test_current_xyz.setReadOnly(True)
        self.movel_test_current_xyz.setPlaceholderText("当前TCP位置: x, y, z")
        self.movel_test_target_xyz = QLineEdit()
        self.movel_test_target_xyz.setReadOnly(True)
        self.movel_test_target_xyz.setPlaceholderText("目标TCP位置: x, y, z")

        movel_test_panel = QWidget()
        movel_test_panel_layout = QVBoxLayout(movel_test_panel)
        movel_test_panel_layout.setContentsMargins(8, 8, 8, 8)
        movel_test_panel_layout.setSpacing(6)
        movel_test_panel_layout.addLayout(movel_test_header)
        movel_test_panel_layout.addWidget(QLabel("当前TCP position [x, y, z] (m)"))
        movel_test_panel_layout.addWidget(self.movel_test_current_xyz)
        movel_test_panel_layout.addWidget(QLabel("目标TCP position [x, y, z] (m)"))
        movel_test_panel_layout.addWidget(self.movel_test_target_xyz)
        movel_test_panel.setStyleSheet("QWidget { border: 1px solid #30363d; border-radius: 6px; }")

        self._movel_test_current_pose = None
        self.movel_test_arm_combo.currentIndexChanged.connect(self._reset_movel_test_preview)
        self.movel_test_direction_combo.currentIndexChanged.connect(self._update_movel_test_target_preview)
        self.movel_test_distance_edit.textChanged.connect(self._update_movel_test_target_preview)

        self.motion_seq_selector = QComboBox()
        self.motion_seq_selector.setMinimumWidth(120)
        self.motion_seq_selector.currentIndexChanged.connect(self._on_motion_sequence_changed)
        self.btn_add_sequence = QPushButton("新增序列")
        self.btn_add_sequence.clicked.connect(self.add_motion_sequence)
        self.btn_rename_sequence = QPushButton("重命名序列")
        self.btn_rename_sequence.clicked.connect(self.rename_current_motion_sequence)
        self.btn_delete_sequence = QPushButton("删除序列")
        self.btn_delete_sequence.clicked.connect(self.delete_current_motion_sequence)
        self.btn_edit_motion_seq = QPushButton("编辑动作序列")
        self.btn_edit_motion_seq.setMinimumWidth(110)
        self.btn_edit_motion_seq.clicked.connect(self.start_motion_edit)
        self.btn_add_motion_row = QPushButton("添加轨迹")
        self.btn_add_motion_row.clicked.connect(self.add_motion_row)
        self.btn_delete_motion_row = QPushButton("删除轨迹")
        self.btn_delete_motion_row.clicked.connect(self.delete_last_motion_row)
        self.btn_finish_motion_seq = QPushButton("完成编辑")
        self.btn_finish_motion_seq.clicked.connect(self.finish_motion_edit)
        self.btn_exec_motion_path = QPushButton("执行动作序列")
        self.btn_exec_motion_path.clicked.connect(self.execute_motion_path)
        self.btn_joint_ctrl_plot = QPushButton("控制Plot")
        self.btn_joint_ctrl_plot.clicked.connect(self.plot_joint_control_trace)
        self.motion_loop_count_edit = QLineEdit("1")
        self.motion_loop_count_edit.setFixedWidth(60)
        self.motion_loop_count_edit.setPlaceholderText("循环")
        self.btn_stop_motion_path = QPushButton("打断执行")
        self.btn_stop_motion_path.clicked.connect(self.stop_motion_path)
        self.btn_stop_motion_path.setEnabled(False)
        self.btn_export_all_sequences = QPushButton("导出序列")
        self.btn_export_all_sequences.setMinimumWidth(120)
        self.btn_export_all_sequences.clicked.connect(self.export_motion_sequences)
        self.btn_import_all_sequences = QPushButton("加载序列")
        self.btn_import_all_sequences.setMinimumWidth(96)
        self.btn_import_all_sequences.clicked.connect(self.import_motion_sequences)
        self.btn_export_motion_seq = QPushButton("导出dance-yaml")
        self.btn_export_motion_seq.clicked.connect(self.export_motion_sequence)
        self.btn_export_motion_py = QPushButton("导出Python")
        self.btn_export_motion_py.setMinimumWidth(96)
        self.btn_export_motion_py.clicked.connect(self.export_motion_sequence_python)

        seq_header = QVBoxLayout()
        seq_header_top = QHBoxLayout()
        seq_header_top.addWidget(QLabel("序列"))
        seq_header_top.addWidget(self.motion_seq_selector)
        seq_header_top.addWidget(self.btn_add_sequence)
        seq_header_top.addWidget(self.btn_rename_sequence)
        seq_header_top.addWidget(self.btn_delete_sequence)
        seq_header_top.addWidget(self.btn_edit_motion_seq)
        seq_header_top.addWidget(self.btn_add_motion_row)
        seq_header_top.addWidget(self.btn_delete_motion_row)
        seq_header_top.addWidget(self.btn_finish_motion_seq)
        seq_header_top.addStretch(1)

        seq_header_bottom = QHBoxLayout()
        seq_header_bottom.addWidget(QLabel("循环次数"))
        seq_header_bottom.addWidget(self.motion_loop_count_edit)
        seq_header_bottom.addWidget(self.btn_exec_motion_path)
        seq_header_bottom.addWidget(self.btn_stop_motion_path)
        seq_header_bottom.addWidget(self.btn_joint_ctrl_plot)
        seq_header_bottom.addWidget(self.btn_import_all_sequences)
        seq_header_bottom.addWidget(self.btn_export_motion_seq)
        seq_header_bottom.addWidget(self.btn_export_all_sequences)
        seq_header_bottom.addWidget(self.btn_export_motion_py)
        seq_header_bottom.addStretch(1)

        self.motion_model_label = QLabel("当前序列型号: -")
        self.motion_model_label.setStyleSheet("color: #58a6ff; font-weight: 700;")
        seq_header_meta = QHBoxLayout()
        seq_header_meta.addWidget(self.motion_model_label)
        seq_header_meta.addStretch(1)

        seq_header.addLayout(seq_header_top)
        seq_header.addLayout(seq_header_bottom)
        seq_header.addLayout(seq_header_meta)

        self.motion_seq_container = QWidget()
        self.motion_seq_layout = QVBoxLayout(self.motion_seq_container)
        self.motion_seq_layout.setContentsMargins(0, 0, 0, 0)
        self.motion_seq_layout.setSpacing(6)
        self.motion_seq_layout.addStretch(1)

        self.motion_seq_scroll = QScrollArea()
        self.motion_seq_scroll.setWidgetResizable(True)
        self.motion_seq_scroll.setWidget(self.motion_seq_container)
        self.motion_seq_scroll.setMinimumHeight(220)

        motion_seq_panel = QWidget()
        motion_seq_panel_layout = QVBoxLayout(motion_seq_panel)
        motion_seq_panel_layout.setContentsMargins(0, 0, 0, 0)
        motion_seq_panel_layout.setSpacing(6)
        motion_seq_panel_layout.addLayout(seq_header)
        motion_seq_panel_layout.addWidget(self.motion_seq_scroll)

        movej_section, self.btn_toggle_movej_test_section = self._build_collapsible_section(
            "MoveJ测试", movej_test_panel, expanded=True
        )
        movel_section, self.btn_toggle_movel_test_section = self._build_collapsible_section(
            "MoveL测试", movel_test_panel, expanded=True
        )
        motion_seq_section, self.btn_toggle_motion_seq_section = self._build_collapsible_section(
            "序列动作", motion_seq_panel, expanded=True
        )

        joint_ctrl_layout.addLayout(teach_line)
        joint_ctrl_layout.addWidget(movej_section)
        joint_ctrl_layout.addWidget(motion_seq_section)
        joint_ctrl_layout.addStretch(1)
        tab_joint_ctrl.setLayout(joint_ctrl_layout)
        self._set_motion_editing(False)
        self._restore_motion_sequences_from_disk()
        self._update_motion_model_label()

        # ---------- Tab 5: 导航 ----------
        tab_nav = QWidget()
        nav_layout = QVBoxLayout()

        self.nav_compose_service = QLineEdit("wa_perception")
        self.nav_compose_service.setPlaceholderText("docker-compose 服务名")
        self.nav_map_name = QLineEdit("kaiao3")
        self.nav_z_floor = QLineEdit("0.1")
        self.nav_z_ceil = QLineEdit("2.0")
        self.nav_scene = QLineEdit("0")
        self.nav_post_method = QLineEdit("0")

        row0 = QHBoxLayout()
        row0.addWidget(QLabel("Compose服务"))
        row0.addWidget(self.nav_compose_service)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("map_name"))
        row1.addWidget(self.nav_map_name)
        row1.addWidget(QLabel("z_floor"))
        row1.addWidget(self.nav_z_floor)
        row1.addWidget(QLabel("z_ceil"))
        row1.addWidget(self.nav_z_ceil)
        row1.addWidget(QLabel("scene"))
        row1.addWidget(self.nav_scene)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("post method"))
        row2.addWidget(self.nav_post_method)
        row2.addStretch(1)

        self.btn_nav_start_mapping = QPushButton("开始建图")
        self.btn_nav_start_mapping.clicked.connect(self.start_mapping)
        self.btn_nav_post_process = QPushButton("建图后处理")
        self.btn_nav_post_process.clicked.connect(self.post_process_mapping)
        self.btn_nav_restart_perception = QPushButton("重启感知容器")
        self.btn_nav_restart_perception.clicked.connect(self.restart_perception_container)

        row3 = QHBoxLayout()
        row3.addWidget(self.btn_nav_start_mapping)
        row3.addWidget(self.btn_nav_post_process)
        row3.addWidget(self.btn_nav_restart_perception)
        row3.addStretch(1)

        nav_layout.addLayout(row0)
        nav_layout.addLayout(row1)
        nav_layout.addLayout(row2)
        nav_layout.addLayout(row3)
        nav_layout.addStretch(1)
        tab_nav.setLayout(nav_layout)

        # ---------- Tab 6: 容器操作 ----------
        tab_container = QWidget()
        container_layout = QVBoxLayout()

        self.container_services = ["mani", "mani22", "sen22", "sen20", "navbrain", "nviz", "envi", "ros", "demos"]

        self.container_service_combo = QComboBox()
        self.container_service_combo.addItems(self.container_services)
        self.container_version_edit = QLineEdit("latest")
        self.container_logs_lines_edit = QLineEdit("100")
        self.container_logs_lines_edit.setFixedWidth(80)

        row_cfg = QHBoxLayout()
        row_cfg.addWidget(QLabel("服务"))
        row_cfg.addWidget(self.container_service_combo)
        row_cfg.addWidget(QLabel("版本"))
        row_cfg.addWidget(self.container_version_edit)
        row_cfg.addWidget(QLabel("日志行数"))
        row_cfg.addWidget(self.container_logs_lines_edit)
        row_cfg.addStretch(1)

        self.btn_container_status = QPushButton("状态")
        self.btn_container_status.clicked.connect(self.container_status)
        self.btn_container_images = QPushButton("镜像")
        self.btn_container_images.clicked.connect(self.container_images)
        self.btn_container_deploy = QPushButton("部署服务")
        self.btn_container_deploy.clicked.connect(self.container_deploy)
        self.btn_container_deploy_all = QPushButton("部署全部")
        self.btn_container_deploy_all.clicked.connect(self.container_deploy_all)
        self.btn_container_stop = QPushButton("停止服务")
        self.btn_container_stop.clicked.connect(self.container_stop)
        self.btn_container_logs = QPushButton("查看日志")
        self.btn_container_logs.clicked.connect(self.container_logs)
        self.btn_container_rollback = QPushButton("回滚服务")
        self.btn_container_rollback.clicked.connect(self.container_rollback)
        self.btn_container_help = QPushButton("帮助")
        self.btn_container_help.clicked.connect(self.container_help)
        self.btn_container_clear = QPushButton("清空输出")
        self.btn_container_clear.clicked.connect(self.container_clear_output)

        self.container_runtime_combo = QComboBox()
        self.container_runtime_combo.currentTextChanged.connect(self._on_container_runtime_selection_changed)
        self.btn_container_runtime_refresh = QPushButton("读取 docker ps -a")
        self.btn_container_runtime_refresh.clicked.connect(self.refresh_container_runtime_list)
        self.btn_container_runtime_restart = QPushButton("重启选中容器")
        self.btn_container_runtime_restart.clicked.connect(self.restart_selected_container)
        self.btn_container_runtime_logs = QPushButton("查看选中日志")
        self.btn_container_runtime_logs.clicked.connect(self.show_selected_container_logs)
        self.container_runtime_logs_lines_edit = QLineEdit("200")
        self.container_runtime_logs_lines_edit.setFixedWidth(80)
        self.container_runtime_info = QLineEdit("未读取容器")
        self.container_runtime_info.setReadOnly(True)
        self.container_runtime_path_edit = QLineEdit("/tmp/test.txt")
        self.container_runtime_path_edit.setPlaceholderText("容器内文件路径，例如 /tmp/test.txt")
        self.btn_container_file_upload = QPushButton("上传到容器")
        self.btn_container_file_upload.clicked.connect(self.upload_file_to_container)
        self.btn_container_file_download = QPushButton("从容器下载")
        self.btn_container_file_download.clicked.connect(self.download_file_from_container)
        self.btn_container_dir_upload = QPushButton("上传文件夹")
        self.btn_container_dir_upload.clicked.connect(self.upload_folder_to_container)
        self.btn_container_dir_download = QPushButton("下载文件夹")
        self.btn_container_dir_download.clicked.connect(self.download_folder_from_container)
        self.btn_container_file_browse = QPushButton("浏览容器路径")
        self.btn_container_file_browse.clicked.connect(self.browse_container_runtime_path)
        self.btn_container_file_open = QPushButton("打开容器文件")
        self.btn_container_file_open.clicked.connect(self.open_container_file_for_edit)
        self.btn_container_file_save = QPushButton("保存到容器")
        self.btn_container_file_save.clicked.connect(self.save_container_file_from_editor)
        self.btn_container_file_save.setEnabled(False)
        self.container_file_status = QLineEdit("未打开容器文件")
        self.container_file_status.setReadOnly(True)
        self.container_file_editor = QPlainTextEdit()
        self.container_file_editor.setPlaceholderText("打开容器内文本文件后可直接编辑")
        self.container_file_editor.setMinimumHeight(180)
        self.container_file_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.container_file_editor.document().setModified(False)

        row_runtime1 = QHBoxLayout()
        row_runtime1.addWidget(QLabel("大脑容器"))
        row_runtime1.addWidget(self.container_runtime_combo)
        row_runtime1.addWidget(self.btn_container_runtime_refresh)
        row_runtime1.addWidget(self.btn_container_runtime_restart)
        row_runtime1.addWidget(QLabel("日志行数"))
        row_runtime1.addWidget(self.container_runtime_logs_lines_edit)
        row_runtime1.addWidget(self.btn_container_runtime_logs)
        row_runtime1.addStretch(1)

        row_runtime2 = QHBoxLayout()
        row_runtime2.addWidget(QLabel("容器文件"))
        row_runtime2.addWidget(self.container_runtime_path_edit)
        row_runtime2.addWidget(self.btn_container_file_upload)
        row_runtime2.addWidget(self.btn_container_file_download)
        row_runtime2.addWidget(self.btn_container_dir_upload)
        row_runtime2.addWidget(self.btn_container_dir_download)
        row_runtime2.addWidget(self.btn_container_file_browse)
        row_runtime2.addWidget(self.btn_container_file_open)
        row_runtime2.addWidget(self.btn_container_file_save)
        row_runtime2.addStretch(1)

        self.container_output = QPlainTextEdit()
        self.container_output.setReadOnly(True)
        container_layout.addWidget(QLabel("大脑 Docker 容器（自动读取 docker ps -a）"))
        container_layout.addLayout(row_runtime1)
        container_layout.addWidget(self.container_runtime_info)
        container_layout.addLayout(row_runtime2)
        container_layout.addWidget(self.container_file_status)
        container_layout.addWidget(self.container_file_editor)
        container_layout.addWidget(QLabel("容器操作输出"))
        container_layout.addWidget(self.container_output)
        tab_container.setLayout(container_layout)

        # ---------- Tab 7: 测试Demo ----------
        tab_humanode_demo = QWidget()
        humanode_demo_layout = QVBoxLayout()

        self.btn_humanode_demo_run = QPushButton("运行 Humanode Demo")
        self.btn_humanode_demo_run.clicked.connect(self.run_humanode_demo)
        self.btn_humanode_demo_refresh_cases = QPushButton("刷新测试用例")
        self.btn_humanode_demo_refresh_cases.clicked.connect(self.refresh_humanode_demo_case_buttons)
        self.btn_humanode_demo_clear = QPushButton("清空输出")
        self.btn_humanode_demo_clear.clicked.connect(lambda: self.humanode_demo_clear_signal.emit())
        self.humanode_demo_local_dir_edit = QLineEdit()
        self.humanode_demo_local_dir_edit.setPlaceholderText("选择本地 Humanode_Naviai-main 目录")
        self.humanode_demo_local_dir_edit.setText(self._local_humanode_demo_dir())
        self.btn_humanode_demo_pick_dir = QPushButton("浏览本地目录")
        self.btn_humanode_demo_pick_dir.clicked.connect(self.pick_humanode_demo_local_dir)
        self.humanode_demo_remote_dir_edit = QLineEdit("test-tools/Humanode")
        self.humanode_demo_remote_dir_edit.setPlaceholderText("远端目录（支持绝对路径，或相对 /home/<user>/）")
        self.humanode_demo_remote_dir_edit.textChanged.connect(
            lambda text: self._sync_humanode_remote_dir_inputs(text, source="humanode")
        )

        humanode_demo_desc = QLabel(
            "执行流程：按“远端目录”定位到目标工程并运行 docker/run1.sh。"
            "远端目录支持手动填写，默认 test-tools/Humanode（会自动按 /home/<大脑用户>/ 解析）；测试用例默认从远端 utils 目录读取。\n"
            "运行流程：执行 docker/run1.sh，随后自动检查容器内是否存在 MotionExecuteAction，缺失时自动在 /shared 下安装中间件。"
        )
        humanode_demo_desc.setWordWrap(True)

        row_humanode_demo_remote_dir = QHBoxLayout()
        row_humanode_demo_remote_dir.addWidget(QLabel("远端目录"))
        row_humanode_demo_remote_dir.addWidget(self.humanode_demo_remote_dir_edit)
        row_humanode_demo_remote_dir.addStretch(1)

        row_humanode_demo = QHBoxLayout()
        row_humanode_demo.addWidget(self.btn_humanode_demo_run)
        row_humanode_demo.addWidget(self.btn_humanode_demo_refresh_cases)
        row_humanode_demo.addWidget(self.btn_humanode_demo_clear)
        row_humanode_demo.addStretch(1)

        self.humanode_demo_case_container = QWidget()
        self.humanode_demo_case_layout = QVBoxLayout(self.humanode_demo_case_container)
        self.humanode_demo_case_layout.setContentsMargins(0, 0, 0, 0)
        self.humanode_demo_case_layout.setSpacing(6)
        self.humanode_demo_case_layout.addStretch(1)

        self.humanode_demo_case_scroll = QScrollArea()
        self.humanode_demo_case_scroll.setWidgetResizable(True)
        self.humanode_demo_case_scroll.setWidget(self.humanode_demo_case_container)
        self.humanode_demo_case_scroll.setMinimumHeight(260)

        self.humanode_demo_output = QPlainTextEdit()
        self.humanode_demo_output.setReadOnly(True)

        humanode_demo_layout.addWidget(humanode_demo_desc)
        humanode_demo_layout.addLayout(row_humanode_demo_remote_dir)
        humanode_demo_layout.addLayout(row_humanode_demo)
        humanode_demo_layout.addWidget(QLabel("utils 测试用例（按型号）"))
        humanode_demo_layout.addWidget(self.humanode_demo_case_scroll)
        humanode_demo_layout.addWidget(QLabel("测试 Demo 输出"))
        humanode_demo_layout.addWidget(self.humanode_demo_output)
        tab_humanode_demo.setLayout(humanode_demo_layout)
        self.refresh_humanode_demo_case_buttons()

        # ---------- Tab 8: 工厂测试 ----------
        tab_factory_test = QWidget()
        factory_test_tab_layout = QVBoxLayout(tab_factory_test)
        factory_test_tab_layout.setContentsMargins(0, 0, 0, 0)

        factory_test_scroll = QScrollArea()
        factory_test_scroll.setWidgetResizable(True)
        factory_test_scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        factory_test_panel = QWidget()
        factory_test_layout = QVBoxLayout(factory_test_panel)

        factory_test_desc = QLabel(
            "工厂测试流程：按步骤执行并点击“完成本项”，全部完成后生成测试报告。"
        )
        factory_test_desc.setWordWrap(True)

        self.factory_flow_status = QLineEdit("未开始")
        self.factory_flow_status.setReadOnly(True)
        self.factory_container_status = QLineEdit("未启动")
        self.factory_container_status.setReadOnly(True)
        self.factory_remote_demo_dir_edit = QLineEdit("test-tools/Humanode")
        self.factory_remote_demo_dir_edit.setPlaceholderText("远端目录（同测试Demo）")
        self.factory_remote_demo_dir_edit.textChanged.connect(
            lambda text: self._sync_humanode_remote_dir_inputs(text, source="factory")
        )
        self.factory_robot_sn_edit = QLineEdit()
        self.factory_robot_sn_edit.setPlaceholderText("手动输入机器人SN码")
        self.btn_factory_flow_start = QPushButton("开始/重置流程")
        self.btn_factory_flow_start.clicked.connect(self.factory_flow_reset)
        self.btn_factory_flow_report = QPushButton("生成测试报告")
        self.btn_factory_flow_report.clicked.connect(self.factory_generate_report)

        self.btn_factory_test_run_container = QPushButton("运行工厂测试容器")
        self.btn_factory_test_run_container.clicked.connect(self.run_factory_test_container)
        self.factory_info_model_value = QLineEdit("-")
        self.factory_info_model_value.setReadOnly(True)
        self.factory_info_embedded_value = QLineEdit("-")
        self.factory_info_embedded_value.setReadOnly(True)
        self.factory_info_middleware_value = QLineEdit("-")
        self.factory_info_middleware_value.setReadOnly(True)
        self.factory_info_upperlimb_value = QLineEdit("-")
        self.factory_info_upperlimb_value.setReadOnly(True)
        self.factory_info_lowerlimb_label = QLabel("下肢版本")
        self.factory_info_lowerlimb_value = QLineEdit("-")
        self.factory_info_lowerlimb_value.setReadOnly(True)
        self.factory_info_lowerlimb_label.setVisible(False)
        self.factory_info_lowerlimb_value.setVisible(False)
        self.factory_info_system_text = QPlainTextEdit()
        self.factory_info_system_text.setReadOnly(True)
        self.factory_info_system_text.setPlaceholderText("等待系统信息...")
        self.factory_info_system_text.setMinimumHeight(120)
        self.btn_factory_step0_refresh = QPushButton("1) 刷新当前信息")
        self.btn_factory_step0_refresh.clicked.connect(self.factory_refresh_info_snapshot)
        self.btn_factory_step0_done = QPushButton("1) 完成本项")
        self.btn_factory_step0_done.clicked.connect(self.factory_finish_info_snapshot)
        self._factory_refresh_info_snapshot_fields()

        self.btn_factory_brainsense_gui = QPushButton("2) 启动 BrainSense GUI（仅Ubuntu生效）")
        self.btn_factory_brainsense_gui.clicked.connect(self.launch_factory_brainsense_gui)
        self.btn_factory_stop_big_middleware = QPushButton("2) 关闭中间件")
        self.btn_factory_stop_big_middleware.clicked.connect(self.factory_stop_big_brain_middleware)
        self.btn_factory_start_big_middleware = QPushButton("2) 开启中间件")
        self.btn_factory_start_big_middleware.clicked.connect(self.factory_start_big_brain_middleware)
        self.btn_factory_test_clear = QPushButton("清空输出")
        self.btn_factory_test_clear.clicked.connect(lambda: self.factory_test_clear_signal.emit())

        self.btn_factory_step1_done = QPushButton("2) 完成本项")
        self.btn_factory_step1_done.clicked.connect(lambda: self.factory_mark_step_done(2, "已确认 BrainSense GUI 与中间件切换"))

        self.factory_topic_service_audit_text = QPlainTextEdit()
        self.factory_topic_service_audit_text.setReadOnly(True)
        self.factory_topic_service_audit_text.setPlaceholderText("点击检测后，显示需监控话题清单的广播和频率结果。")
        self.factory_topic_service_audit_text.setMinimumHeight(220)
        self.btn_factory_step2_audit_run = QPushButton("3) 检测话题")
        self.btn_factory_step2_audit_run.clicked.connect(self.factory_run_topic_service_audit)
        self.btn_factory_step2_audit_done = QPushButton("3) 完成本项")
        self.btn_factory_step2_audit_done.clicked.connect(self.factory_finish_topic_service_audit)

        self.btn_factory_step1_run = QPushButton("4) 运行手指关节测试")
        self.btn_factory_step1_run.clicked.connect(self.run_factory_finger_joint_test)
        self.btn_factory_step2_done = QPushButton("4) 完成本项")
        self.btn_factory_step2_done.clicked.connect(lambda: self.factory_mark_step_done(4, "已确认手指关节运动"))

        self.btn_factory_step2_start = QPushButton("5) 开始压力采集")
        self.btn_factory_step2_start.clicked.connect(self.factory_step2_start_pressure_capture)
        self.btn_factory_step3_done = QPushButton("5) 完成本项")
        self.btn_factory_step3_done.clicked.connect(self.factory_step2_finish)
        self.factory_pressure_max_fields = {"left": [], "right": []}
        self.factory_pressure_max_grid = QGridLayout()
        finger_labels = self._finger_labels_for_side()
        self.factory_pressure_max_grid.addWidget(QLabel("手指"), 0, 0)
        self.factory_pressure_max_grid.addWidget(QLabel("左手最大值"), 0, 1)
        self.factory_pressure_max_grid.addWidget(QLabel("右手最大值"), 0, 2)
        for idx, label_text in enumerate(finger_labels):
            self.factory_pressure_max_grid.addWidget(QLabel(label_text), idx + 1, 0)
            left_field = QLineEdit("-")
            left_field.setReadOnly(True)
            right_field = QLineEdit("-")
            right_field.setReadOnly(True)
            self.factory_pressure_max_fields["left"].append(left_field)
            self.factory_pressure_max_fields["right"].append(right_field)
            self.factory_pressure_max_grid.addWidget(left_field, idx + 1, 1)
            self.factory_pressure_max_grid.addWidget(right_field, idx + 1, 2)

        self.factory_force_fields = {}
        self.factory_force_grid = QGridLayout()
        self.factory_force_grid.addWidget(QLabel("项目"), 0, 0)
        self.factory_force_grid.addWidget(QLabel("左手Force XYZ"), 0, 1)
        self.factory_force_grid.addWidget(QLabel("右手Force XYZ"), 0, 2)
        for row, (field_key, label_text, read_only) in enumerate([
            ("baseline", "初始值", False),
            ("current", "检测值", False),
            ("delta", "偏差", True),
        ], start=1):
            left_edit = QLineEdit()
            right_edit = QLineEdit()
            left_edit.setPlaceholderText(f"6) {label_text} 左手: Fx,Fy,Fz")
            right_edit.setPlaceholderText(f"6) {label_text} 右手: Fx,Fy,Fz")
            left_edit.setReadOnly(read_only)
            right_edit.setReadOnly(read_only)
            self.factory_force_fields[field_key] = {"left": left_edit, "right": right_edit}
            self.factory_force_grid.addWidget(QLabel(label_text), row, 0)
            self.factory_force_grid.addWidget(left_edit, row, 1)
            self.factory_force_grid.addWidget(right_edit, row, 2)
        self.btn_factory_force_capture_baseline = QPushButton("6) 采集Force基线")
        self.btn_factory_force_capture_baseline.clicked.connect(lambda: self.factory_capture_force_values("baseline"))
        self.btn_factory_force_enable_sensor = QPushButton("6) 配置六维力传感器")
        self.btn_factory_force_enable_sensor.clicked.connect(self.factory_enable_force_sensor)
        self.btn_factory_force_restart_middleware = QPushButton("6) 重启中间件")
        self.btn_factory_force_restart_middleware.clicked.connect(self.restart_middleware_service)
        self.btn_factory_force_capture_current = QPushButton("6) 开始Force检测")
        self.btn_factory_force_capture_current.clicked.connect(self.factory_start_force_monitor)
        self.btn_factory_force_calc = QPushButton("6) 停止Force检测")
        self.btn_factory_force_calc.clicked.connect(self.factory_stop_force_monitor)
        self.btn_factory_force_calc.setEnabled(False)
        self.btn_factory_step4_done = QPushButton("6) 完成本项")
        self.btn_factory_step4_done.clicked.connect(self.factory_step3_finish)

        self.factory_tts_text = QLineEdit("你好我来自浙江人形机器人创新中心")
        self.factory_tts_text.setPlaceholderText("输入文本后调用 /zj_humanoid/audio/tts_service")
        self.factory_tts_play_checkbox = QCheckBox("播放语音")
        self.factory_tts_play_checkbox.setChecked(True)
        self.btn_factory_tts = QPushButton("7) 调用TTS")
        self.btn_factory_tts.clicked.connect(self.call_factory_tts)
        self.factory_audio_volume_edit = QLineEdit("50")
        self.factory_audio_volume_edit.setPlaceholderText("音量，例如 50")
        self.btn_factory_audio_set_volume = QPushButton("7) 设置音量")
        self.btn_factory_audio_set_volume.clicked.connect(self.call_factory_set_volume)
        self.btn_factory_step5_done = QPushButton("7) 完成本项")
        self.btn_factory_step5_done.clicked.connect(lambda: self.factory_mark_step_done(7, "语音播报与倾听已确认"))

        self.factory_camera_hint = QLabel("8) 相机图像检测（ROSBridge订阅）：WA1需头部+胸部，WA2只需头部")
        self.factory_camera_status = QLineEdit("未启动")
        self.factory_camera_status.setReadOnly(True)
        self.factory_camera_head_required = QCheckBox("头部相机必须检测")
        self.factory_camera_chest_required = QCheckBox("胸部相机必须检测")
        self.factory_camera_head_required.toggled.connect(self._factory_update_camera_topics_display)
        self.factory_camera_chest_required.toggled.connect(self._factory_update_camera_topics_display)
        self.factory_camera_head_topic_value = QLineEdit("-")
        self.factory_camera_head_topic_value.setReadOnly(True)
        self.factory_camera_chest_topic_value = QLineEdit("-")
        self.factory_camera_chest_topic_value.setReadOnly(True)
        self.btn_factory_step5_start = QPushButton("8) 开始相机订阅")
        self.btn_factory_step5_start.clicked.connect(self.factory_start_camera_sampling)
        self.btn_factory_step5_stop = QPushButton("8) 停止相机订阅")
        self.btn_factory_step5_stop.clicked.connect(self.factory_stop_camera_sampling)

        self.factory_camera_head_preview = QLabel("头部相机等待画面")
        self.factory_camera_head_preview.setMinimumSize(360, 200)
        self.factory_camera_head_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.factory_camera_head_preview.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")
        self.factory_camera_head_path = QLineEdit("-")
        self.factory_camera_head_path.setReadOnly(True)

        self.factory_camera_chest_preview = QLabel("胸部相机等待画面")
        self.factory_camera_chest_preview.setMinimumSize(360, 200)
        self.factory_camera_chest_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.factory_camera_chest_preview.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")
        self.factory_camera_chest_path = QLineEdit("-")
        self.factory_camera_chest_path.setReadOnly(True)

        self.btn_factory_step6_done = QPushButton("8) 完成本项")
        self.btn_factory_step6_done.clicked.connect(self.factory_step5_finish)

        self.btn_factory_test_robot_info = QPushButton("9) 运行 robot_info_validate.py")
        self.btn_factory_test_robot_info.clicked.connect(
            lambda: self.run_factory_test_script("Robot 信息校验", "testcase/wa1/smoke/robot_info_validate.py")
        )
        self.btn_factory_step7_done = QPushButton("9) 完成本项")
        self.btn_factory_step7_done.clicked.connect(lambda: self.factory_mark_step_done(9, "robot_info_validate 已确认"))

        self.btn_factory_test_upperlimb = QPushButton("10) 运行上肢测试")
        self.btn_factory_test_upperlimb.clicked.connect(self.run_factory_upperlimb_test)
        self.btn_factory_upperlimb_home_dual = QPushButton("10) 双臂归位")
        self.btn_factory_upperlimb_home_dual.clicked.connect(self.factory_go_home_dual_arm)
        self.btn_factory_upperlimb_home_waist = QPushButton("10) 腰部归位")
        self.btn_factory_upperlimb_home_waist.clicked.connect(self.factory_go_home_waist)
        self.btn_factory_step8_done = QPushButton("10) 完成本项")
        self.btn_factory_step8_done.clicked.connect(lambda: self.factory_mark_step_done(10, "上肢测试已确认"))

        row_factory_test_actions = QHBoxLayout()
        row_factory_test_actions.addWidget(QLabel("流程状态"))
        row_factory_test_actions.addWidget(self.factory_flow_status)
        row_factory_test_actions.addWidget(QLabel("容器状态"))
        row_factory_test_actions.addWidget(self.factory_container_status)
        row_factory_test_actions.addWidget(QLabel("远端目录"))
        row_factory_test_actions.addWidget(self.factory_remote_demo_dir_edit)
        row_factory_test_actions.addWidget(QLabel("机器人SN"))
        row_factory_test_actions.addWidget(self.factory_robot_sn_edit)
        row_factory_test_actions.addWidget(self.btn_factory_flow_start)
        row_factory_test_actions.addWidget(self.btn_factory_flow_report)
        row_factory_test_actions.addWidget(self.btn_factory_test_run_container)
        row_factory_test_actions.addWidget(self.btn_factory_test_clear)
        row_factory_test_actions.addStretch(1)

        self.factory_info_grid = QGridLayout()
        self.factory_info_grid.addWidget(QLabel("机器人型号"), 0, 0)
        self.factory_info_grid.addWidget(self.factory_info_model_value, 0, 1)
        self.factory_info_grid.addWidget(QLabel("嵌入式版本"), 1, 0)
        self.factory_info_grid.addWidget(self.factory_info_embedded_value, 1, 1)
        self.factory_info_grid.addWidget(QLabel("中间件版本"), 2, 0)
        self.factory_info_grid.addWidget(self.factory_info_middleware_value, 2, 1)
        self.factory_info_grid.addWidget(QLabel("上肢版本"), 3, 0)
        self.factory_info_grid.addWidget(self.factory_info_upperlimb_value, 3, 1)
        self.factory_info_grid.addWidget(self.factory_info_lowerlimb_label, 4, 0)
        self.factory_info_grid.addWidget(self.factory_info_lowerlimb_value, 4, 1)

        row_step0 = QHBoxLayout()
        row_step0.addWidget(self.btn_factory_step0_refresh)
        row_step0.addWidget(self.btn_factory_step0_done)
        row_step0.addStretch(1)

        row_step1 = QHBoxLayout()
        row_step1.addWidget(self.btn_factory_brainsense_gui)
        row_step1.addWidget(self.btn_factory_stop_big_middleware)
        row_step1.addWidget(self.btn_factory_start_big_middleware)
        row_step1.addWidget(self.btn_factory_step1_done)
        row_step1.addStretch(1)

        row_step2_audit = QHBoxLayout()
        row_step2_audit.addWidget(self.btn_factory_step2_audit_run)
        row_step2_audit.addWidget(self.btn_factory_step2_audit_done)
        row_step2_audit.addStretch(1)

        row_step2 = QHBoxLayout()
        row_step2.addWidget(self.btn_factory_step1_run)
        row_step2.addWidget(self.btn_factory_step2_done)
        row_step2.addStretch(1)

        row_step3 = QHBoxLayout()
        row_step3.addWidget(self.btn_factory_step2_start)
        row_step3.addWidget(self.btn_factory_step3_done)
        row_step3.addStretch(1)

        row_step4 = QHBoxLayout()
        row_step4.addWidget(self.btn_factory_force_calc)
        row_step4.addWidget(self.btn_factory_step4_done)
        row_step4.addStretch(1)

        row_step4_capture = QHBoxLayout()
        row_step4_capture.addWidget(self.btn_factory_force_enable_sensor)
        row_step4_capture.addWidget(self.btn_factory_force_restart_middleware)
        row_step4_capture.addWidget(self.btn_factory_force_capture_baseline)
        row_step4_capture.addWidget(self.btn_factory_force_capture_current)
        row_step4_capture.addStretch(1)

        row_step5 = QHBoxLayout()
        row_step5.addWidget(QLabel("TTS文本"))
        row_step5.addWidget(self.factory_tts_text, 1)
        row_step5.addWidget(self.factory_tts_play_checkbox)
        row_step5.addWidget(self.btn_factory_tts)
        row_step5.addWidget(QLabel("音量"))
        row_step5.addWidget(self.factory_audio_volume_edit)
        row_step5.addWidget(self.btn_factory_audio_set_volume)
        row_step5.addWidget(self.btn_factory_step5_done)
        row_step5.addStretch(1)

        row_step6 = QHBoxLayout()
        row_step6.addWidget(QLabel("状态"))
        row_step6.addWidget(self.factory_camera_status)
        row_step6.addWidget(self.factory_camera_head_required)
        row_step6.addWidget(self.factory_camera_chest_required)
        row_step6.addWidget(self.btn_factory_step5_start)
        row_step6.addWidget(self.btn_factory_step5_stop)
        row_step6.addWidget(self.btn_factory_step6_done)
        row_step6.addStretch(1)

        row_step6_topics = QGridLayout()
        self.factory_camera_head_topic_label = QLabel("头部话题")
        row_step6_topics.addWidget(self.factory_camera_head_topic_label, 0, 0)
        row_step6_topics.addWidget(self.factory_camera_head_topic_value, 0, 1)
        self.factory_camera_chest_topic_label = QLabel("胸部话题")
        row_step6_topics.addWidget(self.factory_camera_chest_topic_label, 1, 0)
        row_step6_topics.addWidget(self.factory_camera_chest_topic_value, 1, 1)

        row_step6_preview = QGridLayout()
        self.factory_camera_head_preview_title = QLabel("头部图像")
        self.factory_camera_chest_preview_title = QLabel("胸部图像")
        row_step6_preview.addWidget(self.factory_camera_head_preview_title, 0, 0)
        row_step6_preview.addWidget(self.factory_camera_chest_preview_title, 0, 1)
        row_step6_preview.addWidget(self.factory_camera_head_preview, 1, 0)
        row_step6_preview.addWidget(self.factory_camera_chest_preview, 1, 1)
        row_step6_preview.addWidget(self.factory_camera_head_path, 2, 0)
        row_step6_preview.addWidget(self.factory_camera_chest_path, 2, 1)

        row_step7 = QHBoxLayout()
        row_step7.addWidget(self.btn_factory_test_robot_info)
        row_step7.addWidget(self.btn_factory_step7_done)
        row_step7.addStretch(1)

        row_step8 = QHBoxLayout()
        row_step8.addWidget(self.btn_factory_upperlimb_home_dual)
        row_step8.addWidget(self.btn_factory_upperlimb_home_waist)
        row_step8.addWidget(self.btn_factory_test_upperlimb)
        row_step8.addWidget(self.btn_factory_step8_done)
        row_step8.addStretch(1)

        self.factory_step_selector = QComboBox()
        self.factory_step_selector.addItems([
            "步骤1 机器人信息确认",
            "步骤2 BrainSense与中间件控制",
            "步骤3 ROS话题检测",
            "步骤4 手指关节运动",
            "步骤5 手指压力检测",
            "步骤6 六维力检测",
            "步骤7 语音播报与倾听",
            "步骤8 相机图像检测",
            "步骤9 robot_info 校验",
            "步骤10 upperlimb 运动检测",
        ])
        self.factory_step_stack = QStackedWidget()

        step0_widget = QWidget()
        step0_layout = QVBoxLayout(step0_widget)
        step0_layout.setContentsMargins(0, 0, 0, 0)
        step0_layout.addLayout(self.factory_info_grid)
        step0_layout.addWidget(QLabel("系统信息"))
        step0_layout.addWidget(self.factory_info_system_text)
        step0_layout.addLayout(row_step0)
        step0_layout.addStretch(1)

        step1_widget = QWidget()
        step1_layout = QVBoxLayout(step1_widget)
        step1_layout.setContentsMargins(0, 0, 0, 0)
        step1_layout.addLayout(row_step1)
        step1_layout.addStretch(1)

        step2_audit_widget = QWidget()
        step2_audit_layout = QVBoxLayout(step2_audit_widget)
        step2_audit_layout.setContentsMargins(0, 0, 0, 0)
        step2_audit_layout.addLayout(row_step2_audit)
        step2_audit_layout.addWidget(self.factory_topic_service_audit_text)
        step2_audit_layout.addStretch(1)

        step2_widget = QWidget()
        step2_layout = QVBoxLayout(step2_widget)
        step2_layout.setContentsMargins(0, 0, 0, 0)
        step2_layout.addLayout(row_step2)
        step2_layout.addStretch(1)

        step3_widget = QWidget()
        step3_layout = QVBoxLayout(step3_widget)
        step3_layout.setContentsMargins(0, 0, 0, 0)
        step3_layout.addLayout(row_step3)
        step3_layout.addLayout(self.factory_pressure_max_grid)
        step3_layout.addStretch(1)

        step4_widget = QWidget()
        step4_layout = QVBoxLayout(step4_widget)
        step4_layout.setContentsMargins(0, 0, 0, 0)
        step4_layout.addLayout(self.factory_force_grid)
        step4_layout.addLayout(row_step4_capture)
        step4_layout.addLayout(row_step4)
        step4_layout.addStretch(1)

        step5_widget = QWidget()
        step5_layout = QVBoxLayout(step5_widget)
        step5_layout.setContentsMargins(0, 0, 0, 0)
        step5_layout.addLayout(row_step5)
        step5_layout.addStretch(1)

        step6_widget = QWidget()
        step6_layout = QVBoxLayout(step6_widget)
        step6_layout.setContentsMargins(0, 0, 0, 0)
        step6_layout.addWidget(self.factory_camera_hint)
        step6_layout.addLayout(row_step6)
        step6_layout.addLayout(row_step6_topics)
        step6_layout.addLayout(row_step6_preview)
        step6_layout.addStretch(1)

        step7_widget = QWidget()
        step7_layout = QVBoxLayout(step7_widget)
        step7_layout.setContentsMargins(0, 0, 0, 0)
        step7_layout.addLayout(row_step7)
        step7_layout.addStretch(1)

        step8_widget = QWidget()
        step8_layout = QVBoxLayout(step8_widget)
        step8_layout.setContentsMargins(0, 0, 0, 0)
        step8_layout.addLayout(row_step8)
        step8_layout.addStretch(1)

        self.factory_step_stack.addWidget(step0_widget)
        self.factory_step_stack.addWidget(step1_widget)
        self.factory_step_stack.addWidget(step2_audit_widget)
        self.factory_step_stack.addWidget(step2_widget)
        self.factory_step_stack.addWidget(step3_widget)
        self.factory_step_stack.addWidget(step4_widget)
        self.factory_step_stack.addWidget(step5_widget)
        self.factory_step_stack.addWidget(step6_widget)
        self.factory_step_stack.addWidget(step7_widget)
        self.factory_step_stack.addWidget(step8_widget)
        self.factory_step_selector.currentIndexChanged.connect(self.factory_step_stack.setCurrentIndex)
        self.factory_step_stack.setCurrentIndex(0)
        self.factory_step_stack.setMinimumHeight(320)

        row_factory_step_switch = QHBoxLayout()
        row_factory_step_switch.addWidget(QLabel("步骤选择"))
        row_factory_step_switch.addWidget(self.factory_step_selector)
        row_factory_step_switch.addStretch(1)

        self.factory_test_output = QPlainTextEdit()
        self.factory_test_output.setReadOnly(True)
        self.factory_test_output.setPlaceholderText("点击按钮后，这里会显示容器启动与工厂测试脚本输出。")
        self.factory_test_output.setMaximumHeight(160)

        self.factory_report_text = QPlainTextEdit()
        self.factory_report_text.setReadOnly(True)
        self.factory_report_text.setPlaceholderText("点击“生成测试报告”后，这里显示完整报告。")

        factory_test_layout.addWidget(factory_test_desc)
        factory_test_layout.addLayout(row_factory_test_actions)
        factory_test_layout.addLayout(row_factory_step_switch)
        factory_test_layout.addWidget(self.factory_step_stack)
        factory_test_layout.addWidget(QLabel("工厂测试输出"))
        factory_test_layout.addWidget(self.factory_test_output)
        factory_test_layout.setStretchFactor(self.factory_step_stack, 1)
        factory_test_scroll.setWidget(factory_test_panel)
        factory_test_tab_layout.addWidget(factory_test_scroll)
        self.factory_flow_reset()

        # ---------- Tab: movej_by_path ----------
        tab_huiyang = QWidget()
        huiyang_layout = QVBoxLayout()

        # 机型选择 & 示教控制
        self.huiyang_model_combo = QComboBox()
        self.huiyang_model_combo.addItems([""])
        self.huiyang_model_combo.setEnabled(False)
        self._update_huiyang_mode_label()
        self.btn_huiyang_enter_teach = QPushButton("进入全身示教")
        self.btn_huiyang_enter_teach.setStyleSheet("QPushButton{background:#4caf50;color:white;font-weight:bold;}")
        self.btn_huiyang_enter_teach.clicked.connect(self.huiyang_enter_teach)
        self.btn_huiyang_exit_teach = QPushButton("退出示教")
        self.btn_huiyang_exit_teach.setEnabled(False)
        self.btn_huiyang_exit_teach.clicked.connect(self.huiyang_exit_teach)
        self.huiyang_status_label = QLabel("状态: 就绪")

        hy_teach_row = QHBoxLayout()
        hy_teach_row.addWidget(QLabel("模式"))
        hy_teach_row.addWidget(self.huiyang_model_combo)
        hy_teach_row.addWidget(self.btn_huiyang_enter_teach)
        hy_teach_row.addWidget(self.btn_huiyang_exit_teach)
        hy_teach_row.addWidget(self.huiyang_status_label)
        hy_teach_row.addStretch(1)

        # 数据信息
        self.huiyang_frame_count_label = QLabel("已记录帧数: 0")
        self.huiyang_duration_label = QLabel("时长: 0.00s")
        self.btn_huiyang_clear = QPushButton("清除数据")
        self.btn_huiyang_clear.clicked.connect(self.huiyang_clear_data)

        hy_info_row = QHBoxLayout()
        hy_info_row.addWidget(QLabel("采样间隔: 0.5s"))
        hy_info_row.addWidget(self.huiyang_frame_count_label)
        hy_info_row.addWidget(self.huiyang_duration_label)
        hy_info_row.addWidget(self.btn_huiyang_clear)
        hy_info_row.addStretch(1)

        # 回放设置
        self.huiyang_movej_v_spin = QDoubleSpinBox()
        self.huiyang_movej_v_spin.setRange(0.01, 1.0)
        self.huiyang_movej_v_spin.setSingleStep(0.05)
        self.huiyang_movej_v_spin.setValue(0.2)
        self.huiyang_movej_v_spin.setDecimals(2)
        self.huiyang_time_scale_spin = QDoubleSpinBox()
        self.huiyang_time_scale_spin.setRange(0.1, 5.0)
        self.huiyang_time_scale_spin.setSingleStep(0.1)
        self.huiyang_time_scale_spin.setValue(1.0)
        self.huiyang_time_scale_spin.setDecimals(1)
        self.btn_huiyang_playback = QPushButton("回放")
        self.btn_huiyang_playback.setEnabled(False)
        self.btn_huiyang_playback.clicked.connect(self.huiyang_playback)
        self.btn_huiyang_export_npz = QPushButton("导出NPZ")
        self.btn_huiyang_export_npz.setEnabled(False)
        self.btn_huiyang_export_npz.clicked.connect(self.huiyang_export_npz)

        hy_playback_row = QHBoxLayout()
        hy_playback_row.addWidget(QLabel("MoveJ速度v"))
        hy_playback_row.addWidget(self.huiyang_movej_v_spin)
        hy_playback_row.addWidget(QLabel("回放时间缩放"))
        hy_playback_row.addWidget(self.huiyang_time_scale_spin)
        hy_playback_row.addWidget(self.btn_huiyang_playback)
        hy_playback_row.addWidget(self.btn_huiyang_export_npz)
        hy_playback_row.addStretch(1)

        self.huiyang_output = QPlainTextEdit()
        self.huiyang_output.setReadOnly(True)
        self.huiyang_output.setPlaceholderText("movej_by_path 日志...")
        self.huiyang_output.setMinimumHeight(160)

        huiyang_layout.addLayout(hy_teach_row)
        huiyang_layout.addLayout(hy_info_row)
        huiyang_layout.addLayout(hy_playback_row)
        huiyang_layout.addWidget(QLabel("日志"))
        huiyang_layout.addWidget(self.huiyang_output)
        huiyang_layout.addStretch(1)
        tab_huiyang.setLayout(huiyang_layout)

        # ---------- Tab: ServoJ工具 ----------
        tab_servoj_tool = QWidget()
        servoj_tool_layout = QVBoxLayout()

        servoj_tool_tip = QLabel("用途: 进入全身示教后自动开始录制 joint_states，退出示教自动结束录制；回放时先 MoveJ 回到初始位置，再按固定频率发送 ServoJ。")
        servoj_tool_tip.setWordWrap(True)

        self.servoj_tool_model_combo = QComboBox()
        self.servoj_tool_model_combo.addItems(["WA1", "WA2", "I2"])
        default_servoj_model = str(self.robot_model or "WA2").upper()
        if default_servoj_model == "WA2_LS":
            default_servoj_model = "WA2"
        if default_servoj_model not in {"WA1", "WA2", "I2"}:
            default_servoj_model = "WA2"
        self.servoj_tool_model_combo.setCurrentText(default_servoj_model)

        self.servoj_tool_hz_spin = QDoubleSpinBox()
        self.servoj_tool_hz_spin.setRange(10.0, 500.0)
        self.servoj_tool_hz_spin.setDecimals(0)
        self.servoj_tool_hz_spin.setSingleStep(10.0)
        self.servoj_tool_hz_spin.setValue(200.0)

        self.servoj_tool_mode_combo = QComboBox()
        self.servoj_tool_mode_combo.addItem("ROSBridge录制+发布", "rosbridge")
        self.servoj_tool_mode_combo.addItem("SSH(小脑)录制+发布", "ssh")

        self.btn_servoj_tool_start = QPushButton("进入全身示教")
        self.btn_servoj_tool_start.clicked.connect(self.servoj_tool_enter_teach)
        self.btn_servoj_tool_stop = QPushButton("退出示教")
        self.btn_servoj_tool_stop.setEnabled(False)
        self.btn_servoj_tool_stop.clicked.connect(self.servoj_tool_exit_teach)
        self.btn_servoj_tool_clear = QPushButton("清除数据")
        self.btn_servoj_tool_clear.clicked.connect(self.clear_servoj_tool_recording)
        self.btn_servoj_tool_export = QPushButton("导出固定频率NPZ")
        self.btn_servoj_tool_export.setEnabled(False)
        self.btn_servoj_tool_export.clicked.connect(self.export_servoj_tool_npz)
        self.btn_servoj_tool_playback = QPushButton("回放当前录制")
        self.btn_servoj_tool_playback.setEnabled(False)
        self.btn_servoj_tool_playback.clicked.connect(self.playback_servoj_tool)
        self.servoj_tool_movej_v_spin = QDoubleSpinBox()
        self.servoj_tool_movej_v_spin.setRange(0.01, 1.0)
        self.servoj_tool_movej_v_spin.setDecimals(2)
        self.servoj_tool_movej_v_spin.setSingleStep(0.05)
        self.servoj_tool_movej_v_spin.setValue(0.2)

        self.servoj_tool_status_label = QLabel("状态: 未录制")
        self.servoj_tool_frame_label = QLabel("原始帧数: 0")
        self.servoj_tool_duration_label = QLabel("时长: 0.00s")

        servoj_tool_row1 = QHBoxLayout()
        servoj_tool_row1.addWidget(QLabel("型号"))
        servoj_tool_row1.addWidget(self.servoj_tool_model_combo)
        servoj_tool_row1.addWidget(QLabel("模式"))
        servoj_tool_row1.addWidget(self.servoj_tool_mode_combo)
        servoj_tool_row1.addWidget(QLabel("导出频率Hz"))
        servoj_tool_row1.addWidget(self.servoj_tool_hz_spin)
        servoj_tool_row1.addWidget(self.btn_servoj_tool_start)
        servoj_tool_row1.addWidget(self.btn_servoj_tool_stop)
        servoj_tool_row1.addWidget(self.btn_servoj_tool_clear)
        servoj_tool_row1.addWidget(self.btn_servoj_tool_export)
        servoj_tool_row1.addWidget(QLabel("MoveJ速度v"))
        servoj_tool_row1.addWidget(self.servoj_tool_movej_v_spin)
        servoj_tool_row1.addWidget(self.btn_servoj_tool_playback)
        servoj_tool_row1.addStretch(1)

        servoj_tool_row2 = QHBoxLayout()
        servoj_tool_row2.addWidget(self.servoj_tool_status_label)
        servoj_tool_row2.addWidget(self.servoj_tool_frame_label)
        servoj_tool_row2.addWidget(self.servoj_tool_duration_label)
        servoj_tool_row2.addStretch(1)

        self.servoj_tool_output = QPlainTextEdit()
        self.servoj_tool_output.setReadOnly(True)
        self.servoj_tool_output.setPlaceholderText("ServoJ工具日志...")
        self.servoj_tool_output.setMinimumHeight(160)

        servoj_tool_layout.addWidget(servoj_tool_tip)
        servoj_tool_layout.addLayout(servoj_tool_row1)
        servoj_tool_layout.addLayout(servoj_tool_row2)
        servoj_tool_layout.addWidget(QLabel("日志"))
        servoj_tool_layout.addWidget(self.servoj_tool_output)
        servoj_tool_layout.addStretch(1)
        tab_servoj_tool.setLayout(servoj_tool_layout)

        # ---------- Tab: 回放工具 ----------
        tab_replay_tools = QWidget()
        replay_tools_tab_layout = QVBoxLayout(tab_replay_tools)
        replay_tools_tab_layout.setContentsMargins(0, 0, 0, 0)

        replay_tools_scroll = QScrollArea()
        replay_tools_scroll.setWidgetResizable(True)
        replay_tools_scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        replay_tools_panel = QWidget()
        replay_tools_layout = QVBoxLayout(replay_tools_panel)

        self.replay_huiyang_section, self.btn_toggle_replay_huiyang = self._build_collapsible_section(
            "movej_by_path", tab_huiyang, expanded=True
        )
        self.replay_servoj_tool_section, self.btn_toggle_replay_servoj_tool = self._build_collapsible_section(
            "ServoJ工具", tab_servoj_tool, expanded=True
        )

        replay_tools_layout.addWidget(self.replay_huiyang_section)
        replay_tools_layout.addWidget(self.replay_servoj_tool_section)
        replay_tools_layout.addStretch(1)
        replay_tools_scroll.setWidget(replay_tools_panel)
        replay_tools_tab_layout.addWidget(replay_tools_scroll)

        # ---------- Tab 8: 视觉监控 ----------
        tab_vision = QWidget()
        vision_layout = QVBoxLayout()

        self.vision_topic_edit = QComboBox()
        self.vision_topic_edit.setEditable(False)
        self.vision_topic_edit.setMinimumWidth(420)
        self.vision_mode = QComboBox()
        self.vision_mode.addItems(["单帧", "多帧"])
        self.btn_vision_detect = QPushButton("检测当前帧")
        self.btn_vision_detect.clicked.connect(self.detect_vision_frame)
        self.btn_vision_stop = QPushButton("结束采样")
        self.btn_vision_stop.setEnabled(False)
        self.btn_vision_stop.clicked.connect(self.stop_vision_sampling)
        self.btn_refresh_vision_rates = QPushButton("刷新六路话题频率")
        self.btn_refresh_vision_rates.clicked.connect(self.refresh_vision_topic_rates)
        self.vision_info = QLineEdit("-")
        self.vision_info.setReadOnly(True)

        self.vision_rate_groups = [
            (
                "realsense_up",
                [
                    ("color", "/zj_humanoid/sensor/realsense_up/color/image_raw"),
                    ("depth", "/zj_humanoid/sensor/realsense_up/depth/image_rect_raw"),
                    ("aligned", "/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw"),
                ],
            ),
            (
                "realsense_head",
                [
                    ("color", "/zj_humanoid/sensor/realsense_head/color/image_raw"),
                    ("depth", "/zj_humanoid/sensor/realsense_head/depth/image_rect_raw"),
                    ("aligned", "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw"),
                ],
            ),
        ]
        self.vision_rate_fields = {}
        self._refresh_vision_topic_options([])

        vision_top = QHBoxLayout()
        vision_top.addWidget(QLabel("图像话题"))
        vision_top.addWidget(self.vision_topic_edit)
        vision_top.addWidget(QLabel("模式"))
        vision_top.addWidget(self.vision_mode)
        vision_top.addWidget(self.btn_vision_detect)
        vision_top.addWidget(self.btn_vision_stop)

        vision_rate_grid = QGridLayout()
        for idx, (group_name, _items) in enumerate(self.vision_rate_groups):
            val = QLineEdit("-")
            val.setReadOnly(True)
            self.vision_rate_fields[group_name] = val
            vision_rate_grid.addWidget(QLabel(group_name), idx, 0)
            vision_rate_grid.addWidget(val, idx, 1)

        self.vision_preview = QLabel("等待检测")
        self.vision_preview.setMinimumSize(640, 360)
        self.vision_preview.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")
        self.vision_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)

        vision_layout.addLayout(vision_top)
        vision_layout.addWidget(self.btn_refresh_vision_rates)
        vision_layout.addLayout(vision_rate_grid)
        vision_layout.addWidget(QLabel("输出信息"))
        vision_layout.addWidget(self.vision_info)
        vision_layout.addWidget(self.vision_preview)
        tab_vision.setLayout(vision_layout)

        # ---------- Tab 8: 位姿估计 ----------
        tab_pose = QWidget()
        pose_layout = QVBoxLayout()

        self.pose_remote_coke_edit = QLineEdit("/home/naviai/source/src/fdpose/scripts/debug/track_vis/cola_germany.png")
        self.pose_remote_water_edit = QLineEdit("/home/naviai/source/src/fdpose/scripts/debug/track_vis/vio_medium.png")
        self.pose_interval_edit = QLineEdit("1000")
        self.pose_interval_edit.setPlaceholderText("毫秒，默认1000")

        self.btn_pose_start = QPushButton("开始采样")
        self.btn_pose_start.clicked.connect(self.start_pose_polling)
        self.btn_pose_stop = QPushButton("结束采样")
        self.btn_pose_stop.setEnabled(False)
        self.btn_pose_stop.clicked.connect(self.stop_pose_polling)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("可乐PNG路径"))
        row1.addWidget(self.pose_remote_coke_edit)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("水PNG路径"))
        row2.addWidget(self.pose_remote_water_edit)

        row3 = QHBoxLayout()
        row3.addWidget(QLabel("采样间隔(ms)"))
        row3.addWidget(self.pose_interval_edit)
        row3.addWidget(self.btn_pose_start)
        row3.addWidget(self.btn_pose_stop)
        row3.addStretch(1)

        self.pose_coke_info = QLineEdit("-")
        self.pose_coke_info.setReadOnly(True)
        self.pose_water_info = QLineEdit("-")
        self.pose_water_info.setReadOnly(True)

        self.pose_coke_preview = QLabel("可乐检测图等待采样")
        self.pose_coke_preview.setMinimumSize(420, 280)
        self.pose_coke_preview.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")
        self.pose_coke_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.pose_water_preview = QLabel("水检测图等待采样")
        self.pose_water_preview.setMinimumSize(420, 280)
        self.pose_water_preview.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")
        self.pose_water_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)

        preview_line = QHBoxLayout()
        coke_col = QVBoxLayout()
        coke_col.addWidget(QLabel("可乐检测结果"))
        coke_col.addWidget(self.pose_coke_info)
        coke_col.addWidget(self.pose_coke_preview)
        water_col = QVBoxLayout()
        water_col.addWidget(QLabel("水检测结果"))
        water_col.addWidget(self.pose_water_info)
        water_col.addWidget(self.pose_water_preview)
        preview_line.addLayout(coke_col)
        preview_line.addLayout(water_col)

        pose_layout.addLayout(row1)
        pose_layout.addLayout(row2)
        pose_layout.addLayout(row3)
        pose_layout.addLayout(preview_line)
        tab_pose.setLayout(pose_layout)

        # ---------- Tab 9: 运维测试 ----------
        tab_maintenance = QWidget()
        maintenance_tab_layout = QVBoxLayout(tab_maintenance)
        maintenance_tab_layout.setContentsMargins(0, 0, 0, 0)

        maintenance_scroll = QScrollArea()
        maintenance_scroll.setWidgetResizable(True)
        maintenance_scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        maintenance_panel = QWidget()
        maintenance_layout = QVBoxLayout(maintenance_panel)

        audio_box = QWidget()
        audio_layout = QVBoxLayout(audio_box)
        audio_layout.setContentsMargins(8, 8, 8, 8)
        audio_layout.setSpacing(6)

        self.maintenance_tts_text = QLineEdit()
        self.maintenance_tts_text.setPlaceholderText("输入文本后调用 /zj_humanoid/audio/tts_service")
        self.maintenance_tts_play_checkbox = QCheckBox("播放语音")
        self.maintenance_tts_play_checkbox.setChecked(True)
        self.btn_maintenance_tts = QPushButton("调用 TTS")
        self.btn_maintenance_tts.clicked.connect(self.call_maintenance_tts)
        self.btn_maintenance_mic_list = QPushButton("获取麦克风设备列表")
        self.btn_maintenance_mic_list.clicked.connect(self.refresh_maintenance_microphones)
        self.btn_maintenance_speaker_list = QPushButton("获取扬声器设备列表")
        self.btn_maintenance_speaker_list.clicked.connect(self.refresh_maintenance_speakers)
        self.btn_maintenance_audio_version = QPushButton("获取语音版本模块")
        self.btn_maintenance_audio_version.clicked.connect(self.refresh_maintenance_audio_version)

        audio_row1 = QHBoxLayout()
        audio_row1.addWidget(QLabel("TTS 文本"))
        audio_row1.addWidget(self.maintenance_tts_text, 1)
        audio_row1.addWidget(self.maintenance_tts_play_checkbox)
        audio_row1.addWidget(self.btn_maintenance_tts)

        audio_row2 = QHBoxLayout()
        audio_row2.addWidget(self.btn_maintenance_mic_list)
        audio_row2.addWidget(self.btn_maintenance_speaker_list)
        audio_row2.addWidget(self.btn_maintenance_audio_version)
        audio_row2.addStretch(1)

        audio_layout.addWidget(QLabel("Audio 服务测试"))
        audio_layout.addLayout(audio_row1)
        audio_layout.addLayout(audio_row2)

        pressure_box = QWidget()
        pressure_layout = QVBoxLayout(pressure_box)
        pressure_layout.setContentsMargins(8, 8, 8, 8)
        pressure_layout.setSpacing(6)

        self.maintenance_pressure_status = QLineEdit("未启动")
        self.maintenance_pressure_status.setReadOnly(True)
        self.btn_maintenance_pressure_start = QPushButton("开始压力监听")
        self.btn_maintenance_pressure_start.clicked.connect(self.start_maintenance_pressure_monitor)
        self.btn_maintenance_pressure_stop = QPushButton("结束压力监听")
        self.btn_maintenance_pressure_stop.clicked.connect(self.stop_maintenance_pressure_monitor)
        self.btn_maintenance_pressure_stop.setEnabled(False)

        pressure_top = QHBoxLayout()
        pressure_top.addWidget(QLabel("手部压力"))
        pressure_top.addWidget(self.maintenance_pressure_status)
        pressure_top.addWidget(self.btn_maintenance_pressure_start)
        pressure_top.addWidget(self.btn_maintenance_pressure_stop)
        pressure_top.addStretch(1)

        meta_row = QHBoxLayout()
        self.maintenance_pressure_meta = {
            "left": QLineEdit("seq=- time=-"),
            "right": QLineEdit("seq=- time=-"),
        }
        self.maintenance_pressure_meta["left"].setReadOnly(True)
        self.maintenance_pressure_meta["right"].setReadOnly(True)
        meta_row.addWidget(QLabel("左手元信息"))
        meta_row.addWidget(self.maintenance_pressure_meta["left"])
        meta_row.addWidget(QLabel("右手元信息"))
        meta_row.addWidget(self.maintenance_pressure_meta["right"])

        pressure_grid = QGridLayout()
        pressure_grid.addWidget(QLabel("手指"), 0, 0)
        pressure_grid.addWidget(QLabel("左手压力"), 0, 1)
        pressure_grid.addWidget(QLabel("右手压力"), 0, 2)
        self.maintenance_pressure_fields = {"left": [], "right": []}
        for row, finger_name in enumerate(self._finger_labels_for_side(), start=1):
            left_field = QLineEdit("-")
            left_field.setReadOnly(True)
            right_field = QLineEdit("-")
            right_field.setReadOnly(True)
            self.maintenance_pressure_fields["left"].append(left_field)
            self.maintenance_pressure_fields["right"].append(right_field)
            pressure_grid.addWidget(QLabel(finger_name), row, 0)
            pressure_grid.addWidget(left_field, row, 1)
            pressure_grid.addWidget(right_field, row, 2)

        pressure_layout.addLayout(pressure_top)
        pressure_layout.addLayout(meta_row)
        pressure_layout.addLayout(pressure_grid)

        hand_box = QWidget()
        hand_layout = QVBoxLayout(hand_box)
        hand_layout.setContentsMargins(8, 8, 8, 8)
        hand_layout.setSpacing(6)

        self.maintenance_hand_target_combo = QComboBox()
        self.maintenance_hand_target_combo.addItem("双手", "dual")
        self.maintenance_hand_target_combo.addItem("左手", "left")
        self.maintenance_hand_target_combo.addItem("右手", "right")
        self.maintenance_hand_preset_combo = QComboBox()
        for preset_name, values in self._maintenance_finger_presets:
            self.maintenance_hand_preset_combo.addItem(f"{preset_name} ({len(values)})", preset_name)
        self.btn_maintenance_load_preset = QPushButton("载入预置")
        self.btn_maintenance_load_preset.clicked.connect(self.apply_maintenance_finger_preset)
        self.maintenance_hand_joint_values = QLineEdit()
        self.maintenance_hand_joint_values.setPlaceholderText("输入手指关节数组，例如 [0, 0.2, 1, 0.75, 1, 0]")
        self.btn_maintenance_send_hand = QPushButton("发送手指控制")
        self.btn_maintenance_send_hand.clicked.connect(self.send_maintenance_hand_joints)

        hand_row1 = QHBoxLayout()
        hand_row1.addWidget(QLabel("控制目标"))
        hand_row1.addWidget(self.maintenance_hand_target_combo)
        hand_row1.addWidget(QLabel("预置"))
        hand_row1.addWidget(self.maintenance_hand_preset_combo, 1)
        hand_row1.addWidget(self.btn_maintenance_load_preset)

        hand_row2 = QHBoxLayout()
        hand_row2.addWidget(QLabel("关节值"))
        hand_row2.addWidget(self.maintenance_hand_joint_values, 1)
        hand_row2.addWidget(self.btn_maintenance_send_hand)

        hand_layout.addWidget(QLabel("Hand 手指控制"))
        hand_layout.addLayout(hand_row1)
        hand_layout.addLayout(hand_row2)

        hand_body = QWidget()
        hand_body_layout = QVBoxLayout(hand_body)
        hand_body_layout.setContentsMargins(0, 0, 0, 0)
        hand_body_layout.setSpacing(8)
        hand_body_layout.addWidget(pressure_box)
        hand_body_layout.addWidget(hand_box)

        robot_box = QWidget()
        robot_layout = QVBoxLayout(robot_box)
        robot_layout.setContentsMargins(8, 8, 8, 8)
        robot_layout.setSpacing(6)

        self.btn_maintenance_robot_snapshot = QPushButton("检测 Robot 话题/服务一帧")
        self.btn_maintenance_robot_snapshot.clicked.connect(self.refresh_maintenance_robot_snapshot)

        robot_top = QHBoxLayout()
        robot_top.addWidget(self.btn_maintenance_robot_snapshot)
        robot_top.addStretch(1)

        robot_targets = QPlainTextEdit()
        robot_targets.setReadOnly(True)
        robot_targets.setFixedHeight(180)
        robot_targets.setPlainText(
            "Topics:\n"
            + "\n".join(self._maintenance_robot_topics)
            + "\n\nServices:\n"
            + "\n".join(service_name for _label, service_name in self._maintenance_robot_services)
        )

        robot_layout.addWidget(QLabel("Robot 快照检测"))
        robot_layout.addLayout(robot_top)
        robot_layout.addWidget(robot_targets)

        sensor_box = QWidget()
        sensor_layout = QVBoxLayout(sensor_box)
        sensor_layout.setContentsMargins(8, 8, 8, 8)
        sensor_layout.setSpacing(6)

        self.btn_maintenance_sensor_realsense_serials = QPushButton("显示 Realsense 序列号")
        self.btn_maintenance_sensor_realsense_serials.clicked.connect(self.refresh_maintenance_sensor_realsense_serials)

        sensor_top = QHBoxLayout()
        sensor_top.addWidget(self.btn_maintenance_sensor_realsense_serials)
        sensor_top.addStretch(1)

        sensor_desc = QLabel(
            "通过 SSH(大脑) 进入 ~/navi_project，先尝试 docker-compose exec sensor，再回退 docker compose exec sensor，"
            "在容器内执行 rs-enumerate-devices -S，并显示完整输出。"
        )
        sensor_desc.setWordWrap(True)

        sensor_layout.addWidget(QLabel("Sensor 检测"))
        sensor_layout.addLayout(sensor_top)
        sensor_layout.addWidget(sensor_desc)

        maintenance_brainsense_box = QWidget()
        maintenance_brainsense_layout = QVBoxLayout(maintenance_brainsense_box)
        maintenance_brainsense_layout.setContentsMargins(8, 8, 8, 8)
        maintenance_brainsense_layout.setSpacing(6)

        self.btn_maintenance_brainsense_gui = QPushButton("启动 BrainSense GUI")
        self.btn_maintenance_brainsense_gui.clicked.connect(self.launch_maintenance_brainsense_gui)
        self.btn_maintenance_stop_big_middleware = QPushButton("关闭中间件")
        self.btn_maintenance_stop_big_middleware.clicked.connect(self.maintenance_stop_big_brain_middleware)
        self.btn_maintenance_start_big_middleware = QPushButton("开启中间件")
        self.btn_maintenance_start_big_middleware.clicked.connect(self.maintenance_start_big_brain_middleware)

        maintenance_brainsense_row = QHBoxLayout()
        maintenance_brainsense_row.addWidget(self.btn_maintenance_brainsense_gui)
        maintenance_brainsense_row.addWidget(self.btn_maintenance_stop_big_middleware)
        maintenance_brainsense_row.addWidget(self.btn_maintenance_start_big_middleware)
        maintenance_brainsense_row.addStretch(1)

        maintenance_brainsense_desc = QLabel(
            "通过 SSH(大脑) 固定探测 BrainSense 的 launch.sh 并走本机 X11 回显；中间件控制则在 ~/navi_project 下优先尝试 docker-compose，再回退 docker compose。"
        )
        maintenance_brainsense_desc.setWordWrap(True)

        maintenance_brainsense_layout.addWidget(QLabel("BrainSense 与中间件控制"))
        maintenance_brainsense_layout.addLayout(maintenance_brainsense_row)
        maintenance_brainsense_layout.addWidget(maintenance_brainsense_desc)

        upperlimb_box = QWidget()
        upperlimb_layout = QVBoxLayout(upperlimb_box)
        upperlimb_layout.setContentsMargins(8, 8, 8, 8)
        upperlimb_layout.setSpacing(6)

        self.maintenance_upperlimb_loop_spin = QSpinBox()
        self.maintenance_upperlimb_loop_spin.setRange(1, 100)
        self.maintenance_upperlimb_loop_spin.setValue(1)

        self.btn_maintenance_upperlimb_test = QPushButton("执行wa2_ls上肢测试")
        self.btn_maintenance_upperlimb_test.clicked.connect(self.run_maintenance_upperlimb_test)

        upperlimb_row = QHBoxLayout()
        upperlimb_row.addWidget(QLabel("运行次数"))
        upperlimb_row.addWidget(self.maintenance_upperlimb_loop_spin)
        upperlimb_row.addWidget(self.btn_maintenance_upperlimb_test)
        upperlimb_row.addStretch(1)

        upperlimb_layout.addWidget(QLabel("UpperLimb 功能"))
        upperlimb_layout.addLayout(upperlimb_row)

        self.maintenance_wa1_joint_combo = QComboBox()
        self.maintenance_wa1_joint_combo.currentIndexChanged.connect(self._maintenance_on_wa1_joint_changed)
        self.maintenance_wa1_amp_spin = QDoubleSpinBox()
        self.maintenance_wa1_amp_spin.setRange(0.0001, 0.2)
        self.maintenance_wa1_amp_spin.setDecimals(4)
        self.maintenance_wa1_amp_spin.setSingleStep(0.0001)
        self.maintenance_wa1_amp_spin.setValue(0.0005)
        self.maintenance_wa1_hz_spin = QDoubleSpinBox()
        self.maintenance_wa1_hz_spin.setRange(10.0, 300.0)
        self.maintenance_wa1_hz_spin.setDecimals(0)
        self.maintenance_wa1_hz_spin.setSingleStep(10.0)
        self.maintenance_wa1_hz_spin.setValue(200.0)
        self.maintenance_wa1_duration_spin = QSpinBox()
        self.maintenance_wa1_duration_spin.setRange(10, 3600)
        self.maintenance_wa1_duration_spin.setValue(600)
        self.maintenance_wa1_err_thresh_spin = QDoubleSpinBox()
        self.maintenance_wa1_err_thresh_spin.setRange(0.01, 0.5)
        self.maintenance_wa1_err_thresh_spin.setDecimals(2)
        self.maintenance_wa1_err_thresh_spin.setSingleStep(0.01)
        self.maintenance_wa1_err_thresh_spin.setValue(0.08)
        self.maintenance_wa1_back_movej_v_spin = QDoubleSpinBox()
        self.maintenance_wa1_back_movej_v_spin.setRange(0.01, 1.0)
        self.maintenance_wa1_back_movej_v_spin.setDecimals(2)
        self.maintenance_wa1_back_movej_v_spin.setSingleStep(0.05)
        self.maintenance_wa1_back_movej_v_spin.setValue(0.2)
        self.maintenance_wa1_limit_label = QLabel("限位窗口: -")
        self.maintenance_wa1_status_label = QLabel("状态: 就绪")
        self.btn_maintenance_wa1_single_joint_test = QPushButton("执行WA1单关节测试")
        self.btn_maintenance_wa1_single_joint_test.clicked.connect(self.run_maintenance_wa1_single_joint_test)
        self.btn_maintenance_wa1_single_joint_stop = QPushButton("停止测试")
        self.btn_maintenance_wa1_single_joint_stop.setEnabled(False)
        self.btn_maintenance_wa1_single_joint_stop.clicked.connect(self.stop_maintenance_wa1_single_joint_test)

        wa1_row1 = QHBoxLayout()
        wa1_row1.addWidget(QLabel("关节"))
        wa1_row1.addWidget(self.maintenance_wa1_joint_combo, 1)
        wa1_row1.addWidget(QLabel("增量rad"))
        wa1_row1.addWidget(self.maintenance_wa1_amp_spin)
        wa1_row1.addWidget(QLabel("发布Hz"))
        wa1_row1.addWidget(self.maintenance_wa1_hz_spin)

        wa1_row2 = QHBoxLayout()
        wa1_row2.addWidget(QLabel("时长s"))
        wa1_row2.addWidget(self.maintenance_wa1_duration_spin)
        wa1_row2.addWidget(QLabel("误差阈值rad"))
        wa1_row2.addWidget(self.maintenance_wa1_err_thresh_spin)
        wa1_row2.addWidget(QLabel("回基线MoveJ v"))
        wa1_row2.addWidget(self.maintenance_wa1_back_movej_v_spin)
        wa1_row2.addWidget(self.btn_maintenance_wa1_single_joint_test)
        wa1_row2.addWidget(self.btn_maintenance_wa1_single_joint_stop)
        wa1_row2.addStretch(1)

        self.maintenance_wa1_single_joint_box = QWidget()
        wa1_box_layout = QVBoxLayout(self.maintenance_wa1_single_joint_box)
        wa1_box_layout.setContentsMargins(0, 0, 0, 0)
        wa1_box_layout.setSpacing(4)
        wa1_box_layout.addWidget(QLabel("WA1运维单关节往复测试（默认10分钟，结束自动MoveJ回基线）"))
        wa1_box_layout.addLayout(wa1_row1)
        wa1_box_layout.addLayout(wa1_row2)
        wa1_box_layout.addWidget(self.maintenance_wa1_limit_label)
        wa1_box_layout.addWidget(self.maintenance_wa1_status_label)

        upperlimb_layout.addWidget(self.maintenance_wa1_single_joint_box)
        self._maintenance_refresh_wa1_joint_options()
        self.maintenance_wa1_single_joint_box.setVisible(str(getattr(self, "robot_model", "WA2") or "WA2").upper() == "WA1")

        maintenance_bottom = QHBoxLayout()
        maintenance_bottom.addWidget(QLabel("测试结果"))
        maintenance_bottom.addStretch(1)
        self.btn_show_maintenance_result = QPushButton("显示测试窗口")
        self.btn_show_maintenance_result.clicked.connect(self._show_maintenance_result_window)
        maintenance_bottom.addWidget(self.btn_show_maintenance_result)
        self.btn_maintenance_clear = QPushButton("清空输出")
        self.btn_maintenance_clear.clicked.connect(lambda: self.maintenance_clear_signal.emit())
        maintenance_bottom.addWidget(self.btn_maintenance_clear)

        maintenance_note = QLabel("Audio/Hand/Robot/Sensor 功能按分区折叠显示，测试输出在独立结果窗口中查看。")
        maintenance_note.setWordWrap(True)

        audio_section, self.btn_toggle_maintenance_audio = self._build_collapsible_section(
            "Audio 功能", audio_box, expanded=True
        )
        hand_section, self.btn_toggle_maintenance_hand = self._build_collapsible_section(
            "Hand 功能", hand_body, expanded=True
        )
        maintenance_brainsense_section, self.btn_toggle_maintenance_brainsense = self._build_collapsible_section(
            "BrainSense与中间件控制", maintenance_brainsense_box, expanded=True
        )
        upperlimb_section, self.btn_toggle_maintenance_upperlimb = self._build_collapsible_section(
            "UpperLimb 功能", upperlimb_box, expanded=True
        )
        robot_section, self.btn_toggle_maintenance_robot = self._build_collapsible_section(
            "Robot 功能", robot_box, expanded=True
        )
        sensor_section, self.btn_toggle_maintenance_sensor = self._build_collapsible_section(
            "Sensor 功能", sensor_box, expanded=True
        )

        maintenance_layout.addWidget(maintenance_note)
        maintenance_layout.addWidget(audio_section)
        maintenance_layout.addWidget(hand_section)
        maintenance_layout.addWidget(maintenance_brainsense_section)
        maintenance_layout.addWidget(upperlimb_section)
        maintenance_layout.addWidget(robot_section)
        maintenance_layout.addWidget(sensor_section)
        maintenance_layout.addLayout(maintenance_bottom)
        maintenance_layout.addStretch(1)
        maintenance_scroll.setWidget(maintenance_panel)
        maintenance_tab_layout.addWidget(maintenance_scroll)
        self._set_maintenance_pressure_state(False, "未启动")

        # ---------- Tab 10: 日志排查test ----------
        tab_log_audit = QWidget()
        log_audit_layout = QVBoxLayout(tab_log_audit)

        self.log_audit_mode_combo = QComboBox()
        self.log_audit_mode_combo.addItem("错误驱动式", "error")
        self.log_audit_mode_combo.addItem("指定时段", "range")
        self.log_audit_mode_combo.addItem("就近时间", "near")

        self.log_audit_app_root_edit = QLineEdit("/home/naviai/source/naviai_manip_retail")
        self.log_audit_app_logs_dir_edit = QLineEdit("/home/naviai/source/naviai_manip_retail/logs")
        self.log_audit_target_file_edit = QLineEdit("pick_log_detail.jsonl,place_log_detail.jsonl")
        self.btn_log_audit_browse_app = QPushButton("浏览应用日志")
        self.btn_log_audit_browse_app.clicked.connect(self.browse_log_audit_app_file)
        self.log_audit_keywords_edit = QLineEdit("step3_search,step65_tactile_check")
        self.log_audit_start_time_edit = QLineEdit(datetime.now().strftime("%Y-%m-%d 00:00:00"))
        self.log_audit_end_time_edit = QLineEdit(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        self.log_audit_ref_time_edit = QLineEdit(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        self.log_audit_window_minutes_edit = QLineEdit("10")
        self.log_audit_recent_lines_edit = QLineEdit("200")
        self.log_audit_journal_cmd_edit = QLineEdit("journalctl -u zj_humanoid.service")
        self.log_audit_ubuntu_cmd_edit = QLineEdit("journalctl")

        cfg_grid = QGridLayout()
        cfg_grid.addWidget(QLabel("排查模式"), 0, 0)
        cfg_grid.addWidget(self.log_audit_mode_combo, 0, 1)

        cfg_grid.addWidget(QLabel("待核查jsonl文件"), 1, 0)
        cfg_grid.addWidget(self.log_audit_target_file_edit, 1, 1)
        cfg_grid.addWidget(self.btn_log_audit_browse_app, 1, 2)

        cfg_grid.addWidget(QLabel("失败步骤/关键字(可选)"), 1, 3)
        cfg_grid.addWidget(self.log_audit_keywords_edit, 1, 4)

        cfg_grid.addWidget(QLabel("开始时间"), 2, 0)
        cfg_grid.addWidget(self.log_audit_start_time_edit, 2, 1)
        cfg_grid.addWidget(QLabel("结束时间"), 2, 2)
        cfg_grid.addWidget(self.log_audit_end_time_edit, 2, 3)

        cfg_grid.addWidget(QLabel("参考时间"), 3, 0)
        cfg_grid.addWidget(self.log_audit_ref_time_edit, 3, 1)
        cfg_grid.addWidget(QLabel("窗口分钟"), 3, 2)
        cfg_grid.addWidget(self.log_audit_window_minutes_edit, 3, 3)

        cfg_grid.addWidget(QLabel("预览行数"), 4, 0)
        cfg_grid.addWidget(self.log_audit_recent_lines_edit, 4, 1)

        action_row = QHBoxLayout()
        self.btn_log_audit_run = QPushButton("执行日志排查")
        self.btn_log_audit_run.clicked.connect(self.run_log_audit_test)
        self.btn_log_audit_preview_middleware = QPushButton("查看中间件最新日志")
        self.btn_log_audit_preview_middleware.clicked.connect(self.preview_log_audit_middleware)
        self.btn_log_audit_preview_ubuntu = QPushButton("查看Ubuntu系统最新日志")
        self.btn_log_audit_preview_ubuntu.clicked.connect(self.preview_log_audit_ubuntu)
        action_row.addWidget(self.btn_log_audit_run)
        action_row.addWidget(self.btn_log_audit_preview_middleware)
        action_row.addWidget(self.btn_log_audit_preview_ubuntu)
        self.btn_log_audit_clear = QPushButton("清空输出")
        self.btn_log_audit_clear.clicked.connect(lambda: self.log_audit_clear_signal.emit())
        action_row.addWidget(self.btn_log_audit_clear)
        action_row.addStretch(1)

        self.log_audit_output = QPlainTextEdit()
        self.log_audit_output.setReadOnly(True)
        self.log_audit_output.setPlaceholderText(
            "这里显示应用层jsonl失败事件、中间件日志和Ubuntu系统日志的测试版排查结果。\n"
            "错误驱动模式会扫描 logs 目录下的 jsonl，按 success=false 触发事件并联动切片保存。"
        )

        log_audit_layout.addLayout(cfg_grid)
        log_audit_layout.addLayout(action_row)
        log_audit_layout.addWidget(self.log_audit_output)

        # ---------- Tab 11: 命令 ----------
        tab_cmd = QWidget()
        cmd_layout = QVBoxLayout()

        self.cmd_edit = QLineEdit()
        self.cmd_edit.setPlaceholderText("rosservice call /xxx '{}'  |  rostopic echo /xxx")
        self.btn_run_cmd = QPushButton("执行命令")
        self.btn_run_cmd.clicked.connect(self.run_command)
        self.btn_stop_cmd = QPushButton("停止输出")
        self.btn_stop_cmd.setEnabled(False)
        self.btn_stop_cmd.clicked.connect(self.stop_command_echo)
        self.btn_refresh_ros_lists = QPushButton("刷新话题/服务列表")
        self.btn_refresh_ros_lists.clicked.connect(self.refresh_ros_lists)

        cmd_line = QHBoxLayout()
        cmd_line.addWidget(self.cmd_edit)
        cmd_line.addWidget(self.btn_run_cmd)
        cmd_line.addWidget(self.btn_stop_cmd)
        cmd_line.addWidget(self.btn_refresh_ros_lists)

        list_line = QHBoxLayout()
        self.topic_list = QListWidget()
        self.service_list = QListWidget()
        self.topic_list.itemDoubleClicked.connect(lambda item: self._fill_topic_command_template(item.text() if item else ""))
        self.service_list.itemDoubleClicked.connect(lambda item: self._fill_service_command_template(item.text() if item else ""))

        topic_col = QVBoxLayout()
        topic_col.addWidget(QLabel("rostopic list"))
        topic_col.addWidget(self.topic_list)

        service_col = QVBoxLayout()
        service_col.addWidget(QLabel("rosservice list"))
        service_col.addWidget(self.service_list)

        list_line.addLayout(topic_col)
        list_line.addLayout(service_col)

        cmd_layout.addLayout(cmd_line)
        cmd_layout.addLayout(list_line)
        tab_cmd.setLayout(cmd_layout)
        self._setup_command_completer()

        tab_help = self._build_help_tab()

        # tabs
        self.tab_conn = tab_conn
        self.tab_robot_status = tab_robot_status
        self.tab_transfer = tab_transfer
        self.tab_ros = tab_ros
        self.tab_joint_ctrl = tab_joint_ctrl
        self.tab_nav = tab_nav
        self.tab_container = tab_container
        self.tab_humanode_demo = tab_humanode_demo
        self.tab_factory_test = tab_factory_test
        self.tab_huiyang = tab_huiyang
        self.tab_servoj_tool = tab_servoj_tool
        self.tab_replay_tools = tab_replay_tools
        self.tab_vision = tab_vision
        self.tab_pose = tab_pose
        self.tab_maintenance = tab_maintenance
        self.tab_log_audit = tab_log_audit
        self.tab_cmd = tab_cmd
        self.tab_help = tab_help
        self.tabs.addTab(tab_conn, "连接")
        self.tabs.addTab(tab_robot_status, "状态监控")
        self.tabs.addTab(tab_transfer, "功能栏")
        self.tabs.addTab(tab_ros, "关节监控")
        self.tabs.addTab(tab_joint_ctrl, "关节控制")
        self.tabs.addTab(tab_nav, "导航")
        self.tabs.addTab(tab_humanode_demo, "测试Demo")
        self.tabs.addTab(tab_factory_test, "工厂测试")
        self.tabs.addTab(tab_replay_tools, "回放工具")
        self.tabs.addTab(tab_vision, "视觉监控")
        self.tabs.addTab(tab_pose, "位姿估计")
        self.tabs.addTab(tab_maintenance, "运维测试")
        self.tabs.addTab(tab_log_audit, "日志排查")
        self.tabs.addTab(tab_cmd, "命令")
        self.tabs.addTab(tab_help, "帮助")

        # 全局日志输出区（独立窗口显示）
        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.setMinimumHeight(320)
        self.btn_clear_log = QPushButton("清空日志")
        self.btn_clear_log.clicked.connect(self.clear_log)

        self.conn_quality_label = QLabel("连接质量: -")
        self.conn_quality_label.setStyleSheet("color: #bbb;")
        self.app_version_label = QLabel(f"界面版本: {APP_DISPLAY_VERSION}")
        self.app_version_label.setStyleSheet("color: #666;")
        self.btn_help_center = QPushButton("帮助")
        self.btn_help_center.clicked.connect(self._show_help_tab)
        self.btn_active_disconnect = QPushButton("主动断开")
        self.btn_active_disconnect.clicked.connect(self.active_disconnect_all)
        self.btn_show_log_window = QPushButton("显示日志窗口")
        self.btn_show_log_window.clicked.connect(self._show_log_window)

        self._create_log_window()
        self._create_system_info_window()
        self._create_feature_config_window()
        self._create_humanode_demo_result_window()
        self._create_maintenance_result_window()

        top_bar = QHBoxLayout()
        top_bar.addWidget(self.conn_quality_label)
        top_bar.addStretch(1)
        top_bar.addWidget(self.app_version_label)
        top_bar.addWidget(self.btn_help_center)
        top_bar.addWidget(self.btn_active_disconnect)
        top_bar.addWidget(self.btn_show_log_window)

        root = QVBoxLayout()
        root.addLayout(top_bar)
        root.addWidget(self.tabs)
        self.setLayout(root)

        self._ensure_log_panel_visible()

    def _build_help_tab(self):
        tab_help = QWidget()
        help_layout = QVBoxLayout()

        help_title = QLabel(f"使用帮助 | {APP_DISPLAY_VERSION}")
        help_text = QTextEdit()
        help_text.setReadOnly(True)
        help_text.setPlainText(
            "连接\n"
            "1. 在连接页填写 ROSBridge、SSH(小脑)、SSH(大脑) 地址后，先建立连接。\n"
            "2. 连接成功后再进入其他模块，避免读取状态和调用服务失败。\n\n"
            "状态监控\n"
            "1. 查看电池、机器人状态、Orin/Pico 资源、基础信息。\n"
            "2. 需要版本排查时，优先使用版本检测区。\n\n"
            "功能栏\n"
            "1. 用于大小脑文件上传下载、文件夹同步和升级包部署。\n"
            "2. 部署中间件升级包时，先选择本地 .firmware，再确认 robot_type 与是否强制重装。\n\n"
            "关节监控\n"
            "1. 选择机器人型号后查看关节角度和绘图。\n"
            "2. 适合做状态确认、录制前检查和异常排查。\n\n"
            "关节控制\n"
            "1. 先进入示教模式，再执行 MoveJ、MoveL、动作序列等操作。\n"
            "2. MoveJ 测试直接输入 [] 数组；执行前确认 arm_type、速度、加速度和时间参数。\n"
            "3. 动作序列支持记录当前姿态、单步调试、循环执行与导出。\n\n"
            "导航\n"
            "1. 用于开始建图、建图后处理和感知容器重启。\n"
            "2. 执行前确认导航相关容器和 ROS 服务可用。\n\n"
            "容器操作\n"
            "1. 填写 service 与 version 后执行部署、重启、帮助等脚本动作。\n"
            "2. 建议先看日志输出，再批量部署。\n\n"
            "测试Demo\n"
            "1. 用于运行现场测试用例和查看 Demo 输出结果。\n"
            "2. 执行前确认所需测试资源已同步到目标机器。\n\n"
            "视觉监控\n"
            "1. 选择图像 topic 后查看图像与频率。\n"
            "2. 适合检查相机链路、画面更新和 topic 发布状态。\n\n"
            "位姿估计\n"
            "1. 用于持续拉取识别结果图片并观察更新。\n"
            "2. 适合验证可乐、水等目标识别是否正常。\n\n"
            "运维测试\n"
            "1. 提供压力、语音版本、WiFi 列表、机器人状态等运维检查入口。\n"
            "2. 排障时建议先看这里，再决定是否抓日志或重启模块。\n\n"
            "命令\n"
            "1. 支持 rosservice call、rostopic echo 等命令补全与快捷填充。\n"
            "2. 用于临时调试时，优先复用自动补全模板，减少手工拼命令。"
        )
        help_layout.addWidget(help_title)
        help_layout.addWidget(help_text)
        tab_help.setLayout(help_layout)
        return tab_help

    def _show_help_tab(self):
        if hasattr(self, "tabs") and hasattr(self, "tab_help"):
            self.tabs.setCurrentWidget(self.tab_help)

    def _setup_command_completer(self):
        if not hasattr(self, "cmd_edit"):
            return
        self._cmd_completion_model = QStringListModel(self)
        self._cmd_completer = QCompleter(self._cmd_completion_model, self)
        self._cmd_completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self._cmd_completer.setFilterMode(Qt.MatchFlag.MatchContains)
        self._cmd_completer.setCompletionMode(QCompleter.CompletionMode.PopupCompletion)
        self.cmd_edit.setCompleter(self._cmd_completer)
        self.cmd_edit.installEventFilter(self)
        self._rebuild_command_completions()

    def eventFilter(self, obj, event):
        try:
            if hasattr(self, "cmd_edit") and obj is self.cmd_edit and event.type() == event.Type.KeyPress:
                if event.key() == Qt.Key.Key_Tab:
                    self._apply_tab_completion()
                    return True
        except Exception:
            pass
        return super().eventFilter(obj, event)

    def _rebuild_command_completions(self):
        entries = [
            "rosservice call ",
            "rostopic echo ",
            "rostopic echo --once ",
        ]
        for s in self._cmd_services:
            entries.append(f"rosservice call {s} '{{}}'")
            entries.append(f"rosservice call {s} ")
        for t in self._cmd_topics:
            entries.append(f"rostopic echo {t} --once")
            entries.append(f"rostopic echo {t}")
        self._cmd_completion_entries = sorted({x for x in entries if isinstance(x, str) and x.strip()})
        if hasattr(self, "_cmd_completion_model"):
            self._cmd_completion_model.setStringList(self._cmd_completion_entries)

    def _candidate_vision_topics(self, topics) -> list:
        topic_list = []
        for topic in (topics or []):
            text = str(topic or "").strip()
            if not text.startswith("/"):
                continue
            lower = text.lower()
            if "camera_info" in lower:
                continue

            # 未连接 ROSBridge 时，只保留命名上明显是原始图像流的话题，避免把 compressed 等不可解析格式加进来。
            name_matches = (
                "/image_raw" in lower
                or "/image_rect_raw" in lower
                or "/aligned_depth_to_color/image_raw" in lower
            )
            if not name_matches:
                continue

            if self.ros and self.ros.check_connection():
                try:
                    topic_type = str(self.ros._get_topic_type(text) or "").strip()
                except Exception:
                    topic_type = ""
                if topic_type != "sensor_msgs/Image":
                    continue

            topic_list.append(text)
        return sorted(dict.fromkeys(topic_list))

    def _default_vision_topics(self) -> list:
        defaults = []
        for _group_name, items in getattr(self, "vision_rate_groups", []):
            for item_name, topic in items:
                if item_name in ("color", "depth", "aligned"):
                    defaults.append(str(topic))
        return sorted(dict.fromkeys(defaults))

    def _refresh_vision_topic_options(self, topics):
        if not hasattr(self, "vision_topic_edit"):
            return
        current = self.vision_topic_edit.currentText().strip()
        candidates = self._candidate_vision_topics(topics)
        if not candidates:
            candidates = self._default_vision_topics()
        self.vision_topic_edit.blockSignals(True)
        self.vision_topic_edit.clear()
        for topic in candidates:
            self.vision_topic_edit.addItem(topic)
        if candidates:
            if current and current in candidates:
                self.vision_topic_edit.setCurrentText(current)
            else:
                self.vision_topic_edit.setCurrentIndex(0)
        self.vision_topic_edit.blockSignals(False)

    def _ros_field_default_value_text(self, field_type: str) -> str:
        ft = str(field_type or "").strip()
        is_array = ft.endswith("[]")
        base = ft[:-2] if is_array else ft
        if is_array:
            return "[]"

        mapping = {
            "bool": "false",
            "int8": "0",
            "uint8": "0",
            "int16": "0",
            "uint16": "0",
            "int32": "0",
            "uint32": "0",
            "int64": "0",
            "uint64": "0",
            "float32": "0.0",
            "float64": "0.0",
            "string": '""',
            "time": "{secs: 0, nsecs: 0}",
            "duration": "{secs: 0, nsecs: 0}",
        }
        if base in mapping:
            return mapping[base]
        return "{}"

    def _render_ros_fields_to_yaml_inline(self, fields: list) -> str:
        if not fields:
            return "{}"
        parts = []
        for f in fields:
            name = str(f.get("name") or "").strip()
            ftype = str(f.get("type") or "").strip()
            if not name or not ftype:
                continue
            parts.append(f"{name}: {self._ros_field_default_value_text(ftype)}")
        if not parts:
            return "{}"
        return "{" + ", ".join(parts) + "}"

    def _parse_srv_request_fields_from_rossrv_show(self, text: str) -> list:
        raw = (text or "").strip()
        if not raw:
            return []
        req_part = raw.split("---", 1)[0]
        fields = []
        for ln in req_part.splitlines():
            s = ln.split("#", 1)[0].strip()
            if not s:
                continue
            if "=" in s:
                # 常量定义，不是请求字段
                continue
            cols = s.split()
            if len(cols) < 2:
                continue
            ftype, fname = cols[0].strip(), cols[1].strip()
            if not ftype or not fname:
                continue
            fields.append({"name": fname, "type": ftype})
        return fields

    def _resolve_service_request_fields(self, service_name: str) -> list:
        service = str(service_name or "").strip()
        if not service:
            return []

        # 1) ROSBridge + rosapi
        if self.ros and self.ros.check_connection():
            try:
                resp = self.ros.request_service("/rosapi/service_request_details", {"service": service}, timeout=4.0)
                if isinstance(resp, dict):
                    typedefs = resp.get("typedefs")
                    if isinstance(typedefs, list) and typedefs:
                        root = typedefs[0]
                        names = root.get("fieldnames") or []
                        types = root.get("fieldtypes") or []
                        if isinstance(names, list) and isinstance(types, list) and names and types:
                            fields = []
                            for n, t in zip(names, types):
                                fields.append({"name": str(n), "type": str(t)})
                            return fields
            except Exception:
                pass

            try:
                type_resp = self.ros.request_service("/rosapi/service_type", {"service": service}, timeout=3.0)
                srv_type = str((type_resp or {}).get("type") or "").strip()
                if srv_type:
                    detail_resp = self.ros.request_service("/rosapi/service_request_details", {"type": srv_type}, timeout=4.0)
                    if isinstance(detail_resp, dict):
                        typedefs = detail_resp.get("typedefs")
                        if isinstance(typedefs, list) and typedefs:
                            root = typedefs[0]
                            names = root.get("fieldnames") or []
                            types = root.get("fieldtypes") or []
                            if isinstance(names, list) and isinstance(types, list) and names and types:
                                fields = []
                                for n, t in zip(names, types):
                                    fields.append({"name": str(n), "type": str(t)})
                                return fields
            except Exception:
                pass

        # 2) SSH 兜底: rosservice type + rossrv show
        if self.ssh and self.ssh.ssh and self.ssh.sftp:
            try:
                out_type, err_type = self._run_ros_cli_via_ssh(f"rosservice type {shlex.quote(service)}")
                srv_type = (out_type or "").strip() or (err_type or "").strip()
                srv_type = srv_type.splitlines()[0].strip() if srv_type else ""
                if srv_type and "/" in srv_type:
                    out_show, err_show = self._run_ros_cli_via_ssh(f"rossrv show {shlex.quote(srv_type)}")
                    text_show = ((out_show or "") + "\n" + (err_show or "")).strip()
                    fields = self._parse_srv_request_fields_from_rossrv_show(text_show)
                    if fields:
                        return fields
            except Exception:
                pass

        return []

    def _build_service_request_template_text(self, service_name: str) -> str:
        service = str(service_name or "").strip()
        if not service:
            return "'{}'"
        cached = self._service_req_template_cache.get(service)
        if cached:
            return cached
        fields = self._resolve_service_request_fields(service)
        inline_yaml = self._render_ros_fields_to_yaml_inline(fields)
        templ = f"'{inline_yaml}'"
        self._service_req_template_cache[service] = templ
        return templ

    def _apply_tab_completion(self):
        if not hasattr(self, "cmd_edit"):
            return
        text = self.cmd_edit.text().strip()
        if not text:
            return

        svc_exact = re.match(r"^rosservice\s+call\s+(/\S+)$", text)
        if svc_exact:
            svc = svc_exact.group(1).strip()
            if svc in self._cmd_services:
                self._fill_service_command_template(svc)
                return

        candidates = [x for x in self._cmd_completion_entries if x.startswith(text)]
        if not candidates:
            self.log_signal.emit("[INFO] 无可用补全")
            return
        if len(candidates) == 1:
            self.cmd_edit.setText(candidates[0])
            self.log_signal.emit(f"[OK] 已补全: {candidates[0]}")
            return

        common = os.path.commonprefix(candidates)
        if common and len(common) > len(text):
            self.cmd_edit.setText(common)
            self.log_signal.emit(f"[INFO] 已补全公共前缀 ({len(candidates)}项)")
        else:
            preview = " | ".join(candidates[:6])
            if len(candidates) > 6:
                preview += f" | ... 共{len(candidates)}项"
            self.log_signal.emit(f"[INFO] 补全候选: {preview}")

    def _fill_service_command_template(self, service_name: str):
        service = (service_name or "").strip()
        if not service:
            return
        req_templ = self._build_service_request_template_text(service)
        self.cmd_edit.setText(f"rosservice call {service} {req_templ}")
        self.cmd_edit.setFocus()
        self.log_signal.emit(f"[OK] 已填充服务模板: {service}")

    def _fill_topic_command_template(self, topic_name: str):
        topic = (topic_name or "").strip()
        if not topic:
            return
        self.cmd_edit.setText(f"rostopic echo {topic} --once")
        self.cmd_edit.setFocus()
        self.log_signal.emit(f"[OK] 已填充话题模板: {topic}")

    def _log_text_color(self, text: str) -> str:
        raw = str(text or "")
        if "[中间件-大脑线程]" in raw:
            return "#1f6feb"
        if "[中间件-小脑线程]" in raw:
            return "#2da44e"
        if "[ERR]" in raw:
            return "#f85149"
        if "[WARN]" in raw:
            return "#d29922"
        return "#c9d1d9"

    def _emit_middleware_thread_log(self, target_name: str, text: str):
        self.log_signal.emit(f"[中间件-{target_name}线程] {text}")

    def _append_log(self, text: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        clean = ANSI_ESCAPE_RE.sub("", str(text or ""))
        if len(clean) > 4000:
            clean = f"{clean[:4000]} ... [truncated, total={len(str(text or ''))} chars]"
        raw = f"[{ts}] {clean}"
        color = self._log_text_color(raw)
        escaped = html.escape(raw).replace("\n", "<br>")
        self.output.append(f'<span style="color: {color};">{escaped}</span>')

    def _summarize_movej_path_request(self, req: dict) -> str:
        try:
            if not isinstance(req, dict):
                return str(req)
            path = list(req.get("path") or [])
            points = len(path)
            first_joint = []
            last_joint = []
            if points > 0 and isinstance(path[0], dict):
                first_joint = list(path[0].get("joint") or [])
            if points > 1 and isinstance(path[-1], dict):
                last_joint = list(path[-1].get("joint") or [])
            dims = len(first_joint) if first_joint else (len(last_joint) if last_joint else 0)
            preview_len = 4

            def _fmt(vals: list) -> list:
                out = []
                for v in vals[:preview_len]:
                    try:
                        out.append(round(float(v), 4))
                    except Exception:
                        out.append(v)
                return out

            summary = {
                "arm_type": req.get("arm_type"),
                "is_async": req.get("is_async"),
                "time": req.get("time"),
                "path_points": points,
                "joint_dims": dims,
                "first_joint_preview": _fmt(first_joint),
            }
            if points > 1:
                summary["last_joint_preview"] = _fmt(last_joint)
            return json.dumps(summary, ensure_ascii=False)
        except Exception:
            text = json.dumps(req, ensure_ascii=False) if isinstance(req, dict) else str(req)
            return text if len(text) <= 800 else (text[:800] + " ...")

    def _append_humanode_demo_output(self, text: str):
        clean = ANSI_ESCAPE_RE.sub("", str(text or "")).replace("\r", "")
        if hasattr(self, "humanode_demo_output") and self.humanode_demo_output is not None:
            self.humanode_demo_output.appendPlainText(clean)
        if hasattr(self, "humanode_demo_result_text") and self.humanode_demo_result_text is not None:
            self.humanode_demo_result_text.appendPlainText(clean)

    def _append_factory_test_output(self, text: str):
        clean = ANSI_ESCAPE_RE.sub("", str(text or "")).replace("\r", "")
        if hasattr(self, "factory_test_output") and self.factory_test_output is not None:
            self.factory_test_output.appendPlainText(clean)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._factory_report_records.append({"time": ts, "message": clean})

    def _factory_set_topic_service_audit(self, audit: dict):
        data = audit if isinstance(audit, dict) else {}
        merged = {
            "topics": list(data.get("topics") or []),
            "summary": str(data.get("summary") or "-"),
            "topic_count": int(data.get("topic_count") or 0),
        }
        self._factory_topic_service_audit = merged
        if hasattr(self, "factory_topic_service_audit_text"):
            self.factory_topic_service_audit_text.setPlainText(merged["summary"])

    def _format_resp_text(self, resp) -> str:
        if isinstance(resp, dict):
            return json.dumps(resp, ensure_ascii=False)
        if hasattr(resp, "success") or hasattr(resp, "message"):
            try:
                success = getattr(resp, "success", None)
                message = getattr(resp, "message", None)
                return f"success={success}, message={message}"
            except Exception:
                pass
        return str(resp)

    def clear_log(self):
        self.output.clear()

    def _set_connect_button_state(self, which: str, enabled: bool, text: str):
        if which == "ssh":
            self.btn_ssh.setEnabled(enabled)
            self.btn_ssh.setText(text)
            self._ssh_connecting = not enabled
            if not enabled:
                self.status_signal.emit("ssh", "连接中")
        elif which == "ssh_big":
            self.btn_ssh_big.setEnabled(enabled)
            self.btn_ssh_big.setText(text)
            self._ssh_big_connecting = not enabled
            if not enabled:
                self.status_signal.emit("ssh_big", "连接中")
        elif which == "ros":
            self.btn_ros.setEnabled(enabled)
            self.btn_ros.setText(text)
            self._ros_connecting = not enabled
            if not enabled:
                self.status_signal.emit("ros", "连接中")

    def _set_status_text(self, which: str, text: str):
        if which == "ros":
            self.ros_status_value.setText(text)
        elif which == "ssh":
            self.ssh_status_value.setText(text)
        elif which == "ssh_big":
            self.ssh_big_status_value.setText(text)
            if text == "已连接":
                QTimer.singleShot(0, self.refresh_container_runtime_list)
            else:
                self._clear_container_runtime_list_state()
        self._refresh_robot_status_view()
        self._update_connection_quality()

    def _set_robot_battery_text(self, text: str):
        self.robot_battery_value.setText(text)
        self._refresh_robot_status_view()

    def _set_joint_monitor_health(self, status_text: str, last_update_text: str):
        if hasattr(self, "joint_monitor_status_value"):
            self.joint_monitor_status_value.setText(str(status_text or "-"))
        if hasattr(self, "joint_monitor_last_update_value"):
            self.joint_monitor_last_update_value.setText(str(last_update_text or "-"))

    def _set_robot_status_health(self, status_text: str):
        if hasattr(self, "robot_status_health_value"):
            self.robot_status_health_value.setText(str(status_text or "-"))

    def _set_safety_lock_view(self, text: str, style: str):
        if not hasattr(self, "safety_lock_status_label"):
            return
        self.safety_lock_status_label.setText(str(text or "安全保护: -"))
        self.safety_lock_status_label.setStyleSheet(str(style or "color: #c9d1d9; font-weight: 600;"))

    def _infer_robot_model_from_version(self, robot_version: str) -> str:
        text = str(robot_version or "").strip().upper()
        if not text:
            return ""
        if text.startswith("WA1"):
            return "WA1"
        if text.startswith("I"):
            return "I2"
        if text.startswith("WA2_LS"):
            return "WA2_LS"
        if text.startswith("WA2"):
            return "WA2"
        return ""

    def _is_i_series_model(self, model: str | None = None) -> bool:
        text = str(model or getattr(self, "robot_model", "") or "").strip().upper()
        return bool(text) and text.startswith("I")

    def _factory_model_has_lowerlimb(self, model: str | None = None) -> bool:
        return self._is_i_series_model(model or self._factory_current_model())

    def _factory_camera_slot_labels(self, model: str | None = None) -> dict:
        if self._is_i_series_model(model or self._factory_current_model()):
            return {"head": "头部相机", "chest": "腹部相机"}
        return {"head": "头部相机", "chest": "胸部相机"}

    def _factory_refresh_camera_labels(self):
        labels = self._factory_camera_slot_labels()
        chest_text = labels["chest"]
        if hasattr(self, "factory_camera_head_required"):
            self.factory_camera_head_required.setText(f"{labels['head']}必须检测")
        if hasattr(self, "factory_camera_chest_required"):
            self.factory_camera_chest_required.setText(f"{chest_text}必须检测")
        if hasattr(self, "factory_camera_head_topic_label"):
            self.factory_camera_head_topic_label.setText(f"{labels['head']}话题")
        if hasattr(self, "factory_camera_chest_topic_label"):
            self.factory_camera_chest_topic_label.setText(f"{chest_text}话题")
        if hasattr(self, "factory_camera_head_preview_title"):
            self.factory_camera_head_preview_title.setText(labels["head"])
        if hasattr(self, "factory_camera_chest_preview_title"):
            self.factory_camera_chest_preview_title.setText(chest_text)
        if hasattr(self, "factory_camera_head_preview") and self.factory_camera_head_preview.pixmap() is None:
            self.factory_camera_head_preview.setText(f"{labels['head']}等待画面")
        if hasattr(self, "factory_camera_chest_preview") and self.factory_camera_chest_preview.pixmap() is None:
            self.factory_camera_chest_preview.setText(f"{chest_text}等待画面")

    def _set_robot_basic_info(self, data: dict):
        robot_version = str(data.get("robot_version") or "-")
        self.robot_version_value.setText(robot_version)
        inferred_model = self._infer_robot_model_from_version(robot_version)
        if inferred_model:
            self._set_robot_model(inferred_model)
        self.hardware_version_value.setText(str(data.get("embedded_version") or data.get("hardware_version") or "-"))
        self.software_version_value.setText(str(data.get("software_version") or "-"))
        if hasattr(self, "upperlimb_version_value"):
            self.upperlimb_version_value.setText(str(data.get("upperlimb_version") or "-"))
        show_lowerlimb = bool(data.get("show_lowerlimb", False))
        if hasattr(self, "lowerlimb_version_label"):
            self.lowerlimb_version_label.setVisible(show_lowerlimb)
        if hasattr(self, "lowerlimb_version_value"):
            self.lowerlimb_version_value.setVisible(show_lowerlimb)
            self.lowerlimb_version_value.setText(str(data.get("lowerlimb_version") or "-"))
        if hasattr(self, "factory_info_model_value"):
            self._factory_refresh_info_snapshot_fields()
        if hasattr(self, "factory_camera_head_required"):
            self._factory_refresh_camera_labels()

    def _set_robot_system_info(self, data: dict):
        if not hasattr(self, "system_info_text"):
            return
        lines = [
            f"[小脑 Ubuntu] {str(data.get('small_ubuntu') or '-')}",
            f"[小脑 系统] {str(data.get('small_system') or '-')}",
            "",
            f"[大脑 Ubuntu] {str(data.get('big_ubuntu') or '-')}",
            f"[大脑 JetPack] {str(data.get('big_jetpack') or '-')}",
        ]
        self.system_info_text.setPlainText("\n".join(lines).strip())
        if hasattr(self, "factory_info_system_text"):
            self._factory_refresh_info_snapshot_fields()

    def _emit_version(self, text: str):
        self.log_signal.emit(text)
        self.version_item_signal.emit(text)

    def _emit_peripheral(self, text: str):
        self.log_signal.emit(text)
        self.peripheral_item_signal.emit(text)

    def _emit_container(self, text: str):
        self.log_signal.emit(text)
        self.container_item_signal.emit(text)

    def _emit_humanode_demo(self, text: str):
        self.log_signal.emit(text)
        self.humanode_demo_item_signal.emit(text)

    def _emit_factory_test(self, text: str):
        self.log_signal.emit(text)
        self.factory_test_item_signal.emit(text)

    def _set_humanode_demo_session_state(self, active: bool, status_text: str):
        active = bool(active)
        if hasattr(self, "humanode_demo_session_status"):
            self.humanode_demo_session_status.setText(str(status_text or ("交互会话进行中" if active else "当前无交互会话")))
        if hasattr(self, "humanode_demo_input_edit"):
            self.humanode_demo_input_edit.setEnabled(active)
            if active:
                self.humanode_demo_input_edit.setFocus()
            else:
                self.humanode_demo_input_edit.clear()
        if hasattr(self, "btn_humanode_demo_send_input"):
            self.btn_humanode_demo_send_input.setEnabled(active)
        if hasattr(self, "btn_humanode_demo_send_enter"):
            self.btn_humanode_demo_send_enter.setEnabled(active)
        if hasattr(self, "btn_humanode_demo_interrupt"):
            self.btn_humanode_demo_interrupt.setEnabled(active)
        if hasattr(self, "btn_humanode_demo_close_session"):
            self.btn_humanode_demo_close_session.setEnabled(active)

    def _extract_exit_code(self, text: str):
        exit_code = None
        lines = []
        for line in (text or "").splitlines():
            if line.startswith("__EXIT_CODE__:"):
                try:
                    exit_code = int(line.split(":", 1)[1].strip())
                except Exception:
                    exit_code = 1
                continue
            lines.append(line)
        return "\n".join(lines).strip(), exit_code

    def _humanode_demo_case_local_path(self, _group_name: str, script_name: str) -> str:
        demo_root = self._humanode_demo_local_dir.strip() or os.path.join(
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
            "Humanode_Naviai-main",
        )
        return os.path.join(demo_root, "utils", script_name)

    def _humanode_demo_case_requires_interaction(self, group_name: str, script_name: str) -> bool:
        script_path = self._humanode_demo_case_local_path(group_name, script_name)
        try:
            with open(script_path, "r", encoding="utf-8") as handle:
                text = handle.read()
        except Exception:
            return False
        patterns = (
            r"\binput\s*\(",
            r"termios",
            r"tty\.setcbreak",
            r"sys\.stdin\.read\(",
            r"enable_keyboard\s*=\s*True",
        )
        return any(re.search(pattern, text) for pattern in patterns)

    def _humanode_demo_case_line_handler(self, state: dict, line: str):
        text = ANSI_ESCAPE_RE.sub("", str(line or "")).replace("\r", "")
        if text.startswith("__EXIT_CODE__:"):
            try:
                state["exit_code"] = int(text.split(":", 1)[1].strip())
            except Exception:
                state["exit_code"] = 1
            return
        self._emit_humanode_demo(text)

    def _start_humanode_demo_interactive_case(self, group_name: str, script_name: str, case_label: str):
        self.close_humanode_demo_session(emit_log=False)
        cmd = self._humanode_demo_case_command(group_name, script_name)
        wrapped = f"set +e; {cmd}; rc=$?; printf '\n__EXIT_CODE__:%s\n' \"$rc\""
        state = {"exit_code": None}

        def on_output(line: str):
            self._humanode_demo_case_line_handler(state, line)

        def on_exit(stdout: str, stderr: str, shell_exit_code):
            clean_out, parsed_exit = self._extract_exit_code(stdout)
            clean_err, err_exit = self._extract_exit_code(stderr)
            exit_code = state.get("exit_code")
            if exit_code is None:
                exit_code = parsed_exit if parsed_exit is not None else err_exit
            if exit_code is None:
                exit_code = shell_exit_code if shell_exit_code is not None else 1
            self._humanode_demo_session = None
            self._humanode_demo_session_case_label = ""
            self.humanode_demo_session_state_signal.emit(False, f"交互会话已结束，退出码: {exit_code}")
            if clean_out:
                pass
            if clean_err:
                self._emit_humanode_demo(clean_err)
            if exit_code == 0:
                self._emit_humanode_demo(f"[OK] 测试用例执行完成: {case_label}")
            else:
                self._emit_humanode_demo(f"[ERR] 测试用例执行失败: {case_label}，exit_code={exit_code}")

        self._emit_humanode_demo(f"[INFO] 检测到交互脚本，已开启 SSH 交互会话: {case_label}")
        self.humanode_demo_session_state_signal.emit(True, f"交互会话进行中: {case_label}")
        self._humanode_demo_session_case_label = case_label
        self._humanode_demo_session = self.ssh_big.start_interactive_session(
            wrapped,
            on_output=on_output,
            on_exit=on_exit,
        )

    def send_humanode_demo_input(self):
        session = self._humanode_demo_session
        if session is None or not session.is_active():
            self._emit_humanode_demo("[WARN] 当前没有可发送输入的交互会话")
            return
        text = self.humanode_demo_input_edit.text() if hasattr(self, "humanode_demo_input_edit") else ""
        try:
            session.send(text)
            self._emit_humanode_demo(f">>> {text}")
            if hasattr(self, "humanode_demo_input_edit"):
                self.humanode_demo_input_edit.clear()
        except Exception as e:
            self._emit_humanode_demo(f"[ERR] 发送输入失败: {e}")

    def send_humanode_demo_enter(self):
        session = self._humanode_demo_session
        if session is None or not session.is_active():
            self._emit_humanode_demo("[WARN] 当前没有可发送输入的交互会话")
            return
        try:
            session.send("")
            self._emit_humanode_demo(">>> [ENTER]")
        except Exception as e:
            self._emit_humanode_demo(f"[ERR] 发送回车失败: {e}")

    def interrupt_humanode_demo_session(self):
        session = self._humanode_demo_session
        if session is None or not session.is_active():
            self._emit_humanode_demo("[WARN] 当前没有可中断的交互会话")
            return
        try:
            session.interrupt()
            self._emit_humanode_demo(">>> [CTRL+C]")
        except Exception as e:
            self._emit_humanode_demo(f"[ERR] 发送 Ctrl+C 失败: {e}")

    def close_humanode_demo_session(self, emit_log: bool = True):
        session = self._humanode_demo_session
        self._humanode_demo_session = None
        self._humanode_demo_session_case_label = ""
        self.humanode_demo_session_state_signal.emit(False, "当前无交互会话")
        if session is None:
            return
        try:
            session.close()
            if emit_log:
                self._emit_humanode_demo("[INFO] 已请求关闭交互会话")
        except Exception as e:
            if emit_log:
                self._emit_humanode_demo(f"[ERR] 关闭交互会话失败: {e}")

    def _local_container_script(self) -> str:
        candidates = []

        # PyInstaller 运行态（AppImage/DEB）优先从打包资源目录查找
        if getattr(sys, "frozen", False):
            meipass = getattr(sys, "_MEIPASS", "")
            if meipass:
                candidates.append(os.path.join(meipass, "scripts", "docker_multi_deploy.sh"))
            candidates.append(os.path.join(os.path.dirname(sys.executable), "scripts", "docker_multi_deploy.sh"))

        # 开发态：从项目目录查找
        candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "docker_multi_deploy.sh")))

        for p in candidates:
            if p and os.path.isfile(p):
                return p

        return candidates[0] if candidates else ""

    def _remote_container_script(self) -> str:
        user = self.ssh_big_user.text().strip() if hasattr(self, "ssh_big_user") else "naviai"
        return f"/home/{user}/docker_multi_deploy.sh"

    def _local_humanode_demo_dir(self) -> str:
        candidates = []
        if getattr(sys, "frozen", False):
            meipass = getattr(sys, "_MEIPASS", "")
            if meipass:
                candidates.append(os.path.join(meipass, "Humanode_Naviai-main"))
            candidates.append(os.path.join(os.path.dirname(sys.executable), "Humanode_Naviai-main"))
        candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "Humanode_Naviai-main")))
        for path in candidates:
            if path and os.path.isdir(path):
                return path
        return candidates[0] if candidates else ""

    def pick_humanode_demo_local_dir(self):
        start_dir = ""
        if hasattr(self, "humanode_demo_local_dir_edit"):
            start_dir = self.humanode_demo_local_dir_edit.text().strip()
        if not start_dir:
            start_dir = self._local_humanode_demo_dir()
        chosen = QFileDialog.getExistingDirectory(self, "选择本地 Humanode Demo 目录", start_dir)
        if not chosen:
            return
        self._humanode_demo_local_dir = chosen
        if hasattr(self, "humanode_demo_local_dir_edit"):
            self.humanode_demo_local_dir_edit.setText(chosen)
        self.refresh_humanode_demo_case_buttons()

    def _sync_humanode_remote_dir_inputs(self, text: str, source: str = ""):
        value = str(text or "").strip()
        if source != "humanode" and hasattr(self, "humanode_demo_remote_dir_edit"):
            if self.humanode_demo_remote_dir_edit.text().strip() != value:
                self.humanode_demo_remote_dir_edit.setText(value)
        if source != "factory" and hasattr(self, "factory_remote_demo_dir_edit"):
            if self.factory_remote_demo_dir_edit.text().strip() != value:
                self.factory_remote_demo_dir_edit.setText(value)

    def _remote_humanode_demo_dir(self) -> str:
        user = self.ssh_big_user.text().strip() if hasattr(self, "ssh_big_user") else "naviai"
        raw = ""
        if hasattr(self, "humanode_demo_remote_dir_edit"):
            raw = self.humanode_demo_remote_dir_edit.text().strip()
        if not raw and hasattr(self, "factory_remote_demo_dir_edit"):
            raw = self.factory_remote_demo_dir_edit.text().strip()
        raw = raw or "test-tools/Humanode"

        if raw.startswith("/"):
            return self._normalize_remote_posix_path(raw)
        if raw == "~":
            return self._normalize_remote_posix_path(f"/home/{user}")
        if raw.startswith("~/"):
            return self._normalize_remote_posix_path(f"/home/{user}/{raw[2:]}")
        return self._normalize_remote_posix_path(f"/home/{user}/{raw}")

    def _container_humanode_demo_dir(self) -> str:
        return "/navi_ws/src/humanode"

    def _remote_humanode_demo_docker_dir(self) -> str:
        return f"{self._remote_humanode_demo_dir().rstrip('/')}/docker"

    def _humanode_demo_case_label(self, script_path: str, fallback_name: str) -> str:
        path = str(script_path or "").strip()
        if not path or not os.path.isfile(path):
            return fallback_name
        try:
            with open(path, "r", encoding="utf-8") as handle:
                module = ast.parse(handle.read(), filename=path)
            doc = ast.get_docstring(module, clean=True)
            if doc:
                first_line = next((line.strip() for line in doc.splitlines() if line.strip()), "")
                if first_line:
                    return first_line
        except Exception:
            pass
        return fallback_name

    def _remote_humanode_demo_case_specs(self) -> list[tuple[str, list[tuple[str, str]]]]:
        if not (self.ssh_big and self.ssh_big.ssh and self.ssh_big.sftp):
            return []

        utils_root_dir = f"{self._remote_humanode_demo_dir().rstrip('/')}/utils"
        try:
            if not self._remote_dir_exists(self.ssh_big, utils_root_dir):
                return []
        except Exception:
            return []

        model_up = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
        current_model = "WA2"
        if model_up == "WA1":
            current_model = "WA1"
        elif model_up == "I2":
            current_model = "I2"

        ordered_models = [current_model] + [m for m in ["WA1", "WA2", "I2"] if m != current_model]
        candidate_groups = [("通用", utils_root_dir, "")]
        for model_name in ordered_models:
            group_title = f"{model_name} 型号"
            if model_name == current_model:
                group_title = f"{model_name} 型号（当前）"
            candidate_groups.append((group_title, f"{utils_root_dir}/{model_name}", f"{model_name}/"))

        groups = []
        for group_name, group_dir, rel_prefix in candidate_groups:
            try:
                if not self._remote_dir_exists(self.ssh_big, group_dir):
                    continue
                entries = self.ssh_big.sftp.listdir_attr(group_dir)
            except Exception:
                continue

            scripts = []
            for entry in sorted(entries, key=lambda item: str(getattr(item, "filename", "") or "").lower()):
                file_name = str(getattr(entry, "filename", "") or "").strip()
                if not file_name.endswith(".py") or file_name.startswith("__"):
                    continue
                if stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                    continue
                scripts.append((file_name[:-3], f"{rel_prefix}{file_name}"))
            if scripts:
                groups.append((group_name, scripts))

        return groups

    def _local_humanode_demo_case_specs(self) -> list[tuple[str, list[tuple[str, str]]]]:
        local_demo_dir = ""
        if hasattr(self, "humanode_demo_local_dir_edit"):
            local_demo_dir = self.humanode_demo_local_dir_edit.text().strip()
        local_demo_dir = local_demo_dir or str(getattr(self, "_humanode_demo_local_dir", "") or "").strip() or self._local_humanode_demo_dir()
        utils_root_dir = os.path.join(local_demo_dir, "utils")
        if not os.path.isdir(utils_root_dir):
            return []

        model_up = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
        current_model = "WA2"
        if model_up == "WA1":
            current_model = "WA1"
        elif model_up == "I2":
            current_model = "I2"

        ordered_models = [current_model] + [m for m in ["WA1", "WA2", "I2"] if m != current_model]

        candidate_groups = [("通用", utils_root_dir, "")]
        for model_name in ordered_models:
            group_title = f"{model_name} 型号"
            if model_name == current_model:
                group_title = f"{model_name} 型号（当前）"
            candidate_groups.append(
                (group_title, os.path.join(utils_root_dir, model_name), f"{model_name}/")
            )

        groups = []
        for group_name, group_dir, rel_prefix in candidate_groups:
            if not os.path.isdir(group_dir):
                continue
            scripts = []
            label_counts = {}
            for file_name in sorted(os.listdir(group_dir)):
                if not file_name.endswith(".py"):
                    continue
                if file_name.startswith("__"):
                    continue
                file_path = os.path.join(group_dir, file_name)
                display_name = self._humanode_demo_case_label(file_path, file_name[:-3])
                label_counts[display_name] = label_counts.get(display_name, 0) + 1
                scripts.append((display_name, f"{rel_prefix}{file_name}"))
            if label_counts:
                deduped_scripts = []
                for display_name, rel_script_path in scripts:
                    final_name = display_name
                    if label_counts.get(display_name, 0) > 1:
                        final_name = f"{display_name} [{os.path.basename(rel_script_path)[:-3]}]"
                    deduped_scripts.append((final_name, rel_script_path))
                scripts = deduped_scripts
            if scripts:
                groups.append((group_name, scripts))
        return groups

    def refresh_humanode_demo_case_buttons(self):
        if not hasattr(self, "humanode_demo_case_layout"):
            return
        self._clear_layout(self.humanode_demo_case_layout)

        specs = self._remote_humanode_demo_case_specs()
        source_text = f"远端目录: {self._remote_humanode_demo_dir().rstrip('/')}/utils"
        if not specs:
            specs = self._local_humanode_demo_case_specs()
            source_text = "本地目录: Humanode_Naviai-main/utils"
        if not specs:
            self.humanode_demo_case_layout.addWidget(QLabel("未找到 utils 测试用例，请先连接 SSH(大脑) 并确认远端 ~/test-tools/Humanode/utils 下有脚本。"))
            self.humanode_demo_case_layout.addStretch(1)
            return

        source_label = QLabel(source_text)
        source_label.setStyleSheet("color: #666;")
        self.humanode_demo_case_layout.addWidget(source_label)

        for group_name, scripts in specs:
            group_body = QWidget()
            group_layout = QGridLayout(group_body)
            group_layout.setContentsMargins(0, 0, 0, 0)
            group_layout.setHorizontalSpacing(8)
            group_layout.setVerticalSpacing(8)

            for index, (display_name, script_name) in enumerate(scripts):
                button = QPushButton(display_name)
                button.setToolTip(f"容器内执行: {script_name}")
                button.clicked.connect(
                    lambda checked=False, group=group_name, script=script_name: self.run_humanode_demo_case(group, script)
                )
                row = index // 3
                col = index % 3
                group_layout.addWidget(button, row, col)

            section, _ = self._build_collapsible_section(group_name, group_body, expanded=False)
            self.humanode_demo_case_layout.addWidget(section)

        self.humanode_demo_case_layout.addStretch(1)

    def _humanode_demo_container_name(self) -> str:
        return "test_rosenv"

    def _humanode_demo_image_name(self) -> str:
        return "test_rosenv:R3"

    def _humanode_demo_motion_installer_name(self) -> str:
        return "zj_humanoid_types_dev-v1.3.0+78a9d8e+71.run"

    def _remote_dir_exists(self, client: SshSftpClient, remote_dir: str) -> bool:
        try:
            return stat.S_ISDIR(client.sftp.stat(remote_dir).st_mode)
        except Exception:
            return False

    def _remote_docker_image_exists(self, image_name: str) -> bool:
        name = str(image_name or "").strip()
        if not name:
            return False
        _, err, exit_code = self._run_bash_with_exit_code(
            self.ssh_big,
            f"docker image inspect {shlex.quote(name)} >/dev/null 2>&1",
        )
        err_text = (err or "").strip()
        if err_text and exit_code not in (0, 1):
            raise RuntimeError(err_text)
        return exit_code == 0

    def _remote_docker_container_exists(self, container_name: str) -> bool:
        name = str(container_name or "").strip()
        if not name:
            return False
        _, err, exit_code = self._run_bash_with_exit_code(
            self.ssh_big,
            f"docker container inspect {shlex.quote(name)} >/dev/null 2>&1",
        )
        err_text = (err or "").strip()
        if err_text and exit_code not in (0, 1):
            raise RuntimeError(err_text)
        return exit_code == 0

    def _remove_remote_dir(self, client: SshSftpClient, remote_dir: str):
        target = str(remote_dir or "").strip()
        if not target:
            return
        target_q = shlex.quote(target)
        delete_attempts = [
            f"rm -rf {target_q}",
            f"chmod -R u+w {target_q} >/dev/null 2>&1 || true; rm -rf {target_q}",
            f"sudo -n rm -rf {target_q}",
        ]

        if client is self.ssh_big and self._is_humanode_remote_path(target):
            delete_attempts.append(self._humanode_demo_container_cleanup_command(target))

        last_error = ""
        last_exit_code = 0
        for cmd in delete_attempts:
            _, err, exit_code = self._run_bash_with_exit_code(client, cmd)
            err_text = (err or "").strip()
            if exit_code == 0 and not self._remote_dir_exists(client, target):
                return
            last_error = err_text or last_error
            last_exit_code = exit_code

        raise RuntimeError(last_error or f"删除远端目录失败，exit_code={last_exit_code}")

    def _is_humanode_remote_path(self, remote_path: str) -> bool:
        target = self._normalize_remote_posix_path(remote_path)
        humanode_root = self._normalize_remote_posix_path(self._remote_humanode_demo_dir())
        return target == humanode_root or target.startswith(humanode_root.rstrip("/") + "/")

    def _humanode_demo_container_cleanup_command(self, remote_path: str) -> str:
        remote_target = self._normalize_remote_posix_path(remote_path)
        remote_root = self._normalize_remote_posix_path(self._remote_humanode_demo_dir())
        container_root = self._container_humanode_demo_dir().rstrip("/")
        container_name = self._humanode_demo_container_name()

        rel = remote_target[len(remote_root):].strip("/") if remote_target.startswith(remote_root) else ""
        if rel:
            container_target = f"{container_root}/{rel}"
            inner = f"rm -rf {shlex.quote(container_target)}"
        else:
            inner = (
                f"target={shlex.quote(container_root)}; "
                "find \"$target\" -mindepth 1 -maxdepth 1 -exec rm -rf {} +"
            )

        host_cleanup = f"rm -rf {shlex.quote(remote_target)}"
        return (
            f"if docker container inspect {shlex.quote(container_name)} >/dev/null 2>&1; then "
            f"docker exec -u root {shlex.quote(container_name)} bash -lc {shlex.quote(inner)} >/dev/null 2>&1; "
            "fi; "
            f"sudo -n rm -rf {shlex.quote(remote_target)} >/dev/null 2>&1 || {host_cleanup}"
        )

    def _remove_remote_docker_container(self, container_name: str):
        name = str(container_name or "").strip()
        if not name:
            return
        _, err, exit_code = self._run_bash_with_exit_code(
            self.ssh_big,
            f"docker rm -f {shlex.quote(name)} >/dev/null 2>&1",
        )
        err_text = (err or "").strip()
        if err_text or exit_code != 0:
            raise RuntimeError(err_text or f"删除容器失败，exit_code={exit_code}")

    def _remove_remote_docker_image(self, image_name: str):
        name = str(image_name or "").strip()
        if not name:
            return
        _, err, exit_code = self._run_bash_with_exit_code(
            self.ssh_big,
            f"docker rmi -f {shlex.quote(name)} >/dev/null 2>&1",
        )
        err_text = (err or "").strip()
        if err_text or exit_code != 0:
            raise RuntimeError(err_text or f"删除镜像失败，exit_code={exit_code}")

    def _emit_command_output(self, out: str, err: str):
        text = (out or "").strip()
        err_text = (err or "").strip()
        if text:
            self._emit_humanode_demo(text)
        if err_text:
            self._emit_humanode_demo(err_text)
        if not text and not err_text:
            self._emit_humanode_demo("[OK] 命令执行完成")

    def _run_bash_with_exit_code(self, client: SshSftpClient, cmd: str):
        wrapped = f"set +e; {cmd}; rc=$?; printf '\n__EXIT_CODE__:%s\n' \"$rc\""
        out, err = client.run(f"bash -lc {shlex.quote(wrapped)}")
        exit_code = 0
        lines = []
        for line in (out or "").splitlines():
            if line.startswith("__EXIT_CODE__:"):
                try:
                    exit_code = int(line.split(":", 1)[1].strip())
                except Exception:
                    exit_code = 1
                continue
            lines.append(line)
        return "\n".join(lines).strip(), err, exit_code

    def _run_interactive_bash_with_exit_code(self, client: SshSftpClient, cmd: str, on_output=None):
        wrapped = f"set +e; {cmd}; rc=$?; printf '\n__EXIT_CODE__:%s\n' \"$rc\""
        if on_output is not None and hasattr(client, "run_interactive_stream"):
            out, err = client.run_interactive_stream(wrapped, on_output=on_output)
        else:
            out, err = client.run_interactive(wrapped)
        exit_code = 0
        lines = []
        for line in (out or "").splitlines():
            if line.startswith("__EXIT_CODE__:"):
                try:
                    exit_code = int(line.split(":", 1)[1].strip())
                except Exception:
                    exit_code = 1
                continue
            lines.append(line)
        return "\n".join(lines).strip(), err, exit_code

    def _wait_for_container_running(self, container_name: str, timeout_sec: float = 20.0) -> bool:
        deadline = time.time() + max(1.0, float(timeout_sec))
        while time.time() < deadline:
            try:
                state = self._inspect_container_runtime_state(container_name)
                if state.get("running") and not state.get("restarting"):
                    detail = self._container_runtime_details.get(container_name, {}) or {}
                    detail["status"] = str(state.get("status") or "")
                    self._container_runtime_details[container_name] = detail
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def _humanode_demo_motion_message_exists(self, container_name: str) -> bool:
        inner = (
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
            "if [ -f /navi_ws/devel/setup.bash ]; then source /navi_ws/devel/setup.bash >/dev/null 2>&1; fi; "
            "rosmsg list 2>/dev/null | grep -q MotionExecuteAction"
        )
        cmd = f"docker exec {shlex.quote(container_name)} bash -lc {shlex.quote(inner)}"
        _, err, exit_code = self._run_bash_with_exit_code(self.ssh_big, cmd)
        err_text = (err or "").strip()
        if err_text and exit_code not in (0, 1):
            raise RuntimeError(err_text)
        return exit_code == 0

    def _humanode_demo_run_command(self, remote_docker_dir: str) -> str:
        run_dir = shlex.quote(remote_docker_dir)
        return (
            f"cd {run_dir} && chmod +x ./run1.sh && "
            "if command -v script >/dev/null 2>&1; then "
            "timeout 30s script -q -c './run1.sh' /dev/null; "
            "else "
            "timeout 30s ./run1.sh; "
            "fi"
        )

    def _humanode_demo_case_command(self, _group_name: str, script_name: str) -> str:
        base_dir = self._container_humanode_demo_dir().rstrip("/")
        rel_path = str(script_name or "").strip().lstrip("/")
        script_dir = os.path.dirname(f"{base_dir}/utils/{rel_path}")
        script_file = os.path.basename(rel_path)
        inner = (
            "export PYTHONUNBUFFERED=1; "
            "export LOGURU_COLORIZE=0; "
            "export NO_COLOR=1; "
            "if [ -f /root/.bashrc ]; then source /root/.bashrc >/dev/null 2>&1; fi; "
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; "
            "if [ -f /navi_ws/devel/setup.bash ]; then source /navi_ws/devel/setup.bash >/dev/null 2>&1; fi; "
            f"cd {shlex.quote(script_dir)} && PY_BIN=/root/venv/bin/python3; [ -x \"$PY_BIN\" ] || PY_BIN=python3; \"$PY_BIN\" {shlex.quote(script_file)}"
        )
        return f"docker exec -u root -it {shlex.quote(self._humanode_demo_container_name())} bash -ic {shlex.quote(inner)}"

    def _install_humanode_demo_motion_message(self, container_name: str):
        installer = self._humanode_demo_motion_installer_name()
        inner = (
            f"cd /shared && test -f {shlex.quote(installer)} && chmod +x {shlex.quote('./' + installer)} && {shlex.quote('./' + installer)}"
        )
        cmd = f"docker exec {shlex.quote(container_name)} bash -lc {shlex.quote(inner)}"
        out, err, exit_code = self._run_interactive_bash_with_exit_code(
            self.ssh_big,
            cmd,
            on_output=self._emit_humanode_demo,
        )
        if exit_code != 0:
            self._emit_command_output(out, err)
            raise RuntimeError(f"安装 MotionExecuteAction 相关消息失败，exit_code={exit_code}")

    def run_humanode_demo_case(self, group_name: str, script_name: str):
        case_label = f"{group_name}/{script_name}"
        self._show_humanode_demo_result_window(case_label)

        def worker():
            try:
                self.humanode_demo_clear_signal.emit()
                self._emit_humanode_demo(f"=== 执行测试用例: {case_label} ===")
                self.humanode_demo_session_state_signal.emit(False, "正在准备执行测试用例")
                if not self._ensure_ssh_big():
                    self._emit_humanode_demo("[ERR] 请先连接 SSH(大脑)")
                    return

                container_name = self._humanode_demo_container_name()
                if not self._wait_for_container_running(container_name, timeout_sec=2.0):
                    raise RuntimeError(f"容器未运行，请先点击“运行 Humanode Demo”: {container_name}")

                if self._humanode_demo_case_requires_interaction(group_name, script_name):
                    self._start_humanode_demo_interactive_case(group_name, script_name, case_label)
                    return

                cmd = self._humanode_demo_case_command(group_name, script_name)
                state = {"exit_code": None}

                def _on_case_output(line: str):
                    self._humanode_demo_case_line_handler(state, line)

                out, err, exit_code = self._run_interactive_bash_with_exit_code(
                    self.ssh_big,
                    cmd,
                    on_output=_on_case_output,
                )
                if state.get("exit_code") is not None:
                    exit_code = int(state.get("exit_code"))
                if exit_code != 0:
                    self._emit_command_output(out, err)
                    raise RuntimeError(f"测试用例执行失败，exit_code={exit_code}")
                self.humanode_demo_session_state_signal.emit(False, "当前用例无需交互")
                self._emit_humanode_demo(f"[OK] 测试用例执行完成: {case_label}")
            except Exception as e:
                self.humanode_demo_session_state_signal.emit(False, "当前无交互会话")
                self._emit_humanode_demo(f"[ERR] 测试用例执行失败: {e}")

        self._run_async(worker)

    def _sync_container_script(self) -> str:
        local_script = self._local_container_script()
        remote_script = self._remote_container_script()
        if not os.path.isfile(local_script):
            raise RuntimeError(f"本地未找到脚本: {local_script}")
        self.ssh_big.upload(local_script, remote_script)
        return remote_script

    def _run_container_action(self, args: list, title: str):
        def worker():
            try:
                self._emit_container(f"=== {title} ===")
                if not self._ensure_ssh_big():
                    self._emit_container("[ERR] 请先连接 SSH(大脑)")
                    return

                remote_script = self._sync_container_script()
                arg_text = " ".join(shlex.quote(str(x)) for x in args)
                cmd = f"chmod +x {shlex.quote(remote_script)} && {shlex.quote(remote_script)} {arg_text}".strip()
                out, err = self._run_bash(self.ssh_big, cmd)

                text = (out or "").strip()
                err_text = (err or "").strip()
                if text:
                    self._emit_container(text)
                if err_text:
                    self._emit_container(err_text)
                if not text and not err_text:
                    self._emit_container("[OK] 命令执行完成")
            except Exception as e:
                self._emit_container(f"[ERR] 容器操作失败: {e}")

        self._run_async(worker)

    def deploy_humanode_demo(self):
        def worker():
            try:
                self.humanode_demo_clear_signal.emit()
                self._emit_humanode_demo("=== 部署 Humanode Demo ===")
                if not self._ensure_ssh_big():
                    self._emit_humanode_demo("[ERR] 请先连接 SSH(大脑)")
                    return

                local_demo_dir = ""
                if hasattr(self, "humanode_demo_local_dir_edit"):
                    local_demo_dir = self.humanode_demo_local_dir_edit.text().strip()
                local_demo_dir = local_demo_dir or str(getattr(self, "_humanode_demo_local_dir", "") or "").strip() or self._local_humanode_demo_dir()
                remote_demo_dir = self._remote_humanode_demo_dir()
                remote_docker_dir = self._remote_humanode_demo_docker_dir()
                image_name = self._humanode_demo_image_name()
                container_name = self._humanode_demo_container_name()
                force_rebuild = bool(
                    getattr(self, "humanode_demo_force_rebuild_checkbox", None)
                    and self.humanode_demo_force_rebuild_checkbox.isChecked()
                )

                if not os.path.isdir(local_demo_dir):
                    raise RuntimeError(f"本地未找到 Demo 目录: {local_demo_dir}")

                if force_rebuild:
                    self._emit_humanode_demo("[INFO] 已启用强制重建，开始清理旧目录/容器/镜像")
                    if self._remote_docker_container_exists(container_name):
                        self._emit_humanode_demo(f"[INFO] 删除旧容器: {container_name}")
                        self._remove_remote_docker_container(container_name)
                        self._emit_humanode_demo("[OK] 旧容器删除完成")
                    else:
                        self._emit_humanode_demo(f"[INFO] 未检测到旧容器，跳过删除: {container_name}")

                    if self._remote_docker_image_exists(image_name):
                        self._emit_humanode_demo(f"[INFO] 删除旧镜像: {image_name}")
                        self._remove_remote_docker_image(image_name)
                        self._emit_humanode_demo("[OK] 旧镜像删除完成")
                    else:
                        self._emit_humanode_demo(f"[INFO] 未检测到旧镜像，跳过删除: {image_name}")

                    if self._remote_dir_exists(self.ssh_big, remote_demo_dir):
                        self._emit_humanode_demo(f"[INFO] 删除旧目录: {remote_demo_dir}")
                        self._remove_remote_dir(self.ssh_big, remote_demo_dir)
                        self._emit_humanode_demo("[OK] 旧目录删除完成")
                    else:
                        self._emit_humanode_demo(f"[INFO] 未检测到旧目录，跳过删除: {remote_demo_dir}")

                if self._remote_dir_exists(self.ssh_big, remote_demo_dir):
                    self._emit_humanode_demo(f"[INFO] 大脑已存在同名目录，跳过上传: {remote_demo_dir}")
                else:
                    self._emit_humanode_demo(f"[INFO] 开始上传 Demo 目录: {local_demo_dir} -> {remote_demo_dir}")
                    self.ssh_big.upload_dir(local_demo_dir, remote_demo_dir)
                    self._emit_humanode_demo("[OK] Demo 目录上传完成")

                if self._remote_docker_image_exists(image_name):
                    self._emit_humanode_demo(f"[INFO] 大脑已存在镜像，跳过构建: {image_name}")
                else:
                    self._emit_humanode_demo(f"[INFO] 未检测到镜像，开始构建: {image_name}")
                    build_cmd = f"cd {shlex.quote(remote_docker_dir)} && chmod +x ./build.sh && ./build.sh"
                    out, err, exit_code = self._run_interactive_bash_with_exit_code(
                        self.ssh_big,
                        build_cmd,
                        on_output=self._emit_humanode_demo,
                    )
                    if exit_code != 0:
                        self._emit_command_output(out, err)
                        raise RuntimeError(f"执行 build.sh 失败，exit_code={exit_code}")
                self._emit_humanode_demo("[OK] Humanode Demo 部署完成")
            except Exception as e:
                self._emit_humanode_demo(f"[ERR] 部署 Humanode Demo 失败: {e}")

        self._run_async(worker)

    def update_humanode_demo_remote_dir(self):
        if not self._ensure_ssh_big():
            self._emit_humanode_demo("[ERR] 请先连接 SSH(大脑)")
            return

        start_local_dir = ""
        if hasattr(self, "humanode_demo_local_dir_edit"):
            start_local_dir = self.humanode_demo_local_dir_edit.text().strip()
        start_local_dir = start_local_dir or self._local_humanode_demo_dir()

        local_dir = QFileDialog.getExistingDirectory(self, "选择要更新的本地文件夹", start_local_dir)
        if not local_dir:
            return

        remote_target_dir = self._browse_sftp_path_dialog(
            self.ssh_big,
            "大脑",
            "选择要替换的远端文件夹",
            start_path=self._remote_humanode_demo_dir(),
            select_kind="dir",
        )
        if not remote_target_dir:
            return

        remote_target_dir = self._normalize_remote_posix_path(remote_target_dir)
        if remote_target_dir in ("/", ""):
            QMessageBox.warning(self, "禁止操作", "不能替换根目录。")
            return

        confirm = QMessageBox.question(
            self,
            "确认替换远端文件夹",
            f"将删除远端目录并重新上传本地目录内容。\n\n本地目录:\n{local_dir}\n\n远端目录:\n{remote_target_dir}\n\n是否继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return

        def worker():
            try:
                self.humanode_demo_clear_signal.emit()
                self._emit_humanode_demo("=== 更新 Humanode 远端文件夹 ===")
                self._emit_humanode_demo(f"[INFO] 本地目录: {local_dir}")
                self._emit_humanode_demo(f"[INFO] 远端目录: {remote_target_dir}")

                if not os.path.isdir(local_dir):
                    raise RuntimeError(f"本地目录不存在: {local_dir}")

                if self._remote_dir_exists(self.ssh_big, remote_target_dir):
                    self._emit_humanode_demo(f"[INFO] 删除远端目录: {remote_target_dir}")
                    self._remove_remote_dir(self.ssh_big, remote_target_dir)
                    self._emit_humanode_demo("[OK] 远端旧目录删除完成")
                else:
                    self._emit_humanode_demo(f"[INFO] 远端目录不存在，将直接上传: {remote_target_dir}")

                self._emit_humanode_demo("[INFO] 开始上传本地目录到远端")
                self.ssh_big.upload_dir(local_dir, remote_target_dir)
                self._emit_humanode_demo("[OK] 远端文件夹更新完成")
            except Exception as e:
                self._emit_humanode_demo(f"[ERR] 更新远端文件夹失败: {e}")

        self._run_async(worker)

    def run_humanode_demo(self):
        def worker():
            try:
                self.humanode_demo_clear_signal.emit()
                self._emit_humanode_demo("=== 运行 Humanode Demo ===")
                if not self._ensure_ssh_big():
                    self._emit_humanode_demo("[ERR] 请先连接 SSH(大脑)")
                    return
                container_name = self._ensure_humanode_demo_container_ready(self._emit_humanode_demo)

                self._emit_humanode_demo(f"[OK] Humanode Demo 运行完成，容器: {container_name}")
                QTimer.singleShot(0, self.refresh_container_runtime_list)
            except Exception as e:
                self._emit_humanode_demo(f"[ERR] 运行 Humanode Demo 失败: {e}")

        self._run_async(worker)

    def _ensure_humanode_demo_container_ready(self, emit_line, status_setter=None) -> str:
        container_name = self._humanode_demo_container_name()
        if not self._remote_docker_container_exists(container_name):
            raise RuntimeError(f"docker ps -a 未发现容器: {container_name}")

        state = self._inspect_container_runtime_state(container_name)
        if state.get("running") and (not state.get("restarting")):
            emit_line(f"[INFO] 容器已在运行: {container_name}")
        else:
            status_text = str(state.get("status") or "-")
            emit_line(f"[INFO] 容器当前状态={status_text}，开始执行: docker start {container_name}")
            start_cmd = f"docker start {shlex.quote(container_name)}"
            out, err, exit_code = self._run_bash_with_exit_code(self.ssh_big, start_cmd)
            if exit_code != 0:
                text = (out or "").strip()
                err_text = (err or "").strip()
                if text:
                    emit_line(text)
                if err_text:
                    emit_line(err_text)
                raise RuntimeError(f"docker start 失败，exit_code={exit_code}")
            if (out or "").strip():
                emit_line((out or "").strip())

        if not self._wait_for_container_running(container_name, timeout_sec=20.0):
            raise RuntimeError(f"容器未在预期时间内进入运行态: {container_name}")

        if self._humanode_demo_motion_message_exists(container_name):
            emit_line("[OK] 容器内已存在 MotionExecuteAction，无需安装")
        else:
            emit_line("[INFO] 容器内未检测到 MotionExecuteAction，开始安装中间件")
            self._install_humanode_demo_motion_message(container_name)
            if not self._humanode_demo_motion_message_exists(container_name):
                raise RuntimeError("中间件安装完成后仍未检测到 MotionExecuteAction")
            emit_line("[OK] MotionExecuteAction 相关消息安装完成")

        if status_setter is not None:
            status_setter(f"已启动: {container_name}", "ready")
        return container_name

    def run_factory_test_container(self):
        self.factory_container_status_signal.emit("启动中", "running")
        def worker():
            try:
                self.factory_test_clear_signal.emit()
                self._emit_factory_test("=== 运行工厂测试容器 ===")
                if not self._ensure_ssh_big():
                    self.factory_container_status_signal.emit("未连接大脑SSH", "error")
                    self._emit_factory_test("[ERR] 请先连接 SSH(大脑)")
                    return
                container_name = self._ensure_humanode_demo_container_ready(
                    self._emit_factory_test,
                    status_setter=self.factory_container_status_signal.emit,
                )
                self._emit_factory_test(f"[OK] 工厂测试容器运行完成: {container_name}")
                QTimer.singleShot(0, self.refresh_container_runtime_list)
            except Exception as e:
                self.factory_container_status_signal.emit(f"启动失败: {e}", "error")
                self._emit_factory_test(f"[ERR] 运行工厂测试容器失败: {e}")

        self._run_async(worker)

    def _factory_test_script_command(self, script_rel_path: str) -> str:
        rel = str(script_rel_path or "").strip().lstrip("/")
        if not rel:
            raise RuntimeError("脚本路径不能为空")

        base_dir = self._container_humanode_demo_dir().rstrip("/")
        script_path = f"{base_dir}/{rel}"
        script_dir = os.path.dirname(script_path)
        script_name = os.path.basename(script_path)

        inner = (
            "export PYTHONUNBUFFERED=1; "
            "export LOGURU_COLORIZE=0; "
            "export NO_COLOR=1; "
            "if [ -f /root/.bashrc ]; then source /root/.bashrc >/dev/null 2>&1; fi; "
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; "
            "if [ -f /navi_ws/devel/setup.bash ]; then source /navi_ws/devel/setup.bash >/dev/null 2>&1; fi; "
            f"cd {shlex.quote(script_dir)} && PY_BIN=/root/venv/bin/python3; [ -x \"$PY_BIN\" ] || PY_BIN=python3; \"$PY_BIN\" {shlex.quote(script_name)}"
        )
        return f"docker exec -u root -it {shlex.quote(self._humanode_demo_container_name())} bash -ic {shlex.quote(inner)}"

    def _emit_factory_command_output(self, out: str, err: str):
        text = (out or "").strip()
        err_text = (err or "").strip()
        if text:
            self._emit_factory_test(text)
        if err_text:
            self._emit_factory_test(err_text)
        if not text and not err_text:
            self._emit_factory_test("[OK] 命令执行完成")

    def _run_remote_factory_test_script(self, emit_line, scene_label: str, script_label: str, script_rel_path: str):
        label = str(script_label or script_rel_path or "脚本").strip()
        emit_line(f"=== 执行{scene_label}: {label} ===")
        if not self._ensure_ssh_big():
            emit_line("[ERR] 请先连接 SSH(大脑)")
            return

        container_name = self._humanode_demo_container_name()
        if not self._wait_for_container_running(container_name, timeout_sec=2.0):
            raise RuntimeError(f"容器未运行，请先点击“运行工厂测试容器”: {container_name}")

        cmd = self._factory_test_script_command(script_rel_path)
        out, err, exit_code = self._run_interactive_bash_with_exit_code(
            self.ssh_big,
            cmd,
            on_output=emit_line,
        )
        if exit_code != 0:
            text = (out or "").strip()
            err_text = (err or "").strip()
            if text:
                emit_line(text)
            if err_text:
                emit_line(err_text)
            raise RuntimeError(f"脚本执行失败，exit_code={exit_code}")

        emit_line(f"[OK] {scene_label}执行完成: {label}")

    def _factory_hand_joint_service_name(self) -> str:
        return "/zj_humanoid/hand/joint_switch/dual"

    def _factory_hand_joint_payload(self, joints: list[float]) -> dict:
        service_name = self._factory_hand_joint_service_name()
        joint_values = [float(x) for x in joints]
        return self._maintenance_list_payload(service_name, joint_values, ("joint", "joints", "position", "positions"))

    def _factory_send_hand_joints_via_ros(self, preset_name: str, joints: list[float]):
        if not (self.ros and self.ros.check_connection()):
            raise RuntimeError("ROSBridge 未连接")
        service_name = self._factory_hand_joint_service_name()
        payload = self._factory_hand_joint_payload(joints)
        self._emit_factory_test(
            f"[REQ] 手指关节({preset_name}) ROSBridge: {service_name} {json.dumps(payload, ensure_ascii=False)}"
        )
        resp = self.ros.request_service(service_name, payload, timeout=8.0)
        self._emit_factory_test(f"[OK] 手指关节({preset_name}) ROSBridge完成: {self._format_resp_text(resp)}")

    def _factory_send_hand_joints_via_ssh(self, preset_name: str, joints: list[float]):
        if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
            raise RuntimeError("SSH(小脑) 未连接")
        service_name = self._factory_hand_joint_service_name()
        payload = self._factory_hand_joint_payload(joints)
        cmd = f"rosservice call {service_name} {shlex.quote(json.dumps(payload, ensure_ascii=False))}"
        self._emit_factory_test(f"[REQ] 手指关节({preset_name}) SSH: {cmd}")
        out, err, exit_code = self._run_ros_cli_via_ssh_with_exit_code(cmd)
        text = ((out or "") + "\n" + (err or "")).strip()
        if exit_code != 0:
            raise RuntimeError(text or f"rosservice call 失败，exit_code={exit_code}")
        self._emit_factory_test(f"[OK] 手指关节({preset_name}) SSH完成: {text or '调用完成'}")

    def run_factory_finger_joint_test(self):
        def worker():
            try:
                presets = list(getattr(self, "_maintenance_finger_presets", []) or [])
                if not presets:
                    raise RuntimeError("未找到 finger_joints.yaml 预置数据")

                ros_available = bool(self.ros and self.ros.check_connection())
                ssh_available = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
                if not ros_available and not ssh_available:
                    self._emit_factory_test("[ERR] 请先连接 ROSBridge 或 SSH(小脑)")
                    return

                self._emit_factory_test("=== 执行工厂测试脚本: 手指关节运动 ===")
                use_ssh = not ros_available
                if use_ssh:
                    self._emit_factory_test("[WARN] ROSBridge 未连接，直接使用 SSH(小脑) 执行手指关节测试")

                total = len(presets)
                rounds = 1
                for round_index in range(1, rounds + 1):
                    self._emit_factory_test(f"[INFO] 手指关节测试第 {round_index}/{rounds} 遍开始")
                    for index, (preset_name, joint_values) in enumerate(presets, start=1):
                        step_label = f"{preset_name} (第{round_index}/{rounds}遍 {index}/{total})"
                        if not use_ssh:
                            try:
                                self._factory_send_hand_joints_via_ros(step_label, joint_values)
                            except Exception as e:
                                if not ssh_available:
                                    raise RuntimeError(f"ROSBridge 调用失败，且 SSH(小脑) 不可用: {e}")
                                self._emit_factory_test(f"[WARN] ROSBridge 调用失败，切换 SSH(小脑): {step_label} | {e}")
                                use_ssh = True
                        if use_ssh:
                            self._factory_send_hand_joints_via_ssh(step_label, joint_values)
                        time.sleep(0.35)

                self._emit_factory_test("[OK] 工厂测试脚本执行完成: 手指关节运动")
            except Exception as e:
                self._emit_factory_test(f"[ERR] 工厂测试脚本执行失败: 手指关节运动 | {e}")

        self._run_async(worker)

    def run_factory_test_script(self, script_label: str, script_rel_path: str):
        def worker():
            try:
                self._run_remote_factory_test_script(self._emit_factory_test, "工厂测试脚本", script_label, script_rel_path)
            except Exception as e:
                label = str(script_label or script_rel_path or "脚本").strip()
                self._emit_factory_test(f"[ERR] 工厂测试脚本执行失败: {label} | {e}")

        self._run_async(worker)

    def _factory_wa2_ls_joint_yaml_path(self) -> str:
        candidates = []
        meipass = getattr(sys, "_MEIPASS", "")
        if meipass:
            candidates.append(os.path.join(meipass, "config", "wa2_ls_joint_data.yaml"))
        runtime_dir = os.path.dirname(os.path.abspath(sys.executable)) if getattr(sys, "frozen", False) else ""
        if runtime_dir:
            candidates.append(os.path.join(runtime_dir, "config", "wa2_ls_joint_data.yaml"))
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        candidates.append(os.path.join(project_root, "config", "wa2_ls_joint_data.yaml"))
        for path in candidates:
            if os.path.isfile(path):
                return path
        return candidates[0]

    def _factory_load_wa2_ls_joint_yaml(self, yaml_path: str):
        try:
            with open(yaml_path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or []
        except Exception as e:
            raise RuntimeError(f"读取WA2_LS关节YAML失败: {e}")

        if not isinstance(data, list):
            raise RuntimeError("WA2_LS关节YAML格式错误: 根节点必须是数组")

        joint_names = self._get_joint_names_by_model("WA2_LS")
        expected_count = len(joint_names)
        frames = []
        times = []
        for idx, item in enumerate(data, start=1):
            if not isinstance(item, dict):
                raise RuntimeError(f"第{idx}帧格式错误: 必须是对象")
            positions = item.get("positions")
            if not isinstance(positions, list):
                raise RuntimeError(f"第{idx}帧缺少 positions 数组")
            if len(positions) != expected_count:
                raise RuntimeError(f"第{idx}帧关节数不匹配: 期望{expected_count}，实际{len(positions)}")
            try:
                frame = [float(v) for v in positions]
                ts = float(item.get("time", 0.0))
            except Exception as e:
                raise RuntimeError(f"第{idx}帧存在非数字内容: {e}")
            frames.append(frame)
            times.append(ts)

        if len(frames) < 2:
            raise RuntimeError(f"WA2_LS关节YAML有效帧不足: {len(frames)}，至少需要2帧")

        deltas = []
        for idx in range(1, len(times)):
            dt = float(times[idx]) - float(times[idx - 1])
            if dt > 1e-6:
                deltas.append(dt)
        if deltas:
            ordered = sorted(deltas)
            mid = len(ordered) // 2
            median_dt = ordered[mid] if len(ordered) % 2 == 1 else (ordered[mid - 1] + ordered[mid]) / 2.0
        else:
            median_dt = 0.005
        hz = max(1.0, min(250.0, 1.0 / max(1e-4, float(median_dt))))
        return joint_names, frames, times, hz

    def _factory_wa2_ls_remote_playback_script(self, remote_yaml_path: str, movej_service: str, servoj_topic_name: str, arm_type: int, movej_v: float) -> str:
        return f'''import sys\nimport time\nimport rospy\nfrom upperlimb.msg import Joints\nfrom upperlimb.srv import MoveJ, MoveJRequest, Servo, ServoRequest\n\nYAML_PATH = {remote_yaml_path!r}\nMOVEJ_SERVICE = {movej_service!r}\nTOPIC_NAME = {servoj_topic_name!r}\nARM_TYPE = {int(arm_type)!r}\nMOVEJ_V = {float(movej_v)!r}\nJOINT_COUNT = {len(self._get_joint_names_by_model("WA2_LS"))!r}\nCLEAR_SERVICE = "/zj_humanoid/upperlimb/clear_servo_params"\nSET_SERVICE = "/zj_humanoid/upperlimb/set_servo_params"\n\n\ndef parse_yaml(path):\n    entries = []\n    current = None\n    in_positions = False\n    with open(path, "r", encoding="utf-8") as handle:\n        for raw in handle:\n            stripped = raw.strip()\n            if not stripped:\n                continue\n            if stripped.startswith("- time:"):\n                if current is not None:\n                    entries.append(current)\n                current = {{"time": float(stripped.split(":", 1)[1].strip()), "positions": []}}\n                in_positions = False\n                continue\n            if current is None:\n                continue\n            if stripped == "positions:":\n                in_positions = True\n                continue\n            if in_positions and stripped.startswith("- "):\n                current["positions"].append(float(stripped[2:].strip()))\n    if current is not None:\n        entries.append(current)\n    return entries\n\n\ndef median(values):\n    ordered = sorted(values)\n    if not ordered:\n        return 0.005\n    mid = len(ordered) // 2\n    if len(ordered) % 2 == 1:\n        return ordered[mid]\n    return (ordered[mid - 1] + ordered[mid]) / 2.0\n\n\nentries = parse_yaml(YAML_PATH)\nif len(entries) < 2:\n    raise RuntimeError(f"YAML有效帧不足: {{len(entries)}}")\nframes = []\ntimes = []\nfor idx, item in enumerate(entries, start=1):\n    positions = item.get("positions") or []\n    if len(positions) != JOINT_COUNT:\n        raise RuntimeError(f"第{{idx}}帧关节数不匹配: 期望{{JOINT_COUNT}}，实际{{len(positions)}}")\n    frames.append([float(v) for v in positions])\n    times.append(float(item.get("time", 0.0)))\n\ndeltas = []\nfor idx in range(1, len(times)):\n    dt = times[idx] - times[idx - 1]\n    if dt > 1e-6:\n        deltas.append(dt)\nservo_interval = max(0.001, median(deltas) if deltas else 0.005)\nhz = 1.0 / servo_interval\n\nrospy.init_node("factory_wa2_ls_upperlimb_playback", anonymous=True)\nrospy.wait_for_service(MOVEJ_SERVICE, timeout=30.0)\nrospy.wait_for_service(CLEAR_SERVICE, timeout=10.0)\nrospy.wait_for_service(SET_SERVICE, timeout=10.0)\nmovej = rospy.ServiceProxy(MOVEJ_SERVICE, MoveJ)\nclear_srv = rospy.ServiceProxy(CLEAR_SERVICE, Servo)\nset_srv = rospy.ServiceProxy(SET_SERVICE, Servo)\npub = rospy.Publisher(TOPIC_NAME, Joints, queue_size=1)\ntime.sleep(0.2)\n\nmovej_req = MoveJRequest()\nmovej_req.joints = [float(v) for v in frames[0]]\nmovej_req.v = MOVEJ_V\nmovej_req.acc = 1.0\nmovej_req.t = 5.0\nmovej_req.is_async = False\nmovej_req.arm_type = ARM_TYPE\nprint(f"[REQ] MoveJ回到起始位姿: service={{MOVEJ_SERVICE}}, joints={{len(movej_req.joints)}}")\nsys.stdout.flush()\nmovej_resp = movej(movej_req)\nprint(f"[OK] MoveJ完成: {{movej_resp}}")\nsys.stdout.flush()\n\nclear_req = ServoRequest()\nclear_req.v = 0.0\nclear_req.acc = 0.0\nclear_req.time = 0.0\nclear_req.lookahead_time = 0.0\nclear_req.gain = 0\nclear_req.arm_type = ARM_TYPE\nclear_srv(clear_req)\n\nset_req = ServoRequest()\nset_req.v = 0.1\nset_req.acc = 0.5\nset_req.time = servo_interval\nset_req.lookahead_time = 0.2\nset_req.gain = 100\nset_req.arm_type = ARM_TYPE\nset_srv(set_req)\nprint(f"[INFO] ServoJ参数已设置: interval={{servo_interval:.6f}}s, hz={{hz:.2f}}")\nsys.stdout.flush()\n\nlast_target = time.monotonic()\nfor index, row in enumerate(frames):\n    if index > 0:\n        dt = times[index] - times[index - 1]\n        if dt <= 1e-6:\n            dt = servo_interval\n        last_target += dt\n        while True:\n            remain = last_target - time.monotonic()\n            if remain <= 0:\n                break\n            time.sleep(min(0.001, remain))\n    msg = Joints()\n    msg.joint = [float(v) for v in row]\n    pub.publish(msg)\n    if index == 0 or (index + 1) % max(1, int(round(hz))) == 0 or index == len(frames) - 1:\n        print(f"[INFO] ServoJ已发布 {{index + 1}}/{{len(frames)}} 帧")\n        sys.stdout.flush()\n\nprint("[OK] WA2_LS YAML上肢回放完成")\nsys.stdout.flush()\n'''

    def _run_wa2_ls_upperlimb_yaml_playback(self, emit_line, scene_label: str):
        local_yaml_path = ""
        remote_yaml_path = ""
        remote_script_path = ""
        local_script_path = ""
        try:
            local_yaml_path = self._factory_wa2_ls_joint_yaml_path()
            if not os.path.isfile(local_yaml_path):
                raise RuntimeError(f"未找到WA2_LS关节YAML: {local_yaml_path}")

            joint_names, frames, _times, hz = self._factory_load_wa2_ls_joint_yaml(local_yaml_path)
            movej_v = 0.2
            arm_type = self._servoj_tool_arm_type("WA2_LS")
            movej_service = self._movej_single_service_for_arm_type(arm_type)
            servoj_topic_name = "/zj_humanoid/upperlimb/servoj/whole_body"
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            remote_dir = self._servoj_tool_remote_dir()
            remote_yaml_path = f"{remote_dir}/wa2_ls_upperlimb_{stamp}.yaml"
            remote_script_path = f"{remote_dir}/wa2_ls_upperlimb_{stamp}.py"

            fd_script, local_script_path = tempfile.mkstemp(prefix="wa2_ls_upperlimb_", suffix=".py")
            os.close(fd_script)

            with open(local_script_path, "w", encoding="utf-8") as handle:
                handle.write(
                    self._factory_wa2_ls_remote_playback_script(
                        remote_yaml_path,
                        movej_service,
                        servoj_topic_name,
                        arm_type,
                        movej_v,
                    )
                )

            emit_line(f"=== 执行{scene_label} ===")
            emit_line(f"[INFO] 本地YAML: {local_yaml_path}")
            emit_line(f"[INFO] 解析完成: frames={len(frames)}, joints={len(joint_names)}, hz≈{hz:.2f}")

            self._run_bash(self.ssh, f"mkdir -p {shlex.quote(remote_dir)}")
            self.ssh.upload(local_yaml_path, remote_yaml_path)
            self.ssh.upload(local_script_path, remote_script_path)
            emit_line(f"[OK] 已上传YAML到小脑: {remote_yaml_path}")

            cmd = (
                f"chmod +x {shlex.quote(remote_script_path)}; "
                f"source ~/.bashrc >/dev/null 2>&1; "
                f"source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
                f"python3 {shlex.quote(remote_script_path)}"
            )
            out, err, exit_code = self._run_interactive_bash_with_exit_code(
                self.ssh,
                cmd,
                on_output=emit_line,
            )
            if exit_code != 0:
                raise RuntimeError(((out or "") + "\n" + (err or "")).strip() or f"exit_code={exit_code}")

            emit_line(f"[OK] {scene_label}执行完成")
        finally:
            if local_script_path:
                try:
                    os.remove(local_script_path)
                except Exception:
                    pass
            if remote_yaml_path or remote_script_path:
                try:
                    self._run_bash(
                        self.ssh,
                        f"rm -f {shlex.quote(remote_yaml_path)} {shlex.quote(remote_script_path)}"
                    )
                except Exception:
                    pass

    def _factory_run_wa2_ls_upperlimb_test(self):
        if not self._ensure_ssh():
            self._emit_factory_test("[ERR] 请先连接 SSH(小脑)")
            return

        def worker():
            try:
                self._run_wa2_ls_upperlimb_yaml_playback(self._emit_factory_test, "WA2_LS上肢YAML回放")
            except Exception as e:
                self._emit_factory_test(f"[ERR] WA2_LS上肢测试执行失败: {e}")

        self._run_async(worker)

    def run_factory_upperlimb_test(self):
        model = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
        if model == "WA2_LS":
            self._factory_run_wa2_ls_upperlimb_test()
            return
        self.run_factory_test_script("UpperLimb 激烈运动", "testcase/wa1/smoke/upperlimb_aggresive_move.py")

    def _shared_joint_limits_config_path(self) -> str:
        candidates = []
        meipass = getattr(sys, "_MEIPASS", "")
        if meipass:
            candidates.append(os.path.join(meipass, "config", "joint_limits.yaml"))
        runtime_dir = os.path.dirname(os.path.abspath(sys.executable)) if getattr(sys, "frozen", False) else ""
        if runtime_dir:
            candidates.append(os.path.join(runtime_dir, "config", "joint_limits.yaml"))
        candidates.append(os.path.join(self._get_storage_root(), "config", "joint_limits.yaml"))
        for path in candidates:
            if os.path.isfile(path):
                return path
        return candidates[0]

    def _shared_joint_limits(self, model: str, force_reload: bool = False) -> dict[str, tuple[float, float]]:
        model_tag = str(model or "").strip().upper()
        if model_tag == "WA2":
            model_tag = "WA2_LS"
        if model_tag == "WA2_LS" and (not force_reload) and isinstance(self._maintenance_wa1_joint_limit_cache, dict) and self._maintenance_wa1_joint_limit_cache:
            return dict(self._maintenance_wa1_joint_limit_cache)

        cfg_path = self._shared_joint_limits_config_path()
        if not os.path.isfile(cfg_path):
            raise FileNotFoundError(f"未找到公共关节限位文件: {cfg_path}")
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        src = data.get(model_tag) or {}
        if not isinstance(src, dict) or not src:
            raise RuntimeError(f"{model_tag}关节限位为空")

        margin = float(self._maintenance_wa1_limit_margin)
        mapped: dict[str, tuple[float, float]] = {}
        for name, limits in src.items():
            if not isinstance(limits, (list, tuple)) or len(limits) < 2:
                continue
            try:
                low = float(limits[0]) + margin
                high = float(limits[1]) - margin
            except Exception:
                continue
            if high - low <= 1e-6:
                continue
            mapped[str(name)] = (low, high)

        if not mapped:
            raise RuntimeError(f"{model_tag}关节限位解析后为空(内缩后无有效区间)")
        if model_tag == "WA1":
            self._maintenance_wa1_joint_limit_cache = dict(mapped)
        return mapped

    def _maintenance_load_wa1_joint_limits(self, force_reload: bool = False) -> dict[str, tuple[float, float]]:
        return self._shared_joint_limits("WA1", force_reload=force_reload)

    def _maintenance_update_wa1_status_label(self):
        if not hasattr(self, "maintenance_wa1_status_label"):
            return
        if self._maintenance_wa1_single_joint_running:
            status_text = "状态: 停止中" if self._maintenance_wa1_single_joint_stop_requested else "状态: 运行中"
        else:
            status_text = "状态: 就绪"
        self.maintenance_wa1_status_label.setText(status_text)
        if hasattr(self, "btn_maintenance_wa1_single_joint_test"):
            self.btn_maintenance_wa1_single_joint_test.setEnabled(not self._maintenance_wa1_single_joint_running)
        if hasattr(self, "btn_maintenance_wa1_single_joint_stop"):
            self.btn_maintenance_wa1_single_joint_stop.setEnabled(self._maintenance_wa1_single_joint_running)

    def stop_maintenance_wa1_single_joint_test(self):
        if not self._maintenance_wa1_single_joint_running:
            self._emit_maintenance("[WARN] 当前没有在运行的WA1单关节测试")
            return
        self._maintenance_wa1_single_joint_stop_requested = True
        self._emit_maintenance("[INFO] 已请求停止WA1单关节测试，正在收尾并回基线")
        QTimer.singleShot(0, self._maintenance_update_wa1_status_label)

        pid_text = str(self._maintenance_wa1_single_joint_remote_pid or "").strip()
        if not pid_text.isdigit():
            return
        if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
            return

        def killer():
            try:
                self._run_bash(self.ssh, f"kill -TERM {shlex.quote(pid_text)} >/dev/null 2>&1 || true")
            except Exception:
                pass

        self._run_async(killer)

    def _maintenance_refresh_wa1_joint_options(self):
        if not hasattr(self, "maintenance_wa1_joint_combo"):
            return
        current = self.maintenance_wa1_joint_combo.currentText().strip()
        self.maintenance_wa1_joint_combo.blockSignals(True)
        self.maintenance_wa1_joint_combo.clear()
        try:
            limits = self._maintenance_load_wa1_joint_limits()
        except Exception as e:
            self.maintenance_wa1_joint_combo.addItem("<限位加载失败>")
            if hasattr(self, "maintenance_wa1_limit_label"):
                self.maintenance_wa1_limit_label.setText(f"限位窗口: 读取失败 ({e})")
            self.maintenance_wa1_joint_combo.blockSignals(False)
            return

        preferred = [name for name in self._get_joint_names_by_model("WA1") if name in limits]
        if not preferred:
            preferred = sorted(limits.keys())
        self.maintenance_wa1_joint_combo.addItems(preferred)
        if current and current in preferred:
            self.maintenance_wa1_joint_combo.setCurrentText(current)
        self.maintenance_wa1_joint_combo.blockSignals(False)
        self._maintenance_on_wa1_joint_changed()

    def _maintenance_on_wa1_joint_changed(self, *_args):
        if not hasattr(self, "maintenance_wa1_limit_label") or not hasattr(self, "maintenance_wa1_joint_combo"):
            return
        joint_name = self.maintenance_wa1_joint_combo.currentText().strip()
        try:
            limits = self._maintenance_load_wa1_joint_limits()
            if joint_name in limits:
                low, high = limits[joint_name]
                margin = float(self._maintenance_wa1_limit_margin)
                self.maintenance_wa1_limit_label.setText(f"限位窗口(内缩{margin:g}rad): [{low:.4f}, {high:.4f}]")
            else:
                self.maintenance_wa1_limit_label.setText("限位窗口: 当前关节无定义")
        except Exception as e:
            self.maintenance_wa1_limit_label.setText(f"限位窗口: 读取失败 ({e})")

    def _maintenance_wa1_single_joint_remote_script(self, remote_payload_path: str) -> str:
        return f'''import json\nimport os\nimport signal\nimport time\nimport rospy\nfrom sensor_msgs.msg import JointState\nfrom upperlimb.msg import Joints\nfrom upperlimb.srv import MoveJ, MoveJRequest, Servo, ServoRequest\n\nPAYLOAD_PATH = {remote_payload_path!r}\nSERVO_TOPIC = "/zj_humanoid/upperlimb/servoj/whole_body"\nCLEAR_SERVICE = "/zj_humanoid/upperlimb/clear_servo_params"\nSET_SERVICE = "/zj_humanoid/upperlimb/set_servo_params"\nJOINT_STATE_TOPIC = "/zj_humanoid/upperlimb/joint_states"\n\nwith open(PAYLOAD_PATH, "r", encoding="utf-8") as f:\n    cfg = json.load(f)\n\njoint_names = list(cfg.get("joint_names") or [])\nbaseline = [float(v) for v in (cfg.get("baseline") or [])]\nselected_joint = str(cfg.get("selected_joint") or "")\nduration_sec = float(cfg.get("duration_sec") or 600.0)\nhz = float(cfg.get("hz") or 200.0)\nstep_rad = abs(float(cfg.get("step_rad") or 0.0005))\nstart_value = float(cfg.get("start_value") or 0.0)\nerr_threshold = float(cfg.get("err_threshold") or 0.08)\nmovej_service = str(cfg.get("movej_service") or "")\nmovej_v = float(cfg.get("movej_v") or 0.2)\narm_type = int(cfg.get("arm_type") or 31)\nwindow_low = float(cfg.get("window_low"))\nwindow_high = float(cfg.get("window_high"))\n\nif not joint_names:\n    raise RuntimeError("joint_names为空")\nif len(baseline) != len(joint_names):\n    raise RuntimeError("baseline长度与joint_names不一致")\nif selected_joint not in joint_names:\n    raise RuntimeError(f"选中关节不存在: {{selected_joint}}")\nif not movej_service:\n    raise RuntimeError("movej_service为空")\nif window_high - window_low <= 1e-6:\n    raise RuntimeError("限位窗口无效")\nif step_rad <= 1e-6:\n    raise RuntimeError("step_rad无效")\n\nidx = joint_names.index(selected_joint)\nlatest_map = {{}}\nstop_requested = False\n\ndef js_cb(msg):\n    global latest_map\n    try:\n        latest_map = {{str(n): float(p) for n, p in zip(list(msg.name), list(msg.position))}}\n    except Exception:\n        pass\n\ndef stop_cb(_sig, _frame):\n    global stop_requested\n    stop_requested = True\n\nsignal.signal(signal.SIGINT, stop_cb)\nsignal.signal(signal.SIGTERM, stop_cb)\n\nrospy.init_node("wa1_single_joint_maintenance", anonymous=True, disable_signals=True)\nrospy.Subscriber(JOINT_STATE_TOPIC, JointState, js_cb, queue_size=1)\nrospy.wait_for_service(CLEAR_SERVICE, timeout=10.0)\nrospy.wait_for_service(SET_SERVICE, timeout=10.0)\nrospy.wait_for_service(movej_service, timeout=30.0)\nclear_srv = rospy.ServiceProxy(CLEAR_SERVICE, Servo)\nset_srv = rospy.ServiceProxy(SET_SERVICE, Servo)\nmovej_srv = rospy.ServiceProxy(movej_service, MoveJ)\npub = rospy.Publisher(SERVO_TOPIC, Joints, queue_size=1)\ntime.sleep(0.2)\nprint("__PID__:" + str(os.getpid()), flush=True)\n\ninterval = 1.0 / max(1.0, hz)\nclear_req = ServoRequest()\nclear_req.v = 0.0\nclear_req.acc = 0.0\nclear_req.time = 0.0\nclear_req.lookahead_time = 0.0\nclear_req.gain = 0\nclear_req.arm_type = arm_type\nclear_srv(clear_req)\nset_req = ServoRequest()\nset_req.v = 0.1\nset_req.acc = 0.5\nset_req.time = interval\nset_req.lookahead_time = 0.2\nset_req.gain = 100\nset_req.arm_type = arm_type\nset_srv(set_req)\n\nstart_ts = time.monotonic()\nnext_tick = start_ts\nnext_probe = start_ts\nprobe_interval = 0.2\npeak_err = 0.0\npublished = 0\n\ntarget = min(window_high, max(window_low, start_value))\ndirection = 1\n\nwhile (not stop_requested) and (not rospy.is_shutdown()):\n    now = time.monotonic()\n    elapsed = now - start_ts\n    if elapsed >= duration_sec:\n        break\n\n    cmd = list(baseline)\n    cmd[idx] = float(target)\n    msg = Joints()\n    msg.joint = [float(v) for v in cmd]\n    pub.publish(msg)\n    published += 1\n\n    if now >= next_probe:\n        actual = latest_map.get(selected_joint)\n        if actual is not None:\n            peak_err = max(peak_err, abs(float(actual) - float(target)))\n        next_probe += probe_interval\n\n    next_target = float(target) + float(direction) * float(step_rad)\n    if next_target >= window_high:\n        next_target = window_high\n        direction = -1\n    elif next_target <= window_low:\n        next_target = window_low\n        direction = 1\n    target = next_target\n\n    next_tick += interval\n    while True:\n        remain = next_tick - time.monotonic()\n        if remain <= 0:\n            break\n        time.sleep(min(0.001, remain))\n\nactual_duration = time.monotonic() - start_ts\n\nmovej_req = MoveJRequest()\nmovej_req.joints = [float(v) for v in baseline]\nmovej_req.v = movej_v\nmovej_req.acc = 1.0\nmovej_req.t = 5.0\nmovej_req.is_async = False\nmovej_req.arm_type = arm_type\nmovej_srv(movej_req)\ntime.sleep(1.0)\n\nactual_after = latest_map.get(selected_joint)\nbaseline_val = float(baseline[idx])\nreturn_err = abs(float(actual_after) - baseline_val) if actual_after is not None else None\n\nresult = {{\n    "mode": "ssh",\n    "selected_joint": selected_joint,\n    "peak_error": float(peak_err),\n    "return_error": None if return_err is None else float(return_err),\n    "duration_target": float(duration_sec),\n    "duration_actual": float(actual_duration),\n    "duration_ok": bool(actual_duration >= duration_sec * 0.98),\n    "return_ok": bool(return_err is not None and return_err <= err_threshold),\n    "published": int(published),\n    "error_threshold": float(err_threshold),\n    "aborted": bool(stop_requested),\n}}\nresult["pass"] = bool((not result["aborted"]) and result["duration_ok"] and result["return_ok"])\nprint("__RESULT__:" + json.dumps(result, ensure_ascii=False), flush=True)\n'''

    def _maintenance_run_wa1_single_joint_ros(
        self,
        joint_names: list[str],
        baseline: list[float],
        selected_joint: str,
        window_low: float,
        window_high: float,
        duration_sec: float,
        hz: float,
        step_rad: float,
        start_value: float,
        movej_v: float,
        err_threshold: float,
    ) -> dict:
        arm_type = 31
        movej_service = self._movej_single_service_for_arm_type(arm_type)
        if not movej_service:
            raise RuntimeError("无法解析WA1 whole_body MoveJ服务")
        if not (self.ros and self.ros.check_connection()):
            raise RuntimeError("ROSBridge 不可用")

        clear_req = {"v": 0.0, "acc": 0.0, "time": 0.0, "lookahead_time": 0.0, "gain": 0, "arm_type": arm_type}
        set_req = {"v": 0.1, "acc": 0.5, "time": 1.0 / max(1.0, hz), "lookahead_time": 0.2, "gain": 100, "arm_type": arm_type}
        self.ros.request_service("/zj_humanoid/upperlimb/clear_servo_params", clear_req, service_type="upperlimb/Servo", timeout=10.0)
        self.ros.request_service("/zj_humanoid/upperlimb/set_servo_params", set_req, service_type="upperlimb/Servo", timeout=10.0)

        topic = roslibpy.Topic(self.ros.ros, "/zj_humanoid/upperlimb/servoj/whole_body", "upperlimb/Joints")
        idx = joint_names.index(selected_joint)
        interval = 1.0 / max(1.0, hz)
        start_ts = time.monotonic()
        next_tick = start_ts
        next_probe = start_ts
        peak_err = 0.0
        published = 0
        target = min(float(window_high), max(float(window_low), float(start_value)))
        direction = 1
        try:
            topic.advertise()
            while True:
                if self._maintenance_wa1_single_joint_stop_requested:
                    break
                now = time.monotonic()
                elapsed = now - start_ts
                if elapsed >= duration_sec:
                    break
                if not self.ros.check_connection():
                    raise RuntimeError("执行中ROSBridge断开")
                cmd = list(baseline)
                cmd[idx] = float(target)
                topic.publish(roslibpy.Message({"joint": [float(v) for v in cmd]}))
                published += 1

                if now >= next_probe:
                    state_map = self._fetch_joint_state_once() or {}
                    actual = state_map.get(selected_joint)
                    if actual is not None:
                        peak_err = max(peak_err, abs(float(actual) - float(target)))
                    next_probe += 0.2

                next_target = float(target) + float(direction) * float(step_rad)
                if next_target >= float(window_high):
                    next_target = float(window_high)
                    direction = -1
                elif next_target <= float(window_low):
                    next_target = float(window_low)
                    direction = 1
                target = next_target

                next_tick += interval
                while True:
                    remain = next_tick - time.monotonic()
                    if remain <= 0:
                        break
                    time.sleep(min(0.001, remain))
        finally:
            try:
                topic.unadvertise()
            except Exception:
                pass

        duration_actual = time.monotonic() - start_ts
        movej_req = {
            "joints": [float(v) for v in baseline],
            "v": float(movej_v),
            "acc": 1.0,
            "t": 5.0,
            "is_async": False,
            "arm_type": arm_type,
        }
        self.ros.request_service(movej_service, movej_req, service_type="upperlimb/MoveJ", timeout=30.0)
        time.sleep(1.0)
        state_after = self._fetch_joint_state_once() or {}
        return_err = None
        if selected_joint in state_after:
            return_err = abs(float(state_after[selected_joint]) - float(baseline[idx]))

        return {
            "mode": "rosbridge",
            "selected_joint": selected_joint,
            "peak_error": float(peak_err),
            "return_error": None if return_err is None else float(return_err),
            "duration_target": float(duration_sec),
            "duration_actual": float(duration_actual),
            "duration_ok": bool(duration_actual >= duration_sec * 0.98),
            "return_ok": bool(return_err is not None and return_err <= err_threshold),
            "published": int(published),
            "error_threshold": float(err_threshold),
            "aborted": bool(self._maintenance_wa1_single_joint_stop_requested),
        }

    def _maintenance_run_wa1_single_joint_ssh(
        self,
        joint_names: list[str],
        baseline: list[float],
        selected_joint: str,
        window_low: float,
        window_high: float,
        duration_sec: float,
        hz: float,
        step_rad: float,
        start_value: float,
        movej_v: float,
        err_threshold: float,
    ) -> dict:
        if not self._ensure_ssh():
            raise RuntimeError("SSH(小脑)未连接")

        remote_dir = "/tmp/maintenance_wa1_single_joint"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        remote_payload_path = f"{remote_dir}/payload_{stamp}.json"
        remote_script_path = f"{remote_dir}/runner_{stamp}.py"
        fd_payload, local_payload_path = tempfile.mkstemp(prefix="wa1_single_joint_", suffix=".json")
        os.close(fd_payload)
        fd_script, local_script_path = tempfile.mkstemp(prefix="wa1_single_joint_", suffix=".py")
        os.close(fd_script)
        try:
            payload = {
                "joint_names": list(joint_names),
                "baseline": [float(v) for v in baseline],
                "selected_joint": str(selected_joint),
                "duration_sec": float(duration_sec),
                "hz": float(hz),
                "step_rad": float(step_rad),
                "start_value": float(start_value),
                "err_threshold": float(err_threshold),
                "window_low": float(window_low),
                "window_high": float(window_high),
                "arm_type": 31,
                "movej_service": self._movej_single_service_for_arm_type(31),
                "movej_v": float(movej_v),
            }
            with open(local_payload_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False)
            with open(local_script_path, "w", encoding="utf-8") as f:
                f.write(self._maintenance_wa1_single_joint_remote_script(remote_payload_path))

            self._run_bash(self.ssh, f"mkdir -p {shlex.quote(remote_dir)}")
            self.ssh.upload(local_payload_path, remote_payload_path)
            self.ssh.upload(local_script_path, remote_script_path)

            cmd = (
                f"chmod +x {shlex.quote(remote_script_path)}; "
                f"source ~/.bashrc >/dev/null 2>&1; "
                f"source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
                f"python3 {shlex.quote(remote_script_path)}"
            )
            self._maintenance_wa1_single_joint_remote_pid = ""

            def on_output(line: str):
                text = str(line or "")
                for one in text.splitlines():
                    s = str(one or "").strip()
                    if s.startswith("__PID__:"):
                        self._maintenance_wa1_single_joint_remote_pid = s.split(":", 1)[1].strip()
                        self._emit_maintenance(f"[INFO] SSH远端执行PID: {self._maintenance_wa1_single_joint_remote_pid}")
                    elif s:
                        self._emit_maintenance(s)

            out, err, exit_code = self._run_interactive_bash_with_exit_code(self.ssh, cmd, on_output=on_output)
            if exit_code != 0:
                if self._maintenance_wa1_single_joint_stop_requested:
                    return {
                        "mode": "ssh",
                        "selected_joint": selected_joint,
                        "peak_error": 0.0,
                        "return_error": None,
                        "duration_target": float(duration_sec),
                        "duration_actual": 0.0,
                        "duration_ok": False,
                        "return_ok": False,
                        "published": 0,
                        "error_threshold": float(err_threshold),
                        "aborted": True,
                    }
                raise RuntimeError(((out or "") + "\n" + (err or "")).strip() or f"exit_code={exit_code}")

            merged = "\n".join([out or "", err or ""]).splitlines()
            result_line = ""
            for line in merged:
                if line.startswith("__RESULT__:"):
                    result_line = line.split("__RESULT__:", 1)[1].strip()
            if not result_line:
                raise RuntimeError("远端未返回测试结果")
            return json.loads(result_line)
        finally:
            self._maintenance_wa1_single_joint_remote_pid = ""
            try:
                os.remove(local_payload_path)
            except Exception:
                pass
            try:
                os.remove(local_script_path)
            except Exception:
                pass
            try:
                self._run_bash(self.ssh, f"rm -f {shlex.quote(remote_payload_path)} {shlex.quote(remote_script_path)}")
            except Exception:
                pass

    def run_maintenance_wa1_single_joint_test(self):
        if self._maintenance_wa1_single_joint_running:
            self._emit_maintenance("[WARN] WA1单关节测试正在运行")
            return
        if str(getattr(self, "robot_model", "WA2") or "WA2").upper() != "WA1":
            self._emit_maintenance("[ERR] 当前型号不是WA1，无法执行单关节测试")
            return

        joint_name = self.maintenance_wa1_joint_combo.currentText().strip() if hasattr(self, "maintenance_wa1_joint_combo") else ""
        duration_sec = float(self.maintenance_wa1_duration_spin.value()) if hasattr(self, "maintenance_wa1_duration_spin") else 600.0
        hz = float(self.maintenance_wa1_hz_spin.value()) if hasattr(self, "maintenance_wa1_hz_spin") else 200.0
        movej_v = float(self.maintenance_wa1_back_movej_v_spin.value()) if hasattr(self, "maintenance_wa1_back_movej_v_spin") else 0.2
        step_rad = float(self.maintenance_wa1_amp_spin.value()) if hasattr(self, "maintenance_wa1_amp_spin") else 0.0005
        err_threshold = float(self.maintenance_wa1_err_thresh_spin.value()) if hasattr(self, "maintenance_wa1_err_thresh_spin") else 0.08
        if not joint_name:
            self._emit_maintenance("[ERR] 请选择测试关节")
            return

        try:
            limits = self._maintenance_load_wa1_joint_limits()
            if joint_name not in limits:
                raise RuntimeError(f"关节{joint_name}没有WA1限位定义")
            low, high = limits[joint_name]
        except Exception as e:
            self._emit_maintenance(f"[ERR] WA1限位读取失败: {e}")
            return

        state_map = self._fetch_joint_state_once() or {}
        joint_names = list(self._get_joint_names_by_model("WA1"))
        if joint_name not in joint_names:
            self._emit_maintenance(f"[ERR] WA1关节列表中不存在 {joint_name}")
            return
        if not isinstance(state_map, dict) or not state_map:
            self._emit_maintenance("[ERR] 未读取到有效关节状态，已取消测试")
            return
        missing = [name for name in joint_names if name not in state_map]
        if missing:
            preview = ", ".join(missing[:6])
            suffix = "..." if len(missing) > 6 else ""
            self._emit_maintenance(
                f"[ERR] 关节状态不完整(缺失{len(missing)}个)，已取消测试: {preview}{suffix}"
            )
            return
        baseline = [float(state_map[name]) for name in joint_names]
        idx = joint_names.index(joint_name)
        base_value = baseline[idx]
        window_low = float(low)
        window_high = float(high)
        start_value = min(window_high, max(window_low, base_value))
        if window_high - window_low <= 1e-6:
            self._emit_maintenance(
                f"[ERR] 可执行窗口过小: baseline={base_value:.4f}, limit=[{low:.4f},{high:.4f}]"
            )
            return
        if step_rad <= 1e-6:
            self._emit_maintenance("[ERR] 步长必须大于0")
            return
        if abs(start_value - base_value) > 1e-6:
            self._emit_maintenance(f"[WARN] 当前值{base_value:.4f}超出限位窗口，起点已夹紧为{start_value:.4f}")

        ros_ok = bool(self.ros and self.ros.check_connection())
        if (not ros_ok) and (not self._ensure_ssh()):
            self._emit_maintenance("[ERR] ROSBridge和SSH均不可用，无法执行测试")
            return
        mode = "rosbridge" if ros_ok else "ssh"

        self._maintenance_wa1_single_joint_running = True
        self._maintenance_wa1_single_joint_stop_requested = False
        self._maintenance_wa1_single_joint_remote_pid = ""
        self._maintenance_update_wa1_status_label()
        self._emit_maintenance(
            f"[INFO] WA1单关节测试开始: joint={joint_name}, mode={mode}, duration={duration_sec:.1f}s, hz={hz:.0f}, step={step_rad:.3f}rad, start={start_value:.4f}, sweep=[{window_low:.4f},{window_high:.4f}]"
        )

        def worker():
            try:
                if mode == "rosbridge":
                    result = self._maintenance_run_wa1_single_joint_ros(
                        joint_names,
                        baseline,
                        joint_name,
                        window_low,
                        window_high,
                        duration_sec,
                        hz,
                        step_rad,
                        start_value,
                        movej_v,
                        err_threshold,
                    )
                else:
                    result = self._maintenance_run_wa1_single_joint_ssh(
                        joint_names,
                        baseline,
                        joint_name,
                        window_low,
                        window_high,
                        duration_sec,
                        hz,
                        step_rad,
                        start_value,
                        movej_v,
                        err_threshold,
                    )

                result["pass"] = bool((not result.get("aborted")) and result.get("duration_ok") and result.get("return_ok"))
                peak_err = result.get("peak_error")
                return_err = result.get("return_error")
                self._emit_maintenance(
                    "[INFO] 结果摘要: "
                    f"duration={float(result.get('duration_actual', 0.0)):.2f}/{float(result.get('duration_target', duration_sec)):.2f}s, "
                    f"peak_err={float(peak_err or 0.0):.4f}, "
                    f"return_err={(f'{float(return_err):.4f}' if return_err is not None else 'N/A')}"
                )
                if result["pass"]:
                    self._emit_maintenance("[OK] WA1单关节测试PASS")
                else:
                    fail_reasons = []
                    if result.get("aborted"):
                        fail_reasons.append("人工停止")
                    if not result.get("duration_ok"):
                        fail_reasons.append("运行时长不足")
                    if not result.get("return_ok"):
                        fail_reasons.append("回基线误差超阈值")
                    self._emit_maintenance(f"[FAIL] WA1单关节测试未通过: {'; '.join(fail_reasons) if fail_reasons else '未知原因'}")
            except Exception as e:
                self._emit_maintenance(f"[ERR] WA1单关节测试执行失败: {e}")
            finally:
                self._maintenance_wa1_single_joint_running = False
                self._maintenance_wa1_single_joint_remote_pid = ""
                QTimer.singleShot(0, self._maintenance_update_wa1_status_label)

        self._run_async(worker)

    def run_maintenance_upperlimb_test(self):
        model = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
        if model == "WA1":
            self.run_maintenance_wa1_single_joint_test()
            return
        run_count = int(self.maintenance_upperlimb_loop_spin.value()) if hasattr(self, "maintenance_upperlimb_loop_spin") else 1
        if model == "WA2_LS":
            if not self._ensure_ssh():
                self._emit_maintenance("[ERR] 请先连接 SSH(小脑)")
                return

            def worker():
                try:
                    self._emit_maintenance(f"[INFO] 运维上肢测试开始: model=WA2_LS, count={run_count}")
                    for index in range(run_count):
                        self._emit_maintenance(f"[INFO] 运维上肢测试第 {index + 1}/{run_count} 次")
                        self._run_wa2_ls_upperlimb_yaml_playback(self._emit_maintenance, f"WA2_LS上肢YAML回放 第{index + 1}/{run_count}次")
                    self._emit_maintenance(f"[OK] 运维上肢测试完成: 已执行 {run_count} 次")
                except Exception as e:
                    self._emit_maintenance(f"[ERR] WA2_LS上肢测试执行失败: {e}")

            self._run_async(worker)
            return

        def worker():
            try:
                self._emit_maintenance(f"[INFO] 运维上肢测试开始: model={model}, count={run_count}")
                for index in range(run_count):
                    self._emit_maintenance(f"[INFO] 运维上肢测试第 {index + 1}/{run_count} 次")
                    self._run_remote_factory_test_script(
                        self._emit_maintenance,
                        "运维上肢测试脚本",
                        f"UpperLimb 激烈运动 第{index + 1}/{run_count}次",
                        "testcase/wa1/smoke/upperlimb_aggresive_move.py",
                    )
                self._emit_maintenance(f"[OK] 运维上肢测试完成: 已执行 {run_count} 次")
            except Exception as e:
                self._emit_maintenance(f"[ERR] 运维上肢测试执行失败: {e}")

        self._run_async(worker)

    def _factory_step_label(self, step: int) -> str:
        labels = {
            1: "机器人信息确认",
            2: "BrainSense与中间件控制",
            3: "ROS话题检测",
            4: "手指关节运动",
            5: "手指压力检测",
            6: "六维力检测",
            7: "语音播报与倾听",
            8: "相机图像检测",
            9: "robot_info 校验",
            10: "upperlimb 运动检测",
        }
        return labels.get(int(step), f"步骤{step}")

    def _factory_normalize_flow_state(self) -> dict:
        # 固定步骤状态顺序，避免报告输出受“完成按钮点击顺序”影响。
        normalized = {}
        for step in self._factory_flow_steps:
            raw = self._factory_flow_state.get(step, {}) if isinstance(self._factory_flow_state, dict) else {}
            normalized[int(step)] = {
                "done": bool(raw.get("done")),
                "time": str(raw.get("time") or ""),
                "note": str(raw.get("note") or ""),
            }
        self._factory_flow_state = normalized
        return normalized

    def _factory_update_flow_status(self):
        self._factory_normalize_flow_state()
        done = 0
        total = len(self._factory_flow_steps)
        for step in self._factory_flow_steps:
            if bool(self._factory_flow_state.get(step, {}).get("done")):
                done += 1
        status = f"进行中 {done}/{total}"
        if done >= total:
            status = f"已完成 {done}/{total}"
        if hasattr(self, "factory_flow_status"):
            self.factory_flow_status.setText(status)
        self._factory_refresh_step_buttons()

    def _factory_set_container_status(self, text: str, status: str = "idle"):
        status_key = str(status or "idle").strip().lower()
        styles = {
            "idle": "background: #f3f4f6; color: #374151;",
            "running": "background: #fef3c7; color: #92400e; font-weight: 700;",
            "ready": "background: #dcfce7; color: #166534; font-weight: 700;",
            "error": "background: #fee2e2; color: #991b1b; font-weight: 700;",
        }
        btn_styles = {
            "idle": "",
            "running": "background-color: #d97706; color: #ffffff; border: 1px solid #f59e0b; font-weight: 700;",
            "ready": "background-color: #15803d; color: #ffffff; border: 1px solid #22c55e; font-weight: 700;",
            "error": "background-color: #b91c1c; color: #ffffff; border: 1px solid #ef4444; font-weight: 700;",
        }
        if hasattr(self, "factory_container_status"):
            self.factory_container_status.setText(str(text or "未启动"))
            self.factory_container_status.setStyleSheet(styles.get(status_key, styles["idle"]))
        if hasattr(self, "btn_factory_test_run_container"):
            if status_key == "running":
                self.btn_factory_test_run_container.setText("容器启动中...")
                self.btn_factory_test_run_container.setEnabled(False)
            elif status_key == "ready":
                self.btn_factory_test_run_container.setText("工厂测试容器已启动")
                self.btn_factory_test_run_container.setEnabled(True)
            else:
                self.btn_factory_test_run_container.setText("运行工厂测试容器")
                self.btn_factory_test_run_container.setEnabled(True)
            self.btn_factory_test_run_container.setStyleSheet(btn_styles.get(status_key, ""))

    def _factory_refresh_step_buttons(self):
        button_map = {
            1: getattr(self, "btn_factory_step0_done", None),
            2: getattr(self, "btn_factory_step1_done", None),
            3: getattr(self, "btn_factory_step2_audit_done", None),
            4: getattr(self, "btn_factory_step2_done", None),
            5: getattr(self, "btn_factory_step3_done", None),
            6: getattr(self, "btn_factory_step4_done", None),
            7: getattr(self, "btn_factory_step5_done", None),
            8: getattr(self, "btn_factory_step6_done", None),
            9: getattr(self, "btn_factory_step7_done", None),
            10: getattr(self, "btn_factory_step8_done", None),
        }
        for step in self._factory_flow_steps:
            btn = button_map.get(step)
            if btn is None:
                continue
            done = bool(self._factory_flow_state.get(step, {}).get("done"))
            if done:
                btn.setText(f"{step}) 已确认")
                btn.setStyleSheet("background-color: #15803d; color: #ffffff; border: 1px solid #22c55e; font-weight: 700;")
            else:
                btn.setText(f"{step}) 完成本项")
                btn.setStyleSheet("")

    def _factory_refresh_pressure_max_text(self):
        if not hasattr(self, "factory_pressure_max_fields"):
            return
        left_vals = self._factory_pressure_max.get("left", [])
        right_vals = self._factory_pressure_max.get("right", [])
        left_widgets = self.factory_pressure_max_fields.get("left", [])
        right_widgets = self.factory_pressure_max_fields.get("right", [])
        for idx, widget in enumerate(left_widgets):
            val = left_vals[idx] if idx < len(left_vals) else None
            widget.setText("-" if val is None else f"{float(val):.2f}")
        for idx, widget in enumerate(right_widgets):
            val = right_vals[idx] if idx < len(right_vals) else None
            widget.setText("-" if val is None else f"{float(val):.2f}")

    def _factory_camera_topics_for_model(self, model: str | None = None) -> dict:
        model_name = str(model or self._factory_current_model()).upper()
        topics = {
            "head": "/zj_humanoid/sensor/realsense_head/color/image_raw",
            "chest": "/zj_humanoid/sensor/realsense_up/color/image_raw",
        }
        if self._is_i_series_model(model_name):
            topics["head"] = ""
            topics["chest"] = "/zj_humanoid/sensor/realsense_down/color/image_raw"
        elif model_name != "WA1":
            topics["chest"] = ""
        return topics

    def _factory_update_camera_topics_display(self):
        topics = self._factory_camera_topics_for_model()
        requirement = self._factory_camera_requirement_state()
        labels = self._factory_camera_slot_labels()
        if hasattr(self, "factory_camera_head_required"):
            self._factory_refresh_camera_labels()
        if hasattr(self, "factory_camera_head_topic_value"):
            head_topic = topics.get("head") or (f"N/A({labels['head']}不可用)" if self._is_i_series_model() else "-")
            self.factory_camera_head_topic_value.setText(head_topic if requirement.get("head_required") else f"{head_topic} (未启用)")
        if hasattr(self, "factory_camera_chest_topic_value"):
            if topics.get("chest"):
                chest_topic = topics.get("chest")
            elif self._is_i_series_model():
                chest_topic = "N/A(I系列无第二路上身相机)"
            else:
                chest_topic = f"N/A(非WA1机型无{labels['chest']})"
            if topics.get("chest") and not requirement.get("chest_required"):
                chest_topic = f"{topics.get('chest')} (未启用)"
            self.factory_camera_chest_topic_value.setText(chest_topic)

    def _factory_set_camera_state(self, active: bool, text: str):
        if hasattr(self, "factory_camera_status"):
            self.factory_camera_status.setText(str(text or ("监听中" if active else "未启动")))
        if hasattr(self, "btn_factory_step5_start"):
            self.btn_factory_step5_start.setEnabled(not active)
        if hasattr(self, "btn_factory_step5_stop"):
            self.btn_factory_step5_stop.setEnabled(active)

    def factory_start_camera_sampling(self):
        if self._factory_camera_subscribers:
            self._emit_factory_test("[INFO] 步骤8相机订阅已在运行")
            return
        if not self._ensure_ros():
            return

        all_topics = self._factory_camera_topics_for_model()
        requirement = self._factory_camera_requirement_state()
        topics = {}
        if requirement.get("head_required"):
            topics["head"] = all_topics.get("head") or ""
        if requirement.get("chest_required"):
            topics["chest"] = all_topics.get("chest") or ""
        if not topics:
            self._emit_factory_test("[WARN] 步骤8未启用任何相机检测")
            return

        self._factory_camera_last_image_path = {"head": "", "chest": ""}
        self._factory_camera_last_frame_ts = {"head": 0.0, "chest": 0.0}
        if hasattr(self, "factory_camera_head_path"):
            self.factory_camera_head_path.setText("-")
        if hasattr(self, "factory_camera_chest_path"):
            self.factory_camera_chest_path.setText("-")

        def worker():
            try:
                subscribers = {}
                for key, topic in topics.items():
                    topic_name = str(topic or "").strip()
                    if not topic_name:
                        continue
                    topic_type = str(self.ros._get_topic_type(topic_name) or "").strip()
                    if topic_type != "sensor_msgs/Image":
                        raise RuntimeError(f"话题类型不是 sensor_msgs/Image: {topic_name} ({topic_type or '-'})")

                    sub = roslibpy.Topic(self.ros.ros, topic_name, topic_type)

                    def _make_cb(cam_key: str, cam_topic: str):
                        def _cb(msg: dict):
                            try:
                                img = self._decode_ros_image(msg)
                                path = self._save_qimage(img)
                                self._factory_camera_last_image_path[cam_key] = path
                                first_hit = self._factory_camera_last_frame_ts.get(cam_key, 0.0) <= 0.0
                                self._factory_camera_last_frame_ts[cam_key] = time.time()
                                self.factory_camera_image_signal.emit(cam_key, img, path)
                                if first_hit:
                                    slot_label = self._factory_camera_slot_labels().get(cam_key, cam_key)
                                    self._emit_factory_test(f"[OK] 步骤8收到{slot_label}画面: {cam_topic}")
                            except Exception as e:
                                self._emit_factory_test(f"[ERR] 步骤8图像解析失败({cam_key}): {e}")
                        return _cb

                    sub.subscribe(_make_cb(key, topic_name))
                    subscribers[key] = sub

                self._factory_camera_subscribers = subscribers
                self.factory_camera_state_signal.emit(True, "相机话题监听中")
                self._emit_factory_test("[OK] 步骤8相机订阅已启动")
            except Exception as e:
                for _k, sub in list(locals().get("subscribers", {}).items()):
                    try:
                        sub.unsubscribe()
                    except Exception:
                        pass
                self._factory_camera_subscribers = {}
                self.factory_camera_state_signal.emit(False, f"启动失败: {e}")
                self._emit_factory_test(f"[ERR] 步骤8相机订阅启动失败: {e}")

        self._run_async(worker)

    def factory_stop_camera_sampling(self, emit_log: bool = True):
        for _key, sub in list(self._factory_camera_subscribers.items()):
            try:
                sub.unsubscribe()
            except Exception:
                pass
        self._factory_camera_subscribers = {}
        self._factory_set_camera_state(False, "未启动")
        if emit_log:
            self._emit_factory_test("[INFO] 步骤8相机订阅已停止")

    def _factory_camera_requirement_state(self) -> dict:
        return {
            "head_required": bool(self.factory_camera_head_required.isChecked()) if hasattr(self, "factory_camera_head_required") else True,
            "chest_required": bool(self.factory_camera_chest_required.isChecked()) if hasattr(self, "factory_camera_chest_required") else True,
        }

    def _factory_reset_camera_requirement_defaults(self):
        if hasattr(self, "factory_camera_head_required"):
            self.factory_camera_head_required.setChecked(not self._is_i_series_model())
        if hasattr(self, "factory_camera_chest_required"):
            self.factory_camera_chest_required.setChecked(self._factory_current_model() == "WA1" or self._is_i_series_model())
        if hasattr(self, "factory_camera_head_required"):
            self._factory_refresh_camera_labels()

    def _factory_info_snapshot(self) -> dict:
        def _line_edit_text(attr_name: str) -> str:
            widget = getattr(self, attr_name, None)
            return widget.text().strip() if widget else "-"

        system_widget = getattr(self, "system_info_text", None)
        system_text = system_widget.toPlainText().strip() if system_widget else "-"
        lowerlimb_widget = getattr(self, "lowerlimb_version_value", None)
        show_lowerlimb = bool((lowerlimb_widget and lowerlimb_widget.isVisible()) or self._factory_model_has_lowerlimb())
        return {
            "robot_model": _line_edit_text("robot_version_value"),
            "embedded_version": _line_edit_text("hardware_version_value"),
            "middleware_version": _line_edit_text("software_version_value"),
            "upperlimb_version": _line_edit_text("upperlimb_version_value"),
            "lowerlimb_version": _line_edit_text("lowerlimb_version_value"),
            "show_lowerlimb": show_lowerlimb,
            "system_info": system_text or "-",
        }

    def _factory_refresh_info_snapshot_fields(self):
        info = self._factory_info_snapshot()
        if hasattr(self, "factory_info_model_value"):
            self.factory_info_model_value.setText(info["robot_model"])
        if hasattr(self, "factory_info_embedded_value"):
            self.factory_info_embedded_value.setText(info["embedded_version"])
        if hasattr(self, "factory_info_middleware_value"):
            self.factory_info_middleware_value.setText(info["middleware_version"])
        if hasattr(self, "factory_info_upperlimb_value"):
            self.factory_info_upperlimb_value.setText(info["upperlimb_version"])
        if hasattr(self, "factory_info_lowerlimb_label"):
            self.factory_info_lowerlimb_label.setVisible(bool(info["show_lowerlimb"]))
        if hasattr(self, "factory_info_lowerlimb_value"):
            self.factory_info_lowerlimb_value.setVisible(bool(info["show_lowerlimb"]))
            self.factory_info_lowerlimb_value.setText(info["lowerlimb_version"])
        if hasattr(self, "factory_info_system_text"):
            self.factory_info_system_text.setPlainText(info["system_info"])

    def factory_refresh_info_snapshot(self):
        self._factory_refresh_info_snapshot_fields()
        self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
        self._emit_factory_test("[INFO] 步骤1开始刷新机器人信息快照")

    def factory_finish_info_snapshot(self):
        self._factory_refresh_info_snapshot_fields()
        info = self._factory_info_snapshot()
        if all(str(info.get(key) or "-").strip() == "-" for key in ("robot_model", "embedded_version", "middleware_version", "upperlimb_version")):
            self._emit_factory_test("[ERR] 步骤1未完成：请先刷新并确认机器人版本信息")
            return
        note = (
            f"型号={info['robot_model']}, 嵌入式={info['embedded_version']}, "
            f"中间件={info['middleware_version']}, 上肢={info['upperlimb_version']}"
        )
        if info["show_lowerlimb"]:
            note += f", 下肢={info['lowerlimb_version']}"
        self.factory_mark_step_done(1, note)

    def _factory_monitored_topics_doc_path(self) -> str:
        return os.path.join(self._project_root_path(), "需监控话题.md")

    def _factory_load_monitored_topics(self) -> tuple[list[str], str]:
        topics = list(FACTORY_MONITORED_TOPICS)
        source = "内置清单"
        doc_path = self._factory_monitored_topics_doc_path()
        if os.path.isfile(doc_path):
            file_topics = []
            seen = set()
            with open(doc_path, "r", encoding="utf-8") as handle:
                for raw_line in handle:
                    line = str(raw_line or "").strip()
                    if not line.startswith("| /zj_humanoid/"):
                        continue
                    parts = [item.strip() for item in line.strip("|").split("|")]
                    if not parts:
                        continue
                    topic = parts[0]
                    if topic and topic not in seen:
                        file_topics.append(topic)
                        seen.add(topic)
            if file_topics:
                topics = file_topics
                source = doc_path

        seen = set(topics)

        if self._factory_current_model() == "WA1":
            extra_topics = []
            for topic in list(topics):
                if "/sensor/realsense_head/" not in topic:
                    continue
                extra_topic = topic.replace("/sensor/realsense_head/", "/sensor/realsense_up/")
                if extra_topic not in seen:
                    extra_topics.append(extra_topic)
                    seen.add(extra_topic)
            topics.extend(extra_topics)

        return topics, source

    def _factory_collect_ros_topics(self) -> tuple[list[str], set[str], str, str, str]:
        monitored_topics, doc_path = self._factory_load_monitored_topics()

        ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
        if ssh_ok:
            out_topics, err_topics = self._run_ros_cli_via_ssh("rostopic list")
            raw_topics = [x.strip() for x in ((out_topics or "") + "\n" + (err_topics or "")).splitlines() if x.strip().startswith("/")]

            live_topics = {t.strip() for t in raw_topics if isinstance(t, str) and t.strip().startswith("/")}
            return monitored_topics, live_topics, "SSH(小脑)", doc_path, "ssh"

        if self.ros and self.ros.check_connection():
            try:
                raw_topics = self.ros.list_topics() or []
                live_topics = {t.strip() for t in raw_topics if isinstance(t, str) and t.strip().startswith("/")}
                return monitored_topics, live_topics, "ROSBridge", doc_path, "rosbridge"
            except Exception:
                pass

        raise RuntimeError("请先连接 SSH(小脑)；若不可用可改连 ROSBridge")

    def _factory_service_type(self, service_name: str) -> str:
        service = str(service_name or "").strip()
        if not service:
            return "-"
        if self.ssh and self.ssh.ssh and self.ssh.sftp:
            try:
                out, err = self._run_ros_cli_via_ssh(f"rosservice type {shlex.quote(service)}")
                text = (out or "").strip() or (err or "").strip()
                return text.splitlines()[0].strip() if text else "-"
            except Exception:
                pass
        return "-"

    def _factory_service_request_fields_via_ssh(self, service_name: str) -> list:
        service = str(service_name or "").strip()
        if not service or not (self.ssh and self.ssh.ssh and self.ssh.sftp):
            return []
        try:
            out_type, err_type = self._run_ros_cli_via_ssh(f"rosservice type {shlex.quote(service)}")
            srv_type = (out_type or "").strip() or (err_type or "").strip()
            srv_type = srv_type.splitlines()[0].strip() if srv_type else ""
            if not srv_type or "/" not in srv_type:
                return []
            out_show, err_show = self._run_ros_cli_via_ssh(f"rossrv show {shlex.quote(srv_type)}")
            text_show = ((out_show or "") + "\n" + (err_show or "")).strip()
            return self._parse_srv_request_fields_from_rossrv_show(text_show)
        except Exception:
            return []

    def _factory_topic_rate_text(self, hz) -> str:
        if hz is None:
            return "无数据"
        try:
            return f"{float(hz):.2f} Hz"
        except Exception:
            return str(hz)

    def _factory_topic_sample_simple_text(self, payload, max_len: int = 220) -> str:
        def _shrink(value, depth: int = 0):
            if depth >= 2:
                if isinstance(value, list):
                    return f"list[{len(value)}]"
                if isinstance(value, dict):
                    return f"dict[{len(value)}]"
                return value
            if isinstance(value, dict):
                keys = list(value.keys())[:4]
                result = {str(key): _shrink(value.get(key), depth + 1) for key in keys}
                if len(value) > len(keys):
                    result["..."] = f"+{len(value) - len(keys)}"
                return result
            if isinstance(value, list):
                items = [_shrink(item, depth + 1) for item in value[:4]]
                if len(value) > 4:
                    items.append(f"...+{len(value) - 4}")
                return items
            return value

        try:
            text = json.dumps(_shrink(payload), ensure_ascii=False, separators=(",", ":"))
        except Exception:
            text = str(payload or "")
        if len(text) > max_len:
            return text[: max_len - 3] + "..."
        return text

    def factory_run_topic_service_audit(self):
        self.factory_topic_service_audit_signal.emit({
            "topics": [],
            "summary": "检测中，请稍候...",
            "topic_count": 0,
        })
        self._emit_factory_test("[INFO] 步骤3开始检测需监控话题（SSH优先，失败回退ROSBridge）")

        def worker():
            try:
                topics, live_topics, source, doc_path, transport = self._factory_collect_ros_topics()
                ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
                ros_ok = bool(self.ros and self.ros.check_connection())
                lines = [
                    "=== 步骤3 ROS话题检测 ===",
                    f"数据来源: {source}",
                    f"清单来源: {doc_path}",
                    f"需监控话题数: {len(topics)}",
                    f"当前广播话题数: {len([t for t in topics if t in live_topics])}",
                    "",
                    "[Topics]",
                ]
                self.factory_topic_service_audit_signal.emit({
                    "topics": [],
                    "summary": "\n".join(lines).strip(),
                    "topic_count": len(topics),
                })
                topic_rows = []
                for index, topic in enumerate(topics, start=1):
                    self._emit_factory_test(f"[INFO] 步骤3检测话题 {index}/{len(topics)}: {topic}")
                    exists = topic in live_topics
                    hz = None
                    sample_text = ""
                    if exists:
                        try:
                            if transport == "rosbridge" and ros_ok:
                                hz = self._measure_topic_rate_via_ros(topic)
                            elif ssh_ok:
                                hz = self._measure_topic_rate_via_ssh(topic)
                        except Exception:
                            hz = None
                        if hz is None and transport == "rosbridge" and ssh_ok:
                            try:
                                hz = self._measure_topic_rate_via_ssh(topic)
                            except Exception:
                                hz = None
                        if hz is not None and ros_ok:
                            try:
                                sample_msg = self._maintenance_topic_once(topic, timeout=2.0)
                                if isinstance(sample_msg, dict):
                                    sample_text = self._factory_topic_sample_simple_text(sample_msg)
                                    self._emit_factory_test(f"[DATA] {topic} | sample={sample_text}")
                            except Exception as sample_err:
                                self._emit_factory_test(f"[WARN] 话题取样失败: {topic} | {sample_err}")

                    hz_text = self._factory_topic_rate_text(hz)
                    topic_rows.append({
                        "name": topic,
                        "exists": exists,
                        "hz": hz,
                        "hz_text": hz_text,
                        "sample_text": sample_text,
                    })
                    lines.append(
                        f"{topic} | hz={hz_text}"
                    )
                    self.factory_topic_service_audit_signal.emit({
                        "topics": list(topic_rows),
                        "summary": "\n".join(lines).strip(),
                        "topic_count": len(topics),
                    })

                if not topics:
                    lines.append("未找到 /zj_humanoid 话题")

                missing_count = len([item for item in topic_rows if not item.get("exists")])
                no_hz_count = len([item for item in topic_rows if item.get("exists") and item.get("hz") is None])
                lines.append("")
                lines.append(f"汇总: 缺失话题 {missing_count} 个, 无Hz数据 {no_hz_count} 个")

                audit = {
                    "topics": topic_rows,
                    "summary": "\n".join(lines).strip() or "-",
                    "topic_count": len(topic_rows),
                }
                self.factory_topic_service_audit_signal.emit(audit)
                self._emit_factory_test(f"[OK] 步骤3检测完成: 话题 {len(topic_rows)} 个")
            except Exception as e:
                self.factory_topic_service_audit_signal.emit({"topics": [], "summary": f"检测失败: {e}", "topic_count": 0})
                self._emit_factory_test(f"[ERR] 步骤3检测失败: {e}")

        self._run_async(worker)

    def factory_finish_topic_service_audit(self):
        audit = getattr(self, "_factory_topic_service_audit", {}) or {}
        topic_count = int(audit.get("topic_count") or 0)
        if topic_count <= 0:
            self._emit_factory_test("[ERR] 步骤3未完成：请先执行话题检测")
            return
        note = f"topics={topic_count}"
        self.factory_mark_step_done(3, note)

    def factory_flow_reset(self):
        self.factory_stop_force_monitor(emit_log=False)
        self.factory_stop_camera_sampling(emit_log=False)
        self._factory_flow_state = {step: {"done": False, "time": "", "note": ""} for step in self._factory_flow_steps}
        self._factory_pressure_capture_active = False
        self._factory_pressure_max = {
            "left": [None, None, None, None, None, None],
            "right": [None, None, None, None, None, None],
        }
        self._factory_topic_service_audit = {"topics": [], "summary": "-", "topic_count": 0}
        self._factory_report_records = []
        self._factory_camera_last_image_path = {"head": "", "chest": ""}
        self._factory_camera_last_frame_ts = {"head": 0.0, "chest": 0.0}
        self._factory_force_delta_max = {"left": [None, None, None], "right": [None, None, None]}
        if hasattr(self, "factory_force_fields"):
            for side_fields in self.factory_force_fields.values():
                for edit in side_fields.values():
                    edit.clear()
        if hasattr(self, "factory_robot_sn_edit"):
            self.factory_robot_sn_edit.clear()
        if hasattr(self, "factory_info_system_text"):
            self.factory_info_system_text.clear()
        if hasattr(self, "factory_info_model_value"):
            self._factory_refresh_info_snapshot_fields()
        if hasattr(self, "factory_topic_service_audit_text"):
            self.factory_topic_service_audit_text.clear()
        if hasattr(self, "factory_camera_head_preview"):
            self.factory_camera_head_preview.setPixmap(QPixmap())
            self.factory_camera_head_preview.setText(f"{self._factory_camera_slot_labels()['head']}等待画面")
        if hasattr(self, "factory_camera_chest_preview"):
            self.factory_camera_chest_preview.setPixmap(QPixmap())
            self.factory_camera_chest_preview.setText(f"{self._factory_camera_slot_labels()['chest']}等待画面")
        if hasattr(self, "factory_camera_head_path"):
            self.factory_camera_head_path.setText("-")
        if hasattr(self, "factory_camera_chest_path"):
            self.factory_camera_chest_path.setText("-")
        self._factory_set_container_status("未启动", "idle")
        self._factory_reset_camera_requirement_defaults()
        self._factory_update_camera_topics_display()
        self._factory_refresh_pressure_max_text()
        self._factory_update_flow_status()
        self._emit_factory_test("[INFO] 工厂测试流程已重置")

    def factory_mark_step_done(self, step: int, note: str = ""):
        self._factory_normalize_flow_state()
        step_i = int(step)
        if step_i not in self._factory_flow_steps:
            return
        item = self._factory_flow_state.get(step_i, {"done": False, "time": "", "note": ""})
        item["done"] = True
        item["time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        item["note"] = str(note or "").strip()
        self._factory_flow_state[step_i] = item
        self._factory_update_flow_status()
        self._emit_factory_test(f"[OK] 步骤{step_i}完成: {self._factory_step_label(step_i)}")

    def factory_step2_start_pressure_capture(self):
        self._factory_pressure_capture_active = True
        self._factory_pressure_max = {
            "left": [None, None, None, None, None, None],
            "right": [None, None, None, None, None, None],
        }
        self._factory_refresh_pressure_max_text()
        self.start_maintenance_pressure_monitor()
        self._emit_factory_test("[INFO] 步骤5开始：已开启手指压力采集")

    def factory_step2_finish(self):
        self._factory_pressure_capture_active = False
        self.stop_maintenance_pressure_monitor(emit_log=False)
        self.factory_mark_step_done(5, "已完成手指压力检测并记录最大值")

    def _factory_parse_force_xyz_values(self, text: str) -> list[float]:
        raw = str(text or "").replace("，", ",").strip()
        parts = [x for x in [p.strip() for p in raw.split(",")] if x]
        if len(parts) != 3:
            raise ValueError("请按 Fx,Fy,Fz 输入3个数值")
        return [float(p) for p in parts]

    def _factory_parse_force_text(self, text: str):
        raw = str(text or "").strip()
        if not raw:
            raise ValueError("请输入 Force XYZ 数值")

        if "|" in raw:
            items = [x.strip() for x in raw.split("|") if x.strip()]
            force_map = {}
            for item in items:
                lower = item.lower()
                if lower.startswith("l:") or lower.startswith("left:"):
                    force_map["left"] = self._factory_parse_force_xyz_values(item.split(":", 1)[1])
                elif lower.startswith("r:") or lower.startswith("right:"):
                    force_map["right"] = self._factory_parse_force_xyz_values(item.split(":", 1)[1])
            if "left" in force_map and "right" in force_map:
                return force_map
            raise ValueError("双侧格式应为 L:Fx,Fy,Fz | R:Fx,Fy,Fz")

        return {"single": self._factory_parse_force_xyz_values(raw)}

    def _factory_format_force_values(self, values) -> str:
        def _fmt_triplet(seq) -> str:
            if not isinstance(seq, (list, tuple)) or len(seq) != 3:
                return ""
            parts = []
            has_numeric = False
            for value in seq:
                if value is None:
                    parts.append("-")
                    continue
                parts.append(f"{float(value):.4f}")
                has_numeric = True
            return ",".join(parts) if has_numeric else ""

        if isinstance(values, dict) and "left" in values and "right" in values:
            left = values.get("left") or []
            right = values.get("right") or []
            if len(left) == 3 and len(right) == 3:
                left_text = _fmt_triplet(left)
                right_text = _fmt_triplet(right)
                if not left_text and not right_text:
                    return "-"
                left_text = left_text or "-,-,-"
                right_text = right_text or "-,-,-"
                return f"L:{left_text} | R:{right_text}"
            return "-"
        if isinstance(values, (list, tuple)) and len(values) == 3:
            text = _fmt_triplet(values)
            return text or "-"
        return "-"

    def _factory_force_field_text(self, field_key: str, side: str) -> str:
        field_map = getattr(self, "factory_force_fields", {}) or {}
        edit = field_map.get(str(field_key or "").strip(), {}).get(str(side or "").strip())
        return edit.text().strip() if edit else ""

    def _factory_force_text_by_key(self, field_key: str) -> str:
        left_text = self._factory_force_field_text(field_key, "left")
        right_text = self._factory_force_field_text(field_key, "right")
        return f"L:{left_text or '-'} | R:{right_text or '-'}"

    def _factory_force_has_values(self, field_key: str) -> bool:
        return bool(self._factory_force_field_text(field_key, "left") and self._factory_force_field_text(field_key, "right"))

    def _factory_force_values_by_key(self, field_key: str):
        values = {}
        left_text = self._factory_force_field_text(field_key, "left")
        right_text = self._factory_force_field_text(field_key, "right")
        if left_text:
            values["left"] = self._factory_parse_force_xyz_values(left_text)
        if right_text:
            values["right"] = self._factory_parse_force_xyz_values(right_text)
        if values:
            return values
        raise ValueError("请输入 Force XYZ 数值")

    def _factory_reset_force_delta_max(self):
        self._factory_force_delta_max = {"left": [None, None, None], "right": [None, None, None]}

    def _factory_update_force_delta_max(self, delta: dict):
        if not isinstance(delta, dict):
            return
        current_max = getattr(self, "_factory_force_delta_max", None)
        if not isinstance(current_max, dict):
            self._factory_reset_force_delta_max()
            current_max = self._factory_force_delta_max
        for side in ("left", "right"):
            values = delta.get(side)
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                continue
            side_max = current_max.setdefault(side, [None, None, None])
            for idx, value in enumerate(values):
                abs_value = abs(float(value))
                if side_max[idx] is None or abs_value > float(side_max[idx]):
                    side_max[idx] = abs_value

    def _factory_force_delta_max_text(self) -> str:
        return self._factory_format_force_values(getattr(self, "_factory_force_delta_max", {}))

    def _factory_clear_force_values(self, field_key: str):
        field_map = getattr(self, "factory_force_fields", {}) or {}
        target_fields = field_map.get(str(field_key or "").strip(), {})
        for edit in target_fields.values():
            if edit:
                edit.clear()

    def _factory_set_force_values(self, field_key: str, values):
        field_map = getattr(self, "factory_force_fields", {}) or {}
        target_fields = field_map.get(str(field_key or "").strip(), {})
        left_edit = target_fields.get("left")
        right_edit = target_fields.get("right")
        if not left_edit or not right_edit:
            return
        if isinstance(values, dict) and "left" in values and "right" in values:
            left_edit.setText(self._factory_format_force_values(values.get("left") or []))
            right_edit.setText(self._factory_format_force_values(values.get("right") or []))
        elif isinstance(values, dict):
            left_edit.setText(self._factory_format_force_values(values.get("left") or []) if "left" in values else "")
            right_edit.setText(self._factory_format_force_values(values.get("right") or []) if "right" in values else "")
        else:
            text = self._factory_format_force_values(values)
            left_edit.setText(text if text != "-" else "")
            right_edit.setText(text if text != "-" else "")
        if field_key in {"baseline", "current"}:
            self._factory_refresh_force_delta_display()

    def _factory_set_force_side_value(self, field_key: str, side: str, values):
        field_map = getattr(self, "factory_force_fields", {}) or {}
        target_fields = field_map.get(str(field_key or "").strip(), {})
        edit = target_fields.get(str(side or "").strip())
        if not edit:
            return
        text = self._factory_format_force_values(values)
        edit.setText(text if text != "-" else "")
        if field_key in {"baseline", "current"}:
            self._factory_refresh_force_delta_display()

    def _factory_refresh_force_delta_display(self):
        try:
            baseline = self._factory_force_values_by_key("baseline")
        except Exception:
            baseline = {}
        try:
            current = self._factory_force_values_by_key("current")
        except Exception:
            current = {}

        delta = {}
        for side in ("left", "right"):
            if side in baseline and side in current:
                delta[side] = [current[side][i] - baseline[side][i] for i in range(3)]

        if delta:
            self._factory_update_force_delta_max(delta)
            self._factory_set_force_values("delta", delta)
        else:
            self._factory_clear_force_values("delta")

    def _factory_update_force_live_side(self, side: str, values):
        self._factory_set_force_side_value("current", side, values)

    def _factory_set_force_monitor_state(self, active: bool, text: str):
        if hasattr(self, "btn_factory_force_capture_current"):
            self.btn_factory_force_capture_current.setEnabled(not active)
            self.btn_factory_force_capture_current.setText("6) Force检测中..." if active else "6) 开始Force检测")
        if hasattr(self, "btn_factory_force_calc"):
            self.btn_factory_force_calc.setEnabled(active)
        if text:
            self._emit_factory_test(text)

    def _factory_extract_force_values_from_msg(self, msg: dict) -> list[float]:
        if not isinstance(msg, dict):
            raise ValueError("力传感器消息为空")

        # 只提取 force 的 x/y/z，用于实时观察受力变化量。
        wrench = msg.get("wrench") if isinstance(msg.get("wrench"), dict) else msg
        force = wrench.get("force") if isinstance(wrench.get("force"), dict) else {}

        fx = force.get("x")
        fy = force.get("y")
        fz = force.get("z")

        if all(v is not None for v in [fx, fy, fz]):
            return [float(fx), float(fy), float(fz)]

        alt_force = msg.get("force") if isinstance(msg.get("force"), list) else None
        if isinstance(alt_force, list) and len(alt_force) >= 3:
            return [float(alt_force[0]), float(alt_force[1]), float(alt_force[2])]

        raise ValueError("不支持的 Force XYZ 消息格式")

    def factory_enable_force_sensor(self):
        if not self._ensure_ssh():
            self._emit_factory_test("[ERR] 步骤6配置失败: 请先连接 SSH(小脑)")
            return

        def worker():
            try:
                user = self.ssh_user.text().strip() if hasattr(self, "ssh_user") else "nav01"
                yaml_path = f"/home/{user}/zj_humanoid/config/naviai_default.yaml"
                if user != "nav01":
                    self._emit_factory_test(f"[INFO] 步骤6配置目标用户: {user}")
                self._emit_factory_test(f"[INFO] 步骤6开始配置六维力传感器: {yaml_path}")

                script = (
                    "import re, sys\n"
                    "p = sys.argv[1]\n"
                    "with open(p, 'r', encoding='utf-8') as f:\n"
                    "    lines = f.readlines()\n"
                    "hand_idx = -1\n"
                    "hand_indent = 0\n"
                    "for i, ln in enumerate(lines):\n"
                    "    if re.match(r'^\\s*hand\\s*:\\s*(#.*)?$', ln):\n"
                    "        hand_idx = i\n"
                    "        hand_indent = len(ln) - len(ln.lstrip(' '))\n"
                    "        break\n"
                    "if hand_idx < 0:\n"
                    "    raise RuntimeError('未找到 hand 配置段')\n"
                    "found = False\n"
                    "insert_idx = hand_idx + 1\n"
                    "for j in range(hand_idx + 1, len(lines)):\n"
                    "    cur = lines[j]\n"
                    "    stripped = cur.strip()\n"
                    "    if stripped and (len(cur) - len(cur.lstrip(' ')) <= hand_indent) and (not stripped.startswith('#')):\n"
                    "        break\n"
                    "    insert_idx = j + 1\n"
                    "    m = re.match(r'^(\\s*enable_force_sensor\\s*:\\s*)(true|false)(\\s*(#.*)?)?\\s*$', cur)\n"
                    "    if m:\n"
                    "        lines[j] = f\"{m.group(1)}true{m.group(3) or ''}\\n\"\n"
                    "        found = True\n"
                    "        break\n"
                    "if not found:\n"
                    "    pad = ' ' * (hand_indent + 2)\n"
                    "    lines.insert(insert_idx, f\"{pad}enable_force_sensor: true\\n\")\n"
                    "with open(p, 'w', encoding='utf-8') as f:\n"
                    "    f.writelines(lines)\n"
                    "print('enable_force_sensor=true')\n"
                )
                set_cmd = f"python3 -c {shlex.quote(script)} {shlex.quote(yaml_path)}"
                out, err, exit_code = self._run_bash_with_exit_code(self.ssh, set_cmd)
                if exit_code != 0:
                    msg = ((out or "") + "\n" + (err or "")).strip()
                    raise RuntimeError(msg or f"更新配置失败，exit_code={exit_code}")
                if (out or "").strip():
                    self._emit_factory_test((out or "").strip())
                self._emit_factory_test("[OK] 步骤6六维力传感器已启用（未自动重启中间件）")
            except Exception as e:
                self._emit_factory_test(f"[ERR] 步骤6配置六维力传感器失败: {e}")

        self._run_async(worker)

    def factory_capture_force_values(self, mode: str):
        mode_name = str(mode or "").strip().lower()
        if mode_name != "baseline":
            return
        ros_ok = bool(self.ros and self.ros.check_connection())
        ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
        if not ros_ok and not ssh_ok:
            self._emit_factory_test("[ERR] 步骤6Force基线采集失败: 请先连接 ROSBridge 或 SSH(小脑)")
            return

        left_topic = self._factory_force_topics.get("left", "")
        right_topic = self._factory_force_topics.get("right", "")

        def worker():
            try:
                self._emit_factory_test("[INFO] 步骤6开始采集 Force 基线")
                if ros_ok:
                    left_msg = self._maintenance_topic_once(left_topic, timeout=2.5)
                    right_msg = self._maintenance_topic_once(right_topic, timeout=2.5)
                else:
                    self._emit_factory_test("[INFO] 步骤6Force基线采集使用 SSH(小脑)")
                    left_out, left_err = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {shlex.quote(left_topic)}")
                    right_out, right_err = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {shlex.quote(right_topic)}")
                    left_msg = self._parse_ros_cli_yaml((left_out or "") + "\n" + (left_err or ""))
                    right_msg = self._parse_ros_cli_yaml((right_out or "") + "\n" + (right_err or ""))
                if not isinstance(left_msg, dict):
                    raise RuntimeError(f"左手力传感器话题超时: {left_topic}")
                if not isinstance(right_msg, dict):
                    raise RuntimeError(f"右手力传感器话题超时: {right_topic}")

                left_vals = self._factory_extract_force_values_from_msg(left_msg)
                right_vals = self._factory_extract_force_values_from_msg(right_msg)
                force_values = {"left": left_vals, "right": right_vals}
                text = self._factory_format_force_values(force_values)

                self._factory_reset_force_delta_max()
                self.factory_force_values_signal.emit("baseline", force_values)
                self._emit_factory_test(f"[OK] 步骤6Force基线采集完成: {text}")
            except Exception as e:
                self._emit_factory_test(f"[ERR] 步骤6Force基线采集失败: {e}")

        self._run_async(worker)

    def factory_start_force_monitor(self):
        if self._factory_force_subscribers or self._factory_force_polling_active:
            self._emit_factory_test("[INFO] 步骤6Force检测已在运行")
            return
        ros_ok = bool(self.ros and self.ros.check_connection())
        ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
        if not ros_ok and not ssh_ok:
            self._emit_factory_test("[ERR] 步骤6Force检测启动失败: 请先连接 ROSBridge 或 SSH(小脑)")
            return
        self._factory_force_last_emit_ts = {"left": 0.0, "right": 0.0}
        self._factory_reset_force_delta_max()
        self.factory_force_monitor_state_signal.emit(False, "")

        def worker():
            try:
                if not ros_ok:
                    cmd = self._build_remote_ros_echo_poller_command("force", self._factory_force_topics, interval_sec=0.05)

                    def _on_line(line: str):
                        text = str(line or "").strip()
                        if not text:
                            return
                        try:
                            evt = json.loads(text)
                        except Exception:
                            return
                        kind = str(evt.get("kind") or "")
                        if kind == "sample":
                            side_name = str(evt.get("side") or "")
                            msg = self._parse_ros_cli_yaml(evt.get("text") or "")
                            if not isinstance(msg, dict):
                                return
                            try:
                                now = time.monotonic()
                                last_emit = float(self._factory_force_last_emit_ts.get(side_name, 0.0) or 0.0)
                                if last_emit > 0.0 and (now - last_emit) < 1.0:
                                    return
                                values = self._factory_extract_force_values_from_msg(msg)
                                self._factory_force_last_emit_ts[side_name] = now
                                self.factory_force_live_signal.emit(side_name, values)
                            except Exception as sub_err:
                                self._emit_factory_test(f"[WARN] 步骤6Force解析失败({side_name}): {sub_err}")
                            return
                        if kind == "error":
                            side_name = str(evt.get("side") or "")
                            err_text = str(evt.get("error") or "unknown")
                            self._emit_factory_test(f"[WARN] 步骤6Force检测SSH读取失败({side_name}): {err_text}")

                    def _on_exit(_out: str, _err: str, _exit_code):
                        self._factory_force_ssh_session = None

                    self._factory_force_ssh_session = self.ssh.start_interactive_session(
                        cmd,
                        on_output=_on_line,
                        on_exit=_on_exit,
                    )
                    self._factory_force_polling_active = True
                    self.factory_force_monitor_state_signal.emit(True, "[OK] 步骤6已开始监听双手 Force XYZ(SSH脚本)")
                    return

                subscribers = {}

                def _make_callback(side_name: str):
                    def _callback(msg: dict):
                        try:
                            now = time.monotonic()
                            last_emit = float(self._factory_force_last_emit_ts.get(side_name, 0.0) or 0.0)
                            if last_emit > 0.0 and (now - last_emit) < 1.0:
                                return
                            values = self._factory_extract_force_values_from_msg(msg)
                            self._factory_force_last_emit_ts[side_name] = now
                            self.factory_force_live_signal.emit(side_name, values)
                        except Exception as e:
                            self._emit_factory_test(f"[ERR] 步骤6Force解析失败({side_name}): {e}")
                    return _callback

                for side_name, topic in self._factory_force_topics.items():
                    self.ros.subscribe(topic, _make_callback(side_name))
                    subscribers[side_name] = topic

                self._factory_force_subscribers = subscribers
                self.factory_force_monitor_state_signal.emit(True, "[OK] 步骤6已开始监听双手 Force XYZ")
            except Exception as e:
                try:
                    for topic in self._factory_force_topics.values():
                        self.ros.unsubscribe(topic)
                except Exception:
                    pass
                self._factory_force_subscribers = {}
                self.factory_force_monitor_state_signal.emit(False, f"[ERR] 步骤6Force检测启动失败: {e}")

        self._run_async(worker)

    def factory_stop_force_monitor(self, emit_log: bool = True):
        self._factory_force_polling_active = False
        if self._factory_force_ssh_session:
            try:
                self._factory_force_ssh_session.close()
            except Exception:
                pass
            self._factory_force_ssh_session = None
        ros_client = self.ros
        if ros_client:
            for topic in self._factory_force_topics.values():
                try:
                    ros_client.unsubscribe(topic)
                except Exception:
                    pass
        self._factory_force_subscribers = {}
        self._factory_force_last_emit_ts = {"left": 0.0, "right": 0.0}
        self.factory_force_monitor_state_signal.emit(False, "")
        if emit_log:
            self._emit_factory_test("[INFO] 步骤6已停止 Force 检测")

    def factory_compute_force_delta(self):
        try:
            baseline = self._factory_force_values_by_key("baseline")
            current = self._factory_force_values_by_key("current")

            delta = {}
            for side in ("left", "right"):
                if side in baseline and side in current:
                    delta[side] = [current[side][i] - baseline[side][i] for i in range(3)]
            if not delta:
                raise ValueError("初始值和检测值格式不一致")

            self._factory_set_force_values("delta", delta)
            self._emit_factory_test("[OK] 步骤6Force XYZ 偏差计算完成")
        except Exception as e:
            self._emit_factory_test(f"[ERR] 步骤6Force XYZ 计算失败: {e}")

    def factory_step3_finish(self):
        if not self._factory_force_has_values("delta"):
            self.factory_compute_force_delta()
        max_delta_txt = self._factory_force_delta_max_text()
        if max_delta_txt == "-":
            self._emit_factory_test("[ERR] 步骤6未完成：请先采集基线并开启 Force 检测")
            return
        self.factory_mark_step_done(6, f"Force XYZ 历史最大偏差={max_delta_txt}")

    def call_factory_tts(self):
        text = self.factory_tts_text.text().strip() if hasattr(self, "factory_tts_text") else ""
        if not text:
            self._emit_factory_test("[WARN] 请输入 TTS 文本")
            return
        request = {
            "text": [text],
            "isPlay": bool(self.factory_tts_play_checkbox.isChecked()) if hasattr(self, "factory_tts_play_checkbox") else True,
        }
        self._call_ros_service_async(
            "工厂测试TTS",
            "/zj_humanoid/audio/tts_service",
            request,
            timeout=8.0,
            emit_line=self._emit_factory_test,
        )

    def call_factory_set_volume(self):
        raw = self.factory_audio_volume_edit.text().strip() if hasattr(self, "factory_audio_volume_edit") else ""
        if not raw:
            self._emit_factory_test("[WARN] 请输入音量值")
            return
        try:
            volume = int(float(raw))
        except Exception:
            self._emit_factory_test("[ERR] 音量必须是数字")
            return
        request = {"volume": volume}
        self._call_ros_service_async(
            "工厂测试设置音量",
            "/zj_humanoid/audio/speaker/set_volume",
            request,
            timeout=8.0,
            emit_line=self._emit_factory_test,
        )

    def _factory_current_model(self) -> str:
        model = str(getattr(self, "robot_model", "") or "").strip().upper()
        if model in {"WA1", "WA2", "I2", "WA2_LS"}:
            return model
        if model.startswith("I"):
            return "I2"
        return "WA2"

    def factory_step5_finish(self):
        model = self._factory_current_model()
        requirement = self._factory_camera_requirement_state()
        labels = self._factory_camera_slot_labels(model)
        head_ok = bool(self._factory_camera_last_image_path.get("head"))
        chest_ok = bool(self._factory_camera_last_image_path.get("chest"))

        if requirement["head_required"] and not head_ok:
            self._emit_factory_test(f"[ERR] 步骤8未完成：未收到{labels['head']}图像")
            return
        if requirement["chest_required"] and not chest_ok:
            self._emit_factory_test(f"[ERR] 步骤8未完成：未收到{labels['chest']}图像")
            return

        note = (
            f"model={model}, "
            f"head_required={'Y' if requirement['head_required'] else 'N'}, "
            f"chest_required={'Y' if requirement['chest_required'] else 'N'}, "
            f"head={self._factory_camera_last_image_path.get('head') or '-'}, "
            f"chest={self._factory_camera_last_image_path.get('chest') or '-'}"
        )
        self.factory_mark_step_done(8, note)

    def factory_generate_report(self):
        now = datetime.now()
        flow_state = self._factory_normalize_flow_state()
        factory_info = self._factory_info_snapshot()
        model = str(factory_info.get("robot_model") or "").strip() or self._factory_current_model()
        robot_sn = self.factory_robot_sn_edit.text().strip() if hasattr(self, "factory_robot_sn_edit") else ""
        camera_requirement = self._factory_camera_requirement_state()
        all_done = all(bool(flow_state.get(s, {}).get("done")) for s in self._factory_flow_steps)
        result = "PASS" if all_done else "INCOMPLETE"

        lines = []
        lines.append("=== 工厂测试报告 ===")
        lines.append(f"生成时间: {now.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"机器人型号: {model}")
        lines.append(f"机器人SN: {robot_sn or '-'}")
        lines.append(f"总体结果: {result}")
        lines.append("")
        lines.append("[步骤结果]")
        for step in self._factory_flow_steps:
            item = flow_state.get(step, {})
            done = bool(item.get("done"))
            done_txt = "已检测" if done else "未检测"
            t = item.get("time") or "-"
            lines.append(f"步骤{step}-{self._factory_step_label(step)}: {done_txt} | 时间={t}")

        lines.append("")
        lines.append("[步骤1 机器人信息]")
        lines.append(f"robot_model={factory_info['robot_model']}")
        lines.append(f"embedded_version={factory_info['embedded_version']}")
        lines.append(f"middleware_version={factory_info['middleware_version']}")
        lines.append(f"upperlimb_version={factory_info['upperlimb_version']}")
        if factory_info.get("show_lowerlimb"):
            lines.append(f"lowerlimb_version={factory_info['lowerlimb_version']}")
        lines.append("system_info=")
        lines.append(factory_info["system_info"])

        audit = getattr(self, "_factory_topic_service_audit", {}) or {}
        lines.append("")
        lines.append("[步骤3 ROS话题检测]")
        lines.append(f"topic_count={int(audit.get('topic_count') or 0)}")
        topic_rows = list(audit.get("topics") or [])
        lines.append("topic_details=")
        if topic_rows:
            for item in topic_rows:
                lines.append(
                    f"{item.get('name') or '-'} | hz={item.get('hz_text') or self._factory_topic_rate_text(item.get('hz'))}"
                )
        else:
            lines.append("-")

        lines.append("")
        lines.append("[步骤5 手指压力最大值]")
        left_vals = self._factory_pressure_max.get("left", [])
        right_vals = self._factory_pressure_max.get("right", [])
        lines.append("left=" + ",".join(["-" if v is None else f"{float(v):.2f}" for v in left_vals]))
        lines.append("right=" + ",".join(["-" if v is None else f"{float(v):.2f}" for v in right_vals]))

        lines.append("")
        lines.append("[步骤6 六维力]")
        lines.append(f"baseline={self._factory_force_text_by_key('baseline')}")
        lines.append(f"current={self._factory_force_text_by_key('current')}")
        lines.append(f"delta={self._factory_force_text_by_key('delta')}")
        lines.append(f"delta_max={self._factory_force_delta_max_text()}")

        lines.append("")
        lines.append("[步骤8 相机图像]")
        lines.append(f"head_required={'Y' if camera_requirement['head_required'] else 'N'}")
        lines.append(f"chest_required={'Y' if camera_requirement['chest_required'] else 'N'}")

        lines.append("")
        lines.append("[附件]")
        lines.append("工厂测试输出日志: 见报告附件章节")
        if self._factory_report_records:
            lines.append(f"附件日志条数: {len(self._factory_report_records)}")
        else:
            lines.append("附件日志条数: 0")

        report_text = "\n".join(lines)
        self.factory_report_text.clear()

        try:
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            report_dir = os.path.join(project_root, "logs", "factory_reports")
            os.makedirs(report_dir, exist_ok=True)
            safe_robot_sn = re.sub(r"[^A-Za-z0-9._-]+", "_", robot_sn).strip("_")
            base_name = f"{safe_robot_sn or 'factory_report'}_{now.strftime('%Y%m%d_%H%M%S')}"

            def _copy_report_image(src_path: str, suffix: str) -> str:
                src = str(src_path or "").strip()
                if not src or not os.path.isfile(src):
                    return ""
                ext = os.path.splitext(src)[1].lower() or ".png"
                dst_path = os.path.join(report_dir, f"{base_name}_{suffix}{ext}")
                shutil.copy2(src, dst_path)
                return dst_path

            head_report_image = _copy_report_image(self._factory_camera_last_image_path.get("head"), "head")
            chest_report_image = _copy_report_image(self._factory_camera_last_image_path.get("chest"), "chest")

            txt_path = os.path.join(report_dir, f"{base_name}.txt")
            with open(txt_path, "w", encoding="utf-8") as f:
                f.write(report_text)

            report_data = {
                "generated_at": now.strftime("%Y-%m-%d %H:%M:%S"),
                "robot_model": model,
                "robot_sn": robot_sn,
                "overall_result": result,
                "steps": flow_state,
                "factory_info": dict(factory_info),
                "ros_audit": dict(audit),
                "pressure_max": self._factory_pressure_max,
                "force": {
                    "baseline": self._factory_force_text_by_key("baseline"),
                    "current": self._factory_force_text_by_key("current"),
                    "delta": self._factory_force_text_by_key("delta"),
                    "delta_max": self._factory_force_delta_max_text(),
                    "delta_max_values": self._factory_force_delta_max,
                },
                "camera": {
                    "head_required": camera_requirement["head_required"],
                    "chest_required": camera_requirement["chest_required"],
                    "head": self._factory_camera_last_image_path.get("head") or "",
                    "chest": self._factory_camera_last_image_path.get("chest") or "",
                    "head_report_image": head_report_image,
                    "chest_report_image": chest_report_image,
                },
                "output_records": list(self._factory_report_records),
                "output_text": self.factory_test_output.toPlainText() if hasattr(self, "factory_test_output") else "",
            }

            json_path = os.path.join(report_dir, f"{base_name}.json")
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(report_data, f, ensure_ascii=False, indent=2)

            pdf_path = os.path.join(report_dir, f"{base_name}.pdf")

            def _register_pdf_font() -> str:
                font_name = "STSong-Light"
                if font_name not in pdfmetrics.getRegisteredFontNames():
                    pdfmetrics.registerFont(UnicodeCIDFont(font_name))
                return font_name

            def _pdf_para(text: str, font_name: str, size: int = 10, leading: int | None = None):
                style = ParagraphStyle(
                    name="FactoryPDFText",
                    fontName=font_name,
                    fontSize=size,
                    leading=leading or int(size * 1.35),
                    wordWrap="CJK",
                )
                return Paragraph(html.escape(str(text or "")).replace("\n", "<br/>"), style)

            def _pdf_table(rows: list[list], font_name: str, col_widths=None):
                table = Table(rows, colWidths=col_widths, repeatRows=1)
                table.setStyle(TableStyle([
                    ("FONTNAME", (0, 0), (-1, -1), font_name),
                    ("FONTSIZE", (0, 0), (-1, -1), 9),
                    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#f3f4f6")),
                    ("TEXTCOLOR", (0, 0), (-1, 0), colors.HexColor("#111827")),
                    ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#d1d5db")),
                    ("VALIGN", (0, 0), (-1, -1), "TOP"),
                    ("LEADING", (0, 0), (-1, -1), 12),
                    ("LEFTPADDING", (0, 0), (-1, -1), 4),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                    ("TOPPADDING", (0, 0), (-1, -1), 4),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                ]))
                return table

            def _pdf_image_block(image_path: str, title: str):
                if not image_path or not os.path.isfile(image_path):
                    story.append(_pdf_para(f"{title}: 无图像", font_name, 10))
                    return
                story.append(_pdf_para(title, font_name, 10))
                image_flowable = RLImage(image_path)
                image_flowable._restrictSize(460, 260)
                story.append(image_flowable)
                story.append(Spacer(1, 6))

            font_name = _register_pdf_font()
            doc = SimpleDocTemplate(
                pdf_path,
                pagesize=A4,
                leftMargin=28,
                rightMargin=28,
                topMargin=28,
                bottomMargin=28,
            )
            styles = getSampleStyleSheet()
            title_style = ParagraphStyle(
                name="FactoryPDFTitle",
                parent=styles["Title"],
                fontName=font_name,
                fontSize=18,
                leading=22,
                textColor=colors.HexColor("#111827"),
            )
            header_style = ParagraphStyle(
                name="FactoryPDFHeader",
                parent=styles["Heading2"],
                fontName=font_name,
                fontSize=12,
                leading=15,
                textColor=colors.HexColor("#111827"),
            )
            normal_style = ParagraphStyle(
                name="FactoryPDFNormal",
                parent=styles["BodyText"],
                fontName=font_name,
                fontSize=10,
                leading=14,
                wordWrap="CJK",
            )
            story = [
                Paragraph("工厂测试报告", title_style),
                Spacer(1, 8),
                _pdf_para(f"生成时间: {now.strftime('%Y-%m-%d %H:%M:%S')}", font_name, 10),
                _pdf_para(f"机器人型号: {model}", font_name, 10),
                _pdf_para(f"机器人SN: {robot_sn or '-'}", font_name, 10),
                _pdf_para(f"总体结果: {result}", font_name, 10),
                Spacer(1, 8),
                Paragraph("机器人信息", header_style),
                _pdf_para(f"机器人型号: {factory_info['robot_model']}", font_name, 10),
                _pdf_para(f"嵌入式版本: {factory_info['embedded_version']}", font_name, 10),
                _pdf_para(f"中间件版本: {factory_info['middleware_version']}", font_name, 10),
                _pdf_para(f"上肢版本: {factory_info['upperlimb_version']}", font_name, 10),
                *([_pdf_para(f"下肢版本: {factory_info['lowerlimb_version']}", font_name, 10)] if factory_info.get("show_lowerlimb") else []),
                _pdf_para(f"系统信息:\n{factory_info['system_info']}", font_name, 10),
                Spacer(1, 8),
                Paragraph("步骤结果", header_style),
            ]

            topic_rows = list(audit.get("topics") or [])
            step_table_rows = [["步骤", "名称", "结果", "时间"]]
            for step in self._factory_flow_steps:
                item = flow_state.get(step, {})
                done = bool(item.get("done"))
                step_table_rows.append([
                    _pdf_para(str(step), font_name, 9, 12),
                    _pdf_para(self._factory_step_label(step), font_name, 9, 12),
                    _pdf_para("已检测" if done else "未检测", font_name, 9, 12),
                    _pdf_para(str(item.get("time") or "-"), font_name, 9, 12),
                ])
            story.append(_pdf_table(step_table_rows, font_name, [36, 170, 80, 180]))
            story.append(Spacer(1, 8))

            story.append(Paragraph("步骤3 ROS话题检测", header_style))
            story.append(_pdf_para(f"话题数: {int(audit.get('topic_count') or 0)}", font_name, 10))
            story.append(Spacer(1, 8))
            if topic_rows:
                ros_topic_rows = [["话题", "频率"]]
                for item in topic_rows:
                    ros_topic_rows.append([
                        _pdf_para(str(item.get("name") or "-"), font_name, 8, 11),
                        _pdf_para(str(item.get("hz_text") or self._factory_topic_rate_text(item.get("hz"))), font_name, 8, 11),
                    ])
                story.append(Paragraph("步骤3 话题频率明细", header_style))
                story.append(_pdf_table(ros_topic_rows, font_name, [320, 160]))
                story.append(Spacer(1, 8))

            pressure_rows = [["手指", "左手最大值", "右手最大值"]]
            pressure_names = self._finger_labels_for_side()
            for idx, name in enumerate(pressure_names):
                lv = self._factory_pressure_max.get("left", [None] * 6)[idx]
                rv = self._factory_pressure_max.get("right", [None] * 6)[idx]
                pressure_rows.append([
                    name,
                    "-" if lv is None else f"{float(lv):.2f}",
                    "-" if rv is None else f"{float(rv):.2f}",
                ])
            story.extend([
                Paragraph("步骤5 手指压力最大值", header_style),
                _pdf_table(pressure_rows, font_name, [90, 180, 180]),
                Spacer(1, 8),
                Paragraph("步骤6 六维力", header_style),
                _pdf_para(f"初始值: {self._factory_force_text_by_key('baseline')}", font_name, 10),
                _pdf_para(f"检测值: {self._factory_force_text_by_key('current')}", font_name, 10),
                _pdf_para(f"偏差: {self._factory_force_text_by_key('delta')}", font_name, 10),
                _pdf_para(f"历史最大偏差: {self._factory_force_delta_max_text()}", font_name, 10),
                Spacer(1, 8),
                Paragraph("步骤8 相机输出", header_style),
                _pdf_para(f"头部必须检测: {'是' if camera_requirement['head_required'] else '否'}", font_name, 10),
                _pdf_para(f"胸部必须检测: {'是' if camera_requirement['chest_required'] else '否'}", font_name, 10),
                Spacer(1, 8),
                Paragraph("附件", header_style),
            ])

            _pdf_image_block(head_report_image, "头部相机图片")
            _pdf_image_block(chest_report_image, "胸部相机图片")

            output_lines = list(self._factory_report_records or [])
            if output_lines:
                story.append(_pdf_para("工厂测试输出日志", font_name, 10))
                log_rows = [["时间", "内容"]]
                for item in output_lines:
                    log_rows.append([
                        _pdf_para(str(item.get("time") or "-"), font_name, 9, 12),
                        _pdf_para(str(item.get("message") or ""), font_name, 9, 12),
                    ])
                story.append(_pdf_table(log_rows, font_name, [120, 340]))
            else:
                story.append(_pdf_para("工厂测试输出日志", font_name, 10))
                story.append(_pdf_para(self.factory_test_output.toPlainText() if hasattr(self, "factory_test_output") else "", font_name, 9, 12))

            doc.build(story)

            report_data["pdf_path"] = pdf_path
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(report_data, f, ensure_ascii=False, indent=2)

            def _step_row_html(step: int) -> str:
                item = flow_state.get(step, {})
                done = bool(item.get("done"))
                status = "已检测" if done else "未检测"
                t = html.escape(str(item.get("time") or "-"))
                return (
                    "<tr>"
                    f"<td>{step}</td>"
                    f"<td>{html.escape(self._factory_step_label(step))}</td>"
                    f"<td>{status}</td>"
                    f"<td>{t}</td>"
                    "</tr>"
                )

            html_path = os.path.join(report_dir, f"{base_name}.html")
            topic_rows = list(audit.get("topics") or [])
            ros_topic_rows_html = []
            for item in topic_rows:
                ros_topic_rows_html.append(
                    "<tr>"
                    f"<td>{html.escape(str(item.get('name') or '-'))}</td>"
                    f"<td>{html.escape(str(item.get('hz_text') or self._factory_topic_rate_text(item.get('hz'))))}</td>"
                    "</tr>"
                )
            pressure_names = self._finger_labels_for_side()
            pressure_rows = []
            for idx, name in enumerate(pressure_names):
                lv = self._factory_pressure_max.get("left", [None] * 6)[idx]
                rv = self._factory_pressure_max.get("right", [None] * 6)[idx]
                pressure_rows.append(
                    "<tr>"
                    f"<td>{html.escape(name)}</td>"
                    f"<td>{'-' if lv is None else f'{float(lv):.2f}'}</td>"
                    f"<td>{'-' if rv is None else f'{float(rv):.2f}'}</td>"
                    "</tr>"
                )

            output_html = html.escape(self.factory_test_output.toPlainText() if hasattr(self, "factory_test_output") else "")
            output_records = list(self._factory_report_records or [])
            if output_records:
                attachment_rows = []
                for item in output_records:
                    attachment_rows.append(
                        "<tr>"
                        f"<td>{html.escape(str(item.get('time') or '-'))}</td>"
                        f"<td>{html.escape(str(item.get('message') or ''))}</td>"
                        "</tr>"
                    )
                attachment_html = (
                    "<h3>工厂测试输出日志</h3>"
                    "<table><thead><tr><th>时间</th><th>内容</th></tr></thead><tbody>"
                    + "".join(attachment_rows)
                    + "</tbody></table>"
                )
            else:
                attachment_html = f"<h3>工厂测试输出日志</h3><div class='mono'>{output_html}</div>"

            def _html_image_block(label: str, image_path: str) -> str:
                if not image_path or not os.path.isfile(image_path):
                    return f"<p><b>{html.escape(label)}:</b> 无图像</p>"
                img_name = html.escape(os.path.basename(image_path))
                return (
                    f"<div style='margin: 10px 0;'>"
                    f"<div><b>{html.escape(label)}:</b></div>"
                    f"<img src='{img_name}' style='max-width: 100%; border: 1px solid #d1d5db; padding: 4px; background: #fff;'/>"
                    f"</div>"
                )

            html_text = (
                "<!doctype html>\n"
                "<html><head><meta charset='utf-8'>"
                "<title>Factory Test Report</title>"
                "<style>"
                "body{font-family:Arial,Helvetica,sans-serif;margin:20px;color:#1f2937;}"
                "h1{margin-bottom:8px;}"
                "table{border-collapse:collapse;width:100%;margin-bottom:16px;}"
                "th,td{border:1px solid #d1d5db;padding:6px 8px;text-align:left;font-size:13px;vertical-align:top;white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere;}"
                "th{background:#f3f4f6;}"
                ".mono{white-space:pre-wrap;background:#111827;color:#e5e7eb;padding:10px;border-radius:6px;}"
                "</style></head><body>"
                "<h1>工厂测试报告</h1>"
                f"<p><b>生成时间:</b> {html.escape(now.strftime('%Y-%m-%d %H:%M:%S'))}</p>"
                f"<p><b>机器人型号:</b> {html.escape(model)} | <b>机器人SN:</b> {html.escape(robot_sn or '-')} | <b>总体结果:</b> {html.escape(result)}</p>"
                "<h2>机器人信息</h2>"
                f"<p><b>机器人型号:</b> {html.escape(factory_info['robot_model'])}</p>"
                f"<p><b>嵌入式版本:</b> {html.escape(factory_info['embedded_version'])}</p>"
                f"<p><b>中间件版本:</b> {html.escape(factory_info['middleware_version'])}</p>"
                f"<p><b>上肢版本:</b> {html.escape(factory_info['upperlimb_version'])}</p>"
                + (f"<p><b>下肢版本:</b> {html.escape(factory_info['lowerlimb_version'])}</p>" if factory_info.get("show_lowerlimb") else "")
                + f"<div class='mono'>{html.escape(factory_info['system_info'])}</div>"
                + "<h2>步骤结果</h2>"
                "<table><thead><tr><th>步骤</th><th>名称</th><th>结果</th><th>时间</th></tr></thead><tbody>"
                + "".join([_step_row_html(step) for step in self._factory_flow_steps])
                + "</tbody></table>"
                + "<h2>步骤3 ROS话题检测</h2>"
                + f"<p><b>话题数:</b> {int(audit.get('topic_count') or 0)}</p>"
                + (
                    "<h3>步骤3 话题频率明细</h3>"
                    "<table><thead><tr><th>话题</th><th>频率</th></tr></thead><tbody>"
                    + "".join(ros_topic_rows_html)
                    + "</tbody></table>"
                    if ros_topic_rows_html else ""
                )
                + "<h2>步骤5 手指压力最大值</h2>"
                + "<table><thead><tr><th>手指</th><th>左手</th><th>右手</th></tr></thead><tbody>"
                + "".join(pressure_rows)
                + "</tbody></table>"
                + "<h2>步骤6 六维力</h2>"
                f"<p><b>初始值:</b> {html.escape(self._factory_force_text_by_key('baseline'))}</p>"
                f"<p><b>检测值:</b> {html.escape(self._factory_force_text_by_key('current'))}</p>"
                f"<p><b>偏差:</b> {html.escape(self._factory_force_text_by_key('delta'))}</p>"
                f"<p><b>历史最大偏差:</b> {html.escape(self._factory_force_delta_max_text())}</p>"
                + "<h2>步骤8 相机输出</h2>"
                f"<p><b>头部必须检测:</b> {'是' if camera_requirement['head_required'] else '否'}</p>"
                f"<p><b>胸部必须检测:</b> {'是' if camera_requirement['chest_required'] else '否'}</p>"
                f"{_html_image_block('头部相机图片', head_report_image)}"
                f"{_html_image_block('胸部相机图片', chest_report_image)}"
                + "<h2>附件</h2>"
                f"{attachment_html}"
                "</body></html>"
            )
            with open(html_path, "w", encoding="utf-8") as f:
                f.write(html_text)

            self._emit_factory_test(f"[OK] 工厂测试报告已生成: {txt_path}")
            self._emit_factory_test(f"[OK] PDF报告已生成: {pdf_path}")
            self._emit_factory_test(f"[OK] HTML报告已生成: {html_path}")
            self._emit_factory_test(f"[OK] 原始记录数据已保存: {json_path}")
        except Exception as e:
            self._emit_factory_test(f"[ERR] 报告保存失败: {e}")

        self.factory_report_text.clear()

    def _confirm_container_action(self, title: str, detail: str = "") -> bool:
        text = f"确认执行：{title}"
        if detail:
            text += f"\n{detail}"
        text += "\n\n请再次确认以防误触。"
        ret = QMessageBox.question(
            self,
            "二次确认",
            text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return ret == QMessageBox.StandardButton.Yes

    def container_status(self):
        if not self._confirm_container_action("查看容器状态"):
            return
        self._run_container_action(["status"], "容器状态")

    def container_images(self):
        if not self._confirm_container_action("查看镜像列表"):
            return
        self._run_container_action(["images"], "镜像列表")

    def container_deploy(self):
        service = self.container_service_combo.currentText().strip() if hasattr(self, "container_service_combo") else ""
        version = self.container_version_edit.text().strip() if hasattr(self, "container_version_edit") else "latest"
        if not self._confirm_container_action("部署服务", f"service={service or '-'} version={version or 'latest'}"):
            return
        if not service:
            self._emit_container("[ERR] 请选择服务")
            return
        version = version or "latest"
        self._run_container_action(["deploy", service, version], f"部署服务 {service}")

    def container_deploy_all(self):
        version = self.container_version_edit.text().strip() if hasattr(self, "container_version_edit") else "latest"
        if not self._confirm_container_action("部署全部服务", f"version={version or 'latest'}"):
            return
        version = version or "latest"
        self._run_container_action(["deploy-all", version], f"部署全部服务 {version}")

    def container_stop(self):
        service = self.container_service_combo.currentText().strip() if hasattr(self, "container_service_combo") else ""
        if not self._confirm_container_action("停止服务", f"service={service or '-'}"):
            return
        if not service:
            self._emit_container("[ERR] 请选择服务")
            return
        self._run_container_action(["stop", service], f"停止服务 {service}")

    def container_logs(self):
        service = self.container_service_combo.currentText().strip() if hasattr(self, "container_service_combo") else ""
        lines = self.container_logs_lines_edit.text().strip() if hasattr(self, "container_logs_lines_edit") else "100"
        if not self._confirm_container_action("查看服务日志", f"service={service or '-'} lines={lines or '100'}"):
            return
        if not service:
            self._emit_container("[ERR] 请选择服务")
            return
        try:
            lines_i = int(lines or "100")
        except Exception:
            lines_i = 100
        lines_i = max(1, min(lines_i, 2000))
        self._run_container_action(["logs", service, str(lines_i)], f"服务日志 {service}")

    def container_rollback(self):
        service = self.container_service_combo.currentText().strip() if hasattr(self, "container_service_combo") else ""
        if not self._confirm_container_action("回滚服务", f"service={service or '-'}"):
            return
        if not service:
            self._emit_container("[ERR] 请选择服务")
            return
        self._run_container_action(["rollback", service], f"回滚服务 {service}")

    def container_help(self):
        if not self._confirm_container_action("查看容器脚本帮助"):
            return
        self._run_container_action(["help"], "容器脚本帮助")

    def container_clear_output(self):
        if not self._confirm_container_action("清空容器操作输出"):
            return
        self.container_clear_signal.emit()

    def _clear_container_runtime_list_state(self):
        self._container_runtime_entries = []
        self._container_runtime_details = {}
        if hasattr(self, "container_runtime_combo"):
            self.container_runtime_combo.blockSignals(True)
            self.container_runtime_combo.clear()
            self.container_runtime_combo.blockSignals(False)
        if hasattr(self, "container_runtime_info"):
            self.container_runtime_info.setText("未读取容器")

    def _selected_container_runtime_name(self) -> str:
        if not hasattr(self, "container_runtime_combo"):
            return ""
        return self.container_runtime_combo.currentText().strip()

    def _current_container_runtime_path(self) -> str:
        if not hasattr(self, "container_runtime_path_edit"):
            return ""
        return self.container_runtime_path_edit.text().strip()

    def _list_container_runtime_dir(self, container_name: str, dir_path: str):
        base_dir = self._normalize_remote_posix_path(dir_path or "/")
        script = (
            "target_dir=" + shlex.quote(base_dir) + "; "
            'if [ ! -d "$target_dir" ]; then echo "__ERR__\tNOT_DIR"; exit 17; fi; '
            'cd -- "$target_dir" || exit 17; '
            'printf "__PWD__\t%s\n" "$PWD"; '
            "ls -A1p"
        )
        cmd = f"docker exec {shlex.quote(container_name)} sh -lc {shlex.quote(script)}"
        out, err = self._run_bash(self.ssh_big, cmd)
        raw = ((out or "") + "\n" + (err or "")).strip()
        if "__ERR__\tNOT_DIR" in raw:
            raise RuntimeError(f"目录不存在: {base_dir}")
        lines = [line.rstrip("\n") for line in raw.splitlines() if line.strip()]
        current_dir = base_dir
        dirs = []
        files = []
        for line in lines:
            if line.startswith("__PWD__\t"):
                current_dir = self._normalize_remote_posix_path(line.split("\t", 1)[1])
                continue
            if line.endswith("/"):
                name = line[:-1]
                if name:
                    full = f"{current_dir.rstrip('/')}/{name}" if current_dir != "/" else f"/{name}"
                    dirs.append((name, full))
            else:
                name = line
                if name:
                    full = f"{current_dir.rstrip('/')}/{name}" if current_dir != "/" else f"/{name}"
                    files.append((name, full))
        dirs.sort(key=lambda item: item[0])
        files.sort(key=lambda item: item[0])
        return current_dir, dirs, files

    def _browse_container_path_dialog(self, container_name: str, title: str, start_path: str = "/tmp", select_kind: str = "file", file_filter=None, multi_select: bool = False):
        start_dir = self._normalize_remote_posix_path(start_path or "/tmp")
        dlg = QDialog(self)
        dlg.setWindowTitle(f"{title}({container_name})")
        dlg.resize(760, 520)

        root_layout = QVBoxLayout(dlg)

        nav_row = QHBoxLayout()
        nav_row.addWidget(QLabel("当前目录"))
        path_edit = QLineEdit(start_dir)
        btn_go = QPushButton("跳转")
        btn_up = QPushButton("上一级")
        btn_refresh = QPushButton("刷新")
        nav_row.addWidget(path_edit)
        nav_row.addWidget(btn_go)
        nav_row.addWidget(btn_up)
        nav_row.addWidget(btn_refresh)

        hint_text = "双击目录进入；双击文件直接确认。" if select_kind == "file" else "双击目录进入；选择按钮可选当前目录或列表中的目录。"
        if select_kind == "file" and multi_select:
            hint_text = "双击目录进入；可多选文件后点击选择。"
        hint = QLabel(hint_text)
        list_widget = QListWidget()
        if select_kind == "file" and multi_select:
            list_widget.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)

        action_row = QHBoxLayout()
        btn_ok = QPushButton("选择")
        btn_cancel = QPushButton("取消")
        action_row.addStretch(1)
        action_row.addWidget(btn_ok)
        action_row.addWidget(btn_cancel)

        root_layout.addLayout(nav_row)
        root_layout.addWidget(hint)
        root_layout.addWidget(list_widget)
        root_layout.addLayout(action_row)

        state = {"current_dir": start_dir, "selected_path": "", "selected_paths": []}

        def _accept_paths(chosen_paths):
            normalized = []
            seen = set()
            for chosen_path in chosen_paths:
                value = self._normalize_remote_posix_path(chosen_path)
                if value in seen:
                    continue
                seen.add(value)
                normalized.append(value)
            if not normalized:
                return
            state["selected_paths"] = normalized
            state["selected_path"] = normalized[0]
            dlg.accept()

        def _load_dir(path: str):
            target_dir = self._normalize_remote_posix_path(path)
            try:
                current_dir, dirs, files = self._list_container_runtime_dir(container_name, target_dir)
            except Exception as e:
                QMessageBox.warning(dlg, "读取失败", f"无法读取容器目录:\n{target_dir}\n\n{e}")
                return

            state["current_dir"] = current_dir
            state["selected_path"] = ""
            state["selected_paths"] = []
            path_edit.setText(current_dir)
            list_widget.clear()

            if current_dir != "/":
                parent = os.path.dirname(current_dir.rstrip("/")) or "/"
                item_up = QListWidgetItem("..")
                item_up.setData(Qt.ItemDataRole.UserRole, ("dir", parent))
                list_widget.addItem(item_up)

            for name, full in dirs:
                item = QListWidgetItem(f"[DIR] {name}/")
                item.setData(Qt.ItemDataRole.UserRole, ("dir", full))
                list_widget.addItem(item)

            for name, full in files:
                if file_filter and (not file_filter(full)):
                    continue
                item = QListWidgetItem(name)
                item.setData(Qt.ItemDataRole.UserRole, ("file", full))
                list_widget.addItem(item)

        def _on_item_double_clicked(item: QListWidgetItem):
            data = item.data(Qt.ItemDataRole.UserRole)
            if not data:
                return
            kind, full = data
            if kind == "dir":
                _load_dir(full)
                return
            if kind == "file" and select_kind == "file" and not multi_select:
                _accept_paths([full])

        def _on_select_clicked():
            if select_kind == "file" and multi_select:
                selected_files = []
                for item in list_widget.selectedItems():
                    data = item.data(Qt.ItemDataRole.UserRole)
                    if not data:
                        continue
                    kind, full = data
                    if kind == "file":
                        selected_files.append(full)
                if selected_files:
                    _accept_paths(selected_files)
                    return
            item = list_widget.currentItem()
            if item is not None:
                data = item.data(Qt.ItemDataRole.UserRole)
                if data:
                    kind, full = data
                    if kind == "dir" and select_kind == "dir":
                        _accept_paths([full])
                        return
                    if kind == "file" and select_kind == "file":
                        _accept_paths([full])
                        return
            typed = (path_edit.text() or "").strip()
            if not typed:
                if select_kind == "dir":
                    _accept_paths([state["current_dir"]])
                    return
                QMessageBox.information(dlg, "提示", "请先选择一个文件，或输入完整文件路径。")
                return
            _accept_paths([typed])

        list_widget.itemDoubleClicked.connect(_on_item_double_clicked)
        btn_go.clicked.connect(lambda: _load_dir(path_edit.text()))
        btn_refresh.clicked.connect(lambda: _load_dir(state["current_dir"]))
        btn_up.clicked.connect(lambda: _load_dir(os.path.dirname(state["current_dir"].rstrip("/")) or "/"))
        btn_ok.clicked.connect(_on_select_clicked)
        btn_cancel.clicked.connect(dlg.reject)

        _load_dir(start_dir)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return [] if (select_kind == "file" and multi_select) else ""
        if select_kind == "file" and multi_select:
            return list(state.get("selected_paths") or [])
        return str(state.get("selected_path") or "").strip()

    def _build_container_temp_path(self, remote_path: str) -> str:
        base = os.path.basename(str(remote_path or "").strip()) or "tmp.txt"
        safe_base = re.sub(r"[^A-Za-z0-9._-]+", "_", base)
        return f"/tmp/toolchain_container_{int(time.time() * 1000)}_{safe_base}"

    def _build_container_temp_dir(self, remote_path: str) -> str:
        base = os.path.basename(str(remote_path or "").strip().rstrip("/")) or "tmp_dir"
        safe_base = re.sub(r"[^A-Za-z0-9._-]+", "_", base)
        return f"/tmp/toolchain_container_dir_{int(time.time() * 1000)}_{safe_base}"

    def _container_runtime_detail_text(self, name: str) -> str:
        info = self._container_runtime_details.get(str(name or "").strip(), {})
        image = str(info.get("image") or "-")
        status = str(info.get("status") or "-")
        return f"镜像: {image} | 状态: {status}"

    def _inspect_container_runtime_state(self, container_name: str) -> dict:
        name = str(container_name or "").strip()
        if not name:
            return {}
        out, err = self._run_bash(
            self.ssh_big,
            f"docker inspect --format '{{{{.State.Status}}}}\t{{{{.State.Running}}}}\t{{{{.State.Restarting}}}}' {shlex.quote(name)}",
        )
        raw = (out or "").strip()
        err_text = (err or "").strip()
        if err_text and not raw:
            raise RuntimeError(err_text)
        parts = raw.split("\t") if raw else []
        status = (parts[0].strip() if len(parts) >= 1 else "") or str(
            (self._container_runtime_details.get(name, {}) or {}).get("status") or ""
        )
        running = (parts[1].strip().lower() == "true") if len(parts) >= 2 else ("up" in status.lower())
        restarting = (parts[2].strip().lower() == "true") if len(parts) >= 3 else ("restarting" in status.lower())
        return {
            "status": status,
            "running": running,
            "restarting": restarting,
        }

    def _require_running_container_runtime(self, container_name: str, action_text: str = "执行容器操作"):
        name = str(container_name or "").strip()
        if not name:
            return None
        try:
            state = self._inspect_container_runtime_state(name)
        except Exception as e:
            self._emit_container(f"[ERR] {action_text}前检查容器状态失败: {e}")
            return None
        status = str(state.get("status") or "-")
        detail = self._container_runtime_details.get(name, {}) or {}
        detail["status"] = status
        self._container_runtime_details[name] = detail
        if hasattr(self, "container_runtime_info") and self._selected_container_runtime_name() == name:
            self.container_runtime_info.setText(self._container_runtime_detail_text(name))
        if state.get("restarting"):
            self._emit_container(f"[WARN] 容器正在重启，暂时不能{action_text}: {name} | {status}")
            return None
        if not state.get("running"):
            self._emit_container(f"[WARN] 容器未运行，暂时不能{action_text}: {name} | {status}")
            return None
        return name

    def _require_selected_running_container_runtime(self, action_text: str = "执行容器操作"):
        name = self._require_selected_container_runtime()
        if not name:
            return None
        return self._require_running_container_runtime(name, action_text)

    def _run_container_runtime_command(self, title: str, cmd: str):
        def worker():
            try:
                self._emit_container(f"=== {title} ===")
                if not self._ensure_ssh_big():
                    self._emit_container("[ERR] 请先连接 SSH(大脑)")
                    return
                out, err = self._run_bash(self.ssh_big, cmd)
                text = (out or "").strip()
                err_text = (err or "").strip()
                if text:
                    self._emit_container(text)
                if err_text:
                    self._emit_container(err_text)
                if not text and not err_text:
                    self._emit_container("[OK] 命令执行完成")
            except Exception as e:
                self._emit_container(f"[ERR] {title}失败: {e}")

        self._run_async(worker)

    def refresh_container_runtime_list(self):
        if hasattr(self, "btn_container_runtime_refresh"):
            self.btn_container_runtime_refresh.setEnabled(False)

        def worker():
            try:
                if not self._ensure_ssh_big():
                    self.container_runtime_failed_signal.emit("请先连接 SSH(大脑)")
                    return
                out, err = self._run_bash(
                    self.ssh_big,
                    "docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'",
                )
                raw = (out or "").strip()
                err_text = (err or "").strip()
                if err_text and not raw:
                    raise RuntimeError(err_text)
                entries = []
                for line in raw.splitlines():
                    parts = line.split("\t")
                    if len(parts) < 3:
                        continue
                    entries.append({
                        "name": parts[0].strip(),
                        "image": parts[1].strip(),
                        "status": "\t".join(parts[2:]).strip(),
                    })
                self.container_runtime_list_signal.emit(entries)
            except Exception as e:
                self.container_runtime_failed_signal.emit(f"读取 docker ps -a 失败: {e}")

        self._run_async(worker)

    def _on_container_runtime_list_loaded(self, entries):
        if hasattr(self, "btn_container_runtime_refresh"):
            self.btn_container_runtime_refresh.setEnabled(True)
        current = self._selected_container_runtime_name()
        self._container_runtime_entries = list(entries or [])
        self._container_runtime_details = {
            str(item.get("name") or "").strip(): item for item in self._container_runtime_entries if str(item.get("name") or "").strip()
        }
        names = list(self._container_runtime_details.keys())
        if hasattr(self, "container_runtime_combo"):
            self.container_runtime_combo.blockSignals(True)
            self.container_runtime_combo.clear()
            self.container_runtime_combo.addItems(names)
            if current and current in self._container_runtime_details:
                self.container_runtime_combo.setCurrentText(current)
            elif names:
                self.container_runtime_combo.setCurrentIndex(0)
            self.container_runtime_combo.blockSignals(False)
        selected = self._selected_container_runtime_name()
        if selected:
            self._on_container_runtime_selection_changed(selected)
        else:
            self.container_runtime_info.setText("未发现容器")
        self._emit_container(f"[OK] 已读取 docker ps -a，共 {len(names)} 个容器")

    def _on_container_runtime_selection_changed(self, name: str):
        selected = str(name or "").strip()
        if not selected:
            if hasattr(self, "container_runtime_info"):
                self.container_runtime_info.setText("未选择容器")
            return
        if hasattr(self, "container_runtime_info"):
            self.container_runtime_info.setText(self._container_runtime_detail_text(selected))

    def _require_selected_container_runtime(self):
        if not self._ensure_ssh_big():
            return None
        name = self._selected_container_runtime_name()
        if not name:
            self._emit_container("[ERR] 请先读取并选择一个大脑容器")
            return None
        return name

    def _docker_cp_to_container(self, container_name: str, remote_tmp: str, container_path: str):
        parent = os.path.dirname(container_path.rstrip("/")) or "/"
        prepare_cmd = f"docker exec {shlex.quote(container_name)} sh -lc {shlex.quote(f'mkdir -p {shlex.quote(parent)}')}"
        copy_cmd = f"docker cp {shlex.quote(remote_tmp)} {shlex.quote(f'{container_name}:{container_path}') }"
        cleanup_cmd = f"rm -f {shlex.quote(remote_tmp)}"
        self._run_bash(self.ssh_big, prepare_cmd)
        out, err = self._run_bash(self.ssh_big, f"{copy_cmd} && {cleanup_cmd}")
        err_text = (err or "").strip()
        if err_text:
            raise RuntimeError(err_text)
        return out

    def _docker_cp_dir_to_container(self, container_name: str, remote_tmp_dir: str, container_dir: str):
        prepare_cmd = f"docker exec {shlex.quote(container_name)} sh -lc {shlex.quote(f'mkdir -p {shlex.quote(container_dir)}')}"
        copy_cmd = f"docker cp {shlex.quote(remote_tmp_dir.rstrip('/') + '/.')} {shlex.quote(f'{container_name}:{container_dir}') }"
        self._run_bash(self.ssh_big, prepare_cmd)
        out, err = self._run_bash(self.ssh_big, copy_cmd)
        err_text = (err or "").strip()
        if err_text:
            raise RuntimeError(err_text)
        return out

    def browse_container_runtime_path(self):
        container_name = self._require_selected_running_container_runtime("浏览容器文件")
        if not container_name:
            return
        start_path = self._guess_remote_browse_start_dir(self._current_container_runtime_path() or "/tmp")
        chosen = self._browse_container_path_dialog(
            container_name,
            "浏览容器文件",
            start_path=start_path,
            select_kind="file",
        )
        if not chosen:
            return
        self.container_runtime_path_edit.setText(chosen)
        self._emit_container(f"[OK] 已选择容器路径: {container_name}:{chosen}")

    def restart_selected_container(self):
        container_name = self._require_selected_container_runtime()
        if not container_name:
            return
        if not self._confirm_container_action("重启容器", f"container={container_name}"):
            return

        def worker():
            try:
                self._emit_container(f"=== 重启容器 {container_name} ===")
                out, err = self._run_bash(self.ssh_big, f"docker restart {shlex.quote(container_name)}")
                text = (out or "").strip()
                err_text = (err or "").strip()
                if err_text:
                    raise RuntimeError(err_text)
                self._emit_container(text or f"[OK] 已重启容器: {container_name}")
                self.refresh_container_runtime_list()
            except Exception as e:
                self._emit_container(f"[ERR] 重启容器失败: {e}")

        self._run_async(worker)

    def show_selected_container_logs(self):
        container_name = self._require_selected_container_runtime()
        if not container_name:
            return
        lines = self.container_runtime_logs_lines_edit.text().strip() if hasattr(self, "container_runtime_logs_lines_edit") else "200"
        try:
            lines_i = int(lines or "200")
        except Exception:
            lines_i = 200
        lines_i = max(1, min(lines_i, 5000))
        if not self._confirm_container_action("查看容器日志", f"container={container_name} lines={lines_i}"):
            return
        self._run_container_runtime_command(
            f"容器日志 {container_name}",
            f"docker logs --tail {lines_i} {shlex.quote(container_name)} 2>&1",
        )

    def upload_file_to_container(self):
        container_name = self._require_selected_running_container_runtime("上传文件到容器")
        if not container_name:
            return
        local_files, _ = QFileDialog.getOpenFileNames(self, "选择本地文件")
        if not local_files:
            return
        start_path = self._guess_remote_browse_start_dir(self._current_container_runtime_path() or "/tmp")
        target_dir = self._browse_container_path_dialog(
            container_name,
            "选择容器目标目录",
            start_path=start_path,
            select_kind="dir",
        )
        if not target_dir:
            return
        uploads = []
        for local_file in local_files:
            container_path = f"{target_dir.rstrip('/')}/{os.path.basename(local_file)}" if target_dir != "/" else f"/{os.path.basename(local_file)}"
            uploads.append((local_file, container_path))
        self.container_runtime_path_edit.setText(uploads[0][1] if len(uploads) == 1 else target_dir)

        def worker():
            failed = []
            for local_file, container_path in uploads:
                remote_tmp = self._build_container_temp_path(container_path)
                try:
                    self.ssh_big.upload(local_file, remote_tmp)
                    self._docker_cp_to_container(container_name, remote_tmp, container_path)
                    self._emit_container(f"[OK] 已上传到容器: {local_file} -> {container_name}:{container_path}")
                except Exception as e:
                    failed.append(f"{local_file}: {e}")
                    self._emit_container(f"[ERR] 上传到容器失败: {local_file} -> {container_name}:{container_path} | {e}")
                finally:
                    try:
                        self._run_bash(self.ssh_big, f"rm -f {shlex.quote(remote_tmp)}")
                    except Exception:
                        pass
            if len(uploads) > 1:
                ok_count = len(uploads) - len(failed)
                if failed:
                    self._emit_container(f"[WARN] 批量上传到容器完成: 成功 {ok_count}/{len(uploads)}")
                else:
                    self._emit_container(f"[OK] 批量上传到容器完成: 共 {ok_count} 个文件")

        self._run_async(worker)

    def download_file_from_container(self):
        container_name = self._require_selected_running_container_runtime("从容器下载文件")
        if not container_name:
            return
        start_path = self._guess_remote_browse_start_dir(self._current_container_runtime_path() or "/tmp")
        container_paths = self._browse_container_path_dialog(
            container_name,
            "选择容器源文件",
            start_path=start_path,
            select_kind="file",
            multi_select=True,
        )
        if not container_paths:
            return
        self.container_runtime_path_edit.setText(container_paths[0] if len(container_paths) == 1 else os.path.dirname(container_paths[0]) or "/")
        download_targets = self._choose_local_download_targets(container_paths)
        if not download_targets:
            return

        def worker():
            failed = []
            for container_path, local_path in download_targets:
                remote_tmp = self._build_container_temp_path(container_path)
                try:
                    out, err = self._run_bash(
                        self.ssh_big,
                        f"rm -f {shlex.quote(remote_tmp)} && docker cp {shlex.quote(f'{container_name}:{container_path}')} {shlex.quote(remote_tmp)}",
                    )
                    err_text = (err or "").strip()
                    if err_text:
                        raise RuntimeError(err_text)
                    self.ssh_big.download(remote_tmp, local_path)
                    if (out or "").strip():
                        self._emit_container((out or "").strip())
                    self._emit_container(f"[OK] 已从容器下载: {container_name}:{container_path} -> {local_path}")
                except Exception as e:
                    failed.append(f"{container_path}: {e}")
                    self._emit_container(f"[ERR] 从容器下载失败: {container_name}:{container_path} -> {local_path} | {e}")
                finally:
                    try:
                        self._run_bash(self.ssh_big, f"rm -f {shlex.quote(remote_tmp)}")
                    except Exception:
                        pass
            if len(download_targets) > 1:
                ok_count = len(download_targets) - len(failed)
                if failed:
                    self._emit_container(f"[WARN] 批量从容器下载完成: 成功 {ok_count}/{len(download_targets)}")
                else:
                    self._emit_container(f"[OK] 批量从容器下载完成: 共 {ok_count} 个文件")

        self._run_async(worker)

    def upload_folder_to_container(self):
        container_name = self._require_selected_running_container_runtime("上传文件夹到容器")
        if not container_name:
            return
        local_dir = QFileDialog.getExistingDirectory(self, "选择本地文件夹")
        if not local_dir:
            return
        start_path = self._guess_remote_browse_start_dir(self._current_container_runtime_path() or "/tmp")
        target_parent = self._browse_container_path_dialog(
            container_name,
            "选择容器目标目录",
            start_path=start_path,
            select_kind="dir",
        )
        if not target_parent:
            return
        folder_name = os.path.basename(local_dir.rstrip("/")) or "folder"
        container_dir = f"{target_parent.rstrip('/')}/{folder_name}" if target_parent != "/" else f"/{folder_name}"
        self.container_runtime_path_edit.setText(container_dir)

        def worker():
            remote_tmp_dir = self._build_container_temp_dir(local_dir)
            try:
                self.ssh_big.upload_dir(local_dir, remote_tmp_dir)
                self._docker_cp_dir_to_container(container_name, remote_tmp_dir, container_dir)
                self._emit_container(f"[OK] 文件夹已上传到容器: {local_dir} -> {container_name}:{container_dir}")
            except Exception as e:
                self._emit_container(f"[ERR] 上传文件夹到容器失败: {local_dir} -> {container_name}:{container_dir} | {e}")
            finally:
                try:
                    self._run_bash(self.ssh_big, f"rm -rf {shlex.quote(remote_tmp_dir)}")
                except Exception:
                    pass

        self._run_async(worker)

    def download_folder_from_container(self):
        container_name = self._require_selected_running_container_runtime("从容器下载文件夹")
        if not container_name:
            return
        start_path = self._guess_remote_browse_start_dir(self._current_container_runtime_path() or "/tmp")
        container_dir = self._browse_container_path_dialog(
            container_name,
            "选择容器源目录",
            start_path=start_path,
            select_kind="dir",
        )
        if not container_dir:
            return

        local_parent = QFileDialog.getExistingDirectory(self, "选择本地保存目录")
        if not local_parent:
            return

        folder_name = os.path.basename(container_dir.rstrip("/")) or "container_dir"
        local_dir = os.path.join(local_parent, folder_name)
        self.container_runtime_path_edit.setText(container_dir)

        def worker():
            remote_tmp_dir = self._build_container_temp_dir(container_dir)
            try:
                self._run_bash(self.ssh_big, f"mkdir -p {shlex.quote(remote_tmp_dir)}")
                container_source = f"{container_name}:{container_dir.rstrip('/')}/."
                out, err = self._run_bash(
                    self.ssh_big,
                    f"docker cp {shlex.quote(container_source)} {shlex.quote(remote_tmp_dir)}",
                )
                err_text = (err or "").strip()
                if err_text:
                    raise RuntimeError(err_text)
                self.ssh_big.download_dir(remote_tmp_dir, local_dir)
                if (out or "").strip():
                    self._emit_container((out or "").strip())
                self._emit_container(f"[OK] 已从容器下载文件夹: {container_name}:{container_dir} -> {local_dir}")
            except Exception as e:
                self._emit_container(f"[ERR] 从容器下载文件夹失败: {container_name}:{container_dir} -> {local_dir} | {e}")
            finally:
                try:
                    self._run_bash(self.ssh_big, f"rm -rf {shlex.quote(remote_tmp_dir)}")
                except Exception:
                    pass

        self._run_async(worker)

    def open_container_file_for_edit(self):
        container_name = self._require_selected_running_container_runtime("打开容器文件")
        if not container_name:
            return
        container_path = self._current_container_runtime_path()
        if not container_path:
            self._emit_container("[ERR] 请输入容器内文件路径")
            return
        if not self._is_supported_remote_edit_file(container_path):
            self._emit_container("[ERR] 仅支持编辑常见文本文件，如 .md/.h/.hpp/.py/.cpp/.yaml/.json 等")
            return
        if hasattr(self, "btn_container_file_open"):
            self.btn_container_file_open.setEnabled(False)

        def worker():
            try:
                cmd = (
                    f"docker exec {shlex.quote(container_name)} sh -lc "
                    f"{shlex.quote(f'test -f {shlex.quote(container_path)} && cat {shlex.quote(container_path)}')}"
                )
                out, err = self._run_bash(self.ssh_big, cmd)
                text = out or ""
                err_text = (err or "").strip()
                if err_text and not text:
                    raise RuntimeError(err_text)
                if "\x00" in text:
                    raise RuntimeError("检测到二进制内容，无法作为文本编辑")
                self.container_runtime_file_opened_signal.emit(container_name, container_path, text)
            except Exception as e:
                self.container_runtime_failed_signal.emit(f"打开容器文件失败: {e}")

        self._run_async(worker)

    def save_container_file_from_editor(self):
        container_name = str(getattr(self, "_container_edit_container", "") or "").strip()
        container_path = str(getattr(self, "_container_edit_path", "") or "").strip()
        if not container_name or not container_path:
            self._emit_container("[ERR] 请先打开容器文件后再保存")
            return
        if not self._require_running_container_runtime(container_name, "保存容器文件"):
            return
        if not self._is_supported_remote_edit_file(container_path):
            self._emit_container("[ERR] 当前文件后缀不受支持，无法保存")
            return
        content = self.container_file_editor.toPlainText() if hasattr(self, "container_file_editor") else ""
        if hasattr(self, "btn_container_file_save"):
            self.btn_container_file_save.setEnabled(False)

        def worker():
            remote_tmp = self._build_container_temp_path(container_path)
            try:
                with self.ssh_big.sftp.open(remote_tmp, "wb") as f:
                    f.write(content.encode("utf-8"))
                self._docker_cp_to_container(container_name, remote_tmp, container_path)
                self.container_runtime_file_saved_signal.emit(container_name, container_path)
            except Exception as e:
                try:
                    self._run_bash(self.ssh_big, f"rm -f {shlex.quote(remote_tmp)}")
                except Exception:
                    pass
                self.container_runtime_failed_signal.emit(f"保存容器文件失败: {e}")

        self._run_async(worker)

    def _on_container_runtime_file_opened(self, container_name: str, container_path: str, content: str):
        self._container_edit_container = str(container_name or "")
        self._container_edit_path = str(container_path or "")
        if hasattr(self, "container_runtime_path_edit"):
            self.container_runtime_path_edit.setText(self._container_edit_path)
        if hasattr(self, "container_file_editor"):
            self.container_file_editor.setPlainText(content)
            self.container_file_editor.document().setModified(False)
        if hasattr(self, "container_file_status"):
            self.container_file_status.setText(f"已打开: {self._container_edit_container} | {self._container_edit_path}")
        if hasattr(self, "btn_container_file_open"):
            self.btn_container_file_open.setEnabled(True)
        if hasattr(self, "btn_container_file_save"):
            self.btn_container_file_save.setEnabled(True)
        self._emit_container(f"[OK] 已打开容器文件: {self._container_edit_container}:{self._container_edit_path}")

    def _on_container_runtime_file_saved(self, container_name: str, container_path: str):
        if hasattr(self, "container_file_editor"):
            self.container_file_editor.document().setModified(False)
        if hasattr(self, "btn_container_file_open"):
            self.btn_container_file_open.setEnabled(True)
        if hasattr(self, "btn_container_file_save"):
            self.btn_container_file_save.setEnabled(True)
        if hasattr(self, "container_file_status"):
            self.container_file_status.setText(f"已保存: {container_name} | {container_path}")
        self._emit_container(f"[OK] 已保存容器文件: {container_name}:{container_path}")

    def _on_container_runtime_failed(self, msg: str):
        if hasattr(self, "btn_container_runtime_refresh"):
            self.btn_container_runtime_refresh.setEnabled(True)
        if hasattr(self, "btn_container_file_open"):
            self.btn_container_file_open.setEnabled(True)
        if hasattr(self, "btn_container_file_save"):
            self.btn_container_file_save.setEnabled(bool(getattr(self, "_container_edit_path", "")))
        if hasattr(self, "container_file_status"):
            self.container_file_status.setText(msg)
        self._emit_container(f"[ERR] {msg}")

    def _format_error_flags(self, msg: dict) -> str:
        issues = []
        if bool(msg.get("over_temp", False)):
            issues.append("over_temp")
        if bool(msg.get("over_cpu", False)):
            issues.append("over_cpu")
        if bool(msg.get("over_mem", False)):
            issues.append("over_mem")
        if bool(msg.get("over_disk", False)):
            issues.append("over_disk")
        return "无异常" if not issues else ", ".join(issues)

    def _format_resource(self, msg: dict) -> str:
        cpu = msg.get("cpu_usage", "-")
        temp = msg.get("temperature", "-")
        mem = msg.get("memory_usage", "-")
        disk = msg.get("disk_usage", "-")
        try:
            cpu = f"{float(cpu):.1f}%"
        except Exception:
            pass
        try:
            temp = f"{float(temp):.1f}°C"
        except Exception:
            pass
        try:
            mem = f"{float(mem):.1f}%"
        except Exception:
            pass
        try:
            disk = f"{float(disk):.1f}%"
        except Exception:
            pass
        return f"CPU {cpu} | TEMP {temp} | MEM {mem} | DISK {disk}"

    def _extract_upperlimb_version(self, text: str) -> str:
        raw = (text or "").strip()
        if not raw:
            return "获取失败"

        # 常见返回: success: True\nmessage: "1.2.1"
        s = re.search(r'success\s*:\s*(True|False)', raw, flags=re.IGNORECASE)
        m = re.search(r'message:\s*"?([^"\n]+)"?', raw)
        if s and m and m.group(1).strip():
            return m.group(1).strip()
        if m and m.group(1).strip():
            return m.group(1).strip()
        if s:
            return f"success={s.group(1)}"

        # 兜底: 兼容 version 字段
        v = re.search(r'version\s*:\s*"?([^"\n]+)"?', raw, flags=re.IGNORECASE)
        if v and v.group(1).strip():
            return v.group(1).strip()

        return raw.replace("\n", " | ")

    def _extract_sdk_version(self, text: str) -> str:
        raw = (text or "").strip()
        if not raw:
            return "获取失败"
        match = re.search(r'(^|\n)\s*Version:\s*([^\n]+)', raw, flags=re.IGNORECASE)
        if match and match.group(2).strip():
            return match.group(2).strip()
        return raw.replace("\n", " | ")

    def _extract_apt_package_version(self, text: str) -> str:
        raw = (text or "").strip()
        if not raw:
            return "获取失败"
        first_line = raw.splitlines()[0].strip()
        cols = first_line.split()
        if len(cols) >= 2:
            return cols[1].strip()
        return first_line

    def _default_robot_version_summary(self) -> dict:
        return {
            "robot_version": "-",
            "embedded_version": "-",
            "software_version": "-",
            "upperlimb_version": "-",
            "lowerlimb_version": "-",
            "show_lowerlimb": False,
        }

    def _default_robot_system_info(self) -> dict:
        return {
            "small_ubuntu": "-",
            "small_system": "-",
            "big_ubuntu": "-",
            "big_jetpack": "-",
        }

    def _emit_robot_version_snapshot(self, version_data: dict = None, system_data: dict = None):
        merged_version = self._default_robot_version_summary()
        merged_system = self._default_robot_system_info()
        if isinstance(version_data, dict):
            merged_version.update(version_data)
        if isinstance(system_data, dict):
            merged_system.update(system_data)
        self.robot_basic_info_signal.emit(merged_version)
        self.robot_system_info_signal.emit(merged_system)

    def _merge_status_text(self, err_msg: dict, res_msg: dict) -> str:
        err = self._format_error_flags(err_msg or {})
        res = self._format_resource(res_msg or {})
        return f"{err} || {res}"

    def _measure_latency(self, host: str) -> str:
        host = (host or "").strip()
        if not host:
            return "-"
        try:
            p = subprocess.run(
                ["ping", "-c", "1", "-W", "1", host],
                capture_output=True,
                text=True,
                timeout=2,
                check=False,
            )
            out = (p.stdout or "") + "\n" + (p.stderr or "")
            m = re.search(r"time=([0-9]+(?:\.[0-9]+)?)\s*ms", out)
            if m:
                return f"{float(m.group(1)):.1f} ms"
            return "超时"
        except Exception:
            return "超时"

    def _update_network_latency(self):
        target = "www.baidu.com"
        latency = self._measure_latency(target)
        self.robot_orin_latency_signal.emit(latency)
        self.robot_pico_latency_signal.emit(latency)

    def _subscribe_robot_status_topics(self):
        self._restart_robot_status_subscription("robot状态订阅")
        self._update_network_latency()

    def _disconnect_robot_status_monitor_client(self):
        with self._robot_status_monitor_client_lock:
            client = self.ros_status_monitor
            self.ros_status_monitor = None
            self._robot_status_monitor_recovering = False
            self._robot_status_last_update_ts = 0.0
        if client:
            try:
                client.disconnect()
            except Exception:
                pass
        self.robot_status_health_signal.emit("未连接")

    def _mark_robot_status_update(self):
        self._robot_status_last_update_ts = time.monotonic()
        self.robot_status_health_signal.emit(f"已连接 | 最新 {datetime.now().strftime('%H:%M:%S')}")

    def _robot_status_subscription_alive(self) -> bool:
        with self._robot_status_monitor_client_lock:
            client = self.ros_status_monitor
        if not client or not client.check_connection():
            return False
        try:
            subs = getattr(client, "_subscriptions", {})
            return all(bool(subs.get(topic)) for topic in self._robot_status_topics)
        except Exception:
            return False

    def _restart_robot_status_subscription(self, reason: str = ""):
        if not self._alive:
            return
        if not (self.ros and self.ros.check_connection()):
            self.robot_status_health_signal.emit("未连接")
            return

        with self._robot_status_monitor_client_lock:
            if self._robot_status_monitor_recovering:
                return
            self._robot_status_monitor_recovering = True

        host = self.ros_host.text().strip()
        port_text = self.ros_port.text().strip()

        def worker():
            old_client = None
            try:
                try:
                    port = int(port_text)
                except Exception as e:
                    raise RuntimeError(f"ROSBridge 端口无效: {e}")

                with self._robot_status_monitor_client_lock:
                    old_client = self.ros_status_monitor
                    self.ros_status_monitor = None

                if old_client:
                    try:
                        old_client.disconnect()
                    except Exception:
                        pass

                client = BridgeClient(host, port)
                if not client.connect(timeout=4.0):
                    raise RuntimeError("独立 robot 状态监听通道连接失败")

                battery_topic = "/zj_humanoid/robot/battery_info"
                orin_err_topic = "/zj_humanoid/robot/orin_states/errors"
                orin_res_topic = "/zj_humanoid/robot/orin_states/resource"
                pico_err_topic = "/zj_humanoid/robot/pico_states/errors"
                pico_res_topic = "/zj_humanoid/robot/pico_states/resource"
                robot_state_topic = "/zj_humanoid/robot/robot_state"
                orin_cache = {"err": {}, "res": {}}
                pico_cache = {"err": {}, "res": {}}

                def battery_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    try:
                        value = msg.get("percentage")
                        if value is None:
                            value = msg.get("data")
                        text = self._format_battery_percentage(value) if value is not None else "-"
                        self.battery_signal.emit(text)
                    except RuntimeError:
                        return
                    except Exception:
                        try:
                            self.battery_signal.emit("-")
                        except RuntimeError:
                            pass

                def orin_err_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    orin_cache["err"] = msg or {}
                    try:
                        self.robot_orin_status_signal.emit(self._merge_status_text(orin_cache["err"], orin_cache["res"]))
                    except RuntimeError:
                        pass

                def orin_res_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    orin_cache["res"] = msg or {}
                    try:
                        self.robot_orin_status_signal.emit(self._merge_status_text(orin_cache["err"], orin_cache["res"]))
                    except RuntimeError:
                        pass

                def pico_err_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    pico_cache["err"] = msg or {}
                    try:
                        self.robot_pico_status_signal.emit(self._merge_status_text(pico_cache["err"], pico_cache["res"]))
                    except RuntimeError:
                        pass

                def pico_res_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    pico_cache["res"] = msg or {}
                    try:
                        self.robot_pico_status_signal.emit(self._merge_status_text(pico_cache["err"], pico_cache["res"]))
                    except RuntimeError:
                        pass

                def robot_state_cb(msg: dict):
                    if not self._alive:
                        return
                    self._mark_robot_status_update()
                    state_info = msg.get("state_info") if isinstance(msg, dict) else None
                    try:
                        self.robot_state_info_signal.emit(str(state_info) if state_info else "-")
                    except RuntimeError:
                        pass

                callbacks = {
                    battery_topic: battery_cb,
                    orin_err_topic: orin_err_cb,
                    orin_res_topic: orin_res_cb,
                    pico_err_topic: pico_err_cb,
                    pico_res_topic: pico_res_cb,
                    robot_state_topic: robot_state_cb,
                }
                ok_count = 0
                for topic, cb in callbacks.items():
                    try:
                        client.subscribe(topic, cb)
                        ok_count += 1
                    except Exception as e:
                        self.log_signal.emit(f"[WARN] 跳过不可用话题: {topic} ({e})")

                if ok_count == 0:
                    raise RuntimeError("robot 状态话题全部订阅失败")

                with self._robot_status_monitor_client_lock:
                    self.ros_status_monitor = client

                self._robot_status_last_recover_ts = time.monotonic()
                reason_text = f" ({reason})" if reason else ""
                self.robot_status_health_signal.emit(f"已连接 | {ok_count}/{len(callbacks)} 话题{reason_text}")
                self.log_signal.emit(f"[OK] 已建立独立 robot 状态监听通道: {ok_count}/{len(callbacks)}{reason_text}")
            except Exception as e:
                self.robot_status_health_signal.emit(f"恢复失败: {e}")
                self.log_signal.emit(f"[WARN] 重建 robot 状态监听失败: {e}")
            finally:
                with self._robot_status_monitor_client_lock:
                    self._robot_status_monitor_recovering = False

        self._run_async(worker)

    def _check_robot_status_monitor_health(self):
        if not self._alive:
            return
        if not (self.ros and self.ros.check_connection()):
            self.robot_status_health_signal.emit("未连接")
            return
        now = time.monotonic()
        age = (now - float(self._robot_status_last_update_ts or 0.0)) if self._robot_status_last_update_ts else float("inf")
        alive = self._robot_status_subscription_alive()
        if alive and age <= float(self._robot_status_fresh_timeout_sec):
            return
        if (now - float(self._robot_status_last_recover_ts or 0.0)) < max(3.0, float(self._robot_status_fresh_timeout_sec)):
            return
        self._robot_status_last_recover_ts = now
        reason = []
        if not alive:
            reason.append("订阅缺失")
        if age > float(self._robot_status_fresh_timeout_sec):
            reason.append(f"数据{age:.1f}s未更新")
        reason_text = ", ".join(reason) if reason else "监控异常"
        self.robot_status_health_signal.emit(f"恢复中: {reason_text}")
        self.log_signal.emit(f"[WARN] 检测到 robot 状态监听失活，正在恢复: {reason_text}")
        self._restart_robot_status_subscription(reason_text)

    def _active_disconnect_all(self, emit_log: bool = True):
        self._alive = False

        try:
            self.close_humanode_demo_session(emit_log=False)
        except Exception:
            pass
        try:
            self.stop_maintenance_pressure_monitor(emit_log=False)
        except Exception:
            pass
        try:
            if bool(getattr(self, "_vision_active", False)) and bool(getattr(self, "_vision_topic", None)):
                self.stop_vision_sampling()
            else:
                self._finish_vision_sampling()
        except Exception:
            try:
                self._finish_vision_sampling()
            except Exception:
                pass
        try:
            self.factory_stop_camera_sampling(emit_log=False)
        except Exception:
            pass
        try:
            if bool(getattr(self, "_pose_polling", False)):
                self.stop_pose_polling()
            elif hasattr(self, "_pose_timer") and self._pose_timer is not None:
                self._pose_timer.stop()
        except Exception:
            pass

        try:
            self._disconnect_joint_monitor_client()
        except Exception:
            pass
        try:
            self._disconnect_robot_status_monitor_client()
        except Exception:
            pass

        try:
            if self.ros:
                self.ros.disconnect()
        except Exception:
            pass
        self.ros = None

        try:
            if self.ssh:
                self.ssh.close()
        except Exception:
            pass
        try:
            if self.ssh_big:
                self.ssh_big.close()
        except Exception:
            pass

        self._monitor_started = False
        self._cmd_echo_topic = None
        self.command_btn_signal.emit(False)
        self.status_signal.emit("ros", "未连接")
        self.status_signal.emit("ssh", "未连接")
        self.status_signal.emit("ssh_big", "未连接")
        self._refresh_robot_status_view()

        if emit_log:
            self.log_signal.emit("[OK] 已主动断开：ROSBridge / SSH(小脑) / SSH(大脑) 与监听任务")

    def active_disconnect_all(self):
        self._active_disconnect_all(emit_log=True)
        self._alive = True

    def closeEvent(self, event):
        self._active_disconnect_all(emit_log=False)
        self._dispose_auxiliary_windows()
        super().closeEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_logo_pixmap()

    def _refresh_robot_status_view(self):
        if hasattr(self, "robot_battery_value") and hasattr(self, "battery_conn_value"):
            self.robot_battery_value.setText(self.battery_conn_value.text())
        self._update_connection_quality()

    def _update_connection_quality(self):
        if not hasattr(self, "conn_quality_label"):
            return

        def _norm(text: str) -> str:
            return (text or "").strip()

        ros = _norm(self.ros_status_value.text()) if hasattr(self, "ros_status_value") else "-"
        ssh = _norm(self.ssh_status_value.text()) if hasattr(self, "ssh_status_value") else "-"
        ssh_big = _norm(self.ssh_big_status_value.text()) if hasattr(self, "ssh_big_status_value") else "-"

        parts = [f"ROSBridge:{ros}", f"SSH(小脑):{ssh}", f"SSH(大脑):{ssh_big}"]
        text = "连接质量: " + " | ".join(parts)

        def _score(val: str) -> int:
            if "已连接" in val:
                return 2
            if "连接中" in val:
                return 1
            return 0

        score = max(_score(ros), _score(ssh), _score(ssh_big))
        if score >= 2:
            color = "#3fb950"  # green
        elif score == 1:
            color = "#d29922"  # amber
        else:
            color = "#f85149"  # red

        self.conn_quality_label.setText(text)
        self.conn_quality_label.setStyleSheet(f"color: {color};")
        if hasattr(self, "log_conn_quality_label"):
            self.log_conn_quality_label.setText(text)
            self.log_conn_quality_label.setStyleSheet(f"color: {color};")

    def refresh_robot_status(self):
        ros_ok = bool(self.ros and self.ros.check_connection())
        if not ros_ok and not self._ensure_ssh():
            return

        def worker():
            try:
                if self.ros and self.ros.check_connection():
                    self._subscribe_robot_status_topics()
                else:
                    self.log_signal.emit("[INFO] ROSBridge 未连接，改用 SSH(小脑) 刷新状态")

                    out_bat, err_bat = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/battery_info")
                    bat_text = (out_bat or "") + "\n" + (err_bat or "")
                    bat_msg = self._parse_ros_cli_yaml(bat_text) or {}
                    battery_val = bat_msg.get("percentage", bat_msg.get("data"))
                    if battery_val is None:
                        battery_val = self._extract_text_field(bat_text, "percentage") or self._extract_text_field(bat_text, "data") or "-"
                    self.battery_signal.emit(self._format_battery_percentage(battery_val) if battery_val != "-" else "-")

                    out_state, err_state = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/robot_state")
                    state_text = (out_state or "") + "\n" + (err_state or "")
                    state_msg = self._parse_ros_cli_yaml(state_text) or {}
                    state_info = state_msg.get("state_info")
                    if state_info is None:
                        state_info = self._extract_text_field(state_text, "state_info")
                    self.robot_state_info_signal.emit(str(state_info) if state_info else "-")

                    out_oe, err_oe = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/orin_states/errors")
                    out_or, err_or = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/orin_states/resource")
                    oe_text = (out_oe or "") + "\n" + (err_oe or "")
                    or_text = (out_or or "") + "\n" + (err_or or "")
                    oe_msg = self._parse_ros_cli_yaml(oe_text) or {}
                    or_msg = self._parse_ros_cli_yaml(or_text) or {}
                    self.robot_orin_status_signal.emit(self._merge_status_text(oe_msg, or_msg))

                    out_pe, err_pe = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/pico_states/errors")
                    out_pr, err_pr = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/robot/pico_states/resource")
                    pe_text = (out_pe or "") + "\n" + (err_pe or "")
                    pr_text = (out_pr or "") + "\n" + (err_pr or "")
                    pe_msg = self._parse_ros_cli_yaml(pe_text) or {}
                    pr_msg = self._parse_ros_cli_yaml(pr_text) or {}
                    self.robot_pico_status_signal.emit(self._merge_status_text(pe_msg, pr_msg))

                    self._update_network_latency()
                self._refresh_robot_status_view()
                self.log_signal.emit("[OK] robot状态已刷新")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 刷新 robot状态失败: {e}")

        self._run_async(worker)

    def _update_joint_fields(self, joint_map: dict):
        latest = {}
        for name in self.joint_names:
            val = joint_map.get(name)
            if val is None:
                self.joint_fields[name].setText("-")
            else:
                fv = float(val)
                self.joint_fields[name].setText(f"{fv:.3f}")
                latest[name] = fv
        if latest:
            with self._latest_joint_state_lock:
                self._latest_joint_state_map = latest
                self._latest_joint_state_ts = time.monotonic()

    def _format_monitor_header_text(self, header: dict) -> str:
        if not isinstance(header, dict):
            return "seq=-\nstamp=-\nframe_id=-"
        seq = header.get("seq", "-")
        frame_id = str(header.get("frame_id") or "-")
        stamp = header.get("stamp") if isinstance(header.get("stamp"), dict) else {}
        secs = stamp.get("secs")
        nsecs = stamp.get("nsecs")
        stamp_text = "-"
        if secs is not None:
            try:
                total = float(secs) + (float(nsecs or 0) / 1_000_000_000.0)
                stamp_text = datetime.fromtimestamp(total).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            except Exception:
                stamp_text = f"{secs}.{int(nsecs or 0):09d}"
        return f"seq={seq}\nstamp={stamp_text}\nframe_id={frame_id}"

    def _format_float_vector(self, values, precision: int = 4) -> str:
        if not isinstance(values, list):
            return "-"
        parts = []
        for value in values:
            try:
                parts.append(f"{float(value):.{precision}f}")
            except Exception:
                parts.append(str(value))
        return "[" + ", ".join(parts) + "]"

    def _format_tcp_pose_text(self, msg: dict) -> str:
        if not isinstance(msg, dict) or not msg:
            return "-"
        position = msg.get("position") if isinstance(msg.get("position"), dict) else {}
        quaternion = msg.get("quaternion") if isinstance(msg.get("quaternion"), dict) else {}
        rpy_rad = msg.get("rpy_rad") if isinstance(msg.get("rpy_rad"), list) else []
        rpy_deg = msg.get("rpy_deg") if isinstance(msg.get("rpy_deg"), list) else []
        lines = [
            self._format_monitor_header_text(msg.get("header") if isinstance(msg.get("header"), dict) else {}),
            "",
            "position [m]",
            f"x={float(position.get('x', 0.0)):.4f}  y={float(position.get('y', 0.0)):.4f}  z={float(position.get('z', 0.0)):.4f}" if position else "-",
            "",
            "quaternion",
            f"x={float(quaternion.get('x', 0.0)):.5f}  y={float(quaternion.get('y', 0.0)):.5f}  z={float(quaternion.get('z', 0.0)):.5f}  w={float(quaternion.get('w', 1.0)):.5f}" if quaternion else "-",
            "",
            f"rpy_rad {self._format_float_vector(rpy_rad, precision=5)}",
            f"rpy_deg {self._format_float_vector(rpy_deg, precision=3)}",
        ]
        return "\n".join(lines)

    def _update_tcp_pose_view(self, payload: dict):
        if not hasattr(self, "tcp_pose_left_value") or not hasattr(self, "tcp_pose_right_value"):
            return
        if not isinstance(payload, dict):
            self.tcp_pose_left_value.setPlainText("-")
            self.tcp_pose_right_value.setPlainText("-")
            return
        self.tcp_pose_left_value.setPlainText(self._format_tcp_pose_text(payload.get("left") if isinstance(payload.get("left"), dict) else {}))
        self.tcp_pose_right_value.setPlainText(self._format_tcp_pose_text(payload.get("right") if isinstance(payload.get("right"), dict) else {}))

    def _update_tcp_speed_view(self, msg: dict):
        if not hasattr(self, "tcp_speed_value"):
            return
        if not isinstance(msg, dict) or not msg:
            self.tcp_speed_value.setPlainText("-")
            return
        left_arm = msg.get("left_arm") if isinstance(msg.get("left_arm"), list) else []
        right_arm = msg.get("right_arm") if isinstance(msg.get("right_arm"), list) else []
        lines = [
            self._format_monitor_header_text(msg.get("header") if isinstance(msg.get("header"), dict) else {}),
            "",
            "left_arm [vx, vy, vz, wx, wy, wz] [m/s, rad/s]",
            self._format_float_vector(left_arm, precision=5),
            "",
            "right_arm [vx, vy, vz, wx, wy, wz] [m/s, rad/s]",
            self._format_float_vector(right_arm, precision=5),
        ]
        self.tcp_speed_value.setPlainText("\n".join(lines))

    def _reset_tcp_monitor_views(self):
        self._tcp_pose_monitor_cache = {"left": {}, "right": {}}
        self._update_tcp_pose_view({})
        self._update_tcp_speed_view({})

    def _reset_joint_fields(self):
        with self._latest_joint_state_lock:
            self._latest_joint_state_map = {}
            self._latest_joint_state_ts = 0.0
        for name in self.joint_names:
            self.joint_fields[name].setText("-")
        self._reset_tcp_monitor_views()

    def _format_battery_percentage(self, value):
        try:
            v = float(value)
            if v <= 1.5:
                v *= 100.0
            return f"{v:.1f}%"
        except Exception:
            return str(value)

    def _is_system_topic(self, name: str) -> bool:
        if name in ("/tf", "/tf_static", "/rosout", "/rosout_agg"):
            return True
        prefixes = ("/rosapi/", "/statistics")
        return any(name.startswith(p) for p in prefixes)

    def _is_system_service(self, name: str) -> bool:
        prefixes = ("/rosapi/", "/rosout/")
        return any(name.startswith(p) for p in prefixes)

    def _subscribe_battery_info(self):
        # 兼容旧调用，转发到统一robot状态订阅
        self._subscribe_robot_status_topics()

    def _set_command_button_state(self, echo_running: bool):
        self.btn_stop_cmd.setEnabled(echo_running)
        self.btn_run_cmd.setEnabled(not echo_running)
        self._refresh_robot_status_view()

    def _run_async(self, fn):
        threading.Thread(target=fn, daemon=True).start()

    def _run_movej_async(self, fn):
        self._movej_thread_pool.start(_FnRunnable(fn))

    def _prompt_movej_config(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("movej配置")
        form = QFormLayout(dlg)

        v_edit = QLineEdit("0.2")
        acc_edit = QLineEdit("0.2")
        t_edit = QLineEdit("0.0")
        async_combo = QComboBox()
        async_combo.addItems(["同步", "异步"])

        form.addRow("速度 v", v_edit)
        form.addRow("加速度 acc", acc_edit)
        form.addRow("时间 t", t_edit)
        form.addRow("是否异步", async_combo)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        form.addRow(buttons)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return None

        try:
            v = float((v_edit.text() or "0.5").strip())
            acc = float((acc_edit.text() or "1.0").strip())
            t = float((t_edit.text() or "0.0").strip())
        except Exception:
            self.log_signal.emit("[ERR] movej v/acc/t 格式错误")
            return None

        is_async = async_combo.currentText() == "异步"
        return v, acc, t, is_async

    def _project_root_path(self) -> str:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    def _get_storage_root(self) -> str:
        cached = getattr(self, "_storage_root_cache", None)
        if cached:
            return cached

        project_root = self._project_root_path()
        try:
            probe_dir = os.path.join(project_root, ".runtime")
            os.makedirs(probe_dir, exist_ok=True)
            probe_file = os.path.join(probe_dir, ".write_probe")
            with open(probe_file, "w", encoding="utf-8") as f:
                f.write("ok")
            try:
                os.remove(probe_file)
            except Exception:
                pass
            self._storage_root_cache = project_root
            return self._storage_root_cache
        except Exception:
            pass

        if os.name == "nt":
            local_app_data = os.environ.get("LOCALAPPDATA")
            if local_app_data:
                win_root = os.path.join(local_app_data, "humanoid-robot-delivery-toolchain")
                try:
                    os.makedirs(win_root, exist_ok=True)
                    self._storage_root_cache = win_root
                    return self._storage_root_cache
                except Exception:
                    pass

        xdg_state = os.environ.get("XDG_STATE_HOME")
        if not xdg_state:
            xdg_state = os.path.join(os.path.expanduser("~"), ".local", "state")
        app_root = os.path.join(xdg_state, "humanoid-robot-delivery-toolchain")
        try:
            os.makedirs(app_root, exist_ok=True)
            self._storage_root_cache = app_root
            return self._storage_root_cache
        except Exception:
            fallback = os.path.join("/tmp", "humanoid-robot-delivery-toolchain")
            os.makedirs(fallback, exist_ok=True)
            self._storage_root_cache = fallback
            return self._storage_root_cache

    def _save_qimage(self, img: QImage) -> str:
        project_root = self._get_storage_root()
        if self._instance_tag:
            log_dir = os.path.join(project_root, "logs", self._instance_tag, "vision")
        else:
            log_dir = os.path.join(project_root, "logs", "vision")
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("vision_%Y%m%d_%H%M%S_%f.png")
        path = os.path.join(log_dir, filename)
        img.save(path, "PNG")
        return path

    def _msg_data_to_bytes(self, data_field):
        if isinstance(data_field, list):
            return bytes(data_field)
        if isinstance(data_field, str):
            try:
                return base64.b64decode(data_field)
            except Exception:
                return data_field.encode("latin1", errors="ignore")
        if isinstance(data_field, (bytes, bytearray)):
            return bytes(data_field)
        return b""

    def _decode_ros_image(self, msg: dict) -> QImage:
        width = int(msg.get("width", 0))
        height = int(msg.get("height", 0))
        step = int(msg.get("step", 0))
        encoding = str(msg.get("encoding", "")).lower()
        raw = self._msg_data_to_bytes(msg.get("data"))

        if width <= 0 or height <= 0 or not raw:
            raise RuntimeError("图像消息缺少 width/height/data")

        if encoding in ("rgb8", "bgr8"):
            bytes_per_line = step if step > 0 else width * 3
            img = QImage(raw, width, height, bytes_per_line, QImage.Format.Format_RGB888)
            if encoding == "bgr8":
                img = img.rgbSwapped()
            return img.copy()

        if encoding in ("rgba8", "bgra8"):
            bytes_per_line = step if step > 0 else width * 4
            img = QImage(raw, width, height, bytes_per_line, QImage.Format.Format_RGBA8888)
            if encoding == "bgra8":
                img = img.rgbSwapped()
            return img.copy()

        if encoding in ("mono8", "8uc1"):
            bytes_per_line = step if step > 0 else width
            img = QImage(raw, width, height, bytes_per_line, QImage.Format.Format_Grayscale8)
            return img.copy()

        if encoding in ("16uc1", "mono16"):
            bytes_per_line = step if step > 0 else width * 2
            needed = bytes_per_line * height
            if len(raw) < needed:
                raise RuntimeError("16位图像数据长度不足")

            big_endian = bool(msg.get("is_bigendian", 0))
            depth_vals = []
            gray = bytearray(width * height)

            # 先统计非零深度范围（0通常表示无效）
            for r in range(height):
                row_start = r * bytes_per_line
                for c in range(width):
                    i = row_start + c * 2
                    b0 = raw[i]
                    b1 = raw[i + 1]
                    v = (b0 << 8) | b1 if big_endian else (b1 << 8) | b0
                    if v > 0:
                        depth_vals.append(v)

            if depth_vals:
                vmin = min(depth_vals)
                vmax = max(depth_vals)
                span = max(1, vmax - vmin)
            else:
                vmin, span = 0, 1

            # 映射到8位灰度，便于界面显示/PNG保存
            idx = 0
            for r in range(height):
                row_start = r * bytes_per_line
                for c in range(width):
                    i = row_start + c * 2
                    b0 = raw[i]
                    b1 = raw[i + 1]
                    v = (b0 << 8) | b1 if big_endian else (b1 << 8) | b0
                    if v <= 0:
                        gray[idx] = 0
                    else:
                        gray[idx] = int((v - vmin) * 255 / span)
                    idx += 1

            img = QImage(bytes(gray), width, height, width, QImage.Format.Format_Grayscale8)
            return img.copy()

        raise RuntimeError(f"暂不支持的图像编码: {encoding}")

    def _set_vision_image(self, img: QImage, path: str):
        if img and not img.isNull():
            pix = QPixmap.fromImage(img)
            pix = pix.scaled(
                self.vision_preview.width(),
                self.vision_preview.height(),
                aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio,
                transformMode=Qt.TransformationMode.SmoothTransformation,
            )
            self.vision_preview.setPixmap(pix)
        self.vision_info.setText(path)

    def _set_factory_camera_image(self, key: str, img: QImage, path: str):
        key_name = str(key or "").strip().lower()
        if key_name == "head":
            preview = getattr(self, "factory_camera_head_preview", None)
            path_field = getattr(self, "factory_camera_head_path", None)
        elif key_name == "chest":
            preview = getattr(self, "factory_camera_chest_preview", None)
            path_field = getattr(self, "factory_camera_chest_path", None)
        else:
            return

        if preview is not None and img and not img.isNull():
            pix = QPixmap.fromImage(img).scaled(
                preview.width(),
                preview.height(),
                aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio,
                transformMode=Qt.TransformationMode.SmoothTransformation,
            )
            preview.setPixmap(pix)
        if path_field is not None:
            path_field.setText(str(path or "-"))

    def _set_vision_topic_rate(self, topic: str, text: str):
        field = self.vision_rate_fields.get(topic) if hasattr(self, "vision_rate_fields") else None
        if field is not None:
            field.setText(text)

    def _measure_topic_rate_via_ros(self, topic: str, sample_sec: float = 1.5):
        if not (self.ros and self.ros.check_connection()):
            return None
        hits = []
        lock = threading.Lock()

        def cb(_msg):
            with lock:
                hits.append(time.time())

        try:
            self.ros.subscribe(topic, cb)
        except Exception:
            return None

        try:
            time.sleep(sample_sec)
        finally:
            try:
                self.ros.unsubscribe(topic)
            except Exception:
                pass

        with lock:
            count = len(hits)
            if count >= 2 and hits[-1] > hits[0]:
                return (count - 1) / (hits[-1] - hits[0])
            if count == 1:
                return 0.0
        return None

    def _measure_topic_rate_via_ssh(self, topic: str):
        if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
            return None
        cmd = f"timeout 4s rostopic hz {shlex.quote(topic)}"
        out, err = self._run_ros_cli_via_ssh(cmd)
        text = ((out or "") + "\n" + (err or "")).strip()
        m = re.search(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)", text, flags=re.IGNORECASE)
        if m:
            try:
                return float(m.group(1))
            except Exception:
                return None
        return None

    def refresh_vision_topic_rates(self):
        if self._vision_rate_refreshing:
            self.log_signal.emit("[INFO] 六路话题频率正在刷新，请稍候")
            return

        def worker():
            self._vision_rate_refreshing = True
            try:
                self.log_signal.emit("[INFO] 开始刷新六路话题频率")
                ros_ok = bool(self.ros and self.ros.check_connection())
                ssh_ok = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)

                if not ros_ok and not ssh_ok:
                    self.log_signal.emit("[ERR] 请先连接 ROSBridge 或 SSH(小脑)")
                    for group_name, _items in self.vision_rate_groups:
                        self.vision_topic_rate_signal.emit(group_name, "未连接")
                    return

                for group_name, items in self.vision_rate_groups:
                    parts = []
                    for short_name, topic in items:
                        hz = None
                        if ros_ok:
                            hz = self._measure_topic_rate_via_ros(topic)
                        if hz is None and ssh_ok:
                            hz = self._measure_topic_rate_via_ssh(topic)
                        if hz is None:
                            parts.append(f"{short_name}:无数据")
                        else:
                            parts.append(f"{short_name}:{hz:.2f} Hz")
                    self.vision_topic_rate_signal.emit(group_name, " | ".join(parts))

                self.log_signal.emit("[OK] 六路话题频率已刷新")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 刷新话题频率失败: {e}")
            finally:
                self._vision_rate_refreshing = False

        self._run_async(worker)

    def _finish_vision_sampling(self):
        self._vision_active = False
        self._vision_continuous = False
        self._vision_topic = None
        self.btn_vision_detect.setEnabled(True)
        self.btn_vision_detect.setText("检测当前帧")
        self.btn_vision_stop.setEnabled(False)

    def stop_vision_sampling(self):
        if not self._vision_active or not self._vision_topic:
            self.log_signal.emit("[INFO] 当前没有连续采样任务")
            return

        topic = self._vision_topic
        try:
            if self.ros:
                self.ros.unsubscribe(topic)
        except Exception:
            pass

        self._finish_vision_sampling()
        self.log_signal.emit(f"[OK] 已结束采样: {topic}")

    def detect_vision_frame(self):
        if self._vision_active:
            self.log_signal.emit("[INFO] 视觉检测进行中，请稍候")
            return

        topic = self.vision_topic_edit.currentText().strip() if hasattr(self, "vision_topic_edit") else ""
        if not topic:
            self.log_signal.emit("[ERR] 请先选择图像话题")
            return

        mode_text = self.vision_mode.currentText().strip()
        continuous = mode_text == "多帧"

        ros_ok = bool(self.ros and self.ros.check_connection())
        if not ros_ok and not self._ensure_ssh():
            return
        if not ros_ok and continuous:
            self.log_signal.emit("[WARN] ROSBridge 未连接时，视觉监控仅支持单帧")
            continuous = False

        self._vision_active = True
        self._vision_continuous = continuous
        self._vision_topic = topic
        self.btn_vision_detect.setEnabled(False)
        self.btn_vision_detect.setText("采样中..." if continuous else "检测中...")
        self.btn_vision_stop.setEnabled(continuous)

        def on_msg(msg: dict):
            try:
                img = self._decode_ros_image(msg)
                path = self._save_qimage(img)
                self.vision_image_signal.emit(img, path)
                if self._vision_continuous:
                    self.log_signal.emit(f"[OK] 连续采样已保存: {path}")
                else:
                    self.log_signal.emit(f"[OK] 已保存视觉帧: {path}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 图像解析失败: {e}")
                self.vision_image_signal.emit(QImage(), "解析失败")

            if not self._vision_continuous:
                try:
                    if self.ros:
                        self.ros.unsubscribe(topic)
                except Exception:
                    pass
                self._finish_vision_sampling()

        def worker():
            try:
                if self.ros and self.ros.check_connection():
                    topic_type = self.ros.subscribe(topic, on_msg)
                    if continuous:
                        self.log_signal.emit(f"[INFO] 连续采样已订阅 {topic} ({topic_type})，点击“结束采样”停止")
                    else:
                        self.log_signal.emit(f"[INFO] 视觉检测已订阅 {topic} ({topic_type})，等待首帧...")
                else:
                    self.log_signal.emit("[INFO] ROSBridge 未连接，改用 SSH(小脑) 单帧抓图")
                    out, err = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {shlex.quote(topic)}")
                    msg = self._parse_ros_cli_yaml(out or err or "")
                    if not isinstance(msg, dict):
                        raise RuntimeError((err or out or "未获取到图像消息").strip())
                    img = self._decode_ros_image(msg)
                    path = self._save_qimage(img)
                    self.vision_image_signal.emit(img, path)
                    self.log_signal.emit(f"[OK] 已保存视觉帧: {path}")
                    self._finish_vision_sampling()
            except Exception as e:
                self.log_signal.emit(f"[ERR] 视觉检测订阅失败: {e}")
                self._finish_vision_sampling()

        self._run_async(worker)

    def _save_pose_image(self, img: QImage, prefix: str) -> str:
        project_root = self._get_storage_root()
        if self._instance_tag:
            log_dir = os.path.join(project_root, "logs", self._instance_tag, "pose")
        else:
            log_dir = os.path.join(project_root, "logs", "pose")
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime(f"{prefix}_%Y%m%d_%H%M%S_%f.png")
        path = os.path.join(log_dir, filename)
        img.save(path, "PNG")
        return path

    def _set_pose_image(self, key: str, img: QImage, info: str):
        if key == "coke":
            self.pose_coke_info.setText(info)
            if img and not img.isNull():
                pix = QPixmap.fromImage(img).scaled(
                    self.pose_coke_preview.width(),
                    self.pose_coke_preview.height(),
                    aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio,
                    transformMode=Qt.TransformationMode.SmoothTransformation,
                )
                self.pose_coke_preview.setPixmap(pix)
        elif key == "water":
            self.pose_water_info.setText(info)
            if img and not img.isNull():
                pix = QPixmap.fromImage(img).scaled(
                    self.pose_water_preview.width(),
                    self.pose_water_preview.height(),
                    aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio,
                    transformMode=Qt.TransformationMode.SmoothTransformation,
                )
                self.pose_water_preview.setPixmap(pix)

    def start_pose_polling(self):
        if self._pose_polling:
            self.log_signal.emit("[INFO] 位姿估计采样已在运行")
            return
        if not self._ensure_ssh_big():
            return

        coke = self.pose_remote_coke_edit.text().strip()
        water = self.pose_remote_water_edit.text().strip()
        if not coke or not water:
            self.log_signal.emit("[ERR] 请填写可乐和水两张PNG路径")
            return

        try:
            interval = int(self.pose_interval_edit.text().strip() or "1000")
        except Exception:
            interval = 1000
        interval = max(200, interval)

        self._pose_remote_coke = coke
        self._pose_remote_water = water
        self._pose_last_hash = {"coke": None, "water": None}
        self._pose_polling = True
        self._pose_busy = False
        self._pose_timer.setInterval(interval)
        self._pose_timer.start()
        self.btn_pose_start.setEnabled(False)
        self.btn_pose_stop.setEnabled(True)
        self.log_signal.emit(f"[OK] 位姿估计开始采样，间隔 {interval}ms")
        self._poll_pose_images()

    def stop_pose_polling(self):
        if not self._pose_polling:
            self.log_signal.emit("[INFO] 位姿估计当前未采样")
            return
        self._pose_polling = False
        self._pose_timer.stop()
        self.btn_pose_start.setEnabled(True)
        self.btn_pose_stop.setEnabled(False)
        self.log_signal.emit("[OK] 位姿估计已结束采样")

    def _poll_pose_images(self):
        if not self._pose_polling or self._pose_busy:
            return
        if not self.ssh_big or not self.ssh_big.sftp:
            self.stop_pose_polling()
            self.log_signal.emit("[ERR] 大脑SSH未连接，位姿估计采样已停止")
            return

        self._pose_busy = True

        def worker():
            try:
                for key, remote_path in (("coke", self._pose_remote_coke), ("water", self._pose_remote_water)):
                    try:
                        with self.ssh_big.sftp.open(remote_path, "rb") as f:
                            blob = f.read()

                        digest = hashlib.md5(blob).hexdigest()
                        if self._pose_last_hash.get(key) == digest:
                            # 文件未变化：跳过保存
                            continue

                        img = QImage.fromData(blob, "PNG")
                        if img.isNull():
                            raise RuntimeError("文件不是有效PNG")
                        save_path = self._save_pose_image(img, key)
                        self._pose_last_hash[key] = digest
                        self.pose_image_signal.emit(key, img, save_path)
                    except Exception as e:
                        self.pose_image_signal.emit(key, QImage(), f"读取失败: {e}")
            finally:
                self._pose_busy = False

        self._run_async(worker)

    def _ensure_ssh(self) -> bool:
        if not self.ssh or not self.ssh.ssh or not self.ssh.sftp:
            self.log_signal.emit("[ERR] 请先连接 SSH(小脑)")
            return False
        return True

    def _ensure_ssh_big(self) -> bool:
        if not self.ssh_big or not self.ssh_big.ssh or not self.ssh_big.sftp:
            self.log_signal.emit("[ERR] 请先连接 SSH(大脑)")
            return False
        return True

    def _get_transfer_client(self):
        target = self.transfer_target.currentText().strip() if hasattr(self, "transfer_target") else "小脑"
        if target == "大脑":
            if not self._ensure_ssh_big():
                return None, None
            return self.ssh_big, "大脑"
        if not self._ensure_ssh():
            return None, None
        return self.ssh, "小脑"

    def _get_transfer_client_by_name(self, target_name: str):
        target = str(target_name or "").strip()
        if target == "大脑":
            if not self._ensure_ssh_big():
                return None
            return self.ssh_big
        if not self._ensure_ssh():
            return None
        return self.ssh

    def _is_supported_remote_edit_file(self, remote_path: str) -> bool:
        path = str(remote_path or "").strip()
        if not path:
            return False
        name = os.path.basename(path)
        ext = os.path.splitext(name)[1].lower()
        if ext in self._remote_edit_supported_exts:
            return True
        if name in self._remote_edit_supported_names:
            return True
        if name.startswith(".") and len(name) > 1 and ("." not in name[1:]):
            return True
        return False

    def _get_remote_edit_dir_input_text(self) -> str:
        if hasattr(self, "remote_edit_dir_input"):
            return self.remote_edit_dir_input.currentText().strip()
        return ""

    def _set_remote_edit_dir_input_text(self, path: str):
        text = str(path or "").strip()
        if not text or (not hasattr(self, "remote_edit_dir_input")):
            return
        self._add_remote_edit_dir_candidate(text)
        self.remote_edit_dir_input.setCurrentText(text)

    def _add_remote_edit_dir_candidate(self, path: str):
        text = str(path or "").strip()
        if not text:
            return
        items = [self.remote_edit_dir_input.itemText(i).strip() for i in range(self.remote_edit_dir_input.count())]
        items = [x for x in items if x and x != text]
        items.insert(0, text)
        if len(items) > 100:
            items = items[:100]
        self.remote_edit_dir_input.blockSignals(True)
        self.remote_edit_dir_input.clear()
        self.remote_edit_dir_input.addItems(items)
        self.remote_edit_dir_input.setCurrentText(text)
        self.remote_edit_dir_input.blockSignals(False)
        self._remote_edit_dir_history = items

    def _get_remote_edit_file_input_text(self) -> str:
        if hasattr(self, "remote_edit_file_input"):
            return self.remote_edit_file_input.currentText().strip()
        return ""

    def _set_remote_edit_file_input_text(self, name: str):
        text = str(name or "").strip()
        if not text or (not hasattr(self, "remote_edit_file_input")):
            return
        self._add_remote_edit_file_candidate(text)
        self.remote_edit_file_input.setCurrentText(text)

    def _add_remote_edit_file_candidate(self, name: str):
        text = str(name or "").strip()
        if not text:
            return
        items = [self.remote_edit_file_input.itemText(i).strip() for i in range(self.remote_edit_file_input.count())]
        items = [x for x in items if x and x != text]
        items.insert(0, text)
        if len(items) > 200:
            items = items[:200]
        self.remote_edit_file_input.blockSignals(True)
        self.remote_edit_file_input.clear()
        self.remote_edit_file_input.addItems(items)
        self.remote_edit_file_input.setCurrentText(text)
        self.remote_edit_file_input.blockSignals(False)
        self._remote_edit_file_history = items

    def _build_remote_edit_path_from_inputs(self) -> str:
        dir_text = self._get_remote_edit_dir_input_text().rstrip("/")
        file_text = self._get_remote_edit_file_input_text()
        if not file_text:
            return ""
        if file_text.startswith("/"):
            return file_text
        if not dir_text:
            return f"/{file_text}"
        return f"{dir_text}/{file_text}"

    def _fill_remote_edit_inputs_from_path(self, remote_path: str):
        full = str(remote_path or "").strip()
        if not full:
            return
        self._add_remote_edit_dir_candidate(os.path.dirname(full) or "/")
        self._set_remote_edit_dir_input_text(os.path.dirname(full) or "/")
        self._set_remote_edit_file_input_text(os.path.basename(full) or full)

    def _normalize_remote_posix_path(self, path: str) -> str:
        raw = str(path or "").strip()
        if not raw:
            return "/"
        if not raw.startswith("/"):
            raw = f"/{raw}"
        parts = [x for x in raw.split("/") if x and x != "."]
        stack = []
        for item in parts:
            if item == "..":
                if stack:
                    stack.pop()
            else:
                stack.append(item)
        return "/" + "/".join(stack)

    def _guess_remote_browse_start_dir(self, path: str, fallback: str = "/tmp") -> str:
        normalized = self._normalize_remote_posix_path(path or fallback)
        if normalized == "/":
            return normalized
        leaf = os.path.basename(normalized.rstrip("/"))
        if os.path.splitext(leaf)[1]:
            return os.path.dirname(normalized.rstrip("/")) or "/"
        return normalized

    def _browse_sftp_path_dialog(self, client: SshSftpClient, target_name: str, title: str, start_path: str = "/tmp", select_kind: str = "file", file_filter=None, multi_select: bool = False):
        start_dir = self._normalize_remote_posix_path(start_path or "/tmp")

        dlg = QDialog(self)
        dlg.setWindowTitle(f"{title}({target_name})")
        dlg.resize(760, 520)

        root_layout = QVBoxLayout(dlg)

        nav_row = QHBoxLayout()
        nav_row.addWidget(QLabel("当前目录"))
        path_edit = QLineEdit(start_dir)
        btn_go = QPushButton("跳转")
        btn_up = QPushButton("上一级")
        btn_refresh = QPushButton("刷新")
        nav_row.addWidget(path_edit)
        nav_row.addWidget(btn_go)
        nav_row.addWidget(btn_up)
        nav_row.addWidget(btn_refresh)

        hint_text = "双击目录进入；双击文件直接确认。" if select_kind == "file" else "双击目录进入；选择按钮可选当前目录或列表中的目录。"
        if select_kind == "file" and multi_select:
            hint_text = "双击目录进入；可多选文件后点击选择。"
        hint = QLabel(hint_text)
        list_widget = QListWidget()
        if select_kind == "file" and multi_select:
            list_widget.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)

        action_row = QHBoxLayout()
        btn_ok = QPushButton("选择")
        btn_cancel = QPushButton("取消")
        action_row.addStretch(1)
        action_row.addWidget(btn_ok)
        action_row.addWidget(btn_cancel)

        root_layout.addLayout(nav_row)
        root_layout.addWidget(hint)
        root_layout.addWidget(list_widget)
        root_layout.addLayout(action_row)

        state = {"current_dir": start_dir, "selected_path": "", "selected_paths": []}

        def _accept_paths(chosen_paths):
            normalized = []
            seen = set()
            for chosen_path in chosen_paths:
                value = self._normalize_remote_posix_path(chosen_path)
                if value in seen:
                    continue
                seen.add(value)
                normalized.append(value)
            if not normalized:
                return
            state["selected_paths"] = normalized
            state["selected_path"] = normalized[0]
            dlg.accept()

        def _load_dir(path: str):
            target_dir = self._normalize_remote_posix_path(path)
            try:
                entries = client.sftp.listdir_attr(target_dir)
            except Exception as e:
                QMessageBox.warning(dlg, "读取失败", f"无法读取目录:\n{target_dir}\n\n{e}")
                return

            dirs = []
            files = []
            for entry in entries:
                name = str(getattr(entry, "filename", "") or "").strip()
                if not name:
                    continue
                full = f"{target_dir.rstrip('/')}/{name}" if target_dir != "/" else f"/{name}"
                if stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                    dirs.append((name, full))
                else:
                    if file_filter and (not file_filter(full)):
                        continue
                    files.append((name, full))

            dirs.sort(key=lambda item: item[0])
            files.sort(key=lambda item: item[0])

            state["current_dir"] = target_dir
            state["selected_path"] = ""
            state["selected_paths"] = []
            path_edit.setText(target_dir)
            list_widget.clear()

            if target_dir != "/":
                parent = os.path.dirname(target_dir.rstrip("/")) or "/"
                item_up = QListWidgetItem("..")
                item_up.setData(Qt.ItemDataRole.UserRole, ("dir", parent))
                list_widget.addItem(item_up)

            for name, full in dirs:
                item = QListWidgetItem(f"[DIR] {name}/")
                item.setData(Qt.ItemDataRole.UserRole, ("dir", full))
                list_widget.addItem(item)

            for name, full in files:
                item = QListWidgetItem(name)
                item.setData(Qt.ItemDataRole.UserRole, ("file", full))
                list_widget.addItem(item)

        def _on_item_double_clicked(item: QListWidgetItem):
            data = item.data(Qt.ItemDataRole.UserRole)
            if not data:
                return
            kind, full = data
            if kind == "dir":
                _load_dir(full)
                return
            if kind == "file" and select_kind == "file" and not multi_select:
                _accept_paths([full])

        def _on_select_clicked():
            if select_kind == "file" and multi_select:
                selected_files = []
                for item in list_widget.selectedItems():
                    data = item.data(Qt.ItemDataRole.UserRole)
                    if not data:
                        continue
                    kind, full = data
                    if kind == "file":
                        selected_files.append(full)
                if selected_files:
                    _accept_paths(selected_files)
                    return
            item = list_widget.currentItem()
            if item is not None:
                data = item.data(Qt.ItemDataRole.UserRole)
                if data:
                    kind, full = data
                    if kind == "dir" and select_kind == "dir":
                        _accept_paths([full])
                        return
                    if kind == "file" and select_kind == "file":
                        _accept_paths([full])
                        return
            typed = (path_edit.text() or "").strip()
            if not typed:
                if select_kind == "dir":
                    _accept_paths([state["current_dir"]])
                    return
                QMessageBox.information(dlg, "提示", "请先选择一个文件，或输入完整文件路径。")
                return
            _accept_paths([typed])

        list_widget.itemDoubleClicked.connect(_on_item_double_clicked)
        btn_go.clicked.connect(lambda: _load_dir(path_edit.text()))
        btn_refresh.clicked.connect(lambda: _load_dir(state["current_dir"]))
        btn_up.clicked.connect(lambda: _load_dir(os.path.dirname(state["current_dir"].rstrip("/")) or "/"))
        btn_ok.clicked.connect(_on_select_clicked)
        btn_cancel.clicked.connect(dlg.reject)

        _load_dir(start_dir)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return [] if (select_kind == "file" and multi_select) else ""
        if select_kind == "file" and multi_select:
            return list(state.get("selected_paths") or [])
        return str(state.get("selected_path") or "").strip()

    def _choose_local_download_targets(self, remote_paths, single_title: str = "选择本地保存路径", multi_title: str = "选择本地保存目录"):
        paths = [str(item or "").strip() for item in (remote_paths or []) if str(item or "").strip()]
        if not paths:
            return []
        if len(paths) == 1:
            default_name = os.path.basename(paths[0].rstrip("/")) or "download.bin"
            local_path, _ = QFileDialog.getSaveFileName(self, single_title, default_name)
            if not local_path:
                return []
            return [(paths[0], local_path)]

        local_parent = QFileDialog.getExistingDirectory(self, multi_title)
        if not local_parent:
            return []

        targets = []
        used_paths = set()
        for remote_path in paths:
            base_name = os.path.basename(remote_path.rstrip("/")) or "download.bin"
            local_path = os.path.join(local_parent, base_name)
            if local_path in used_paths:
                stem, ext = os.path.splitext(base_name)
                index = 2
                while True:
                    candidate_name = f"{stem}_{index}{ext}"
                    candidate_path = os.path.join(local_parent, candidate_name)
                    if candidate_path not in used_paths:
                        local_path = candidate_path
                        break
                    index += 1
            used_paths.add(local_path)
            targets.append((remote_path, local_path))
        return targets

    def pick_remote_file_dialog(self):
        client, target_name = self._get_transfer_client()
        if not client:
            return

        start_dir = self._get_remote_edit_dir_input_text().strip() or "/tmp"
        if not start_dir.startswith("/"):
            start_dir = f"/{start_dir}"

        dlg = QDialog(self)
        dlg.setWindowTitle(f"远程文件选择({target_name})")
        dlg.resize(760, 520)

        root_layout = QVBoxLayout(dlg)

        nav_row = QHBoxLayout()
        nav_row.addWidget(QLabel("当前目录"))
        path_edit = QLineEdit(start_dir)
        btn_go = QPushButton("跳转")
        btn_up = QPushButton("上一级")
        btn_refresh = QPushButton("刷新")
        nav_row.addWidget(path_edit)
        nav_row.addWidget(btn_go)
        nav_row.addWidget(btn_up)
        nav_row.addWidget(btn_refresh)

        hint = QLabel("双击目录进入；双击文件直接确认。仅展示可编辑文件类型。")
        list_widget = QListWidget()

        action_row = QHBoxLayout()
        btn_ok = QPushButton("选择")
        btn_cancel = QPushButton("取消")
        action_row.addStretch(1)
        action_row.addWidget(btn_ok)
        action_row.addWidget(btn_cancel)

        root_layout.addLayout(nav_row)
        root_layout.addWidget(hint)
        root_layout.addWidget(list_widget)
        root_layout.addLayout(action_row)

        state = {"current_dir": start_dir, "selected_file": ""}

        def _norm_dir(p: str) -> str:
            raw = str(p or "").strip()
            if not raw:
                return "/"
            if not raw.startswith("/"):
                raw = f"/{raw}"
            parts = [x for x in raw.split("/") if x and x != "."]
            stack = []
            for x in parts:
                if x == "..":
                    if stack:
                        stack.pop()
                else:
                    stack.append(x)
            return "/" + "/".join(stack)

        def _load_dir(path: str):
            target_dir = _norm_dir(path)
            try:
                entries = client.sftp.listdir_attr(target_dir)
            except Exception as e:
                QMessageBox.warning(dlg, "读取失败", f"无法读取目录:\n{target_dir}\n\n{e}")
                return

            dirs = []
            files = []
            for entry in entries:
                name = str(getattr(entry, "filename", "") or "").strip()
                if not name:
                    continue
                if stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                    dirs.append(name)
                else:
                    full = f"{target_dir.rstrip('/')}/{name}" if target_dir != "/" else f"/{name}"
                    if self._is_supported_remote_edit_file(full):
                        files.append(name)

            dirs.sort()
            files.sort()

            state["current_dir"] = target_dir
            state["selected_file"] = ""
            path_edit.setText(target_dir)
            list_widget.clear()

            if target_dir != "/":
                parent = os.path.dirname(target_dir.rstrip("/")) or "/"
                item_up = QListWidgetItem("..")
                item_up.setData(Qt.ItemDataRole.UserRole, ("dir", parent))
                list_widget.addItem(item_up)

            for name in dirs:
                full = f"{target_dir.rstrip('/')}/{name}" if target_dir != "/" else f"/{name}"
                item = QListWidgetItem(f"[DIR] {name}/")
                item.setData(Qt.ItemDataRole.UserRole, ("dir", full))
                list_widget.addItem(item)

            for name in files:
                full = f"{target_dir.rstrip('/')}/{name}" if target_dir != "/" else f"/{name}"
                item = QListWidgetItem(name)
                item.setData(Qt.ItemDataRole.UserRole, ("file", full))
                list_widget.addItem(item)

        def _on_item_double_clicked(item: QListWidgetItem):
            data = item.data(Qt.ItemDataRole.UserRole)
            if not data:
                return
            kind, full = data
            if kind == "dir":
                _load_dir(full)
                return
            if kind == "file":
                state["selected_file"] = full
                dlg.accept()

        def _on_select_clicked():
            item = list_widget.currentItem()
            if item is not None:
                data = item.data(Qt.ItemDataRole.UserRole)
                if data:
                    kind, full = data
                    if kind == "dir":
                        _load_dir(full)
                        return
                    if kind == "file":
                        state["selected_file"] = full
                        dlg.accept()
                        return

            typed = (path_edit.text() or "").strip()
            if typed and self._is_supported_remote_edit_file(typed):
                state["selected_file"] = typed
                dlg.accept()
                return
            QMessageBox.information(dlg, "提示", "请先选择一个文件，或输入完整文件路径。")

        list_widget.itemDoubleClicked.connect(_on_item_double_clicked)
        btn_go.clicked.connect(lambda: _load_dir(path_edit.text()))
        btn_refresh.clicked.connect(lambda: _load_dir(state["current_dir"]))
        btn_up.clicked.connect(lambda: _load_dir(os.path.dirname(state["current_dir"].rstrip("/")) or "/"))
        btn_ok.clicked.connect(_on_select_clicked)
        btn_cancel.clicked.connect(dlg.reject)

        _load_dir(start_dir)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        chosen = str(state.get("selected_file") or "").strip()
        if not chosen:
            return
        self._fill_remote_edit_inputs_from_path(chosen)
        self.log_signal.emit(f"[OK] 已选择远程文件({target_name}): {chosen}")

    def focus_remote_find_input(self):
        if not hasattr(self, "remote_find_input"):
            return
        self.remote_find_input.setFocus()
        self.remote_find_input.selectAll()

    def remote_find_text(self, backward: bool = False):
        if not hasattr(self, "remote_file_editor") or not hasattr(self, "remote_find_input"):
            return
        needle = (self.remote_find_input.text() or "").strip()
        if not needle:
            self.log_signal.emit("[WARN] 请输入要搜索的文本")
            return

        flags = QTextDocument.FindFlag(0)
        if backward:
            flags |= QTextDocument.FindFlag.FindBackward

        found = self.remote_file_editor.find(needle, flags)
        if not found:
            cursor = self.remote_file_editor.textCursor()
            if backward:
                cursor.movePosition(QTextCursor.MoveOperation.End)
            else:
                cursor.movePosition(QTextCursor.MoveOperation.Start)
            self.remote_file_editor.setTextCursor(cursor)
            found = self.remote_file_editor.find(needle, flags)

        if not found:
            self.log_signal.emit(f"[WARN] 未找到: {needle}")

    def _remote_comment_style(self):
        ext = os.path.splitext(str(getattr(self, "_remote_edit_path", "") or "").strip())[1].lower()
        if ext in (".py", ".sh", ".txt"):
            return "hash"
        if ext in (".cpp",):
            return "slash"
        if ext in (".launch",):
            return "xml"
        return "hash"

    def toggle_remote_comment_selection(self):
        if not hasattr(self, "remote_file_editor"):
            return

        editor = self.remote_file_editor
        doc = editor.document()
        cursor = editor.textCursor()
        if not cursor.hasSelection():
            cursor.select(QTextCursor.SelectionType.LineUnderCursor)

        sel_start = cursor.selectionStart()
        sel_end = cursor.selectionEnd()
        start_block = doc.findBlock(sel_start)
        end_pos = sel_end - 1 if sel_end > sel_start else sel_end
        end_block = doc.findBlock(end_pos)

        start_no = start_block.blockNumber()
        end_no = end_block.blockNumber()
        if start_no < 0 or end_no < start_no:
            return

        style = self._remote_comment_style()
        lines = []
        for bn in range(start_no, end_no + 1):
            blk = doc.findBlockByNumber(bn)
            if not blk.isValid():
                continue
            lines.append(blk.text())

        non_empty = [ln for ln in lines if ln.strip()]
        if not non_empty:
            return

        if style == "xml":
            all_commented = all(ln.lstrip().startswith("<!--") and ln.rstrip().endswith("-->") for ln in non_empty)
        elif style == "slash":
            all_commented = all(ln.lstrip().startswith("//") for ln in non_empty)
        else:
            all_commented = all(ln.lstrip().startswith("#") for ln in non_empty)

        cursor.beginEditBlock()
        try:
            for bn in range(start_no, end_no + 1):
                blk = doc.findBlockByNumber(bn)
                if not blk.isValid():
                    continue
                old_line = blk.text()
                new_line = old_line

                if style == "xml":
                    stripped = old_line.strip()
                    if not stripped:
                        new_line = old_line
                    elif all_commented and stripped.startswith("<!--") and stripped.endswith("-->"):
                        indent_len = len(old_line) - len(old_line.lstrip())
                        inner = stripped[4:-3].strip()
                        new_line = (old_line[:indent_len] + inner)
                    elif not all_commented:
                        indent_len = len(old_line) - len(old_line.lstrip())
                        content = old_line[indent_len:]
                        new_line = f"{old_line[:indent_len]}<!-- {content} -->"
                else:
                    marker = "//" if style == "slash" else "#"
                    indent_len = len(old_line) - len(old_line.lstrip())
                    indent = old_line[:indent_len]
                    content = old_line[indent_len:]

                    if not content.strip():
                        new_line = old_line
                    elif all_commented:
                        if content.startswith(marker):
                            rest = content[len(marker):]
                            if rest.startswith(" "):
                                rest = rest[1:]
                            new_line = indent + rest
                    else:
                        new_line = f"{indent}{marker} {content}"

                block_cursor = QTextCursor(blk)
                block_cursor.select(QTextCursor.SelectionType.LineUnderCursor)
                block_cursor.insertText(new_line)
        finally:
            cursor.endEditBlock()

    def refresh_remote_edit_directories(self):
        client, target_name = self._get_transfer_client()
        if not client:
            return

        current_dir = self._get_remote_edit_dir_input_text() or "/tmp"
        base_dir = current_dir.rstrip("/") or "/"

        if hasattr(self, "btn_remote_edit_refresh_dirs"):
            self.btn_remote_edit_refresh_dirs.setEnabled(False)

        def worker():
            try:
                candidates = [base_dir]
                for entry in client.sftp.listdir_attr(base_dir):
                    name = str(getattr(entry, "filename", "") or "").strip()
                    if not name:
                        continue
                    if stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                        full = f"{base_dir.rstrip('/')}/{name}" if base_dir != "/" else f"/{name}"
                        candidates.append(full)
                candidates.sort()
                self.remote_edit_dirs_signal.emit(candidates, base_dir)
            except Exception as e:
                self.remote_edit_failed_signal.emit(f"刷新目录失败: {e}")

        self._run_async(worker)

    def _on_remote_edit_dirs_loaded(self, candidates: list, current_dir: str):
        if hasattr(self, "btn_remote_edit_refresh_dirs"):
            self.btn_remote_edit_refresh_dirs.setEnabled(True)
        merged = [str(x).strip() for x in (candidates or []) if str(x).strip()]
        for old in getattr(self, "_remote_edit_dir_history", []):
            old_s = str(old).strip()
            if old_s and old_s not in merged:
                merged.append(old_s)
        if len(merged) > 100:
            merged = merged[:100]
        current = self._get_remote_edit_dir_input_text()
        self.remote_edit_dir_input.blockSignals(True)
        self.remote_edit_dir_input.clear()
        self.remote_edit_dir_input.addItems(merged)
        if current:
            self.remote_edit_dir_input.setCurrentText(current)
        self.remote_edit_dir_input.blockSignals(False)
        self._remote_edit_dir_history = merged
        self.log_signal.emit(f"[OK] 已刷新可选目录({current_dir})，共 {len(candidates or [])} 个")
        self.refresh_remote_edit_files()

    def enter_remote_edit_directory(self):
        current_dir = self._get_remote_edit_dir_input_text() or "/tmp"
        self._set_remote_edit_dir_input_text(current_dir)
        self.refresh_remote_edit_directories()

    def go_remote_edit_parent_directory(self):
        current_dir = self._get_remote_edit_dir_input_text() or "/tmp"
        base_dir = current_dir.rstrip("/") or "/"
        parent_dir = os.path.dirname(base_dir.rstrip("/")) or "/"
        self._set_remote_edit_dir_input_text(parent_dir)
        self.refresh_remote_edit_directories()

    def refresh_remote_edit_files(self):
        client, target_name = self._get_transfer_client()
        if not client:
            return

        base_dir = self._get_remote_edit_dir_input_text().rstrip("/") or "/"
        if hasattr(self, "btn_remote_edit_refresh_files"):
            self.btn_remote_edit_refresh_files.setEnabled(False)

        def worker():
            try:
                names = []
                for entry in client.sftp.listdir_attr(base_dir):
                    name = str(getattr(entry, "filename", "") or "").strip()
                    if not name or stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                        continue
                    full = f"{base_dir.rstrip('/')}/{name}" if base_dir != "/" else f"/{name}"
                    if self._is_supported_remote_edit_file(full):
                        names.append(name)
                names.sort()
                self.remote_edit_files_signal.emit(names, base_dir)
            except Exception as e:
                self.remote_edit_failed_signal.emit(f"刷新文件失败: {e}")

        self._run_async(worker)

    def _on_remote_edit_files_loaded(self, file_names: list, base_dir: str):
        if hasattr(self, "btn_remote_edit_refresh_files"):
            self.btn_remote_edit_refresh_files.setEnabled(True)
        merged = [str(x).strip() for x in (file_names or []) if str(x).strip()]
        for old in getattr(self, "_remote_edit_file_history", []):
            old_s = str(old).strip()
            if old_s and old_s not in merged:
                merged.append(old_s)
        if len(merged) > 200:
            merged = merged[:200]
        current = self._get_remote_edit_file_input_text()
        self.remote_edit_file_input.blockSignals(True)
        self.remote_edit_file_input.clear()
        self.remote_edit_file_input.addItems(merged)
        if current:
            self.remote_edit_file_input.setCurrentText(current)
        self.remote_edit_file_input.blockSignals(False)
        self._remote_edit_file_history = merged
        self.log_signal.emit(f"[OK] 已刷新可选文件({base_dir})，共 {len(file_names or [])} 个")

    def _run_bash(self, client: SshSftpClient, cmd: str):
        return client.run(f"bash -lc {shlex.quote(cmd)}")

    def _run_sudo_bash(self, client: SshSftpClient, password: str, cmd: str):
        sudo_cmd = f"echo {shlex.quote(password)} | sudo -S -p '' bash -lc {shlex.quote(cmd)}"
        return client.run(sudo_cmd)

    def _run_sudo_bash_with_exit_code(self, client: SshSftpClient, password: str, cmd: str):
        wrapped = f"set +e; {cmd}; rc=$?; printf '\n__EXIT_CODE__:%s\n' \"$rc\""
        out, err = self._run_sudo_bash(client, password, wrapped)
        exit_code = 0
        lines = []
        for line in (out or "").splitlines():
            if line.startswith("__EXIT_CODE__:"):
                try:
                    exit_code = int(line.split(":", 1)[1].strip())
                except Exception:
                    exit_code = 1
                continue
            lines.append(line)
        return "\n".join(lines).strip(), err, exit_code

    def _run_ros_cli_via_ssh(self, cmd: str):
        # ROSBridge 断开时，通过小脑 SSH 以“模拟终端”方式执行 ROS 命令
        # 这样更接近手工 ssh 后在 shell 里执行的行为
        full_cmd = f"source ~/.bashrc >/dev/null 2>&1; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; {cmd}"
        return self.ssh.run_interactive(full_cmd)

    def _run_ros_cli_via_ssh_with_exit_code(self, cmd: str):
        full_cmd = f"source ~/.bashrc >/dev/null 2>&1; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; {cmd}"
        return self._run_bash_with_exit_code(self.ssh, full_cmd)

    def _run_ros_cli_via_ssh_interactive(self, cmd: str):
        # 兼容旧调用，统一走模拟终端实现
        return self._run_ros_cli_via_ssh(cmd)

    def _build_remote_ros_echo_poller_command(self, mode: str, topics: dict, interval_sec: float = 0.08) -> str:
        cfg = {
            "mode": str(mode or "monitor").strip() or "monitor",
            "topics": dict(topics or {}),
            "interval": float(interval_sec),
        }
        cfg_text = json.dumps(cfg, ensure_ascii=False)
        script = (
            "import json, shlex, subprocess, sys, time\n"
            "cfg = json.loads(sys.argv[1])\n"
            "mode = str(cfg.get('mode') or 'monitor')\n"
            "topics = cfg.get('topics') or {}\n"
            "interval = float(cfg.get('interval') or 0.08)\n"
            "while True:\n"
            "    for side, topic in topics.items():\n"
            "        cmd = 'source ~/.bashrc >/dev/null 2>&1; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; rostopic echo -n 1 ' + shlex.quote(str(topic))\n"
            "        try:\n"
            "            p = subprocess.run(['bash', '-lc', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.5)\n"
            "            text = (p.stdout or '') + '\\n' + (p.stderr or '')\n"
            "            evt = {'kind': 'sample', 'mode': mode, 'side': str(side), 'topic': str(topic), 'text': text}\n"
            "        except Exception as e:\n"
            "            evt = {'kind': 'error', 'mode': mode, 'side': str(side), 'topic': str(topic), 'error': str(e)}\n"
            "        print(json.dumps(evt, ensure_ascii=False), flush=True)\n"
            "    time.sleep(interval if interval > 0.0 else 0.01)\n"
        )
        return f"python3 -u -c {shlex.quote(script)} {shlex.quote(cfg_text)}"

    def _emit_log_audit(self, text: str):
        self.log_audit_item_signal.emit(str(text or ""))

    def _parse_log_audit_keywords(self, text: str) -> list[str]:
        raw = str(text or "")
        parts = re.split(r"[,;\n\r\t]+", raw)
        values = []
        seen = set()
        for part in parts:
            item = part.strip()
            if not item:
                continue
            lowered = item.lower()
            if lowered in seen:
                continue
            seen.add(lowered)
            values.append(item)
        return values

    def _parse_log_audit_target_names(self, text: str) -> list[str]:
        raw = str(text or "")
        parts = re.split(r"[,;\n\r\t]+", raw)
        values = []
        seen = set()
        for part in parts:
            item = str(part or "").strip()
            if not item:
                continue
            lowered = item.lower()
            if lowered in seen:
                continue
            seen.add(lowered)
            values.append(item)
        return values

    def _parse_log_audit_datetime(self, text: str) -> datetime:
        raw = str(text or "").strip()
        if not raw:
            raise ValueError("时间不能为空")
        normalized = raw.replace("/", "-").replace("T", " ").strip()
        try:
            return datetime.fromisoformat(normalized)
        except Exception:
            pass
        for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d %H:%M:%S.%f"):
            try:
                return datetime.strptime(normalized, fmt)
            except Exception:
                continue
        raise ValueError(f"无法解析时间: {raw}，请使用 YYYY-MM-DD HH:MM:SS")

    def _parse_log_line_timestamp(self, text: str):
        line = str(text or "")
        patterns = [
            r"(\d{4}[-/]\d{2}[-/]\d{2}[ T]\d{2}:\d{2}:\d{2}(?:[\.,]\d{1,6})?)",
            r"(\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}(?:[\.,]\d{1,6})?)",
        ]
        current_year = datetime.now().year
        for pattern in patterns:
            match = re.search(pattern, line)
            if not match:
                continue
            raw = match.group(1).replace("/", "-").replace("T", " ").replace(",", ".")
            candidates = []
            if re.match(r"^\d{2}-\d{2} ", raw):
                candidates.append(f"{current_year}-{raw}")
            candidates.append(raw)
            for candidate in candidates:
                for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
                    try:
                        return datetime.strptime(candidate, fmt)
                    except Exception:
                        continue
        localized_match = re.search(r"(\d{1,2})月\s*(\d{1,2})\s+(\d{2}:\d{2}:\d{2})", line)
        if localized_match:
            month_text, day_text, time_text = localized_match.groups()
            candidate = f"{current_year}-{int(month_text):02d}-{int(day_text):02d} {time_text}"
            try:
                return datetime.strptime(candidate, "%Y-%m-%d %H:%M:%S")
            except Exception:
                pass
        return None

    def _normalize_log_audit_remote_path(self, log_dir: str, target_file: str) -> str:
        log_dir_value = self._normalize_remote_posix_path(log_dir or "/home/naviai/source/naviai_manip_retail/logs")
        target_value = str(target_file or "").strip()
        if not target_value:
            return ""
        if target_value.startswith("/"):
            return self._normalize_remote_posix_path(target_value)
        return self._normalize_remote_posix_path(f"{log_dir_value.rstrip('/')}/{target_value}")

    def _load_remote_log_records(self, client: SshSftpClient, remote_path: str):
        records = []
        last_timestamp = None
        with client.sftp.open(remote_path, "r") as handle:
            for raw_line in handle:
                if isinstance(raw_line, bytes):
                    line = raw_line.decode("utf-8", errors="ignore")
                else:
                    line = str(raw_line)
                line = line.rstrip("\r\n")
                timestamp = self._parse_log_line_timestamp(line)
                if timestamp is not None:
                    last_timestamp = timestamp
                records.append((last_timestamp, line))
        return records

    def _list_log_audit_remote_jsonl_files(self, client: SshSftpClient, log_dir: str, target_names: list[str]):
        base_dir = self._normalize_remote_posix_path(log_dir or "/home/naviai/source/naviai_manip_retail/logs")
        requested = {str(item).strip().lower() for item in (target_names or []) if str(item).strip()}
        requested_basenames = {os.path.basename(item).strip().lower() for item in requested}
        matched = []
        try:
            entries = client.sftp.listdir_attr(base_dir)
        except Exception:
            return []
        for entry in entries:
            name = str(getattr(entry, "filename", "") or "").strip()
            if not name or stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                continue
            if not name.lower().endswith(".jsonl"):
                continue
            full = f"{base_dir.rstrip('/')}/{name}" if base_dir != "/" else f"/{name}"
            if requested and name.lower() not in requested_basenames and full.lower() not in requested:
                continue
            matched.append(full)
        matched.sort()
        return matched

    def _load_remote_jsonl_entries(self, client: SshSftpClient, remote_path: str):
        items = []
        with client.sftp.open(remote_path, "r") as handle:
            for index, raw_line in enumerate(handle, start=1):
                if isinstance(raw_line, bytes):
                    line = raw_line.decode("utf-8", errors="ignore")
                else:
                    line = str(raw_line)
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                except Exception:
                    continue
                if not isinstance(payload, dict):
                    continue
                timestamp_text = str(payload.get("timestamp") or "").strip()
                stamp = self._parse_log_audit_datetime(timestamp_text) if timestamp_text else None
                items.append(
                    {
                        "timestamp": stamp,
                        "payload": payload,
                        "raw": line,
                        "source": remote_path,
                        "line_no": index,
                    }
                )
        return items

    def _failed_step_names_from_payload(self, payload: dict) -> list[str]:
        failed = []
        step_times = payload.get("step_times") if isinstance(payload, dict) else None
        if isinstance(step_times, dict):
            for step_name, step_info in step_times.items():
                if isinstance(step_info, dict) and step_info.get("success") is False:
                    failed.append(str(step_name))
        return failed

    def _match_log_audit_failure_filter(self, payload: dict, filters: list[str]) -> bool:
        if not filters:
            return True
        payload_text = json.dumps(payload or {}, ensure_ascii=False).lower()
        failed_steps = " ".join(self._failed_step_names_from_payload(payload)).lower()
        return any(str(item).lower() in payload_text or str(item).lower() in failed_steps for item in filters)

    def _render_log_audit_app_entries(self, entries, max_lines: int = 400) -> str:
        lines = []
        for item in (entries or []):
            payload = item.get("payload") if isinstance(item, dict) else None
            source = os.path.basename(str(item.get("source") or "")) if isinstance(item, dict) else ""
            if isinstance(payload, dict):
                lines.append(f"[{source}] {json.dumps(payload, ensure_ascii=False)}")
        return self._format_log_audit_lines(lines, max_lines=max_lines)

    def _save_log_audit_bundle(self, bundle_name: str, app_entries, middleware_text: str, ubuntu_text: str, summary_lines: list[str]):
        storage_root = self._get_storage_root()
        if self._instance_tag:
            bundle_root = os.path.join(storage_root, "logs", self._instance_tag, "log_audit_test", bundle_name)
        else:
            bundle_root = os.path.join(storage_root, "logs", "log_audit_test", bundle_name)
        os.makedirs(bundle_root, exist_ok=True)

        with open(os.path.join(bundle_root, "application_events.jsonl"), "w", encoding="utf-8") as f:
            for item in (app_entries or []):
                raw = str((item or {}).get("raw") or "").strip()
                if raw:
                    f.write(raw + "\n")

        with open(os.path.join(bundle_root, "middleware.log"), "w", encoding="utf-8") as f:
            f.write(str(middleware_text or ""))

        with open(os.path.join(bundle_root, "ubuntu_system.log"), "w", encoding="utf-8") as f:
            f.write(str(ubuntu_text or ""))

        with open(os.path.join(bundle_root, "summary.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join([str(item) for item in (summary_lines or [])]))

        return bundle_root

    def _is_embedded_archive_name(self, path: str) -> bool:
        low = str(path or "").strip().lower()
        return low.endswith((".tar.gz", ".tgz", ".tar", ".zip", ".gz"))

    def _parse_embedded_archive_start(self, dir_name: str, file_name: str):
        target_name = str(file_name or "").strip()
        if not target_name:
            return None

        fallback_date = None
        dir_text = str(dir_name or "").strip()
        dir_digits = re.sub(r"\D", "", dir_text)
        if len(dir_digits) >= 8:
            fallback_date = dir_digits[:8]

        stem = target_name
        lower_name = target_name.lower()
        for suffix in (".tar.gz", ".tgz", ".tar", ".zip", ".gz"):
            if lower_name.endswith(suffix):
                stem = target_name[: -len(suffix)]
                break

        stem_digits = re.sub(r"\D", "", stem)
        candidates = []
        if len(stem_digits) >= 14:
            candidates.append(stem_digits[:14])
        if fallback_date and len(stem_digits) >= 6:
            candidates.append(fallback_date + stem_digits[-6:])
        if fallback_date and len(stem_digits) >= 4:
            candidates.append(fallback_date + stem_digits[-4:] + "00")

        seen = set()
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            for fmt in ("%Y%m%d%H%M%S", "%Y%m%d%H%M"):
                try:
                    parsed = datetime.strptime(candidate[: len(fmt.replace('%', ''))], fmt)
                    return parsed
                except Exception:
                    continue
        return None

    def _find_log_audit_embedded_archive(self, client: SshSftpClient, event_time: datetime, root_dir: str = "/var/www/html/log/sdk"):
        if event_time is None:
            return None
        base_dir = self._normalize_remote_posix_path(root_dir or "/var/www/html/log/sdk")
        target_date_digits = event_time.strftime("%Y%m%d")
        date_dirs = []
        try:
            entries = client.sftp.listdir_attr(base_dir)
        except Exception:
            return None
        for entry in entries:
            name = str(getattr(entry, "filename", "") or "").strip()
            if not name or not stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                continue
            if re.sub(r"\D", "", name) == target_date_digits:
                date_dirs.append(name)

        if not date_dirs:
            return None

        candidates = []
        for dir_name in date_dirs:
            remote_dir = f"{base_dir.rstrip('/')}/{dir_name}" if base_dir != "/" else f"/{dir_name}"
            for entry in client.sftp.listdir_attr(remote_dir):
                file_name = str(getattr(entry, "filename", "") or "").strip()
                if not file_name or stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                    continue
                if not self._is_embedded_archive_name(file_name):
                    continue
                start_time = self._parse_embedded_archive_start(dir_name, file_name)
                remote_path = f"{remote_dir.rstrip('/')}/{file_name}" if remote_dir != "/" else f"/{file_name}"
                candidates.append(
                    {
                        "remote_path": remote_path,
                        "file_name": file_name,
                        "dir_name": dir_name,
                        "start_time": start_time,
                    }
                )

        if not candidates:
            return None

        with_start = [item for item in candidates if item.get("start_time") is not None]
        if with_start:
            earlier = [item for item in with_start if item["start_time"] <= event_time]
            if earlier:
                earlier.sort(key=lambda item: item["start_time"], reverse=True)
                selected = earlier[0]
                selected["match_rule"] = "latest_not_after_event"
                return selected
            with_start.sort(key=lambda item: item["start_time"])
            selected = with_start[0]
            selected["match_rule"] = "earliest_after_event"
            return selected

        candidates.sort(key=lambda item: item["file_name"])
        selected = candidates[0]
        selected["match_rule"] = "first_archive_without_time"
        return selected

    def _find_log_audit_embedded_archives_in_range(
        self,
        client: SshSftpClient,
        start_time: datetime,
        end_time: datetime,
        root_dir: str = "/var/www/html/log/sdk",
    ):
        if start_time is None or end_time is None:
            return []
        if start_time > end_time:
            start_time, end_time = end_time, start_time

        base_dir = self._normalize_remote_posix_path(root_dir or "/var/www/html/log/sdk")
        date_dirs = []
        cursor_date = start_time.date()
        end_date = end_time.date()
        expected_dates = set()
        while cursor_date <= end_date:
            expected_dates.add(cursor_date.strftime("%Y%m%d"))
            cursor_date += timedelta(days=1)

        try:
            entries = client.sftp.listdir_attr(base_dir)
        except Exception:
            return []
        for entry in entries:
            name = str(getattr(entry, "filename", "") or "").strip()
            if not name or not stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                continue
            if re.sub(r"\D", "", name) in expected_dates:
                date_dirs.append(name)

        candidates = []
        for dir_name in sorted(date_dirs):
            remote_dir = f"{base_dir.rstrip('/')}/{dir_name}" if base_dir != "/" else f"/{dir_name}"
            for entry in client.sftp.listdir_attr(remote_dir):
                file_name = str(getattr(entry, "filename", "") or "").strip()
                if not file_name or stat.S_ISDIR(getattr(entry, "st_mode", 0)):
                    continue
                if not self._is_embedded_archive_name(file_name):
                    continue
                start_value = self._parse_embedded_archive_start(dir_name, file_name)
                remote_path = f"{remote_dir.rstrip('/')}/{file_name}" if remote_dir != "/" else f"/{file_name}"
                item = {
                    "remote_path": remote_path,
                    "file_name": file_name,
                    "dir_name": dir_name,
                    "start_time": start_value,
                }
                if start_value is None or start_time <= start_value <= end_time:
                    candidates.append(item)

        seen = set()
        results = []
        for item in sorted(candidates, key=lambda value: (value.get("start_time") or datetime.min, value.get("file_name") or "")):
            key = item.get("remote_path")
            if not key or key in seen:
                continue
            seen.add(key)
            results.append(item)
        return results

    def _download_log_audit_embedded_archive(self, remote_path: str, bundle_root: str) -> str:
        if not remote_path:
            return ""
        os.makedirs(bundle_root, exist_ok=True)
        local_path = os.path.join(bundle_root, os.path.basename(remote_path.rstrip("/")) or "embedded_log.tar.gz")
        self.ssh.download(remote_path, local_path)
        return local_path

    def _download_log_audit_embedded_archives(self, remote_paths, bundle_root: str):
        local_paths = []
        for remote_path in (remote_paths or []):
            target = str(remote_path or "").strip()
            if not target:
                continue
            local_paths.append(self._download_log_audit_embedded_archive(target, bundle_root))
        return local_paths

    def _download_log_audit_embedded_fallback_dir(
        self,
        bundle_root: str,
        remote_dir: str = "/home/nav01/CodeFiles/sdk/zjhrobotsdkreleaseproject/LogFile",
    ) -> str:
        if not (self.ssh and self.ssh.sftp):
            raise RuntimeError("请先连接 SSH(小脑)")
        target_remote_dir = self._normalize_remote_posix_path(remote_dir)
        self.ssh.sftp.stat(target_remote_dir)
        local_dir = os.path.join(bundle_root, os.path.basename(target_remote_dir.rstrip("/")) or "LogFile")
        self.ssh.download_dir(target_remote_dir, local_dir)
        return local_dir

    def _filter_log_records_by_range(self, records, start_time: datetime, end_time: datetime):
        return [line for stamp, line in records if stamp is not None and start_time <= stamp <= end_time]

    def _format_log_audit_lines(self, lines, max_lines: int = 400) -> str:
        values = [str(line) for line in (lines or [])]
        if not values:
            return "(无匹配日志)"
        if len(values) > max_lines:
            values = values[:max_lines] + [f"... 已截断，其余 {len(values) - max_lines} 行未显示"]
        return "\n".join(values)

    def _format_log_audit_text(self, text: str, max_lines: int = 400, empty_text: str = "(无匹配日志)") -> str:
        raw = str(text or "").strip()
        if not raw:
            return empty_text
        return self._format_log_audit_lines(raw.splitlines(), max_lines=max_lines)

    def _normalize_log_audit_journal_cmd(self, cmd: str, default_cmd: str) -> str:
        value = str(cmd or default_cmd or "").strip()
        if not value:
            value = str(default_cmd or "").strip()
        if not value:
            return ""
        lowered = value.lower()
        if "journalctl" in lowered and "--no-pager" not in lowered:
            value = f"{value} --no-pager"
        return value

    def _run_log_audit_small_brain_log_range(self, base_cmd: str, default_cmd: str, start_time: datetime, end_time: datetime, preview_lines: int | None = None) -> str:
        if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
            raise RuntimeError("请先连接 SSH(小脑)")
        cmd = self._normalize_log_audit_journal_cmd(base_cmd, default_cmd)
        cmd = (
            f"{cmd} --since {shlex.quote(start_time.strftime('%Y-%m-%d %H:%M:%S'))} "
            f"--until {shlex.quote(end_time.strftime('%Y-%m-%d %H:%M:%S'))}"
        )
        if preview_lines is not None and int(preview_lines) > 0:
            cmd = f"{cmd} -n {int(preview_lines)}"
        out, err = self.ssh.run_interactive(cmd)
        text = ((out or "") + ("\n" + err if err else "")).strip()
        return text or "(无中间件日志输出)"

    def _run_log_audit_middleware_range(self, base_cmd: str, start_time: datetime, end_time: datetime, preview_lines: int | None = None) -> str:
        return self._run_log_audit_small_brain_log_range(
            base_cmd,
            "journalctl -u zj_humanoid.service",
            start_time,
            end_time,
            preview_lines=preview_lines,
        )

    def _run_log_audit_ubuntu_range(self, base_cmd: str, start_time: datetime, end_time: datetime, preview_lines: int | None = None) -> str:
        text = self._run_log_audit_small_brain_log_range(
            base_cmd,
            "journalctl",
            start_time,
            end_time,
            preview_lines=preview_lines,
        )
        return text if text != "(无中间件日志输出)" else "(无Ubuntu系统日志输出)"

    def browse_log_audit_app_file(self):
        if not self._ensure_ssh_big():
            return
        start_dir = self.log_audit_app_logs_dir_edit.text().strip() if hasattr(self, "log_audit_app_logs_dir_edit") else "/home/naviai/source/naviai_manip_retail/logs"
        selected = self._browse_sftp_path_dialog(
            self.ssh_big,
            "大脑",
            "选择应用日志文件",
            start_path=start_dir or "/home/naviai/source/naviai_manip_retail/logs",
            select_kind="file",
        )
        if not selected:
            return
        self.log_audit_app_logs_dir_edit.setText(os.path.dirname(selected) or "/")
        selected_name = os.path.basename(selected) or selected
        existing = self._parse_log_audit_target_names(self.log_audit_target_file_edit.text() if hasattr(self, "log_audit_target_file_edit") else "")
        if selected_name not in existing:
            existing.append(selected_name)
        self.log_audit_target_file_edit.setText(",".join(existing) if existing else selected_name)

    def preview_log_audit_middleware(self):
        preview_text = self.log_audit_recent_lines_edit.text().strip() if hasattr(self, "log_audit_recent_lines_edit") else "200"
        try:
            preview_lines = max(1, int(preview_text or "200"))
        except Exception:
            QMessageBox.warning(self, "提示", "预览行数必须是正整数")
            return
        base_cmd = self.log_audit_journal_cmd_edit.text().strip() if hasattr(self, "log_audit_journal_cmd_edit") else ""

        def worker():
            try:
                self.log_audit_clear_signal.emit()
                self._emit_log_audit("[INFO] 开始读取小脑中间件最新日志")
                if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
                    raise RuntimeError("请先连接 SSH(小脑)")
                cmd = self._normalize_log_audit_journal_cmd(base_cmd, "journalctl -u zj_humanoid.service")
                cmd = f"{cmd} -n {preview_lines}"
                self._emit_log_audit(f"[REQ] {cmd}")
                out, err = self.ssh.run_interactive(cmd)
                text = ((out or "") + ("\n" + err if err else "")).strip()
                self._emit_log_audit(text or "(无中间件日志输出)")
            except Exception as e:
                self._emit_log_audit(f"[ERR] 读取中间件日志失败: {e}")

        self._run_async(worker)

    def preview_log_audit_ubuntu(self):
        preview_text = self.log_audit_recent_lines_edit.text().strip() if hasattr(self, "log_audit_recent_lines_edit") else "200"
        try:
            preview_lines = max(1, int(preview_text or "200"))
        except Exception:
            QMessageBox.warning(self, "提示", "预览行数必须是正整数")
            return
        base_cmd = self.log_audit_ubuntu_cmd_edit.text().strip() if hasattr(self, "log_audit_ubuntu_cmd_edit") else ""

        def worker():
            try:
                self.log_audit_clear_signal.emit()
                self._emit_log_audit("[INFO] 开始读取小脑 Ubuntu 系统最新日志")
                if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
                    raise RuntimeError("请先连接 SSH(小脑)")
                cmd = self._normalize_log_audit_journal_cmd(base_cmd, "journalctl")
                cmd = f"{cmd} -n {preview_lines}"
                self._emit_log_audit(f"[REQ] {cmd}")
                out, err = self.ssh.run_interactive(cmd)
                text = ((out or "") + ("\n" + err if err else "")).strip()
                self._emit_log_audit(text or "(无Ubuntu系统日志输出)")
            except Exception as e:
                self._emit_log_audit(f"[ERR] 读取Ubuntu系统日志失败: {e}")

        self._run_async(worker)

    def run_log_audit_test(self):
        mode = self.log_audit_mode_combo.currentData() if hasattr(self, "log_audit_mode_combo") else "error"
        mode_text = self.log_audit_mode_combo.currentText() if hasattr(self, "log_audit_mode_combo") else "错误驱动式"
        app_root = self.log_audit_app_root_edit.text().strip() if hasattr(self, "log_audit_app_root_edit") else "/home/naviai/source/naviai_manip_retail"
        app_log_dir = self.log_audit_app_logs_dir_edit.text().strip() if hasattr(self, "log_audit_app_logs_dir_edit") else "/home/naviai/source/naviai_manip_retail/logs"
        target_file = self.log_audit_target_file_edit.text().strip() if hasattr(self, "log_audit_target_file_edit") else "pick_log_detail.jsonl,place_log_detail.jsonl"
        keyword_text = self.log_audit_keywords_edit.text().strip() if hasattr(self, "log_audit_keywords_edit") else ""
        base_cmd = self.log_audit_journal_cmd_edit.text().strip() if hasattr(self, "log_audit_journal_cmd_edit") else ""
        ubuntu_cmd = self.log_audit_ubuntu_cmd_edit.text().strip() if hasattr(self, "log_audit_ubuntu_cmd_edit") else ""
        preview_text = self.log_audit_recent_lines_edit.text().strip() if hasattr(self, "log_audit_recent_lines_edit") else "200"
        start_text = self.log_audit_start_time_edit.text().strip() if hasattr(self, "log_audit_start_time_edit") else ""
        end_text = self.log_audit_end_time_edit.text().strip() if hasattr(self, "log_audit_end_time_edit") else ""
        ref_text = self.log_audit_ref_time_edit.text().strip() if hasattr(self, "log_audit_ref_time_edit") else ""
        window_text = self.log_audit_window_minutes_edit.text().strip() if hasattr(self, "log_audit_window_minutes_edit") else "10"

        target_names = self._parse_log_audit_target_names(target_file)
        keywords = self._parse_log_audit_keywords(keyword_text)
        try:
            preview_lines = max(50, int(preview_text or "200"))
        except Exception:
            QMessageBox.warning(self, "提示", "预览行数必须是整数")
            return
        try:
            window_minutes = max(1, int(window_text or "10"))
        except Exception:
            QMessageBox.warning(self, "提示", "窗口分钟必须是正整数")
            return

        try:
            if mode == "range":
                range_start = self._parse_log_audit_datetime(start_text)
                range_end = self._parse_log_audit_datetime(end_text)
                if range_start > range_end:
                    raise ValueError("开始时间不能晚于结束时间")
            elif mode == "near":
                ref_time = self._parse_log_audit_datetime(ref_text)
                range_start = ref_time - timedelta(minutes=window_minutes)
                range_end = ref_time + timedelta(minutes=window_minutes)
            else:
                range_start = None
                range_end = None
        except Exception as e:
            QMessageBox.warning(self, "提示", str(e))
            return

        def worker():
            try:
                self.log_audit_clear_signal.emit()
                self._emit_log_audit(f"[INFO] 日志排查test启动: 模式={mode_text}")
                self._emit_log_audit(f"[INFO] 目标jsonl: {', '.join(target_names) if target_names else '默认jsonl'}")

                remote_files = []
                ssh_big_ready = bool(self.ssh_big and self.ssh_big.ssh and self.ssh_big.sftp)
                if mode == "error" and not ssh_big_ready:
                    raise RuntimeError("错误驱动式日志排查请先连接 SSH(大脑)")
                if ssh_big_ready:
                    try:
                        self.ssh_big.sftp.stat(app_log_dir)
                        remote_files = self._list_log_audit_remote_jsonl_files(self.ssh_big, app_log_dir, target_names)
                    except Exception:
                        self._emit_log_audit("[WARN] 应用日志目录不存在或不可访问，将继续提取其他日志")
                elif mode != "error":
                    self._emit_log_audit("[INFO] 当前不是错误驱动式，未连接 SSH(大脑)，跳过应用层 jsonl 提取")

                if remote_files:
                    self._emit_log_audit(f"[OK] 应用层匹配jsonl文件 {len(remote_files)} 个")
                else:
                    self._emit_log_audit("[WARN] 应用层未找到匹配的 jsonl 文件，将继续提取其他日志")

                app_entries = []
                for remote_file in remote_files:
                    loaded = self._load_remote_jsonl_entries(self.ssh_big, remote_file)
                    app_entries.extend(loaded)
                    self._emit_log_audit(f"[OK] 已加载 {os.path.basename(remote_file)}: {len(loaded)} 条")

                app_entries = [item for item in app_entries if item.get("timestamp") is not None]
                app_entries.sort(key=lambda item: item.get("timestamp"))
                self._emit_log_audit(f"[OK] 应用层有效事件总数: {len(app_entries)}")

                if mode == "error":
                    matched = []
                    for item in app_entries:
                        payload = item.get("payload") or {}
                        stamp = item.get("timestamp")
                        if stamp is None:
                            continue
                        if payload.get("success") is not False:
                            continue
                        if not self._match_log_audit_failure_filter(payload, keywords):
                            continue
                        matched.append(item)
                    if not matched:
                        self._emit_log_audit("[WARN] 应用层 jsonl 中未发现匹配的失败事件")
                        return

                    total_matches = len(matched)
                    now_time = datetime.now()
                    shown_matches = sorted(
                        matched,
                        key=lambda item: abs((item.get("timestamp") - now_time).total_seconds()) if item.get("timestamp") else float("inf"),
                    )[:20]
                    shown_matches.sort(key=lambda item: item.get("timestamp") or datetime.min)
                    self._emit_log_audit(
                        f"[OK] 命中失败事件 {total_matches} 条，当前按距离现在最近的 {len(shown_matches)} 条进行处理"
                    )
                    for index, item in enumerate(shown_matches, start=1):
                        stamp = item.get("timestamp")
                        payload = item.get("payload") or {}
                        source_name = os.path.basename(str(item.get("source") or ""))
                        section_start = stamp - timedelta(minutes=window_minutes)
                        section_end = stamp + timedelta(minutes=window_minutes)
                        window_entries = [entry for entry in app_entries if entry.get("timestamp") and section_start <= entry.get("timestamp") <= section_end]
                        middleware_text = self._run_log_audit_middleware_range(base_cmd, section_start, section_end, preview_lines=None)
                        ubuntu_text = self._run_log_audit_ubuntu_range(ubuntu_cmd, section_start, section_end, preview_lines=None)
                        middleware_preview = self._format_log_audit_text(middleware_text, max_lines=preview_lines, empty_text="(无中间件日志输出)")
                        ubuntu_preview = self._format_log_audit_text(ubuntu_text, max_lines=preview_lines, empty_text="(无Ubuntu系统日志输出)")
                        embedded_match = self._find_log_audit_embedded_archive(self.ssh, stamp)
                        failed_steps = self._failed_step_names_from_payload(payload)
                        bundle_name = f"error_{index:02d}_{stamp.strftime('%Y%m%d_%H%M%S')}"
                        summary_lines = [
                            f"mode=error",
                            f"source={source_name}",
                            f"timestamp={stamp.strftime('%Y-%m-%d %H:%M:%S')}",
                            f"failed_steps={','.join(failed_steps) if failed_steps else '-'}",
                            f"window_start={section_start.strftime('%Y-%m-%d %H:%M:%S')}",
                            f"window_end={section_end.strftime('%Y-%m-%d %H:%M:%S')}",
                            f"embedded_archive={embedded_match.get('remote_path') if embedded_match else '-'}",
                            f"embedded_fallback_dir=/home/nav01/CodeFiles/sdk/zjhrobotsdkreleaseproject/LogFile",
                            f"event={json.dumps(payload, ensure_ascii=False)}",
                        ]
                        saved_dir = self._save_log_audit_bundle(bundle_name, window_entries, middleware_text, ubuntu_text, summary_lines)
                        embedded_local = ""
                        embedded_desc = "(未匹配到嵌入式压缩日志)"
                        if embedded_match:
                            try:
                                embedded_local = self._download_log_audit_embedded_archive(embedded_match.get("remote_path"), saved_dir)
                                start_value = embedded_match.get("start_time")
                                start_text = start_value.strftime('%Y-%m-%d %H:%M:%S') if start_value else "-"
                                embedded_desc = (
                                    f"{embedded_match.get('remote_path')} -> {embedded_local} "
                                    f"(起始时间={start_text}, 规则={embedded_match.get('match_rule')})"
                                )
                            except Exception as embedded_error:
                                embedded_desc = f"{embedded_match.get('remote_path')} (下载失败: {embedded_error})"
                        else:
                            try:
                                fallback_local_dir = self._download_log_audit_embedded_fallback_dir(saved_dir)
                                embedded_desc = f"未匹配到压缩包，已回退提取目录 -> {fallback_local_dir}"
                            except Exception as embedded_error:
                                embedded_desc = f"未匹配到压缩包，且回退目录提取失败: {embedded_error}"
                        self._emit_log_audit(
                            f"\n===== 错误#{index} =====\n"
                            f"来源文件: {source_name}\n"
                            f"命中时间: {stamp.strftime('%Y-%m-%d %H:%M:%S')}\n"
                            f"失败步骤: {', '.join(failed_steps) if failed_steps else '-'}\n"
                            f"失败事件: {json.dumps(payload, ensure_ascii=False)}\n"
                            f"时间窗: {section_start.strftime('%Y-%m-%d %H:%M:%S')} ~ {section_end.strftime('%Y-%m-%d %H:%M:%S')}\n"
                            f"保存目录: {saved_dir}\n"
                            f"嵌入式日志: {embedded_desc}\n"
                            f"\n--- 应用层事件 ---\n{self._render_log_audit_app_entries(window_entries, max_lines=preview_lines)}\n"
                            f"\n--- 中间件日志 ---\n{middleware_preview}\n"
                            f"\n--- Ubuntu系统日志 ---\n{ubuntu_preview}"
                        )
                    return

                window_entries = [entry for entry in app_entries if entry.get("timestamp") and range_start <= entry.get("timestamp") <= range_end]
                middleware_text = self._run_log_audit_middleware_range(base_cmd, range_start, range_end, preview_lines=None)
                ubuntu_text = self._run_log_audit_ubuntu_range(ubuntu_cmd, range_start, range_end, preview_lines=None)
                middleware_preview = self._format_log_audit_text(middleware_text, max_lines=preview_lines, empty_text="(无中间件日志输出)")
                ubuntu_preview = self._format_log_audit_text(ubuntu_text, max_lines=preview_lines, empty_text="(无Ubuntu系统日志输出)")
                embedded_matches = self._find_log_audit_embedded_archives_in_range(self.ssh, range_start, range_end)
                title = "指定时段" if mode == "range" else "就近时间"
                bundle_name = f"{mode}_{range_start.strftime('%Y%m%d_%H%M%S')}_{range_end.strftime('%Y%m%d_%H%M%S')}"
                summary_lines = [
                    f"mode={mode}",
                    f"range_start={range_start.strftime('%Y-%m-%d %H:%M:%S')}",
                    f"range_end={range_end.strftime('%Y-%m-%d %H:%M:%S')}",
                    f"embedded_archive_count={len(embedded_matches)}",
                    f"embedded_archives={';'.join([item.get('remote_path') or '' for item in embedded_matches]) if embedded_matches else '-'}",
                    f"embedded_fallback_dir=/home/nav01/CodeFiles/sdk/zjhrobotsdkreleaseproject/LogFile",
                    f"target_jsonl={','.join(target_names) if target_names else 'all'}",
                ]
                saved_dir = self._save_log_audit_bundle(bundle_name, window_entries, middleware_text, ubuntu_text, summary_lines)
                embedded_desc = "(未匹配到嵌入式压缩日志，注意该时段可能对应 0 个或多个压缩包)"
                if embedded_matches:
                    try:
                        embedded_local_paths = self._download_log_audit_embedded_archives(
                            [item.get("remote_path") for item in embedded_matches],
                            saved_dir,
                        )
                        embedded_lines = []
                        for item, local_path in zip(embedded_matches, embedded_local_paths):
                            start_value = item.get("start_time")
                            start_text = start_value.strftime('%Y-%m-%d %H:%M:%S') if start_value else "-"
                            embedded_lines.append(
                                f"{item.get('remote_path')} -> {local_path} (起始时间={start_text})"
                            )
                        embedded_desc = "\n".join(embedded_lines)
                    except Exception as embedded_error:
                        embedded_desc = f"部分嵌入式压缩日志下载失败: {embedded_error}"
                else:
                    try:
                        fallback_local_dir = self._download_log_audit_embedded_fallback_dir(saved_dir)
                        embedded_desc = f"未匹配到压缩包，已回退提取目录 -> {fallback_local_dir}"
                    except Exception as embedded_error:
                        embedded_desc = f"未匹配到压缩包，且回退目录提取失败: {embedded_error}"
                self._emit_log_audit(
                    f"\n===== {title}日志排查 =====\n"
                    f"范围: {range_start.strftime('%Y-%m-%d %H:%M:%S')} ~ {range_end.strftime('%Y-%m-%d %H:%M:%S')}\n"
                    f"保存目录: {saved_dir}\n"
                    f"应用层jsonl: {'已命中事件' if remote_files else '未找到jsonl，已继续提取其他日志'}\n"
                    f"嵌入式日志: {embedded_desc}\n"
                    f"\n--- 应用层事件 ---\n{self._render_log_audit_app_entries(window_entries, max_lines=preview_lines)}\n"
                    f"\n--- 中间件日志 ---\n{middleware_preview}\n"
                    f"\n--- Ubuntu系统日志 ---\n{ubuntu_preview}"
                )
            except Exception as e:
                self._emit_log_audit(f"[ERR] 日志排查test失败: {e}")

        self._run_async(worker)

    def _parse_ros_cli_yaml(self, text: str):
        raw = (text or "").strip()
        if not raw:
            return None
        try:
            for doc in yaml.safe_load_all(raw):
                if isinstance(doc, dict):
                    return doc
        except Exception:
            pass

        try:
            cleaned_lines = []
            for ln in raw.splitlines():
                s = ln.strip()
                if not s:
                    continue
                if s.startswith("WARNING:") or s.startswith("Usage:") or s.startswith("rostopic:"):
                    continue
                if re.match(r"^[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+:.*[$#]\s?", s):
                    continue
                if s.startswith("rosservice call ") or s.startswith("rostopic echo "):
                    continue
                if s in ("---", "..."):
                    continue
                cleaned_lines.append(ln)
            cleaned = "\n".join(cleaned_lines).strip()
            if cleaned:
                obj = yaml.safe_load(cleaned)
                if isinstance(obj, dict):
                    return obj
        except Exception:
            pass
        return None

    def _extract_text_field(self, text: str, key: str):
        m = re.search(rf"(^|\\n)\\s*{re.escape(key)}\\s*:\\s*([^\\n]+)", text or "")
        if not m:
            return None
        return m.group(2).strip().strip("\"'")

    def _extract_float_list_field(self, text: str, key: str):
        m = re.search(rf"(^|\\n)\\s*{re.escape(key)}\\s*:\\s*\[([^\]]*)\]", text or "")
        vals = []
        if m:
            for item in m.group(2).split(","):
                s = item.strip().strip("\"'")
                if not s:
                    continue
                try:
                    vals.append(float(s))
                except Exception:
                    continue
            return vals

        m2 = re.search(rf"(^|\\n)\\s*{re.escape(key)}\\s*:\\s*((?:\\n\\s*-\\s*[^\\n]+)+)", text or "")
        if not m2:
            return []
        for ln in m2.group(2).splitlines():
            s = ln.strip()
            if not s.startswith("-"):
                continue
            item = s[1:].strip().strip("\"'")
            if not item:
                continue
            try:
                vals.append(float(item))
            except Exception:
                continue
        return vals

    def _extract_string_list_field(self, text: str, key: str):
        m = re.search(rf"(^|\\n)\\s*{re.escape(key)}\\s*:\\s*\[([^\]]*)\]", text or "")
        vals = []
        if m:
            for item in m.group(2).split(","):
                s = item.strip().strip("\"'")
                if s:
                    vals.append(s)
            return vals

        m2 = re.search(rf"(^|\\n)\\s*{re.escape(key)}\\s*:\\s*((?:\\n\\s*-\\s*[^\\n]+)+)", text or "")
        if not m2:
            return []
        for ln in m2.group(2).splitlines():
            s = ln.strip()
            if not s.startswith("-"):
                continue
            item = s[1:].strip().strip("\"'")
            if item:
                vals.append(item)
        return vals

    def _local_jetpack_script(self) -> str:
        candidates = []

        # PyInstaller 运行态（AppImage/DEB）优先从打包资源目录查找
        if getattr(sys, "frozen", False):
            meipass = getattr(sys, "_MEIPASS", "")
            if meipass:
                candidates.append(os.path.join(meipass, "check", "jetpack_check.sh"))
            candidates.append(os.path.join(os.path.dirname(sys.executable), "check", "jetpack_check.sh"))

        # 开发态：从项目目录查找
        candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "check", "jetpack_check.sh")))

        for p in candidates:
            if p and os.path.isfile(p):
                return p

        # 兜底返回首选路径，便于日志定位
        return candidates[0] if candidates else ""

    def _inline_jetpack_check_cmd(self) -> str:
        return (
            "map_jetpack(){ "
            "case \"$1\" in "
            "36.4.4) echo 'JetPack 6.2.1' ;; "
            "36.4.3) echo 'JetPack 6.2' ;; "
            "36.4) echo 'JetPack 6.1' ;; "
            "36.3) echo 'JetPack 6.0' ;; "
            "36.2) echo 'JetPack 6.0 DP' ;; "
            "35.6.2|35.6.1) echo 'JetPack 5.1.5' ;; "
            "35.6.0) echo 'JetPack 5.1.4' ;; "
            "35.5.0) echo 'JetPack 5.1.3' ;; "
            "35.4.1) echo 'JetPack 5.1.2' ;; "
            "35.3.1) echo 'JetPack 5.1.1' ;; "
            "35.2.1) echo 'JetPack 5.1' ;; "
            "35.1) echo 'JetPack 5.0.2' ;; "
            "34.1.1) echo 'JetPack 5.0.1 DP' ;; "
            "34.1) echo 'JetPack 5.0 DP' ;; "
            "*) echo 'Unknown' ;; "
            "esac; "
            "}; "
            "BOOT_L4T_RAW=$(dpkg-query --showformat='${Version}\\n' --show nvidia-l4t-core 2>/dev/null | cut -d- -f1); "
            "ROOTFS_L4T_RAW=$(grep -oE 'R[0-9]+ \\(release\\), REVISION: [0-9.]+' /etc/nv_tegra_release 2>/dev/null | sed -E 's/R([0-9]+) \\(release\\), REVISION: ([0-9.]+)/\\1.\\2/' | head -n 1); "
            "[ -n \"$BOOT_L4T_RAW\" ] || BOOT_L4T_RAW='Unknown'; "
            "[ -n \"$ROOTFS_L4T_RAW\" ] || ROOTFS_L4T_RAW='Unknown'; "
            "BOOT_JETPACK=$(map_jetpack \"$BOOT_L4T_RAW\"); "
            "ROOTFS_JETPACK=$(map_jetpack \"$ROOTFS_L4T_RAW\"); "
            "printf '=== Jetson L4T / JetPack Consistency Check ===\\n'; "
            "printf 'Bootloader L4T    : %s\\n' \"$BOOT_L4T_RAW\"; "
            "printf 'Bootloader JetPack : %s\\n' \"$BOOT_JETPACK\"; "
            "printf 'Rootfs L4T        : %s\\n' \"$ROOTFS_L4T_RAW\"; "
            "printf 'Rootfs JetPack     : %s\\n' \"$ROOTFS_JETPACK\""
        )

    def _run_rosservice_in_perception_container(self, service_name: str, req_text: str):
        compose_service = self.nav_compose_service.text().strip() if hasattr(self, "nav_compose_service") else "wa_perception"
        if not compose_service:
            compose_service = "wa_perception"

        inner = f"source /opt/ros/noetic/setup.bash && rosservice call {service_name} {shlex.quote(req_text)}"
        cmd = f"cd ~/navi_project && docker-compose exec -T {shlex.quote(compose_service)} bash -lc {shlex.quote(inner)}"
        return self._run_bash(self.ssh_big, cmd)

    def _ensure_ros(self) -> bool:
        if not self.ros or not self.ros.check_connection():
            self.log_signal.emit("[ERR] 请先连接 ROSBridge")
            return False
        return True

    def start_mapping(self):
        if not self._ensure_ssh_big():
            return

        map_name = self.nav_map_name.text().strip() if hasattr(self, "nav_map_name") else "kaiao3"
        z_floor = self.nav_z_floor.text().strip() if hasattr(self, "nav_z_floor") else "0.1"
        z_ceil = self.nav_z_ceil.text().strip() if hasattr(self, "nav_z_ceil") else "2.0"
        scene = self.nav_scene.text().strip() if hasattr(self, "nav_scene") else "0"

        req = f"{{map_name: '{map_name}', z_floor: {z_floor}, z_ceil: {z_ceil}, scene: {scene}}}"

        def worker():
            try:
                out, err = self._run_rosservice_in_perception_container("/perception/mapping_service", req)
                msg = (out.strip() or err.strip() or "已下发建图请求")
                self.log_signal.emit(f"[OK] 开始建图: {msg}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 开始建图失败: {e}")

        self._run_async(worker)

    def post_process_mapping(self):
        if not self._ensure_ssh_big():
            return

        method = self.nav_post_method.text().strip() if hasattr(self, "nav_post_method") else "0"
        req = f"{{method: {method}}}"

        def worker():
            try:
                out, err = self._run_rosservice_in_perception_container("/perception/post_processing", req)
                msg = (out.strip() or err.strip() or "已下发后处理请求")
                self.log_signal.emit(f"[OK] 建图后处理: {msg}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 建图后处理失败: {e}")

        self._run_async(worker)

    def restart_perception_container(self):
        if not self._ensure_ssh_big():
            return

        def worker():
            try:
                cmd = "cd ~/navi_project && ./start_robot.sh wa2 restart wa_perception"
                out, err = self._run_bash(self.ssh_big, cmd)
                msg = (out.strip() or err.strip() or "已下发重启感知容器指令")
                self.log_signal.emit(f"[OK] 重启感知容器: {msg}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 重启感知容器失败: {e}")

        self._run_async(worker)

    def _current_teach_arm_type(self) -> int:
        data = self.joint_ctrl_part.currentData()
        try:
            return int(data)
        except Exception:
            text = self.joint_ctrl_part.currentText().strip()
            # 示教模式全身统一映射到 15，升降仍通过 16 单独控制
            full_body_type = 15
            mapping = {
                "左臂": 1,
                "右臂": 2,
                "双臂": 3,
                "脖子": 4,
                "腰部": 8,
                "升降": 16,
                "全身": full_body_type,
            }
            return mapping.get(text, 1)

    def _teach_service_candidates(self, action: str):
        return [
            f"/zj_humanoid/upperlimb/teach_mode/{action}",
        ]

    def _joint_names_for_arm_type(self, arm_type: int):
        if str(getattr(self, "robot_model", "WA2")).upper() == "WA1":
            if arm_type == 1:
                return [
                    "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                    "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                ]
            if arm_type == 2:
                return [
                    "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                    "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                ]
            if arm_type == 3:
                return [
                    "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                    "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                    "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                    "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                ]
            if arm_type == 4:
                return ["Neck_Z", "Neck_Y"]
            if arm_type == 8:
                return ["Waist_Z", "Waist_Y"]
            if arm_type == 16:
                return ["Lifting_Z"]
            if arm_type in (15, 31):
                return self._wa1_full_body_names()
            return []
        if str(getattr(self, "robot_model", "WA2")).upper() == "I2":
            if arm_type == 1:
                return [
                    "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                    "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                ]
            if arm_type == 2:
                return [
                    "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                    "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                ]
            if arm_type == 3:
                return [
                    "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                    "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                    "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                    "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
                ]
            if arm_type == 4:
                return ["Neck_Z", "Neck_Y"]
            if arm_type == 8:
                return ["A_Waist"]
            if arm_type == 15:
                return self._i2_full_body_names()
            return []
        if arm_type == 1:
            return [
                "Chest_Z_L", "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
            ]
        if arm_type == 2:
            return [
                "Chest_Z_R", "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
            ]
        if arm_type == 3:
            return [
                "Chest_Z_L", "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
                "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
                "Chest_Z_R", "Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
                "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
            ]
        if arm_type == 4:
            return ["Neck_Z", "Neck_Y"]
        if arm_type == 8:
            return ["Pitch_Y_B", "Pitch_Y_M", "Waist_Z", "Waist_Y"]
        if arm_type == 15:
            return list(self.joint_names)
        return []

    def _arm_type_text(self, arm_type: int) -> str:
        mapping = {1: "左臂", 2: "右臂", 3: "双臂", 4: "脖子", 8: "腰部", 15: "全身", 16: "升降", 31: "全身"}
        return mapping.get(int(arm_type), f"arm_type={arm_type}")

    def _home_service_for_part(self, arm_type: int) -> str:
        # 需求：双臂归位用 go_down/dual_arm，脖子/腰用 go_home
        if arm_type in (1, 2, 3):
            return "/zj_humanoid/upperlimb/go_down/dual_arm"
        if arm_type == 4:
            return "/zj_humanoid/upperlimb/go_home/neck"
        if arm_type == 8:
            return "/zj_humanoid/upperlimb/go_home/waist"
        return "/zj_humanoid/upperlimb/go_down/dual_arm"

    def _movej_service_for_arm_type(self, arm_type: int) -> str:
        mapping = {
            1: "/zj_humanoid/upperlimb/movej_by_path/left_arm",
            2: "/zj_humanoid/upperlimb/movej_by_path/right_arm",
            4: "/zj_humanoid/upperlimb/movej_by_path/neck",
            8: "/zj_humanoid/upperlimb/movej_by_path/waist",
            16: "/zj_humanoid/upperlimb/movej_by_path/lifting",
            3: "/zj_humanoid/upperlimb/movej_by_path/dual_arm",
            15: "/zj_humanoid/upperlimb/movej_by_path/whole_body",
            31: "/zj_humanoid/upperlimb/movej_by_path/whole_body",
        }
        try:
            return mapping.get(int(arm_type))
        except Exception:
            return None

    def _movej_single_service_for_arm_type(self, arm_type: int) -> str:
        mapping = {
            1: "/zj_humanoid/upperlimb/movej/left_arm",
            2: "/zj_humanoid/upperlimb/movej/right_arm",
            3: "/zj_humanoid/upperlimb/movej/dual_arm",
            4: "/zj_humanoid/upperlimb/movej/neck",
            8: "/zj_humanoid/upperlimb/movej/waist",
            16: "/zj_humanoid/upperlimb/movej/lifting",
            15: "/zj_humanoid/upperlimb/movej/whole_body",
            31: "/zj_humanoid/upperlimb/movej/whole_body",
        }
        try:
            return mapping.get(int(arm_type))
        except Exception:
            return None

    def _motion_state_file_path_for_model(self, model: str) -> str:
        project_root = self._get_storage_root()
        state_dir = os.path.join(project_root, ".runtime")
        os.makedirs(state_dir, exist_ok=True)
        model_tag = self._motion_sequence_model_tag(model)
        if self._instance_tag:
            return os.path.join(state_dir, f"motion_sequences_{model_tag}_{self._instance_tag}.json")
        return os.path.join(state_dir, f"motion_sequences_{model_tag}.json")

    def _motion_sequence_model_tag(self, model: str = None) -> str:
        model_up = str(model or getattr(self, "robot_model", "WA2") or "WA2").upper()
        if model_up == "WA2":
            return "WA2"
        if model_up == "WA2_LS":
            return "WA2_LS"
        if model_up == "WA1":
            return "WA1"
        if model_up == "I2":
            return "I2"
        return "WA2"

    def _legacy_motion_state_file_path_for_model(self, model: str) -> str:
        project_root = self._get_storage_root()
        state_dir = os.path.join(project_root, ".runtime")
        os.makedirs(state_dir, exist_ok=True)
        model_tag = str(model or "WA2").upper()
        if self._instance_tag:
            return os.path.join(state_dir, f"motion_sequences_{model_tag}_{self._instance_tag}.json")
        return os.path.join(state_dir, f"motion_sequences_{model_tag}.json")

    def _ensure_motion_sequence_storage_migrated(self, model: str):
        model_tag = self._motion_sequence_model_tag(model)
        target_path = self._motion_state_file_path_for_model(model_tag)
        if os.path.isfile(target_path):
            return

        legacy_candidates = []
        legacy_same = self._legacy_motion_state_file_path_for_model(model_tag)
        legacy_wa2 = self._legacy_motion_state_file_path_for_model("WA2")
        legacy_wa2_ls = self._legacy_motion_state_file_path_for_model("WA2_LS")

        if model_tag == "WA2":
            legacy_candidates = [legacy_wa2, legacy_wa2_ls]
        elif model_tag == "WA2_LS":
            legacy_candidates = [legacy_wa2_ls, legacy_wa2]
        else:
            legacy_candidates = [legacy_same]

        legacy_path = ""
        for p in legacy_candidates:
            if p and p != target_path and os.path.isfile(p):
                legacy_path = p
                break
        if not legacy_path:
            return

        try:
            with open(legacy_path, "r", encoding="utf-8") as fr:
                legacy_payload = json.load(fr)
            migrated = self._normalize_motion_sequences_payload(legacy_payload or {}, fallback_model=model_tag, ensure_default=True)
            save_obj = {
                "version": 1,
                "current_index": migrated["current_index"],
                "sequence_seq": migrated["sequence_seq"],
                "sequences": migrated["sequences"],
            }
            with open(target_path, "w", encoding="utf-8") as fw:
                json.dump(save_obj, fw, ensure_ascii=False, indent=2)
            self.log_signal.emit(f"[INFO] 已将动作序列存档从 {os.path.basename(legacy_path)} 迁移到 {model_tag}")
        except Exception as e:
            self.log_signal.emit(f"[WARN] 迁移动作序列存档失败: {e}")

    def _motion_state_file_path(self) -> str:
        model_tag = self._motion_sequence_model_tag(getattr(self, "robot_model", "WA2"))
        return self._motion_state_file_path_for_model(model_tag)

    def _save_motion_sequences_to_disk(self):
        if self._suspend_motion_persist:
            return
        try:
            payload = {
                "version": 1,
                "current_index": int(self._motion_current_sequence_idx),
                "sequence_seq": int(self._motion_sequence_seq),
                "sequences": self._motion_sequences,
            }
            path = self._motion_state_file_path()
            with open(path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.log_signal.emit(f"[WARN] 保存动作序列失败: {e}")

    def _restore_motion_sequences_from_disk(self):
        self._ensure_motion_sequence_storage_migrated(getattr(self, "robot_model", "WA2"))
        path = self._motion_state_file_path()
        self._suspend_motion_persist = True
        valid_arm_types = self._valid_arm_types_for_model(getattr(self, "robot_model", "WA2"))
        try:
            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.clear()
            self.motion_seq_selector.blockSignals(False)
            self._motion_sequences = []
            self._motion_sequence_seq = 0
            self._motion_current_sequence_idx = -1

            loaded = None
            if os.path.isfile(path):
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        loaded = json.load(f)
                except Exception as e:
                    self.log_signal.emit(f"[WARN] 读取动作序列存档失败，将使用空配置: {e}")

            if isinstance(loaded, dict):
                seq_list = loaded.get("sequences")
                if isinstance(seq_list, list):
                    for seq in seq_list:
                        if not isinstance(seq, dict):
                            continue
                        sid = seq.get("id")
                        name = str(seq.get("name") or "序列")
                        rows = seq.get("rows") if isinstance(seq.get("rows"), list) else []
                        safe_rows = []
                        for r in rows:
                            if not isinstance(r, dict):
                                continue
                            pose = r.get("pose")
                            if not (isinstance(pose, list) or pose is None):
                                pose = None
                            arm_type = r.get("arm_type")
                            if arm_type not in valid_arm_types:
                                try:
                                    arm_type = int(arm_type)
                                except Exception:
                                    arm_type = None
                            if arm_type not in valid_arm_types:
                                arm_type = None
                            try:
                                duration = float(r.get("duration", 2.0))
                            except Exception:
                                duration = 2.0
                            safe_rows.append({"duration": duration, "pose": pose, "arm_type": arm_type})
                        try:
                            sid = int(sid)
                        except Exception:
                            sid = len(self._motion_sequences) + 1
                        self._motion_sequences.append({"id": sid, "name": name, "rows": safe_rows})

                try:
                    self._motion_sequence_seq = int(loaded.get("sequence_seq", 0))
                except Exception:
                    self._motion_sequence_seq = 0

                if self._motion_sequence_seq < len(self._motion_sequences):
                    self._motion_sequence_seq = len(self._motion_sequences)

                try:
                    wanted_idx = int(loaded.get("current_index", 0))
                except Exception:
                    wanted_idx = 0
            else:
                wanted_idx = 0

            if not self._motion_sequences:
                self._motion_sequence_seq = 1
                self._motion_sequences = [{"id": 1, "name": "序列1", "rows": []}]

            self.motion_seq_selector.blockSignals(True)
            for seq in self._motion_sequences:
                self.motion_seq_selector.addItem(seq.get("name") or "序列")
            self.motion_seq_selector.blockSignals(False)
            self.motion_seq_selector.update()
            self.motion_seq_selector.repaint()

            wanted_idx = max(0, min(wanted_idx, len(self._motion_sequences) - 1))
            self._motion_current_sequence_idx = wanted_idx
            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.setCurrentIndex(wanted_idx)
            self.motion_seq_selector.blockSignals(False)
            self._load_current_sequence_rows()
            self._set_motion_editing(False)
            self._update_motion_model_label()
            names = [str(seq.get("name") or "序列") for seq in self._motion_sequences]
            self.log_signal.emit(
                f"[INFO] 已从存档加载动作序列: model={getattr(self, 'robot_model', 'WA2')}, count={len(names)}, selector_count={self.motion_seq_selector.count()}, current_index={self._motion_current_sequence_idx}, names={names}"
            )
        finally:
            self._suspend_motion_persist = False
            self._save_motion_sequences_to_disk()

    def _fetch_joint_state_once(self):
        # 若监控已经持续订阅，优先使用最新缓存，避免重复订阅影响主监控
        if self._monitor_started:
            latest = self._get_latest_joint_state_map(self._joint_state_fresh_timeout_sec)
            if isinstance(latest, dict) and latest:
                return latest

        # 1) ROSBridge 优先
        if self.ros and self.ros.check_connection():
            done = threading.Event()
            box = {"data": None}

            def cb(msg: dict):
                try:
                    names = msg.get("name") or []
                    pos = msg.get("position") or []
                    if isinstance(names, list) and isinstance(pos, list) and names and pos:
                        box["data"] = {str(n): float(p) for n, p in zip(names, pos)}
                    elif isinstance(pos, list) and pos:
                        tmp = {}
                        for i, n in enumerate(self.joint_names):
                            if i < len(pos):
                                tmp[n] = float(pos[i])
                        box["data"] = tmp
                except Exception:
                    box["data"] = None
                finally:
                    done.set()

            topic = "/zj_humanoid/upperlimb/joint_states"
            try:
                topic_type = self.ros._get_topic_type(topic)
                if topic_type:
                    # 用临时 roslibpy.Topic 做一次性采样，不写入 BridgeClient 的全局订阅表
                    tmp_topic = roslibpy.Topic(self.ros.ros, topic, topic_type)
                    tmp_topic.subscribe(cb)
                else:
                    tmp_topic = None
                done.wait(timeout=2.0)
            except Exception:
                pass
            finally:
                try:
                    if 'tmp_topic' in locals() and tmp_topic is not None:
                        tmp_topic.unsubscribe()
                except Exception:
                    pass

            if isinstance(box.get("data"), dict) and box["data"]:
                with self._latest_joint_state_lock:
                    self._latest_joint_state_map = dict(box["data"])
                    self._latest_joint_state_ts = time.monotonic()
                return box["data"]

        # 2) SSH 兜底
        if self.ssh and self.ssh.ssh and self.ssh.sftp:
            try:
                out, err = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/upperlimb/joint_states")
                text = (out or "") + "\n" + (err or "")
                msg = self._parse_ros_cli_yaml(text) or {}
                names = msg.get("name") or self._extract_string_list_field(text, "name") or []
                pos = msg.get("position") or self._extract_float_list_field(text, "position") or []
                if isinstance(names, list) and isinstance(pos, list) and names and pos:
                    data = {str(n): float(p) for n, p in zip(names, pos)}
                    with self._latest_joint_state_lock:
                        self._latest_joint_state_map = dict(data)
                        self._latest_joint_state_ts = time.monotonic()
                    return data
                if isinstance(pos, list) and pos:
                    tmp = {}
                    for i, n in enumerate(self.joint_names):
                        if i < len(pos):
                            tmp[n] = float(pos[i])
                    if tmp:
                        with self._latest_joint_state_lock:
                            self._latest_joint_state_map = dict(tmp)
                            self._latest_joint_state_ts = time.monotonic()
                    return tmp
            except Exception:
                pass

        # 3) 最后回退：读取关节监控区当前值
        data = {}
        for n in self.joint_names:
            t = self.joint_fields.get(n).text() if n in self.joint_fields else ""
            try:
                data[n] = float(t)
            except Exception:
                continue
        if data:
            with self._latest_joint_state_lock:
                self._latest_joint_state_map = dict(data)
                self._latest_joint_state_ts = time.monotonic()
        return data

    def _reindex_motion_rows(self):
        for idx, row in enumerate(self._motion_rows, start=1):
            row["index_label"].setText(f"动作{idx}")

    def _current_sequence(self):
        if 0 <= self._motion_current_sequence_idx < len(self._motion_sequences):
            return self._motion_sequences[self._motion_current_sequence_idx]
        return None

    def _snapshot_motion_rows(self):
        rows = []
        for row in self._motion_rows:
            try:
                duration = float((row["duration_edit"].text() or "2.0").strip())
            except Exception:
                duration = 2.0
            pose = row.get("pose")
            if isinstance(pose, list):
                pose = [float(x) for x in pose]
            else:
                pose = None
            arm_type_combo = row.get("arm_type_combo")
            if arm_type_combo is not None:
                try:
                    arm_type = int(arm_type_combo.currentData())
                except Exception:
                    arm_type = row.get("arm_type")
            else:
                arm_type = row.get("arm_type")
            rows.append({"duration": duration, "pose": pose, "arm_type": arm_type})
        return rows

    def _wa2_empty_frame(self):
        return {
            "left_arm": [],
            "right_arm": [],
            "neck": [],
            "waist": [],
            "left_hand": [],
            "right_hand": [],
        }

    def _wa2_fill(self, values, size: int):
        out = []
        for v in values:
            out.append(0.0 if v is None else float(v))
        if len(out) < size:
            out.extend([0.0] * (size - len(out)))
        if len(out) > size:
            out = out[:size]
        return out

    def _wa2_frame_from_row(self, row: dict):
        frame = self._wa2_empty_frame()
        pose = row.get("pose")
        if not pose:
            return frame
        arm_type = row.get("arm_type")
        try:
            arm_type = int(arm_type)
        except Exception:
            arm_type = None

        if arm_type == 1:
            frame["left_arm"] = self._wa2_fill(pose, 8)
        elif arm_type == 2:
            frame["right_arm"] = self._wa2_fill(pose, 8)
        elif arm_type == 3:
            frame["left_arm"] = self._wa2_fill(pose[:8], 8)
            frame["right_arm"] = self._wa2_fill(pose[8:16], 8)
        elif arm_type == 4:
            frame["neck"] = self._wa2_fill(pose, 2)
        elif arm_type == 8:
            if len(pose) >= 4:
                frame["waist"] = self._wa2_fill(pose[:4], 4)
            elif len(pose) == 2:
                frame["waist"] = [0.0, 0.0, float(pose[0]), float(pose[1])]
            else:
                frame["waist"] = self._wa2_fill(pose, 4)
        elif arm_type == 15:
            name_map = {}
            for n, v in zip(self.joint_names, pose):
                name_map[n] = float(v)
            left_names = self._joint_names_for_arm_type(1)
            right_names = self._joint_names_for_arm_type(2)
            neck_names = self._joint_names_for_arm_type(4)
            waist_names = ["Pitch_Y_B", "Pitch_Y_M", "Waist_Z", "Waist_Y"]
            frame["left_arm"] = self._wa2_fill([name_map.get(n) for n in left_names], 8)
            frame["right_arm"] = self._wa2_fill([name_map.get(n) for n in right_names], 8)
            frame["neck"] = self._wa2_fill([name_map.get(n) for n in neck_names], 2)
            frame["waist"] = self._wa2_fill([name_map.get(n) for n in waist_names], 4)

        return frame

    def _wa1_empty_frame(self):
        return {
            "left_arm": [],
            "right_arm": [],
            "neck": [],
            "waist": [],
            "lifting": [],
            "left_hand": [],
            "right_hand": [],
        }

    def _wa1_frame_from_row(self, row: dict):
        frame = self._wa1_empty_frame()
        pose = row.get("pose")
        if not pose:
            return frame
        arm_type = row.get("arm_type")
        try:
            arm_type = int(arm_type)
        except Exception:
            arm_type = None

        if arm_type == 1:
            frame["left_arm"] = self._wa2_fill(pose, 7)
        elif arm_type == 2:
            frame["right_arm"] = self._wa2_fill(pose, 7)
        elif arm_type == 3:
            frame["left_arm"] = self._wa2_fill(pose[:7], 7)
            frame["right_arm"] = self._wa2_fill(pose[7:14], 7)
        elif arm_type == 4:
            frame["neck"] = self._wa2_fill(pose, 2)
        elif arm_type == 8:
            frame["waist"] = self._wa2_fill(pose, 2)
        elif arm_type == 16:
            frame["lifting"] = self._wa2_fill(pose, 1)
        elif arm_type == 31:
            name_map = {}
            for n, v in zip(self._wa1_full_body_names(), pose):
                name_map[n] = float(v)
            frame["left_arm"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(1)], 7)
            frame["right_arm"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(2)], 7)
            frame["neck"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(4)], 2)
            frame["waist"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(8)], 2)
            frame["lifting"] = self._wa2_fill([name_map.get("Lifting_Z")], 1)

        return frame

    def _i2_empty_frame(self):
        return {
            "left_arm": [],
            "right_arm": [],
            "neck": [],
            "waist": [],
            "left_hand": [],
            "right_hand": [],
        }

    def _i2_frame_from_row(self, row: dict):
        frame = self._i2_empty_frame()
        pose = row.get("pose")
        if not pose:
            return frame
        arm_type = row.get("arm_type")
        try:
            arm_type = int(arm_type)
        except Exception:
            arm_type = None

        if arm_type == 1:
            frame["left_arm"] = self._wa2_fill(pose, 7)
        elif arm_type == 2:
            frame["right_arm"] = self._wa2_fill(pose, 7)
        elif arm_type == 3:
            frame["left_arm"] = self._wa2_fill(pose[:7], 7)
            frame["right_arm"] = self._wa2_fill(pose[7:14], 7)
        elif arm_type == 4:
            frame["neck"] = self._wa2_fill(pose, 2)
        elif arm_type == 8:
            frame["waist"] = self._wa2_fill(pose, 1)
        elif arm_type == 15:
            name_map = {}
            for n, v in zip(self._i2_full_body_names(), pose):
                name_map[n] = float(v)
            frame["left_arm"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(1)], 7)
            frame["right_arm"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(2)], 7)
            frame["neck"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(4)], 2)
            frame["waist"] = self._wa2_fill([name_map.get(n) for n in self._joint_names_for_arm_type(8)], 1)

        return frame

    def export_motion_sequence(self):
        seq = self._current_sequence()
        if not seq:
            self.log_signal.emit("[ERR] 当前没有可导出的动作序列")
            return
        model = str(getattr(self, "robot_model", "WA2")).upper()

        rows = self._snapshot_motion_rows()
        full_body_type = 31 if model == "WA1" else 15
        for idx, r in enumerate(rows, start=1):
            if not r.get("pose"):
                continue
            arm_type = r.get("arm_type")
            try:
                arm_type = int(arm_type)
            except Exception:
                arm_type = None
            if arm_type != full_body_type:
                self.log_signal.emit(f"[ERR] 动作{idx} 不是全身(arm_type={full_body_type})，无法导出")
                return
        frames = []
        for r in rows:
            if r.get("pose"):
                if model == "WA1":
                    frames.append(self._wa1_frame_from_row(r))
                elif model == "I2":
                    frames.append(self._i2_frame_from_row(r))
                else:
                    frames.append(self._wa2_frame_from_row(r))

        if not frames:
            self.log_signal.emit("[ERR] 当前序列没有已记录的轨迹")
            return

        payload = {"model": model}
        for idx, frame in enumerate(frames):
            payload[str(idx)] = frame

        default_name = f"{seq.get('name', 'sequence')}.yaml"
        save_path, _ = QFileDialog.getSaveFileName(self, "导出动作序列", default_name, "YAML Files (*.yaml *.yml)")
        if not save_path:
            return

        try:
            class _FlowListDumper(yaml.SafeDumper):
                pass

            def _list_representer(dumper, data):
                return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)

            _FlowListDumper.add_representer(list, _list_representer)

            with open(save_path, "w", encoding="utf-8") as f:
                yaml.dump(payload, f, Dumper=_FlowListDumper, allow_unicode=True, sort_keys=False)
            self.log_signal.emit(f"[OK] 动作序列已导出: {save_path}")
        except Exception as e:
            self.log_signal.emit(f"[ERR] 导出失败: {e}")

    def export_motion_sequence_python(self):
        seq = self._current_sequence()
        if not seq:
            self.log_signal.emit("[ERR] 当前没有可导出的动作序列")
            return

        rows = self._snapshot_motion_rows()
        items = []
        for row in rows:
            pose = row.get("pose")
            if not pose:
                continue
            arm_type = row.get("arm_type")
            if arm_type is None:
                self.log_signal.emit("[ERR] 存在未记录部位的轨迹，请重新记录")
                return
            try:
                t = float((row.get("duration") or 2.0))
            except Exception:
                self.log_signal.emit("[ERR] 轨迹时长格式错误")
                return
            if t <= 0:
                self.log_signal.emit("[ERR] 轨迹时长必须>0")
                return
            items.append({"arm_type": int(arm_type), "pose": list(pose), "duration": t})

        if not items:
            self.log_signal.emit("[ERR] 当前序列没有已记录的轨迹")
            return

        node_name, ok = QInputDialog.getText(self, "ROS节点名", "请输入节点名", text="motion_seq_runner")
        if not ok:
            return
        node_name = (node_name or "").strip()
        if not node_name:
            self.log_signal.emit("[ERR] 节点名不能为空")
            return

        groups = []
        for item in items:
            if not groups or groups[-1]["arm_type"] != item["arm_type"]:
                groups.append({"arm_type": item["arm_type"], "poses": [item["pose"]], "durations": [item["duration"]]})
            else:
                groups[-1]["poses"].append(item["pose"])
                groups[-1]["durations"].append(item["duration"])

        calls = []
        for g in groups:
            arm_type = g["arm_type"]
            service = self._movej_service_for_arm_type(arm_type)
            if not service:
                self.log_signal.emit(f"[ERR] 未找到movej_by_path服务: arm_type={arm_type}")
                return
            total_time = float(sum(g["durations"])) if g["durations"] else 1.0
            if total_time <= 0:
                total_time = 1.0
            path = [{"joint": [float(v) for v in p]} for p in g["poses"]]
            payload = {"path": path, "time": total_time, "is_async": False, "arm_type": arm_type}
            calls.append({"service": service, "payload": payload, "arm_type": arm_type})

        default_name = f"{seq.get('name', 'sequence')}.py"
        file_name, ok = QInputDialog.getText(self, "脚本文件名", "请输入导出文件名", text=default_name)
        if not ok:
            return
        file_name = (file_name or "").strip() or default_name
        if not file_name.lower().endswith(".py"):
            file_name += ".py"
        save_path, _ = QFileDialog.getSaveFileName(self, "导出可执行脚本", file_name, "Python Files (*.py)")
        if not save_path:
            return

        try:
            script = self._render_motion_script(node_name, calls)
            with open(save_path, "w", encoding="utf-8") as f:
                f.write(script)
            try:
                os.chmod(save_path, 0o755)
            except Exception:
                pass
            self.log_signal.emit(f"[OK] 可执行脚本已生成: {save_path}")
        except Exception as e:
            self.log_signal.emit(f"[ERR] 导出脚本失败: {e}")

    def _render_motion_script(self, node_name: str, calls: list) -> str:
        import pprint
        calls_text = pprint.pformat(calls, width=120)
        return (
            "#!/usr/bin/env python3\n"
            "# -*- coding: utf-8 -*-\n"
            "\n"
            "import os\n"
            "import sys\n"
            "from datetime import datetime\n"
            "import sys\n"
            "\n"
            "try:\n"
            "    import rospy\n"
            "    from upperlimb.msg import Joints\n"
            "    from upperlimb.srv import MoveJByPath, MoveJByPathRequest\n"
            "except Exception:\n"
            "    rospy = None\n"
            "\n"
            f"NODE_NAME = {json.dumps(node_name, ensure_ascii=False)}\n"
            f"CALLS = {calls_text}\n"
            "\n"
            "LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')\n"
            "os.makedirs(LOG_DIR, exist_ok=True)\n"
            "LOG_FILE = os.path.join(LOG_DIR, f'motion_seq_{datetime.now().strftime(\"%Y%m%d_%H%M%S\")}.log')\n"
            "\n"
            "def log_print(msg):\n"
            "    now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')\n"
            "    text = f'[{now_str}] {msg}'\n"
            "    print(text)\n"
            "    try:\n"
            "        with open(LOG_FILE, 'a', encoding='utf-8') as f:\n"
            "            f.write(text + '\\n')\n"
            "    except Exception:\n"
            "        pass\n"
            "\n"
            "def call_movej_by_path(service_name, payload):\n"
            "    rospy.wait_for_service(service_name, timeout=10.0)\n"
            "    client = rospy.ServiceProxy(service_name, MoveJByPath)\n"
            "    req = MoveJByPathRequest()\n"
            "\n"
            "    raw_path = payload.get('path', [])\n"
            "    for item in raw_path:\n"
            "        joints = item.get('joint', []) if isinstance(item, dict) else []\n"
            "        jm = Joints()\n"
            "        jm.joint = [float(v) for v in joints]\n"
            "        req.path.append(jm)\n"
            "\n"
            "    req.time = float(payload.get('time', 1.0))\n"
            "    req.is_async = bool(payload.get('is_async', False))\n"
            "    req.arm_type = int(payload.get('arm_type', 0))\n"
            "\n"
            "    return client(req)\n"
            "\n"
            "def main():\n"
            "    if rospy is None:\n"
            "        raise RuntimeError('请在ROS环境中运行该脚本')\n"
            "    rospy.init_node(NODE_NAME, anonymous=True)\n"
            "    log_print(f'节点启动: {NODE_NAME}')\n"
            "    for idx, item in enumerate(CALLS, start=1):\n"
            "        service = item.get('service')\n"
            "        payload = item.get('payload')\n"
            "        arm_type = item.get('arm_type')\n"
            "        log_print(f'[INFO] {idx}/{len(CALLS)} 调用 {service}, arm_type={arm_type}')\n"
            "        resp = call_movej_by_path(service, payload)\n"
            "        ok = bool(getattr(resp, 'success', True))\n"
            "        msg = str(getattr(resp, 'message', resp))\n"
            "        if not ok:\n"
            "            raise RuntimeError(f'服务返回失败: {service}, message={msg}')\n"
            "        log_print(f'[OK] {service}: {msg}')\n"
            "\n"
            "if __name__ == '__main__':\n"
            "    try:\n"
            "        main()\n"
            "    except Exception as e:\n"
            "        print(f'[ERR] {e}')\n"
            "        sys.exit(1)\n"
        )

    def _persist_current_sequence_rows(self):
        seq = self._current_sequence()
        if seq is None:
            return
        seq["rows"] = self._snapshot_motion_rows()
        self._save_motion_sequences_to_disk()

    def _clear_motion_row_widgets(self):
        for row in self._motion_rows:
            w = row.get("widget")
            if w:
                w.setParent(None)
                w.deleteLater()
        self._motion_rows = []

    def _load_current_sequence_rows(self):
        seq = self._current_sequence()
        self._clear_motion_row_widgets()
        if not seq:
            self._set_motion_editing(False)
            return
        for item in seq.get("rows", []):
            self.add_motion_row(
                force=True,
                initial_duration=item.get("duration", 2.0),
                initial_pose=item.get("pose"),
                initial_arm_type=item.get("arm_type"),
                silent=True,
            )
        self._set_motion_editing(False)

    def _on_motion_sequence_changed(self, index: int):
        if index < 0:
            return
        if self._motion_editing:
            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.setCurrentIndex(self._motion_current_sequence_idx)
            self.motion_seq_selector.blockSignals(False)
            self.log_signal.emit("[ERR] 编辑中不能切换序列，请先“完成编辑”")
            return
        if index == self._motion_current_sequence_idx:
            return
        self._persist_current_sequence_rows()
        self._motion_current_sequence_idx = index
        self._load_current_sequence_rows()
        self._update_motion_model_label()

    def _motion_sequences_export_payload(self) -> dict:
        return {
            "version": 1,
            "model": self._motion_sequence_model_tag(getattr(self, "robot_model", "WA2")),
            "current_index": int(self._motion_current_sequence_idx),
            "sequence_seq": int(self._motion_sequence_seq),
            "sequences": self._motion_sequences,
        }

    def _normalize_motion_sequences_payload(self, payload: dict, fallback_model: str = None, ensure_default: bool = True) -> dict:
        model = self._motion_sequence_model_tag(fallback_model or payload.get("model") or getattr(self, "robot_model", "WA2"))

        valid_arm_types = self._valid_arm_types_for_model(model)
        seq_list = payload.get("sequences") if isinstance(payload.get("sequences"), list) else []
        seqs = []
        for seq in seq_list:
            if not isinstance(seq, dict):
                continue
            sid = seq.get("id")
            try:
                sid = int(sid)
            except Exception:
                sid = len(seqs) + 1
            name = str(seq.get("name") or f"序列{sid}")
            rows = seq.get("rows") if isinstance(seq.get("rows"), list) else []
            safe_rows = []
            for r in rows:
                if not isinstance(r, dict):
                    continue
                pose = r.get("pose")
                if not (isinstance(pose, list) or pose is None):
                    pose = None
                arm_type = r.get("arm_type")
                try:
                    arm_type = int(arm_type)
                except Exception:
                    arm_type = None
                if arm_type not in valid_arm_types:
                    arm_type = None
                try:
                    duration = float(r.get("duration", 2.0))
                except Exception:
                    duration = 2.0
                safe_rows.append({"duration": duration, "pose": pose, "arm_type": arm_type})
            seqs.append({"id": sid, "name": name, "rows": safe_rows})

        if not seqs and ensure_default:
            seqs = [{"id": 1, "name": "序列1", "rows": []}]

        try:
            sequence_seq = int(payload.get("sequence_seq", 0))
        except Exception:
            sequence_seq = 0
        if sequence_seq < len(seqs):
            sequence_seq = len(seqs)

        try:
            current_idx = int(payload.get("current_index", 0))
        except Exception:
            current_idx = 0
        if seqs:
            current_idx = max(0, min(current_idx, len(seqs) - 1))
        else:
            current_idx = 0

        return {
            "version": 1,
            "model": model,
            "current_index": current_idx,
            "sequence_seq": sequence_seq,
            "sequences": seqs,
        }

    def _merge_motion_sequences_payload(self, base_payload: dict, imported_payload: dict) -> dict:
        base = self._normalize_motion_sequences_payload(base_payload or {}, ensure_default=True)
        imported = self._normalize_motion_sequences_payload(imported_payload or {}, fallback_model=base["model"], ensure_default=False)
        if not imported["sequences"]:
            return base

        merged_sequences = copy.deepcopy(base["sequences"])
        next_sequence_id = max(int(base.get("sequence_seq", 0)), len(merged_sequences))
        first_imported_index = len(merged_sequences)
        for seq in imported["sequences"]:
            next_sequence_id += 1
            merged_seq = copy.deepcopy(seq)
            merged_seq["id"] = next_sequence_id
            original_name = str(merged_seq.get("name") or "").strip()
            if re.fullmatch(r"序列\d+", original_name):
                merged_seq["name"] = f"序列{next_sequence_id}"
            elif original_name:
                merged_seq["name"] = f"{original_name}(序列{next_sequence_id})"
            else:
                merged_seq["name"] = f"序列{next_sequence_id}"
            merged_sequences.append(merged_seq)

        return {
            "version": 1,
            "model": base["model"],
            "current_index": first_imported_index,
            "sequence_seq": max(next_sequence_id, len(merged_sequences)),
            "sequences": merged_sequences,
        }

    def _write_motion_sequences_payload_for_model(self, model: str, payload: dict):
        normalized = self._normalize_motion_sequences_payload(payload or {}, fallback_model=model, ensure_default=True)
        path = self._motion_state_file_path_for_model(normalized["model"])
        save_obj = {
            "version": 1,
            "current_index": normalized["current_index"],
            "sequence_seq": normalized["sequence_seq"],
            "sequences": normalized["sequences"],
        }
        with open(path, "w", encoding="utf-8") as fw:
            json.dump(save_obj, fw, ensure_ascii=False, indent=2)

    def _ask_motion_sequences_import_mode(self) -> str:
        options = ["覆盖现有序列", "追加到现有序列"]
        selected, ok = QInputDialog.getItem(self, "加载方式", "请选择加载方式", options, 0, False)
        if not ok:
            return ""
        return "append" if selected == "追加到现有序列" else "overwrite"

    def export_motion_sequences(self):
        self._persist_current_sequence_rows()
        options = ["导出全部序列（WA2/WA1/I2）", "导出当前选中序列"]
        selected, ok = QInputDialog.getItem(self, "导出范围", "请选择导出范围", options, 0, False)
        if not ok:
            return
        if selected == "导出当前选中序列":
            self._export_current_motion_sequence_json()
            return
        self.export_all_motion_sequences()

    def _export_current_motion_sequence_json(self):
        seq = self._current_sequence()
        if not seq:
            self.log_signal.emit("[ERR] 当前没有可导出的动作序列")
            return

        model = str(getattr(self, "robot_model", "WA2")).upper()
        exported_seq = copy.deepcopy(seq)
        exported_seq["id"] = 1
        exported_seq["name"] = str(exported_seq.get("name") or "序列1")
        payload = {
            "version": 1,
            "model": model,
            "current_index": 0,
            "sequence_seq": 1,
            "sequences": [exported_seq],
        }

        default_name = f"{exported_seq['name']}.json"
        save_path, _ = QFileDialog.getSaveFileName(self, "导出当前选中序列", default_name, "JSON Files (*.json)")
        if not save_path:
            return
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            self.log_signal.emit(f"[OK] 已导出当前选中序列: {save_path}")
        except Exception as e:
            self.log_signal.emit(f"[ERR] 导出当前选中序列失败: {e}")

    def _load_motion_payload_for_model(self, model: str) -> dict:
        model = str(model or "WA2").upper()
        path = self._motion_state_file_path_for_model(model)
        loaded = None
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
            except Exception:
                loaded = None
        if not isinstance(loaded, dict):
            loaded = {}
        return {
            "version": 1,
            "model": model,
            "current_index": int(loaded.get("current_index", 0)) if isinstance(loaded.get("current_index", 0), (int, float, str)) else 0,
            "sequence_seq": int(loaded.get("sequence_seq", 0)) if isinstance(loaded.get("sequence_seq", 0), (int, float, str)) else 0,
            "sequences": loaded.get("sequences") if isinstance(loaded.get("sequences"), list) else [],
        }

    def export_all_motion_sequences(self):
        self._persist_current_sequence_rows()
        current_model = str(getattr(self, "robot_model", "WA2")).upper()
        models = {}
        for m in ("WA2", "WA2_LS", "WA1", "I2"):
            if m == current_model:
                p = self._motion_sequences_export_payload()
                p["model"] = m
                models[m] = p
            else:
                models[m] = self._load_motion_payload_for_model(m)

        payload = {
            "version": 2,
            "format": "all_models",
            "current_model": current_model,
            "models": models,
        }

        total_count = 0
        for m in ("WA2", "WA2_LS", "WA1", "I2"):
            total_count += len((models.get(m) or {}).get("sequences") or [])
        if total_count <= 0:
            self.log_signal.emit("[ERR] 当前没有可导出的序列")
            return

        default_name = "motion_sequences_all_models.json"
        save_path, _ = QFileDialog.getSaveFileName(self, "导出全部序列", default_name, "JSON Files (*.json)")
        if not save_path:
            return
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            self.log_signal.emit(f"[OK] 已导出全部序列(含WA2/WA2_LS/WA1/I2): {save_path}")
        except Exception as e:
            self.log_signal.emit(f"[ERR] 导出全部序列失败: {e}")

    def _apply_imported_motion_sequences(self, payload: dict):
        normalized = self._normalize_motion_sequences_payload(payload or {}, ensure_default=True)
        model = normalized["model"]
        self._set_robot_model(model)

        self._suspend_motion_persist = True
        try:
            self._motion_sequences = normalized["sequences"]
            self._motion_sequence_seq = normalized["sequence_seq"]
            self._motion_current_sequence_idx = normalized["current_index"]

            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.clear()
            for seq in self._motion_sequences:
                self.motion_seq_selector.addItem(seq.get("name") or "序列")
            self.motion_seq_selector.setCurrentIndex(self._motion_current_sequence_idx)
            self.motion_seq_selector.blockSignals(False)
            self.motion_seq_selector.update()
            self.motion_seq_selector.repaint()

            self._load_current_sequence_rows()
            self._set_motion_editing(False)
            self._update_motion_model_label()
            names = [str(seq.get("name") or "序列") for seq in self._motion_sequences]
            self.log_signal.emit(
                f"[INFO] 已应用导入动作序列: model={model}, count={len(names)}, selector_count={self.motion_seq_selector.count()}, current_index={self._motion_current_sequence_idx}, names={names}"
            )
        finally:
            self._suspend_motion_persist = False
            self._save_motion_sequences_to_disk()

    def import_motion_sequences(self):
        if self._motion_editing:
            self.log_signal.emit("[ERR] 请先完成当前序列编辑")
            return
        self._persist_current_sequence_rows()
        file_path, _ = QFileDialog.getOpenFileName(self, "加载序列", "", "JSON Files (*.json)")
        if not file_path:
            return
        import_mode = self._ask_motion_sequences_import_mode()
        if not import_mode:
            return
        is_append_mode = import_mode == "append"
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if not isinstance(payload, dict):
                raise RuntimeError("文件格式错误")

            # 新格式：一次导入WA2/WA2_LS/WA1/I2全部序列
            if payload.get("format") == "all_models" and isinstance(payload.get("models"), dict):
                models = payload.get("models") or {}
                applied_models = {}
                for m in ("WA2", "WA2_LS", "WA1", "I2"):
                    mp = models.get(m)
                    if not isinstance(mp, dict):
                        continue
                    incoming_payload = dict(mp)
                    incoming_payload["model"] = m
                    if is_append_mode:
                        base_payload = self._motion_sequences_export_payload() if m == str(getattr(self, "robot_model", "WA2")).upper() else self._load_motion_payload_for_model(m)
                        final_payload = self._merge_motion_sequences_payload(base_payload, incoming_payload)
                    else:
                        final_payload = self._normalize_motion_sequences_payload(incoming_payload, fallback_model=m, ensure_default=True)
                    self._write_motion_sequences_payload_for_model(m, final_payload)
                    applied_models[m] = final_payload

                target_model = str(payload.get("current_model") or getattr(self, "robot_model", "WA2")).upper()
                if target_model not in ("WA2", "WA2_LS", "WA1", "I2"):
                    target_model = "WA2"
                target_payload = applied_models.get(target_model)
                if isinstance(target_payload, dict):
                    self._apply_imported_motion_sequences(target_payload)
                else:
                    current_model = str(getattr(self, "robot_model", "WA2") or "WA2").upper()
                    if current_model == target_model:
                        self._restore_motion_sequences_from_disk()
                        self._update_motion_model_label()
                    else:
                        self._set_robot_model(target_model)
                mode_text = "追加" if is_append_mode else "覆盖"
                self.log_signal.emit(f"[OK] 已加载全部序列: {file_path} (current_model={target_model}, mode={mode_text})")
                return

            # 兼容旧格式：单型号序列
            model = str(payload.get("model") or getattr(self, "robot_model", "WA2")).upper()
            if model not in ("WA2", "WA2_LS", "WA1", "I2"):
                model = "WA2"
            if is_append_mode:
                base_payload = self._motion_sequences_export_payload() if model == str(getattr(self, "robot_model", "WA2")).upper() else self._load_motion_payload_for_model(model)
                final_payload = self._merge_motion_sequences_payload(base_payload, dict(payload, model=model))
            else:
                final_payload = self._normalize_motion_sequences_payload(dict(payload, model=model), fallback_model=model, ensure_default=True)
            self._apply_imported_motion_sequences(final_payload)
            mode_text = "追加" if is_append_mode else "覆盖"
            self.log_signal.emit(f"[OK] 已加载序列: {file_path} (model={model}, mode={mode_text})")
        except Exception as e:
            self.log_signal.emit(f"[ERR] 加载序列失败: {e}")

    def _ask_motion_sequence_name(self, default_name: str = "") -> str:
        text, ok = QInputDialog.getText(self, "序列命名", "请输入序列名称", text=default_name or "")
        if not ok:
            return ""
        return str(text or "").strip()

    def add_motion_sequence(self, initial: bool = False):
        if self._motion_editing and (not initial):
            self.log_signal.emit("[ERR] 请先完成当前序列编辑")
            return

        self._persist_current_sequence_rows()
        self._motion_sequence_seq += 1
        default_name = f"序列{self._motion_sequence_seq}"
        sequence_name = default_name
        if not initial:
            custom_name = self._ask_motion_sequence_name(default_name)
            if custom_name == "":
                self._motion_sequence_seq -= 1
                return
            sequence_name = custom_name
        seq = {
            "id": self._motion_sequence_seq,
            "name": sequence_name,
            "rows": [],
        }
        self._motion_sequences.append(seq)

        self.motion_seq_selector.blockSignals(True)
        self.motion_seq_selector.addItem(seq["name"])
        self.motion_seq_selector.setCurrentIndex(len(self._motion_sequences) - 1)
        self.motion_seq_selector.blockSignals(False)

        self._motion_current_sequence_idx = len(self._motion_sequences) - 1
        self._load_current_sequence_rows()
        self._save_motion_sequences_to_disk()
        if not initial:
            self.log_signal.emit(f"[OK] 已新增动作序列: {seq['name']}")

    def rename_current_motion_sequence(self):
        if self._motion_editing:
            self.log_signal.emit("[ERR] 请先完成当前序列编辑")
            return
        seq = self._current_sequence()
        if not seq:
            self.log_signal.emit("[ERR] 当前没有可重命名的动作序列")
            return
        old_name = str(seq.get("name") or "").strip() or f"序列{seq.get('id', 1)}"
        new_name = self._ask_motion_sequence_name(old_name)
        if not new_name:
            return
        if new_name == old_name:
            return
        seq["name"] = new_name
        idx = self._motion_current_sequence_idx
        if idx >= 0:
            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.setItemText(idx, new_name)
            self.motion_seq_selector.blockSignals(False)
        self._save_motion_sequences_to_disk()
        self.log_signal.emit(f"[OK] 已重命名动作序列: {old_name} -> {new_name}")

    def delete_current_motion_sequence(self):
        if self._motion_editing:
            self.log_signal.emit("[ERR] 请先完成当前序列编辑")
            return
        if not self._motion_sequences:
            self.log_signal.emit("[INFO] 当前没有可删除序列")
            return

        idx = self._motion_current_sequence_idx
        if idx < 0 or idx >= len(self._motion_sequences):
            idx = len(self._motion_sequences) - 1

        removed = self._motion_sequences.pop(idx)
        self.motion_seq_selector.blockSignals(True)
        self.motion_seq_selector.removeItem(idx)
        self.motion_seq_selector.blockSignals(False)

        if not self._motion_sequences:
            self._motion_current_sequence_idx = -1
            self._clear_motion_row_widgets()
            self.add_motion_sequence(initial=True)
        else:
            new_idx = min(idx, len(self._motion_sequences) - 1)
            self._motion_current_sequence_idx = new_idx
            self.motion_seq_selector.blockSignals(True)
            self.motion_seq_selector.setCurrentIndex(new_idx)
            self.motion_seq_selector.blockSignals(False)
            self._load_current_sequence_rows()

        self._save_motion_sequences_to_disk()

        self.log_signal.emit(f"[OK] 已删除动作序列: {removed['name']}")

    def _set_motion_editing(self, editing: bool):
        self._motion_editing = bool(editing)
        self.motion_seq_selector.setEnabled(not self._motion_editing)
        self.btn_add_sequence.setEnabled(not self._motion_editing)
        self.btn_delete_sequence.setEnabled((not self._motion_editing) and self.motion_seq_selector.count() > 1)
        self.btn_edit_motion_seq.setEnabled(not self._motion_editing)
        self.btn_add_motion_row.setEnabled(self._motion_editing)
        self.btn_delete_motion_row.setEnabled(self._motion_editing and bool(self._motion_rows))
        self.btn_finish_motion_seq.setEnabled(self._motion_editing)

        for row in self._motion_rows:
            row["duration_edit"].setReadOnly(not self._motion_editing)
            row["btn_record"].setEnabled(self._motion_editing)
            row["btn_delete"].setEnabled(self._motion_editing)
            if row.get("arm_type_combo") is not None:
                row["arm_type_combo"].setEnabled(self._motion_editing)
            if row.get("btn_movej") is not None:
                row["btn_movej"].setEnabled(True)

        self._sync_motion_execute_button()

    def _sync_motion_execute_button(self):
        has_pose = any(bool(r.get("pose")) for r in self._motion_rows)
        self.btn_exec_motion_path.setEnabled((not self._motion_editing) and has_pose and (not self._motion_exec_running))
        if hasattr(self, "btn_stop_motion_path"):
            self.btn_stop_motion_path.setEnabled(self._motion_exec_running)

    def _set_motion_exec_running(self, running: bool):
        self._motion_exec_running = bool(running)
        if hasattr(self, "motion_loop_count_edit"):
            self.motion_loop_count_edit.setEnabled(not self._motion_exec_running)
        if hasattr(self, "btn_edit_motion_seq"):
            self.btn_edit_motion_seq.setEnabled((not self._motion_editing) and (not self._motion_exec_running))
        if hasattr(self, "btn_add_sequence"):
            self.btn_add_sequence.setEnabled((not self._motion_editing) and (not self._motion_exec_running))
        if hasattr(self, "btn_delete_sequence"):
            self.btn_delete_sequence.setEnabled((not self._motion_editing) and (not self._motion_exec_running) and self.motion_seq_selector.count() > 1)
        self._sync_motion_execute_button()

    def stop_motion_path(self):
        if not self._motion_exec_running:
            self.log_signal.emit("[INFO] 当前没有正在执行的动作序列")
            return
        self._motion_exec_stop_event.set()
        self.log_signal.emit("[INFO] 已请求打断动作序列，将在当前调用结束后停止")

    def start_motion_edit(self):
        if self._motion_exec_running:
            self.log_signal.emit("[ERR] 动作序列执行中，无法进入编辑")
            return
        if self._motion_editing:
            self.log_signal.emit("[INFO] 当前已在编辑动作序列")
            return
        if not self._motion_rows:
            self.add_motion_row(force=True)
        self._set_motion_editing(True)
        self.log_signal.emit("[OK] 已进入动作序列编辑")

    def finish_motion_edit(self):
        if not self._motion_editing:
            self.log_signal.emit("[INFO] 当前未处于编辑状态")
            return
        self._persist_current_sequence_rows()
        self._set_motion_editing(False)
        total = len(self._motion_rows)
        recorded = sum(1 for r in self._motion_rows if r.get("pose"))
        self.log_signal.emit(f"[OK] 已完成动作序列编辑: 共{total}条，已记录{recorded}条")

    def delete_last_motion_row(self):
        if not self._motion_editing:
            self.log_signal.emit("[ERR] 请先点击“编辑动作序列”")
            return
        if not self._motion_rows:
            self.log_signal.emit("[INFO] 当前没有可删除的轨迹")
            return
        row = self._motion_rows.pop()
        w = row.get("widget")
        if w:
            w.setParent(None)
            w.deleteLater()
        self._reindex_motion_rows()
        self._set_motion_editing(True)
        self._persist_current_sequence_rows()

    def add_motion_row(self, force: bool = False, initial_duration: float = 2.0, initial_pose=None, initial_arm_type=None, silent: bool = False, insert_index: int = None):
        if (not force) and (not self._motion_editing):
            self.log_signal.emit("[ERR] 请先点击“编辑动作序列”")
            return
        self._motion_row_seq += 1

        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)

        index_label = QLabel(f"动作{len(self._motion_rows) + 1}")
        arm_type_combo = QComboBox()
        arm_type_combo.addItem("左臂", 1)
        arm_type_combo.addItem("右臂", 2)
        arm_type_combo.addItem("双臂", 3)
        arm_type_combo.addItem("脖子", 4)
        arm_type_combo.addItem("腰部", 8)
        if str(getattr(self, "robot_model", "WA2")).upper() == "WA1":
            arm_type_combo.addItem("升降", 16)
        full_body_type = 31 if str(getattr(self, "robot_model", "WA2")).upper() == "WA1" else 15
        arm_type_combo.addItem("全身", full_body_type)
        idx_map = {1: 0, 2: 1, 3: 2, 4: 3, 8: 4}
        next_index = 5
        if str(getattr(self, "robot_model", "WA2")).upper() == "WA1":
            idx_map[16] = next_index
            next_index += 1
        idx_map[full_body_type] = next_index
        if initial_arm_type in idx_map:
            if int(initial_arm_type) in idx_map:
                arm_type_combo.setCurrentIndex(idx_map[int(initial_arm_type)])
        else:
            # 默认沿用当前主下拉框选择
            d = self.joint_ctrl_part.currentData()
            try:
                d = int(d)
            except Exception:
                d = 1
            arm_type_combo.setCurrentIndex(idx_map.get(d, 0))
        duration_edit = QLineEdit(str(initial_duration if initial_duration else 2.0))
        duration_edit.setFixedWidth(70)
        duration_edit.setPlaceholderText("时长")
        pose_preview = QLineEdit("未记录")
        pose_preview.setReadOnly(True)
        btn_record = QPushButton("记录姿态")
        btn_movej = QPushButton("单步调试")
        btn_insert = QPushButton("插入")
        btn_delete = QPushButton("删除")

        row_layout.addWidget(index_label)
        row_layout.addWidget(QLabel("部位"))
        row_layout.addWidget(arm_type_combo)
        row_layout.addWidget(QLabel("时长(s)"))
        row_layout.addWidget(duration_edit)
        row_layout.addWidget(pose_preview, 1)
        row_layout.addWidget(btn_record)
        row_layout.addWidget(btn_movej)
        row_layout.addWidget(btn_insert)
        row_layout.addWidget(btn_delete)

        row = {
            "id": self._motion_row_seq,
            "widget": row_widget,
            "index_label": index_label,
            "duration_edit": duration_edit,
            "pose_preview": pose_preview,
            "arm_type_combo": arm_type_combo,
            "btn_record": btn_record,
            "btn_movej": btn_movej,
            "btn_insert": btn_insert,
            "btn_delete": btn_delete,
            "pose": list(initial_pose) if isinstance(initial_pose, list) else None,
            "arm_type": int(arm_type_combo.currentData()),
        }

        if isinstance(row["pose"], list) and row["pose"]:
            arm_text = self._arm_type_text(row["arm_type"]) if row.get("arm_type") is not None else "未知"
            pose_preview.setText(f"[{arm_text}] " + ", ".join(f"{float(v):.3f}" for v in row["pose"]))

        def update_record_button_text():
            try:
                current_arm = int(arm_type_combo.currentData())
            except Exception:
                current_arm = 1
            btn_record.setText("填写升降" if self._is_wa1_lifting_arm_type(current_arm) else "记录姿态")

        update_record_button_text()

        def on_arm_type_changed():
            try:
                new_arm = int(arm_type_combo.currentData())
            except Exception:
                new_arm = 1
            if row.get("arm_type") == new_arm:
                return
            row["arm_type"] = new_arm
            # 部位切换后清空已记录姿态，避免左右臂数据错配
            if row.get("pose"):
                row["pose"] = None
                pose_preview.setText("未记录")
                self.log_signal.emit(f"[INFO] {index_label.text()} 已切换为{self._arm_type_text(new_arm)}，请重新记录姿态")
            update_record_button_text()
            self._sync_motion_execute_button()
            self._persist_current_sequence_rows()

        arm_type_combo.currentIndexChanged.connect(lambda _: on_arm_type_changed())

        def do_record():
            try:
                arm_type = int(arm_type_combo.currentData())
            except Exception:
                arm_type = 1
            names = self._joint_names_for_arm_type(arm_type)
            if not names:
                self.log_signal.emit("[ERR] 当前部位不支持记录")
                return
            if self._is_wa1_lifting_arm_type(arm_type):
                current_value = None
                existing_pose = row.get("pose")
                if isinstance(existing_pose, list) and existing_pose:
                    try:
                        current_value = float(existing_pose[0])
                    except Exception:
                        current_value = None
                if current_value is None:
                    data = self._fetch_joint_state_once() or {}
                    try:
                        current_value = float(data.get("Lifting_Z")) if ("Lifting_Z" in data) else None
                    except Exception:
                        current_value = None
                value = self._prompt_wa1_lifting_value(f"{index_label.text()} 升降", current_value)
                if value is None:
                    return
                vals = self._validate_pose_for_arm_type(arm_type, [value])
                row["pose"] = vals
                row["arm_type"] = arm_type
                pose_preview.setText(f"[{self._arm_type_text(arm_type)}] " + ", ".join(f"{v:.3f}" for v in vals))
                self.log_signal.emit(f"[OK] 已填写{index_label.text()}: {self._arm_type_text(arm_type)}={vals[0]:.3f}")
                self._sync_motion_execute_button()
                self._persist_current_sequence_rows()
                return
            data = self._fetch_joint_state_once() or {}
            vals = []
            missing = []
            for n in names:
                if n not in data:
                    missing.append(n)
                else:
                    vals.append(float(data[n]))
            if missing:
                self.log_signal.emit(f"[ERR] 记录失败，缺少关节数据: {', '.join(missing)}")
                return
            row["pose"] = vals
            row["arm_type"] = arm_type
            pose_preview.setText(f"[{self._arm_type_text(arm_type)}] " + ", ".join(f"{v:.3f}" for v in vals))
            self.log_signal.emit(f"[OK] 已记录{index_label.text()}: {self._arm_type_text(arm_type)}(arm_type={arm_type}), joints={len(vals)}")
            self._sync_motion_execute_button()
            self._persist_current_sequence_rows()

        def do_delete():
            for i, x in enumerate(self._motion_rows):
                if x["id"] == row["id"]:
                    self._motion_rows.pop(i)
                    break
            row_widget.setParent(None)
            row_widget.deleteLater()
            self._reindex_motion_rows()
            self._set_motion_editing(self._motion_editing)
            self._persist_current_sequence_rows()

        def do_insert():
            if not self._motion_editing:
                self.log_signal.emit("[ERR] 请先点击“编辑动作序列”")
                return
            try:
                idx = self._motion_rows.index(row)
            except ValueError:
                idx = len(self._motion_rows) - 1
            self.add_motion_row(force=True, insert_index=idx + 1)

        def do_movej():
            pose = row.get("pose")
            if not pose:
                self.log_signal.emit(f"[ERR] {index_label.text()} 未记录姿态，无法单步调试")
                return
            arm_type = row.get("arm_type")
            self._execute_single_movej(int(arm_type), list(pose), index_label.text())

        btn_record.clicked.connect(do_record)
        btn_movej.clicked.connect(do_movej)
        btn_insert.clicked.connect(do_insert)
        btn_delete.clicked.connect(do_delete)

        if insert_index is None:
            self.motion_seq_layout.insertWidget(max(0, self.motion_seq_layout.count() - 1), row_widget)
            self._motion_rows.append(row)
        else:
            insert_index = max(0, min(int(insert_index), len(self._motion_rows)))
            self.motion_seq_layout.insertWidget(insert_index, row_widget)
            self._motion_rows.insert(insert_index, row)
        self._reindex_motion_rows()
        self._set_motion_editing(self._motion_editing)
        self._persist_current_sequence_rows()
        if (not silent) and (not force):
            self.log_signal.emit(f"[OK] 已添加轨迹: 动作{len(self._motion_rows)}")

    def clear_motion_rows(self):
        self._clear_motion_row_widgets()
        self._persist_current_sequence_rows()
        self._set_motion_editing(False)
        self.log_signal.emit("[OK] 动作序列已清空")

    def execute_motion_path(self):
        if self._motion_editing:
            self.log_signal.emit("[ERR] 请先点击“完成编辑”再执行动作序列")
            return
        if self._motion_exec_running:
            self.log_signal.emit("[INFO] 动作序列正在执行中，请先打断或等待完成")
            return
        service_name = "auto"
        use_auto_service = True
        seq = self._current_sequence()
        seq_name = seq.get("name") if isinstance(seq, dict) else "序列?"
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")
        try:
            loop_count = int((self.motion_loop_count_edit.text() or "1").strip())
        except Exception:
            self.log_signal.emit("[ERR] 循环次数格式错误")
            return
        if loop_count <= 0:
            self.log_signal.emit("[ERR] 循环次数必须>0")
            return

        items = []
        for row in self._motion_rows:
            pose = row.get("pose")
            if not pose:
                continue
            arm_type = row.get("arm_type")
            if arm_type is None:
                self.log_signal.emit(f"[ERR] {row['index_label'].text()} 未记录部位，请重新记录该轨迹")
                return
            try:
                t = float((row["duration_edit"].text() or "2.0").strip())
            except Exception:
                self.log_signal.emit(f"[ERR] {row['index_label'].text()} 时长格式错误")
                return
            if t <= 0:
                self.log_signal.emit(f"[ERR] {row['index_label'].text()} 时长必须>0")
                return
            try:
                valid_pose = self._validate_pose_for_arm_type(int(arm_type), list(pose))
            except Exception as e:
                self.log_signal.emit(f"[ERR] {row['index_label'].text()} 目标无效: {e}")
                return
            items.append({"arm_type": int(arm_type), "pose": valid_pose, "duration": t})

        if not items:
            self.log_signal.emit("[ERR] 请先添加并记录至少一组动作")
            return

        if not self._monitor_started:
            self.start_monitor()

        # 按“连续同部位”分段调用 movejbypath，支持一个序列内混合左右臂/脖子/腰部
        groups = []
        for item in items:
            if not groups or groups[-1]["arm_type"] != item["arm_type"]:
                groups.append({"arm_type": item["arm_type"], "poses": [item["pose"]], "durations": [item["duration"]]})
            else:
                groups[-1]["poses"].append(item["pose"])
                groups[-1]["durations"].append(item["duration"])

        total_tracks = len(items)

        is_async = False

        def _req_candidates(arm_type: int, poses: list, durations: list):
            total_time = float(sum(durations)) if durations else 1.0
            if total_time <= 0:
                total_time = 1.0
            path_joints = [{"joint": [float(v) for v in p]} for p in poses]
            return [
                {"path": path_joints, "time": total_time, "is_async": is_async, "arm_type": arm_type}
            ]

        def worker():
            try:
                self._clear_joint_control_trace("seq")
                seq_trace_start = time.monotonic()
                self.log_signal.emit(
                    f"[REQ] movej_by_path: sequence={seq_name}, mode={transport_mode}, service={service_name}, tracks={total_tracks}, groups={len(groups)}(按连续同部位合并), loops={loop_count}, is_async={is_async}"
                )

                def _resp_success(resp_obj) -> bool:
                    if isinstance(resp_obj, dict) and ("success" in resp_obj):
                        return bool(resp_obj.get("success"))
                    if hasattr(resp_obj, "success"):
                        try:
                            return bool(getattr(resp_obj, "success"))
                        except Exception:
                            return True
                    return True

                def _text_success(text: str) -> bool:
                    low = (text or "").lower()
                    if "success: false" in low:
                        return False
                    if "success=false" in low:
                        return False
                    if "is not available" in low or "error" in low or "exception" in low:
                        return False
                    return True

                def _pose_reached(target_map: dict, actual_map: dict, tol: float = 0.08) -> bool:
                    if not target_map or not actual_map:
                        return False
                    for joint_name, target in target_map.items():
                        if joint_name not in actual_map:
                            return False
                        try:
                            if abs(float(actual_map[joint_name]) - float(target)) > float(tol):
                                return False
                        except Exception:
                            return False
                    return True

                for loop_idx in range(1, loop_count + 1):
                    if self._motion_exec_stop_event.is_set():
                        self.log_signal.emit(f"[INFO] 动作序列已打断: {seq_name}，在第{loop_idx}轮开始前停止")
                        break

                    self.log_signal.emit(f"[INFO] 开始第 {loop_idx}/{loop_count} 轮: {seq_name}")

                    for gi, g in enumerate(groups, start=1):
                        if self._motion_exec_stop_event.is_set():
                            self.log_signal.emit(f"[INFO] 动作序列已打断: {seq_name}，在第{loop_idx}轮分组{gi}前停止")
                            break

                        arm_type = g["arm_type"]
                        poses = g["poses"]
                        durations = g["durations"]
                        total_time = float(sum(durations)) if durations else 1.0
                        service_timeout = max(60.0, total_time + 10.0)
                        desired_last = self._desired_map_for_group(arm_type, poses, durations, total_time)
                        req_candidates = _req_candidates(arm_type, poses, durations)
                        group_service = self._movej_service_for_arm_type(arm_type) if use_auto_service else service_name
                        if not group_service:
                            group_service = service_name
                        group_tag = f"loop={loop_idx},group={gi},{self._arm_type_text(arm_type)}"
                        group_offset = max(0.0, time.monotonic() - seq_trace_start)
                        sampler_stop = threading.Event()
                        sampler = threading.Thread(
                            target=self._sample_control_trace,
                            args=(
                                "seq",
                                seq_trace_start,
                                group_offset + max(0.5, total_time),
                                lambda et, _arm=arm_type, _poses=poses, _durs=durations, _off=group_offset: self._desired_map_for_group(_arm, _poses, _durs, max(0.0, et - _off)),
                                sampler_stop,
                                group_tag,
                            ),
                            daemon=True,
                        )
                        group_start = time.monotonic()
                        sampler.start()
                        self.log_signal.emit(
                            f"[INFO] 第{loop_idx}轮 分组 {gi}/{len(groups)}: {self._arm_type_text(arm_type)} points={len(poses)}, time={total_time:.3f}s, service={group_service}"
                        )
                        for ri, req in enumerate(req_candidates, start=1):
                            self.log_signal.emit(
                                f"[REQ] 第{loop_idx}轮 分组{gi} 请求体#{ri}: {self._summarize_movej_path_request(req)}"
                            )

                        done = False
                        last_err = None

                        try:
                            if transport_mode == "ros":
                                if not (self.ros and self.ros.check_connection()):
                                    raise RuntimeError("ROSBridge 未连接")
                                for ri, req in enumerate(req_candidates, start=1):
                                    try:
                                        self.log_signal.emit(
                                            f"[REQ] 第{loop_idx}轮 分组{gi} ROSBridge调用#{ri}: {group_service}, timeout={service_timeout:.1f}s"
                                        )
                                        resp = self.ros.request_service(group_service, req, timeout=service_timeout)
                                        if not _resp_success(resp):
                                            last_err = RuntimeError(f"服务返回 success=false: {json.dumps(resp, ensure_ascii=False)}")
                                            continue
                                        self.log_signal.emit(
                                            f"[OK] 第{loop_idx}轮 分组{gi} movej_by_path成功: {self._format_resp_text(resp)}"
                                        )
                                        done = True
                                        break
                                    except TimeoutError:
                                        verify_deadline = time.monotonic() + 3.0
                                        reached = False
                                        while time.monotonic() < verify_deadline:
                                            if _pose_reached(desired_last, self._get_latest_joint_state_map()):
                                                reached = True
                                                break
                                            time.sleep(0.2)
                                        if reached:
                                            self.log_signal.emit(
                                                f"[WARN] 第{loop_idx}轮 分组{gi} ROSBridge响应超时，但关节已到位，按成功处理"
                                            )
                                            done = True
                                            break
                                        last_err = TimeoutError(
                                            f"Service call timeout: {group_service} (timeout={service_timeout:.1f}s)"
                                        )
                                    except Exception as e:
                                        last_err = e

                            if (not done) and transport_mode == "ssh":
                                if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
                                    if not self._ensure_ssh():
                                        raise RuntimeError("SSH 未连接")
                                if last_err:
                                    self.log_signal.emit(f"[WARN] 第{loop_idx}轮 分组{gi} 上次调用失败: {last_err}")
                                last_text = ""
                                for ri, req in enumerate(req_candidates, start=1):
                                    req_text = json.dumps(req, ensure_ascii=False)
                                    cmd = f"rosservice call {group_service} {shlex.quote(req_text)}"
                                    self.log_signal.emit(f"[REQ] 第{loop_idx}轮 分组{gi} SSH调用#{ri}: {cmd}")
                                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                                    text = ((out or "") + "\n" + (err or "")).strip()
                                    last_text = text or "调用完成"
                                    if not _text_success(text):
                                        continue
                                    self.log_signal.emit(f"[OK] 第{loop_idx}轮 分组{gi} movej_by_path成功: {last_text}")
                                    done = True
                                    break
                                if (not done) and last_text:
                                    last_err = RuntimeError(last_text)
                        finally:
                            sampler_stop.set()
                            try:
                                sampler.join(timeout=1.0)
                            except Exception:
                                pass
                            self._append_joint_control_trace_sample(
                                "seq",
                                time.monotonic() - seq_trace_start,
                                desired_last,
                                self._get_latest_joint_state_map(),
                                tag=group_tag,
                            )
                            spent = time.monotonic() - group_start
                            self.log_signal.emit(f"[INFO] 第{loop_idx}轮 分组{gi} 控制采样结束，耗时 {spent:.2f}s")

                        if not done:
                            if transport_mode == "ros":
                                raise RuntimeError("ROSBridge 调用失败")
                            if transport_mode == "ssh":
                                raise RuntimeError("SSH 调用失败")
                            if last_err and "/rosapi/service_type" in str(last_err):
                                raise RuntimeError(
                                    f"分组{gi}调用失败: {last_err}；提示: ROSBridge已连通但rosapi不可用/超时，请在机器人侧确认rosapi已启动，或连接SSH后走兜底调用"
                                )
                            raise RuntimeError(f"分组{gi}调用失败: {last_err}")

                    if self._motion_exec_stop_event.is_set():
                        break

                if self._motion_exec_stop_event.is_set():
                    self.log_signal.emit(f"[INFO] 动作序列执行已打断: {seq_name}")
                else:
                    self.log_signal.emit(f"[OK] 动作序列执行完成: {seq_name}")
                self.log_signal.emit("[INFO] 已记录动作序列控制轨迹，可点击 控制Plot 查看")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 执行动作序列失败: {e}")
            finally:
                self.motion_exec_state_signal.emit(False)
                self._motion_exec_stop_event.clear()

        self._motion_exec_stop_event.clear()
        self.motion_exec_state_signal.emit(True)
        self._run_async(worker)

    def enter_teach_mode(self):
        arm_type = self._current_teach_arm_type()
        part = self.joint_ctrl_part.currentText().strip()
        service_used = None
        req_payload = {"arm_type": arm_type}
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

        def worker():
            try:
                self.log_signal.emit(f"[REQ] enter_teach_mode({transport_mode}): part={part}, payload={json.dumps(req_payload, ensure_ascii=False)}")
                if transport_mode == "ros":
                    if not (self.ros and self.ros.check_connection()):
                        raise RuntimeError("ROSBridge 未连接")
                    last_err = None
                    for srv in self._teach_service_candidates("enter"):
                        try:
                            resp = self.ros.request_service(srv, req_payload)
                            service_used = srv
                            self.log_signal.emit(f"[OK] 已进入示教模式: {part}(arm_type={arm_type}) [{service_used}] -> {json.dumps(resp, ensure_ascii=False)}")
                            return
                        except Exception as e:
                            last_err = e
                    raise RuntimeError(last_err or "示教服务不可用")

                if not self._ensure_ssh():
                    return
                self.log_signal.emit("[INFO] 使用 SSH(小脑) 进入示教模式")
                last_msg = ""
                for srv in self._teach_service_candidates("enter"):
                    cmd = f"rosservice call {srv} \"arm_type: {arm_type}\""
                    self.log_signal.emit(f"[REQ] {cmd}")
                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                    text = ((out or "") + "\n" + (err or "")).strip()
                    last_msg = text or "调用完成"
                    if "is not available" in (text or ""):
                        continue
                    service_used = srv
                    self.log_signal.emit(f"[OK] 已进入示教模式: {part}(arm_type={arm_type}) [{service_used}] -> {last_msg}")
                    return
                raise RuntimeError(last_msg or "示教服务不可用")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 进入示教模式失败: {part}(arm_type={arm_type}) -> {e}")

        self._run_async(worker)

    def exit_teach_mode(self):
        arm_type = self._current_teach_arm_type()
        part = self.joint_ctrl_part.currentText().strip()
        service_used = None
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

        def worker():
            try:
                if transport_mode == "ros":
                    if not (self.ros and self.ros.check_connection()):
                        raise RuntimeError("ROSBridge 未连接")
                    last_err = None
                    for srv in self._teach_service_candidates("exit"):
                        try:
                            resp = self.ros.request_service(srv, {"arm_type": arm_type})
                            service_used = srv
                            self.log_signal.emit(f"[OK] 已退出示教模式: {part}(arm_type={arm_type}) [{service_used}] -> {json.dumps(resp, ensure_ascii=False)}")
                            return
                        except Exception as e:
                            last_err = e
                    raise RuntimeError(last_err or "示教服务不可用")

                if not self._ensure_ssh():
                    return
                self.log_signal.emit("[INFO] 使用 SSH(小脑) 退出示教模式")
                last_msg = ""
                for srv in self._teach_service_candidates("exit"):
                    cmd = f"rosservice call {srv} \"arm_type: {arm_type}\""
                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                    text = ((out or "") + "\n" + (err or "")).strip()
                    last_msg = text or "调用完成"
                    if "is not available" in (text or ""):
                        continue
                    service_used = srv
                    self.log_signal.emit(f"[OK] 已退出示教模式: {part}(arm_type={arm_type}) [{service_used}] -> {last_msg}")
                    return
                raise RuntimeError(last_msg or "示教服务不可用")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 退出示教模式失败: {part}(arm_type={arm_type}) -> {e}")

        self._run_async(worker)

    def connect_ros(self):
        if self._ros_connecting:
            self.log_signal.emit("[INFO] ROSBridge 正在连接中，请勿重复点击")
            return
        op_seq = self._next_ros_op_seq()
        self.connect_btn_signal.emit("ros", False, "连接中...")

        def worker():
            new_client = None
            try:
                old_client = self.ros
                self.ros = None
                self._disconnect_joint_monitor_client()
                self._disconnect_robot_status_monitor_client()
                if old_client:
                    try:
                        old_client.disconnect()
                    except Exception:
                        pass

                new_client = BridgeClient(self.ros_host.text().strip(), int(self.ros_port.text().strip()))
                ok = new_client.connect()

                if not self._is_current_ros_op_seq(op_seq):
                    try:
                        new_client.disconnect()
                    except Exception:
                        pass
                    return

                if ok:
                    self.ros = new_client
                    self.status_signal.emit("ros", "已连接")
                    try:
                        self._subscribe_battery_info()
                    except Exception as sub_e:
                        self.log_signal.emit(f"[WARN] ROSBridge 已连接，但部分状态话题订阅失败: {sub_e}")
                    if self._monitor_started:
                        self._restart_joint_monitor_subscription("ROSBridge重连")
                    self._restart_robot_status_subscription("ROSBridge重连")
                    QTimer.singleShot(0, self._poll_safety_lock_state)
                    self.log_signal.emit("[OK] ROSBridge 已连接")
                else:
                    self.ros = None
                    self.status_signal.emit("ros", "未连接")
                    self.log_signal.emit("[ERR] ROSBridge 连接失败")
            except Exception as e:
                if self._is_current_ros_op_seq(op_seq):
                    self.ros = None
                self.status_signal.emit("ros", "未连接")
                self.log_signal.emit(f"[ERR] ROSBridge: {e}")
            finally:
                if self._is_current_ros_op_seq(op_seq):
                    self.connect_btn_signal.emit("ros", True, "连接 ROSBridge")
        self._run_async(worker)

    def disconnect_ros(self):
        if self._ros_connecting:
            self.log_signal.emit("[INFO] ROSBridge 正在连接中，请稍后再断开")
            return
        op_seq = self._next_ros_op_seq()
        self.connect_btn_signal.emit("ros", False, "断开中...")

        def worker():
            try:
                old_client = self.ros
                self.ros = None
                self._disconnect_joint_monitor_client()
                self._disconnect_robot_status_monitor_client()
                if old_client:
                    old_client.disconnect()

                if not self._is_current_ros_op_seq(op_seq):
                    return

                if old_client:
                    self._monitor_started = False
                    self._cmd_echo_topic = None
                    self.stop_maintenance_pressure_monitor(emit_log=False)
                    self.command_btn_signal.emit(False)
                    self.status_signal.emit("ros", "未连接")
                    self.safety_lock_view_signal.emit("安全保护: 未连接", "color: #8b949e; font-weight: 600;")
                    self.battery_signal.emit("-")
                    self.robot_orin_status_signal.emit("-")
                    self.robot_pico_status_signal.emit("-")
                    self.robot_orin_latency_signal.emit("-")
                    self.robot_pico_latency_signal.emit("-")
                    self.robot_state_info_signal.emit("-")
                    self._finish_vision_sampling()
                    self._reset_joint_fields()
                    self._refresh_robot_status_view()
                    self.log_signal.emit("[OK] ROSBridge 已断开")
                else:
                    self.status_signal.emit("ros", "未连接")
                    self.stop_maintenance_pressure_monitor(emit_log=False)
                    self._finish_vision_sampling()
                    self._refresh_robot_status_view()
                    self.log_signal.emit("[INFO] ROSBridge 当前未连接")
            except Exception as e:
                self.log_signal.emit(f"[ERR] ROSBridge 断开失败: {e}")
            finally:
                if self._is_current_ros_op_seq(op_seq):
                    self.connect_btn_signal.emit("ros", True, "连接 ROSBridge")

        self._run_async(worker)

    def connect_ssh(self):
        if self._ssh_connecting:
            self.log_signal.emit("[INFO] SSH 正在连接中，请勿重复点击")
            return
        self.connect_btn_signal.emit("ssh", False, "连接中...")

        def worker():
            try:
                if self.ssh:
                    try:
                        self.ssh.close()
                    except Exception:
                        pass
                small_host = self.ssh_host.text().strip()
                small_port = int(self.ssh_port.text().strip())
                small_user = self.ssh_user.text().strip()
                small_pwd = self.ssh_pwd.text()

                try:
                    self.ssh.connect(
                        host=small_host,
                        port=small_port,
                        username=small_user,
                        password=small_pwd,
                    )
                    self._ssh_small_password = small_pwd
                    self.status_signal.emit("ssh", "已连接(直连)")
                    self.log_signal.emit("[OK] SSH/SFTP(小脑) 已直连")
                    self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
                except Exception as direct_err:
                    big_connected = bool(self.ssh_big and self.ssh_big.ssh and self.ssh_big.sftp)
                    if not big_connected:
                        raise direct_err

                    self.log_signal.emit("[WARN] 小脑直连失败，尝试通过大脑跳转连接...")
                    self.ssh.connect_via_jump(
                        jump_client=self.ssh_big,
                        host=small_host,
                        port=small_port,
                        username=small_user,
                        password=small_pwd,
                    )
                    self._ssh_small_password = small_pwd
                    self.status_signal.emit("ssh", "已连接(经大脑跳转)")
                    self.log_signal.emit("[OK] SSH/SFTP(小脑) 已通过大脑跳转连接")
                    self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
            except Exception as e:
                self.status_signal.emit("ssh", "未连接")
                self.log_signal.emit(f"[ERR] SSH 连接失败: {e}")
            finally:
                self.connect_btn_signal.emit("ssh", True, "连接 SSH(小脑)")
        self._run_async(worker)

    def disconnect_ssh(self):
        if self._ssh_connecting:
            self.log_signal.emit("[INFO] SSH(小脑) 正在连接中，请稍后再断开")
            return

        def worker():
            try:
                if self.ssh:
                    self.ssh.close()
                    self._ssh_small_password = ""
                    self.status_signal.emit("ssh", "未连接")
                    self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
                    self.log_signal.emit("[OK] SSH/SFTP(小脑) 已断开")
                else:
                    self._ssh_small_password = ""
                    self.status_signal.emit("ssh", "未连接")
                    self.log_signal.emit("[INFO] SSH(小脑) 当前未连接")
            except Exception as e:
                self.log_signal.emit(f"[ERR] SSH(小脑) 断开失败: {e}")

        self._run_async(worker)

    def connect_ssh_big(self):
        if self._ssh_big_connecting:
            self.log_signal.emit("[INFO] SSH(大脑) 正在连接中，请勿重复点击")
            return
        self.connect_btn_signal.emit("ssh_big", False, "连接中...")

        def worker():
            try:
                if self.ssh_big:
                    try:
                        self.ssh_big.close()
                    except Exception:
                        pass
                self.ssh_big.connect(
                    host=self.ssh_big_host.text().strip(),
                    port=int(self.ssh_big_port.text().strip()),
                    username=self.ssh_big_user.text().strip(),
                    password=self.ssh_big_pwd.text(),
                )
                self.status_signal.emit("ssh_big", "已连接")
                self.log_signal.emit("[OK] SSH/SFTP(大脑) 已连接")
                self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
            except Exception as e:
                self.status_signal.emit("ssh_big", "未连接")
                self.log_signal.emit(f"[ERR] SSH(大脑) 连接失败: {e}")
            finally:
                self.connect_btn_signal.emit("ssh_big", True, "连接 SSH(大脑)")
        self._run_async(worker)

    def _build_x11_terminal_command(self, ssh_cmd: list[str]):
        bundled_xterm = ""
        try:
            if getattr(sys, "frozen", False):
                candidate = os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "tools", "bin", "xterm")
                if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                    bundled_xterm = candidate
        except Exception:
            bundled_xterm = ""

        xterm = bundled_xterm or shutil.which("xterm")
        if xterm:
            return [xterm, "-hold", "-e", *ssh_cmd], "xterm"

        gnome_terminal = shutil.which("gnome-terminal")
        if gnome_terminal:
            return [gnome_terminal, "--", *ssh_cmd], "gnome-terminal"

        terminal = shutil.which("x-terminal-emulator")
        if terminal:
            resolved = os.path.realpath(terminal)
            if os.path.basename(resolved).startswith("gnome-terminal") and gnome_terminal:
                return [gnome_terminal, "--", *ssh_cmd], "gnome-terminal"

            shell_cmd = " ".join(shlex.quote(part) for part in ssh_cmd)
            shell_cmd += "; rc=$?; echo; echo \"[X11 会话结束] exit_code=$rc\"; echo \"按回车关闭窗口...\"; read _"
            return [terminal, "-e", "bash", "-lc", shell_cmd], os.path.basename(resolved or terminal)

        raise RuntimeError("未找到可用终端，请先安装 xterm 或 gnome-terminal")

    def _x11_launcher_log_path(self, target_name: str, terminal_name: str) -> str:
        safe_target = re.sub(r"[^0-9A-Za-z._-]+", "_", str(target_name or "x11"))
        safe_terminal = re.sub(r"[^0-9A-Za-z._-]+", "_", str(terminal_name or "terminal"))
        base_dir = os.path.join(os.path.expanduser("~"), ".local", "state", "humanoid-robot-delivery-toolchain")
        os.makedirs(base_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return os.path.join(base_dir, f"x11_{safe_target}_{safe_terminal}_{stamp}.log")

    def _read_tail_text(self, file_path: str, max_lines: int = 60) -> str:
        try:
            with open(file_path, "r", encoding="utf-8", errors="replace") as handle:
                lines = handle.readlines()
        except Exception:
            return ""
        text = "".join(lines[-max_lines:]).strip()
        return text

    def _check_x11_terminal_launch(self, proc: subprocess.Popen, terminal_name: str, log_path: str, target_name: str):
        try:
            exit_code = proc.poll()
        except Exception:
            exit_code = None
        if exit_code is None:
            return

        detail = self._read_tail_text(log_path)
        msg = f"{terminal_name} 启动 {target_name} X11 会话失败，exit_code={exit_code}"
        if detail:
            self.log_signal.emit(f"[ERR] {msg}\n{detail}")
        else:
            self.log_signal.emit(f"[ERR] {msg}")
        QMessageBox.warning(
            self,
            "X11 启动失败",
            f"{msg}\n\n日志文件:\n{log_path}" + (f"\n\n最后输出:\n{detail}" if detail else "")
        )

    def _launch_x11_remote_gui(self, host: str, port: str, user: str, target_name: str, remote_cmd: str = ""):
        if sys.platform != "linux":
            QMessageBox.warning(self, "不支持", "该入口当前仅支持 Ubuntu/Linux。")
            return

        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            QMessageBox.warning(self, "本地图形环境缺失", "当前会话没有 DISPLAY/WAYLAND_DISPLAY，无法接收远端 GUI。")
            return

        if not host or not user:
            QMessageBox.warning(self, "参数不完整", f"请先填写 {target_name} 的 Host 和 User。")
            return

        ssh_cmd = ["ssh", "-C", "-Y"]
        if port:
            ssh_cmd.extend(["-p", port])
        ssh_cmd.append(f"{user}@{host}")
        if remote_cmd:
            remote_shell = remote_cmd + '; rc=$?; echo; echo "[远端命令结束] exit_code=$rc"; exit "$rc"'
            ssh_cmd.append(f"bash -lc {shlex.quote(remote_shell)}")

        try:
            term_cmd, terminal_name = self._build_x11_terminal_command(ssh_cmd)
            log_path = self._x11_launcher_log_path(target_name, terminal_name)
            with open(log_path, "ab") as log_handle:
                proc = subprocess.Popen(
                    term_cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    close_fds=True,
                    start_new_session=True,
                )
            if remote_cmd:
                self.log_signal.emit(f"[INFO] 已通过 {terminal_name} 启动 {target_name} X11 会话: {remote_cmd}")
            else:
                self.log_signal.emit(f"[INFO] 已通过 {terminal_name} 启动 {target_name} X11 shell")
            self.log_signal.emit(f"[INFO] X11 启动日志: {log_path}")
            if terminal_name != "xterm":
                self.log_signal.emit("[INFO] 当前系统未检测到 xterm，已回退到较重的终端实现；如仍卡顿，建议安装 xterm 后重试")
            QTimer.singleShot(
                1500,
                lambda proc=proc, terminal_name=terminal_name, log_path=log_path, target_name=target_name: self._check_x11_terminal_launch(
                    proc,
                    terminal_name,
                    log_path,
                    target_name,
                ),
            )
        except Exception as e:
            QMessageBox.warning(self, "启动失败", f"无法启动 Ubuntu X11 会话: {e}")
            self.log_signal.emit(f"[ERR] Ubuntu X11 会话启动失败: {e}")

    def _brainsense_remote_launch_command(self) -> str:
        candidates = [
            "$HOME/test-tools/BrainSense/scripts",
            "$HOME/test-tools/BrainSense",
            "$HOME/BrainSense/scripts",
            "$HOME/source/test-tools/BrainSense/scripts",
            "$HOME/navi_project/test-tools/BrainSense/scripts",
        ]
        candidate_expr = " ".join(f'"{path}"' for path in candidates)
        tried_text = " | ".join(candidates)
        return (
            'brainsense_dir=""; '
            f'for candidate in {candidate_expr}; do '
            'if [ -f "$candidate/launch.sh" ]; then brainsense_dir="$candidate"; break; fi; '
            'done; '
            'if [ -z "$brainsense_dir" ]; then '
            'found="$(find "$HOME" -maxdepth 6 -type f -path "*/BrainSense/scripts/launch.sh" 2>/dev/null | head -n 1)"; '
            'if [ -n "$found" ]; then brainsense_dir="$(dirname "$found")"; fi; '
            'fi; '
            'if [ -z "$brainsense_dir" ]; then '
            f'echo "[ERR] 未找到 BrainSense launch.sh，已尝试: {tried_text}"; '
            'exit 1; '
            'fi; '
            'cd "$brainsense_dir" && chmod +x ./launch.sh && echo "[INFO] BrainSense目录: $brainsense_dir" && ./launch.sh'
        )

    def _launch_brainsense_gui(self, emit_line):
        host = self.ssh_big_host.text().strip() if hasattr(self, "ssh_big_host") else ""
        port = self.ssh_big_port.text().strip() if hasattr(self, "ssh_big_port") else ""
        user = self.ssh_big_user.text().strip() if hasattr(self, "ssh_big_user") else ""
        remote_cmd = self._brainsense_remote_launch_command()
        emit_line(
            f"[INFO] 准备启动 BrainSense GUI: {user or '-'}@{host or '-'}:{port or '22'} 自动探测 BrainSense 目录并执行 launch.sh"
        )
        self._launch_x11_remote_gui(host, port, user, "大脑", remote_cmd)

    def launch_factory_brainsense_gui(self):
        self._launch_brainsense_gui(self._emit_factory_test)

    def launch_maintenance_brainsense_gui(self):
        self._launch_brainsense_gui(self._emit_maintenance)

    def _run_big_brain_middleware_compose(self, action: str, emit_line=None):
        action_name = str(action or "").strip().lower()
        if action_name not in ("stop", "start"):
            raise RuntimeError(f"不支持的中间件操作: {action}")
        log = emit_line or self._emit_factory_test

        compose_cmds = [
            f"cd ~/navi_project && docker-compose {action_name}",
            f"cd ~/navi_project && docker compose {action_name}",
        ]
        last_error = ""

        for index, cmd in enumerate(compose_cmds, start=1):
            log(f"[REQ] 尝试{index}: {cmd}")
            try:
                out, err, exit_code = self._run_bash_with_exit_code(self.ssh_big, cmd)
            except Exception as e:
                last_error = str(e)
                log(f"[WARN] 尝试{index}执行异常: {e}")
                continue

            out_text = (out or "").strip()
            err_text = (err or "").strip()
            merged = "\n".join(part for part in [out_text, err_text] if part).strip()
            if exit_code == 0:
                if merged:
                    log(merged)
                return merged or "已执行完成"

            last_error = merged or f"exit_code={exit_code}"
            log(f"[WARN] 尝试{index}失败: {last_error}")

        raise RuntimeError(last_error or f"docker compose {action_name} 执行失败")

    def _control_big_brain_middleware(self, action: str, emit_line):
        if not self._ensure_ssh_big():
            emit_line("[ERR] 请先连接 SSH(大脑)")
            return

        action_name = str(action or "").strip().lower()
        action_label = "关闭" if action_name == "stop" else "开启"

        def worker():
            try:
                emit_line(f"=== {action_label}大脑中间件 ===")
                detail = self._run_big_brain_middleware_compose(action_name, emit_line=emit_line)
                emit_line(f"[OK] {action_label}大脑中间件完成")
                if detail and detail != "已执行完成":
                    emit_line(f"[INFO] {action_label}结果:\n{detail}")
            except Exception as e:
                emit_line(f"[ERR] {action_label}大脑中间件失败: {e}")

        self._run_async(worker)

    def _factory_control_big_brain_middleware(self, action: str):
        self._control_big_brain_middleware(action, self._emit_factory_test)

    def factory_stop_big_brain_middleware(self):
        self._factory_control_big_brain_middleware("stop")

    def factory_start_big_brain_middleware(self):
        self._factory_control_big_brain_middleware("start")

    def maintenance_stop_big_brain_middleware(self):
        self._control_big_brain_middleware("stop", self._emit_maintenance)

    def maintenance_start_big_brain_middleware(self):
        self._control_big_brain_middleware("start", self._emit_maintenance)

    def disconnect_ssh_big(self):
        if self._ssh_big_connecting:
            self.log_signal.emit("[INFO] SSH(大脑) 正在连接中，请稍后再断开")
            return

        def worker():
            try:
                if self.ssh_big:
                    self.ssh_big.close()
                    self.status_signal.emit("ssh_big", "未连接")
                    self._refresh_version_snapshot_async(emit_lines=False, clear_lines=False)
                    self.log_signal.emit("[OK] SSH/SFTP(大脑) 已断开")
                else:
                    self.status_signal.emit("ssh_big", "未连接")
                    self.log_signal.emit("[INFO] SSH(大脑) 当前未连接")
            except Exception as e:
                self.log_signal.emit(f"[ERR] SSH(大脑) 断开失败: {e}")

        self._run_async(worker)

    def _refresh_version_snapshot_async(self, emit_lines: bool = False, clear_lines: bool = False):
        with self._version_refresh_lock:
            if self._version_refreshing:
                return
            self._version_refreshing = True

        def _emit_line(text: str):
            self.log_signal.emit(text)
            if emit_lines:
                self.version_item_signal.emit(text)

        def worker():
            version_data = self._default_robot_version_summary()
            system_data = self._default_robot_system_info()
            try:
                if clear_lines and emit_lines:
                    self.version_clear_signal.emit()

                small_connected = bool(self.ssh and self.ssh.ssh and self.ssh.sftp)
                big_connected = bool(self.ssh_big and self.ssh_big.ssh and self.ssh_big.sftp)

                if not small_connected and not big_connected:
                    if emit_lines:
                        _emit_line("[ERR] 请至少连接一个 SSH（小脑/大脑）")
                    self._emit_robot_version_snapshot(version_data, system_data)
                    return

                if small_connected:
                    if emit_lines:
                        _emit_line("=== 小脑(192.168.217.66) 版本检查 ===")

                    out, err = self._run_bash(self.ssh, "if command -v lsb_release >/dev/null 2>&1; then lsb_release -ds; else source /etc/os-release && echo ${PRETTY_NAME:-Unknown}; fi")
                    system_data["small_ubuntu"] = (out.strip() or err.strip() or "Unknown")
                    if emit_lines:
                        _emit_line(f"[小脑 Ubuntu] {system_data['small_ubuntu']}")

                    out, err = self._run_bash(self.ssh, "echo -n ${ROBOT_TYPE}")
                    robot_type = (out.strip() or err.strip() or "Unknown")
                    version_data["robot_version"] = robot_type
                    if emit_lines:
                        _emit_line(f"[小脑 机器人型号] {robot_type}")

                    out, err = self._run_bash(self.ssh, "echo -n ${ZJ_VERSION}")
                    version_data["software_version"] = (out.strip() or err.strip() or "-")
                    if emit_lines:
                        _emit_line(f"[小脑 软件大版本] {version_data['software_version']}")

                    sdk_cmd = "SDK_CANDIDATES=($HOME/CodeFiles/sdk/zjhrobotsdkreleaseproject /home/nav01/CodeFiles/sdk/zjhrobotsdkreleaseproject /home/robot/CodeFiles/sdk/zjhrobotsdkreleaseproject); FOUND=''; for d in ${SDK_CANDIDATES[@]}; do if [ -d \"$d\" ]; then FOUND=\"$d\"; break; fi; done; if [ -n \"$FOUND\" ]; then cd \"$FOUND\" && ./runcmd.sh version; else echo 'SDK目录不存在'; fi"
                    out, err = self._run_sudo_bash(self.ssh, self.ssh_pwd.text(), sdk_cmd)
                    sdk_msg = (out.strip() or err.strip() or "获取失败")
                    version_data["embedded_version"] = self._extract_sdk_version(sdk_msg)
                    if emit_lines:
                        _emit_line(f"[小脑 SDK] {version_data['embedded_version']}")

                    out, err = self._run_bash(self.ssh, "rosservice call /zj_humanoid/upperlimb/version")
                    upper_text = "\n".join([x for x in [out, err] if x])
                    version_data["upperlimb_version"] = self._extract_upperlimb_version(upper_text)
                    if emit_lines:
                        _emit_line(f"[小脑 上肢] {version_data['upperlimb_version']}")

                    if self._factory_model_has_lowerlimb(robot_type or self._factory_current_model()):
                        out, err = self._run_sudo_bash(self.ssh, self.ssh_pwd.text(), "apt list 2>/dev/null | grep legged || true")
                        lower_raw = (out.strip() or err.strip() or "未匹配到 legged 包")
                        version_data["show_lowerlimb"] = True
                        version_data["lowerlimb_version"] = self._extract_apt_package_version(lower_raw)
                        if emit_lines:
                            _emit_line(f"[小脑 下肢] {version_data['lowerlimb_version']}")

                    sys_cmd = "LOGICAL_CORES=$(grep -c '^processor' /proc/cpuinfo 2>/dev/null || echo Unknown); PHYSICAL_CORES=$(grep -E '^physical id|^core id' /proc/cpuinfo 2>/dev/null | paste -d' ' - - | sort -u | wc -l || echo Unknown); MEM=$(free -h | awk '/Mem:/ {print \"Total: \"$2\", Used: \"$3\", Free: \"$4}'); echo CPU Logical Cores: $LOGICAL_CORES; echo CPU Physical Cores: $PHYSICAL_CORES; echo Memory: $MEM"
                    out, _ = self._run_bash(self.ssh, sys_cmd)
                    system_data["small_system"] = (out or "").strip().replace(chr(10), ' | ') or "-"
                    if emit_lines:
                        _emit_line(f"[小脑 系统] {system_data['small_system']}")

                if big_connected:
                    if emit_lines:
                        _emit_line("=== 大脑(192.168.217.100) 版本检查 ===")

                    local_jetpack = self._local_jetpack_script()
                    remote_user = self.ssh_big_user.text().strip()
                    remote_jetpack = f"/home/{remote_user}/jetpack_check.sh"
                    script_uploaded = False
                    if os.path.isfile(local_jetpack):
                        try:
                            self.ssh_big.upload(local_jetpack, remote_jetpack)
                            script_uploaded = True
                            if emit_lines:
                                _emit_line(f"[大脑] 已上传 jetpack_check.sh -> {remote_jetpack}")
                        except Exception as e:
                            if emit_lines:
                                _emit_line(f"[WARN] 上传 jetpack_check.sh 失败: {e}")
                    elif emit_lines:
                        _emit_line(f"[WARN] 本地未找到脚本: {local_jetpack}")

                    if emit_lines and not script_uploaded:
                        _emit_line("[INFO] JetPack检查将使用远端内联命令（无需本地脚本）")

                    jp_cmd = (
                        f"if [ -f {shlex.quote(remote_jetpack)} ]; then "
                        f"chmod +x {shlex.quote(remote_jetpack)} && {shlex.quote(remote_jetpack)}; "
                        f"else {self._inline_jetpack_check_cmd()}; fi"
                    )
                    out, err = self._run_sudo_bash(self.ssh_big, self.ssh_big_pwd.text(), jp_cmd)
                    system_data["big_jetpack"] = (out.strip() or err.strip() or "检查失败")
                    if emit_lines:
                        _emit_line(f"[大脑 JetPack] {system_data['big_jetpack']}")

                    out, err = self._run_bash(self.ssh_big, "if command -v lsb_release >/dev/null 2>&1; then lsb_release -ds; else source /etc/os-release && echo ${PRETTY_NAME:-Unknown}; fi")
                    system_data["big_ubuntu"] = (out.strip() or err.strip() or "Unknown")
                    if emit_lines:
                        _emit_line(f"[大脑 Ubuntu] {system_data['big_ubuntu']}")

                self._emit_robot_version_snapshot(version_data, system_data)
            except Exception as e:
                if emit_lines:
                    _emit_line(f"[ERR] 版本检查失败: {e}")
                else:
                    self.log_signal.emit(f"[ERR] 自动刷新版本信息失败: {e}")
            finally:
                with self._version_refresh_lock:
                    self._version_refreshing = False

        self._run_async(worker)

    def check_version(self):
        self._refresh_version_snapshot_async(emit_lines=True, clear_lines=True)

    def check_peripherals(self):
        def worker():
            try:
                self.peripheral_clear_signal.emit()
                self._emit_peripheral("=== 外设状态检查(大脑) ===")

                if not (self.ssh_big and self.ssh_big.ssh and self.ssh_big.sftp):
                    self._emit_peripheral("[ERR] 请先连接 SSH(大脑)")
                    return

                devices = {
                    "camera": ["realsense", "camera", "uvc"],
                    "lidar": ["rplidar", "ydlidar", "velodyne"],
                    "imu": ["imu", "mpu"],
                    "radar": ["radar", "mmwave"],
                }

                self._emit_peripheral("\n=== USB设备检查 ===")
                try:
                    usb_out, usb_err = self._run_bash(self.ssh_big, "lsusb")
                    if usb_err and not usb_out:
                        self._emit_peripheral(f"[WARN] lsusb 输出错误: {usb_err.strip()}")
                    found = False
                    for line in (usb_out or "").splitlines():
                        low = line.lower()
                        for dev_type, keywords in devices.items():
                            if any(kw in low for kw in keywords):
                                self._emit_peripheral(f"[OK] [{dev_type}] {line}")
                                found = True
                                break
                    if not found:
                        self._emit_peripheral("[WARN] 未匹配到常见外设关键字")
                except Exception as e:
                    self._emit_peripheral(f"[ERR] USB检查失败: {e}")

                self._emit_peripheral("\n=== 串口设备检查 ===")
                try:
                    serial_cmd = "ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* /dev/ttyS* 2>/dev/null || true"
                    serial_out, _ = self._run_bash(self.ssh_big, serial_cmd)
                    serial_nodes = sorted({x.strip() for x in (serial_out or "").split() if x.strip().startswith('/dev/')})
                    if serial_nodes:
                        for node in serial_nodes:
                            self._emit_peripheral(f"[OK] {node}")
                    else:
                        self._emit_peripheral("[WARN] 未检测到常见串口设备")
                except Exception as e:
                    self._emit_peripheral(f"[ERR] 串口检查失败: {e}")

                self._emit_peripheral("\n=== 网络接口检查 ===")
                try:
                    net_cmd = (
                        "for ifn in $(ip -o link show | awk -F': ' '{print $2}' | cut -d@ -f1); do "
                        "  [ -z \"$ifn\" ] && continue; "
                        "  if [[ \"$ifn\" =~ ^(lo|docker[0-9]*|br-.*|veth.*|virbr.*|cni.*|flannel.*|tunl.*|vxlan.*|kube.*|zt.*|tailscale.*|dummy.*|ifb.*|tap.*|vnet.*)$ ]]; then continue; fi; "
                        "  state=$(ip -o link show dev \"$ifn\" | sed -n 's/.*state \\([A-Z]\\+\\).*/\\1/p'); "
                        "  [ \"$state\" != \"UP\" ] && continue; "
                        "  ips=$(ip -o -4 addr show dev \"$ifn\" | awk '{print $4}' | paste -sd ',' -); "
                        "  [ -z \"$ips\" ] && ips='-'; "
                        "  echo \"$ifn|$ips\"; "
                        "done"
                    )
                    net_out, net_err = self._run_bash(self.ssh_big, net_cmd)
                    if net_err and not net_out:
                        self._emit_peripheral(f"[WARN] 网络接口检查输出错误: {net_err.strip()}")
                    ups = []
                    for line in (net_out or "").splitlines():
                        line = line.strip()
                        if not line or "|" not in line:
                            continue
                        iface, ips = line.split("|", 1)
                        ups.append((iface.strip(), ips.strip()))
                    if ups:
                        for iface, ips in ups:
                            self._emit_peripheral(f"[OK] {iface} | IPv4: {ips}")
                    else:
                        self._emit_peripheral("[WARN] 未检测到 UP 状态的非容器网卡")
                except Exception as e:
                    self._emit_peripheral(f"[ERR] 网络接口检查失败: {e}")

                self._emit_peripheral("\n=== 相机设备检查 ===")
                try:
                    video_out, _ = self._run_bash(self.ssh_big, "ls /dev/video* 2>/dev/null || true")
                    videos = [x.strip() for x in (video_out or "").split() if x.strip().startswith('/dev/video')]
                    if videos:
                        self._emit_peripheral(f"[OK] Video设备: {' '.join(videos)}")
                    else:
                        self._emit_peripheral("[WARN] 未检测到 /dev/video* 设备")
                except Exception as e:
                    self._emit_peripheral(f"[ERR] 相机设备检查失败: {e}")

                try:
                    rs_cmd = "if command -v rs-enumerate-devices >/dev/null 2>&1; then rs-enumerate-devices | grep 'Device Name' || true; fi"
                    rs_out, _ = self._run_bash(self.ssh_big, rs_cmd)
                    rs_out = (rs_out or "").strip()
                    if rs_out:
                        self._emit_peripheral(f"[OK] RealSense: {rs_out.replace(chr(10), ' | ')}")
                    else:
                        self._emit_peripheral("[INFO] 未检测到 RealSense 设备或未安装 rs-enumerate-devices")
                except Exception as e:
                    self._emit_peripheral(f"[ERR] RealSense检查失败: {e}")

                self._emit_peripheral("\n[OK] 外设状态检查完成")
            except Exception as e:
                self._emit_peripheral(f"[ERR] 外设状态检查失败: {e}")

        self._run_async(worker)

    def upload_file(self):
        if not self._ensure_smallbrain_feature("small_brain_upload_file"):
            return
        client, target_name = self._get_transfer_client()
        if not client:
            return
        local_files, _ = QFileDialog.getOpenFileNames(self, "选择本地文件")
        if not local_files:
            return

        default_remote = f"/tmp/{os.path.basename(local_files[0])}"
        remote_dir = self._browse_sftp_path_dialog(
            client,
            target_name,
            "选择上传目标目录",
            start_path=self._guess_remote_browse_start_dir(default_remote),
            select_kind="dir",
        )
        if not remote_dir:
            return
        upload_targets = []
        for local_file in local_files:
            remote_path = f"{remote_dir.rstrip('/')}/{os.path.basename(local_file)}" if remote_dir != "/" else f"/{os.path.basename(local_file)}"
            upload_targets.append((local_file, remote_path))

        def worker():
            failed = []
            for local_file, remote_path in upload_targets:
                try:
                    client.upload(local_file, remote_path)
                    self.log_signal.emit(f"[OK] 上传成功({target_name}): {local_file} -> {remote_path}")
                except Exception as e:
                    failed.append(f"{local_file}: {e}")
                    self.log_signal.emit(f"[ERR] 上传失败({target_name}): {local_file} -> {remote_path} | {e}")
            if len(upload_targets) > 1:
                ok_count = len(upload_targets) - len(failed)
                if failed:
                    self.log_signal.emit(f"[WARN] 批量上传完成({target_name}): 成功 {ok_count}/{len(upload_targets)}")
                else:
                    self.log_signal.emit(f"[OK] 批量上传完成({target_name}): 共 {ok_count} 个文件")
        self._run_async(worker)

    def open_remote_file_for_edit(self):
        if not self._ensure_feature_enabled("remote_file_read"):
            return
        client, target_name = self._get_transfer_client()
        if not client:
            return

        remote_path = self._build_remote_edit_path_from_inputs()
        if not remote_path:
            self.log_signal.emit("[ERR] 请选择目录并输入文件名")
            return
        if not self._is_supported_remote_edit_file(remote_path):
            self.log_signal.emit("[ERR] 仅支持编辑常见文本文件，如 .md/.h/.hpp/.py/.cpp/.yaml/.json 等")
            return

        if hasattr(self, "btn_remote_edit_open"):
            self.btn_remote_edit_open.setEnabled(False)

        def worker():
            try:
                with client.sftp.open(remote_path, "rb") as f:
                    blob = f.read()
                if b"\x00" in blob:
                    raise RuntimeError("检测到二进制内容，无法作为文本编辑")
                try:
                    text = blob.decode("utf-8")
                except Exception:
                    text = blob.decode("gb18030")
                self.remote_edit_opened_signal.emit(target_name, remote_path, text)
            except Exception as e:
                self.remote_edit_failed_signal.emit(f"打开远程文件失败: {e}")

        self._run_async(worker)

    def reload_remote_file_for_edit(self):
        if not self._ensure_feature_enabled("remote_file_read"):
            return
        target_name = str(getattr(self, "_remote_edit_target", "") or "").strip()
        remote_path = str(getattr(self, "_remote_edit_path", "") or "").strip()
        if target_name and remote_path:
            client = self._get_transfer_client_by_name(target_name)
            if not client:
                return
            if hasattr(self, "btn_remote_edit_open"):
                self.btn_remote_edit_open.setEnabled(False)

            def worker():
                try:
                    with client.sftp.open(remote_path, "rb") as f:
                        blob = f.read()
                    if b"\x00" in blob:
                        raise RuntimeError("检测到二进制内容，无法作为文本编辑")
                    try:
                        text = blob.decode("utf-8")
                    except Exception:
                        text = blob.decode("gb18030")
                    self.remote_edit_opened_signal.emit(target_name, remote_path, text)
                except Exception as e:
                    self.remote_edit_failed_signal.emit(f"重新加载失败: {e}")

            self._run_async(worker)
            return
        self.open_remote_file_for_edit()

    def save_remote_file_from_editor(self):
        if not self._ensure_feature_enabled("remote_file_write"):
            return
        target_name = str(getattr(self, "_remote_edit_target", "") or "").strip()
        remote_path = str(getattr(self, "_remote_edit_path", "") or "").strip()
        if not target_name or not remote_path:
            self.log_signal.emit("[ERR] 请先打开远程文件后再保存")
            return
        if not self._is_supported_remote_edit_file(remote_path):
            self.log_signal.emit("[ERR] 当前文件后缀不受支持，无法保存")
            return

        client = self._get_transfer_client_by_name(target_name)
        if not client:
            return

        content = self.remote_file_editor.toPlainText() if hasattr(self, "remote_file_editor") else ""
        if hasattr(self, "btn_remote_edit_save"):
            self.btn_remote_edit_save.setEnabled(False)

        def worker():
            try:
                data = content.encode("utf-8")
                with client.sftp.open(remote_path, "wb") as f:
                    f.write(data)
                self.remote_edit_saved_signal.emit(target_name, remote_path)
            except Exception as e:
                self.remote_edit_failed_signal.emit(f"保存远程文件失败: {e}")

        self._run_async(worker)

    def undo_remote_file_edit(self):
        if hasattr(self, "remote_file_editor"):
            self.remote_file_editor.undo()

    def _on_remote_edit_opened(self, target_name: str, remote_path: str, content: str):
        self._remote_edit_target = str(target_name or "")
        self._remote_edit_path = str(remote_path or "")
        self._fill_remote_edit_inputs_from_path(self._remote_edit_path)
        if hasattr(self, "remote_file_editor"):
            self.remote_file_editor.setPlainText(content)
            self.remote_file_editor.document().setModified(False)
        self._refresh_feature_control_states()
        if hasattr(self, "remote_edit_status"):
            self.remote_edit_status.setText(f"已打开: {self._remote_edit_target} | {self._remote_edit_path}")
        if hasattr(self, "btn_remote_edit_reload"):
            self.btn_remote_edit_reload.setEnabled(self._feature_enabled("remote_file_read"))
        if hasattr(self, "btn_remote_edit_save"):
            self.btn_remote_edit_save.setEnabled(self._feature_enabled("remote_file_write"))
        self.log_signal.emit(f"[OK] 已打开远程文件({self._remote_edit_target}): {self._remote_edit_path}")

    def _on_remote_edit_saved(self, target_name: str, remote_path: str):
        self._fill_remote_edit_inputs_from_path(remote_path)
        if hasattr(self, "remote_file_editor"):
            self.remote_file_editor.document().setModified(False)
        self._refresh_feature_control_states()
        if hasattr(self, "remote_edit_status"):
            self.remote_edit_status.setText(f"已保存: {target_name} | {remote_path}")
        self.log_signal.emit(f"[OK] 已保存远程文件({target_name}): {remote_path}")

    def _on_remote_edit_failed(self, msg: str):
        self._refresh_feature_control_states()
        if hasattr(self, "btn_remote_edit_save"):
            self.btn_remote_edit_save.setEnabled(self._feature_enabled("remote_file_write") and bool(getattr(self, "_remote_edit_path", "")))
        if hasattr(self, "btn_remote_edit_reload"):
            self.btn_remote_edit_reload.setEnabled(self._feature_enabled("remote_file_read") and bool(getattr(self, "_remote_edit_path", "")))
        if hasattr(self, "remote_edit_status"):
            self.remote_edit_status.setText(msg)
        self.log_signal.emit(f"[ERR] {msg}")

    def download_file(self):
        if not self._ensure_smallbrain_feature("small_brain_download_file"):
            return
        client, target_name = self._get_transfer_client()
        if not client:
            return
        remote_paths = self._browse_sftp_path_dialog(
            client,
            target_name,
            "选择下载源文件",
            start_path="/tmp",
            select_kind="file",
            multi_select=True,
        )
        if not remote_paths:
            return
        download_targets = self._choose_local_download_targets(remote_paths)
        if not download_targets:
            return

        def worker():
            failed = []
            for remote_path, local_path in download_targets:
                try:
                    client.download(remote_path, local_path)
                    self.log_signal.emit(f"[OK] 下载成功({target_name}): {remote_path} -> {local_path}")
                except Exception as e:
                    failed.append(f"{remote_path}: {e}")
                    self.log_signal.emit(f"[ERR] 下载失败({target_name}): {remote_path} -> {local_path} | {e}")
            if len(download_targets) > 1:
                ok_count = len(download_targets) - len(failed)
                if failed:
                    self.log_signal.emit(f"[WARN] 批量下载完成({target_name}): 成功 {ok_count}/{len(download_targets)}")
                else:
                    self.log_signal.emit(f"[OK] 批量下载完成({target_name}): 共 {ok_count} 个文件")
        self._run_async(worker)

    def upload_folder(self):
        if not self._ensure_smallbrain_feature("small_brain_upload_folder"):
            return
        client, target_name = self._get_transfer_client()
        if not client:
            return
        local_dir = QFileDialog.getExistingDirectory(self, "选择本地文件夹")
        if not local_dir:
            return

        default_remote = f"/tmp/{os.path.basename(local_dir)}"
        remote_parent = self._browse_sftp_path_dialog(
            client,
            target_name,
            "选择上传目标目录",
            start_path=self._guess_remote_browse_start_dir(default_remote),
            select_kind="dir",
        )
        if not remote_parent:
            return
        remote_dir = f"{remote_parent.rstrip('/')}/{os.path.basename(local_dir)}" if remote_parent != "/" else f"/{os.path.basename(local_dir)}"

        def worker():
            try:
                client.upload_dir(local_dir, remote_dir)
                self.log_signal.emit(f"[OK] 文件夹上传成功({target_name}): {local_dir} -> {remote_dir}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 文件夹上传失败: {e}")
        self._run_async(worker)

    def download_folder(self):
        if not self._ensure_smallbrain_feature("small_brain_download_folder"):
            return
        client, target_name = self._get_transfer_client()
        if not client:
            return
        remote_dir = self._browse_sftp_path_dialog(
            client,
            target_name,
            "选择下载源目录",
            start_path="/tmp",
            select_kind="dir",
        )
        if not remote_dir:
            return

        local_parent = QFileDialog.getExistingDirectory(self, "选择本地保存目录")
        if not local_parent:
            return

        folder_name = os.path.basename(remote_dir.rstrip("/")) or "remote_dir"
        local_dir = os.path.join(local_parent, folder_name)

        def worker():
            try:
                client.download_dir(remote_dir, local_dir)
                self.log_signal.emit(f"[OK] 文件夹下载成功({target_name}): {remote_dir} -> {local_dir}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 文件夹下载失败: {e}")
        self._run_async(worker)

    def restart_embedded_service(self):
        if not self._ensure_feature_enabled("small_brain_restart_embedded"):
            return
        if not self._ensure_ssh():
            return

        def worker():
            try:
                cmd = "systemctl restart sdk_autostart.service --no-block"
                out, err = self._run_sudo_bash(self.ssh, self.ssh_pwd.text(), cmd)
                msg = (out.strip() or err.strip() or "已下发重启指令")
                self.log_signal.emit(f"[OK] 小脑重启嵌入式完成: {msg}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 小脑重启嵌入式失败: {e}")

        self._run_async(worker)

    def restart_middleware_service(self):
        if not self._ensure_feature_enabled("small_brain_restart_middleware"):
            return
        if not self._ensure_ssh():
            return

        def worker():
            try:
                cmd = "systemctl restart zj_humanoid.service"
                out, err = self._run_sudo_bash(self.ssh, self.ssh_pwd.text(), cmd)
                msg = (out.strip() or err.strip() or "已下发重启指令")
                self.log_signal.emit(f"[OK] 小脑重启中间件完成: {msg}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 小脑重启中间件失败: {e}")

        self._run_async(worker)

    def browse_middleware_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        local_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 Middleware 升级包",
            "",
            "Firmware (*.firmware);;All Files (*)",
        )
        if not local_path:
            return
        self._middleware_deploy_local_path = local_path
        if hasattr(self, "middleware_deploy_pkg_edit"):
            self.middleware_deploy_pkg_edit.setText(local_path)

    def _middleware_deploy_remote_command(self, remote_path: str, robot_type: str, force_reinstall: bool = False) -> str:
        remote_name = os.path.basename(str(remote_path or "").strip())
        force_arg = " --force" if force_reinstall else ""
        return (
            f"cd /tmp && chmod +x {shlex.quote(remote_name)} && ./" + shlex.quote(remote_name) +
            f" -- --robot_type={shlex.quote(robot_type)}{force_arg}"
        )

    def _run_middleware_deploy_in_remote_terminal(self, client: SshSftpClient, cmd: str, on_output=None):
        return self._run_interactive_bash_with_exit_code(client, cmd, on_output=on_output)

    def _ensure_remote_middleware_package(self, client: SshSftpClient, target_name: str, local_path: str, remote_path: str):
        local_size = None
        try:
            local_size = os.path.getsize(local_path)
        except Exception:
            local_size = None

        try:
            remote_stat = client.sftp.stat(remote_path)
            remote_size = int(getattr(remote_stat, "st_size", -1))
            if local_size is not None and remote_size == int(local_size):
                self._emit_middleware_thread_log(target_name, f"检测到远端已存在同名升级包且大小一致，跳过上传: {remote_path}")
                return
            self._emit_middleware_thread_log(
                target_name,
                f"检测到远端已存在同名升级包，但大小不同，重新上传: {remote_path} (remote={remote_size}, local={local_size})",
            )
        except Exception:
            self._emit_middleware_thread_log(target_name, f"远端未发现升级包，开始上传: {local_path} -> {remote_path}")
            client.upload(local_path, remote_path)
            self._emit_middleware_thread_log(target_name, f"上传完成: {remote_path}")
            return

        self._emit_middleware_thread_log(target_name, f"开始上传升级包: {local_path} -> {remote_path}")
        client.upload(local_path, remote_path)
        self._emit_middleware_thread_log(target_name, f"上传完成: {remote_path}")

    def _run_middleware_deploy_on_target(self, client: SshSftpClient, target_name: str, local_path: str, robot_type: str, force_reinstall: bool = False):
        remote_path = f"/tmp/{os.path.basename(local_path)}"
        self._ensure_remote_middleware_package(client, target_name, local_path, remote_path)

        cmd = self._middleware_deploy_remote_command(remote_path, robot_type, force_reinstall)
        force_text = " --force" if force_reinstall else ""
        self._emit_middleware_thread_log(target_name, f"开始远程终端升级: {os.path.basename(local_path)} --robot_type={robot_type}{force_text}")
        out, err, exit_code = self._run_middleware_deploy_in_remote_terminal(
            client,
            cmd,
            on_output=lambda line: self._emit_middleware_thread_log(target_name, line),
        )
        detail = ((out or "") + "\n" + (err or "")).strip()
        if exit_code != 0:
            raise RuntimeError(detail or f"远端执行失败，exit_code={exit_code}")
        self._emit_middleware_thread_log(target_name, "升级执行完成")

    def _middleware_deploy_selected_targets(self):
        targets = []
        if bool(self.middleware_deploy_big_checkbox.isChecked()) if hasattr(self, "middleware_deploy_big_checkbox") else False:
            targets.append((self.ssh_big, "大脑", self._ensure_ssh_big))
        if bool(self.middleware_deploy_small_checkbox.isChecked()) if hasattr(self, "middleware_deploy_small_checkbox") else False:
            targets.append((self.ssh, "小脑", self._ensure_ssh))
        return targets

    def deploy_middleware_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        local_path = ""
        if hasattr(self, "middleware_deploy_pkg_edit"):
            local_path = self.middleware_deploy_pkg_edit.text().strip()
        local_path = local_path or str(getattr(self, "_middleware_deploy_local_path", "") or "").strip()
        if not local_path:
            self.log_signal.emit("[ERR] 请先选择本地 Middleware 升级包")
            return
        if not os.path.isfile(local_path):
            self.log_signal.emit(f"[ERR] 升级包不存在: {local_path}")
            return
        robot_type = self.middleware_deploy_robot_type_combo.currentText().strip() if hasattr(self, "middleware_deploy_robot_type_combo") else ""
        if not robot_type:
            self.log_signal.emit("[ERR] 请输入 robot_type，例如 WA2_LS 或 WA1")
            return
        force_reinstall = bool(self.middleware_deploy_force_checkbox.isChecked()) if hasattr(self, "middleware_deploy_force_checkbox") else False
        targets = self._middleware_deploy_selected_targets()
        if not targets:
            self.log_signal.emit("[ERR] 请至少选择一个部署目标（大脑/小脑）")
            return
        selected_targets = []
        for client, target_name, ensure_fn in targets:
            if not ensure_fn():
                return
            selected_targets.append((client, target_name))

        if hasattr(self, "btn_deploy_middleware_pkg"):
            self.btn_deploy_middleware_pkg.setEnabled(False)
        if hasattr(self, "btn_browse_middleware_pkg"):
            self.btn_browse_middleware_pkg.setEnabled(False)
        if hasattr(self, "middleware_deploy_big_checkbox"):
            self.middleware_deploy_big_checkbox.setEnabled(False)
        if hasattr(self, "middleware_deploy_small_checkbox"):
            self.middleware_deploy_small_checkbox.setEnabled(False)
        if hasattr(self, "middleware_deploy_force_checkbox"):
            self.middleware_deploy_force_checkbox.setEnabled(False)

        def worker():
            try:
                force_text = " | force=on" if force_reinstall else " | force=off"
                target_text = "、".join([name for _, name in selected_targets])
                self.log_signal.emit(
                    f"[INFO] 开始部署中间件升级包: {os.path.basename(local_path)} | robot_type={robot_type} | 目标={target_text}{force_text}"
                )
                errors = []
                errors_lock = threading.Lock()

                def _deploy_target(client: SshSftpClient, target_name: str):
                    try:
                        self._emit_middleware_thread_log(target_name, "线程开始")
                        self._run_middleware_deploy_on_target(client, target_name, local_path, robot_type, force_reinstall)
                        self._emit_middleware_thread_log(target_name, "线程结束: 成功")
                    except Exception as e:
                        self._emit_middleware_thread_log(target_name, f"线程结束: 失败 | {e}")
                        with errors_lock:
                            errors.append(f"{target_name}: {e}")

                threads = [
                    threading.Thread(target=_deploy_target, args=(client, target_name), daemon=True)
                    for client, target_name in selected_targets
                ]
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join()

                if errors:
                    raise RuntimeError(" | ".join(errors))
                self.log_signal.emit(
                    f"[OK] 中间件升级包部署完成: {os.path.basename(local_path)} | robot_type={robot_type} | 目标={target_text}{force_text}"
                )
            except Exception as e:
                self.log_signal.emit(f"[ERR] 中间件升级包部署失败: {e}")
            finally:
                if hasattr(self, "btn_deploy_middleware_pkg"):
                    self.btn_deploy_middleware_pkg.setEnabled(True)
                if hasattr(self, "btn_browse_middleware_pkg"):
                    self.btn_browse_middleware_pkg.setEnabled(True)
                if hasattr(self, "middleware_deploy_big_checkbox"):
                    self.middleware_deploy_big_checkbox.setEnabled(True)
                if hasattr(self, "middleware_deploy_small_checkbox"):
                    self.middleware_deploy_small_checkbox.setEnabled(True)
                if hasattr(self, "middleware_deploy_force_checkbox"):
                    self.middleware_deploy_force_checkbox.setEnabled(True)

        self._run_async(worker)

    def browse_mpc_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        local_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 MPC 部署包",
            "",
            "Archive (*.zip *.7z *.tar *.tar.gz *.tgz);;All Files (*)",
        )
        if not local_path:
            return
        self._mpc_deploy_local_path = local_path
        if hasattr(self, "mpc_deploy_pkg_edit"):
            self.mpc_deploy_pkg_edit.setText(local_path)

    def browse_mpc_image_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        local_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 MPC 镜像包",
            "",
            "Image Tar (*.tar *.tar.gz *.tgz);;All Files (*)",
        )
        if not local_path:
            return
        self._mpc_image_local_path = local_path
        if hasattr(self, "mpc_image_pkg_edit"):
            self.mpc_image_pkg_edit.setText(local_path)

    def browse_mpc_docker_offline_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        local_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择离线 Docker 包",
            "",
            "Docker Tarball (*.tgz *.tar.gz *.tar);;All Files (*)",
        )
        if not local_path:
            return
        self._mpc_docker_offline_local_path = local_path
        if hasattr(self, "mpc_docker_offline_pkg_edit"):
            self.mpc_docker_offline_pkg_edit.setText(local_path)

    def _mpc_collect_deploy_params(self):
        image_pkg = self.mpc_image_pkg_edit.text().strip() if hasattr(self, "mpc_image_pkg_edit") else ""
        image_pkg = image_pkg or str(getattr(self, "_mpc_image_local_path", "") or "").strip()
        mpc_pkg = self.mpc_deploy_pkg_edit.text().strip() if hasattr(self, "mpc_deploy_pkg_edit") else ""
        mpc_pkg = mpc_pkg or str(getattr(self, "_mpc_deploy_local_path", "") or "").strip()
        docker_offline_pkg = self.mpc_docker_offline_pkg_edit.text().strip() if hasattr(self, "mpc_docker_offline_pkg_edit") else ""
        docker_offline_pkg = docker_offline_pkg or str(getattr(self, "_mpc_docker_offline_local_path", "") or "").strip()
        return {
            "image_pkg": image_pkg,
            "mpc_pkg": mpc_pkg,
            "docker_offline_pkg": docker_offline_pkg,
            "image_name": (self.mpc_image_name_edit.text().strip() if hasattr(self, "mpc_image_name_edit") else "") or "tele-image:delivery-v1",
            "container_name": (self.mpc_container_name_edit.text().strip() if hasattr(self, "mpc_container_name_edit") else "") or "tele-delivery-container",
            "remote_dir": (self.mpc_deploy_remote_dir_edit.text().strip() if hasattr(self, "mpc_deploy_remote_dir_edit") else "") or "/home/nav01/tele-workspace/tele-delivery",
            "ros_master_uri": (self.mpc_ros_master_uri_edit.text().strip() if hasattr(self, "mpc_ros_master_uri_edit") else "") or "http://192.168.217.1:11311",
            "ros_ip": (self.mpc_ros_ip_edit.text().strip() if hasattr(self, "mpc_ros_ip_edit") else "") or "192.168.217.66",
            "auto_extract": bool(self.mpc_deploy_auto_extract_checkbox.isChecked()) if hasattr(self, "mpc_deploy_auto_extract_checkbox") else True,
        }

    def _run_bash_checked(self, client: SshSftpClient, cmd: str, desc: str):
        out, err, exit_code = self._run_bash_with_exit_code(client, cmd)
        merged = ((out or "") + "\n" + (err or "")).strip()
        if exit_code != 0:
            raise RuntimeError(f"{desc}失败: {merged or ('exit_code=' + str(exit_code))}")
        return merged

    def _get_small_brain_sudo_password(self) -> str:
        pwd = self.ssh_pwd.text() if hasattr(self, "ssh_pwd") else ""
        pwd = str(pwd or "")
        if pwd.strip():
            return pwd
        return str(getattr(self, "_ssh_small_password", "") or "")

    def _prompt_small_brain_sudo_password(self, reason: str = "") -> str:
        cached = self._get_small_brain_sudo_password()
        if cached.strip():
            return cached

        done = threading.Event()
        box = {"password": "", "accepted": False, "reason": str(reason or "").strip()}
        self.small_brain_sudo_prompt_signal.emit((box, done))
        done.wait()

        password = str(box.get("password") or "")
        if box.get("accepted") and password.strip():
            self._ssh_small_password = password
            return password
        return ""

    def _on_request_small_brain_sudo_password(self, payload):
        box, done = payload
        try:
            reason = str((box or {}).get("reason") or "").strip()
            prompt = "请输入小脑 sudo 密码"
            if reason:
                prompt = f"{prompt}\n\n用途: {reason}"
            text, ok = QInputDialog.getText(
                self,
                "小脑 sudo 密码",
                prompt,
                QLineEdit.EchoMode.Password,
                str(getattr(self, "_ssh_small_password", "") or ""),
            )
            if ok and str(text or "").strip():
                password = str(text)
                box["password"] = password
                box["accepted"] = True
                if hasattr(self, "ssh_pwd") and self.ssh_pwd is not None:
                    self.ssh_pwd.setText(password)
            else:
                box["password"] = ""
                box["accepted"] = False
        finally:
            done.set()

    def _is_docker_socket_permission_error(self, text: str) -> bool:
        msg = str(text or "").lower()
        return "permission denied" in msg and ("/var/run/docker.sock" in msg or "docker daemon socket" in msg)

    def _run_small_brain_docker_checked(self, docker_cmd: str, desc: str) -> str:
        try:
            return self._run_bash_checked(self.ssh, docker_cmd, desc)
        except Exception as e:
            if not self._is_docker_socket_permission_error(str(e)):
                raise

            sudo_pwd = self._prompt_small_brain_sudo_password(desc)
            if not sudo_pwd.strip():
                raise RuntimeError(f"{desc}失败: Docker 权限不足，且缺少小脑 sudo 密码") from e

            self.log_signal.emit(f"[WARN] {desc}检测到 Docker 权限不足，尝试使用 sudo 重试")
            out, err, exit_code = self._run_sudo_bash_with_exit_code(self.ssh, sudo_pwd, docker_cmd)
            merged = ((out or "") + "\n" + (err or "")).strip()
            if exit_code != 0:
                raise RuntimeError(f"{desc}失败: {merged or ('exit_code=' + str(exit_code))}") from e
            return merged

    def _run_small_brain_docker_exit_code(self, docker_cmd: str):
        out, err, exit_code = self._run_bash_with_exit_code(self.ssh, docker_cmd)
        merged = ((out or "") + "\n" + (err or "")).strip()
        if exit_code == 0:
            return out, err, exit_code

        if not self._is_docker_socket_permission_error(merged):
            return out, err, exit_code

        sudo_pwd = self._prompt_small_brain_sudo_password("Docker 状态检查")
        if not sudo_pwd.strip():
            return out, err, exit_code

        self.log_signal.emit("[WARN] Docker 权限不足，使用 sudo 重试状态检查")
        return self._run_sudo_bash_with_exit_code(self.ssh, sudo_pwd, docker_cmd)

    def _small_brain_docker_image_exists(self, image_name: str) -> bool:
        name = str(image_name or "").strip()
        if not name:
            return False
        _, err, exit_code = self._run_small_brain_docker_exit_code(
            f"docker image inspect {shlex.quote(name)} >/dev/null 2>&1"
        )
        err_text = (err or "").strip()
        if err_text and exit_code not in (0, 1):
            raise RuntimeError(err_text)
        return exit_code == 0

    def _ensure_small_brain_file_uploaded(self, local_path: str, remote_path: str, label: str):
        local_size = None
        try:
            local_size = os.path.getsize(local_path)
        except Exception:
            local_size = None

        try:
            remote_stat = self.ssh.sftp.stat(remote_path)
            remote_size = int(getattr(remote_stat, "st_size", -1))
            if local_size is not None and remote_size == int(local_size):
                self.log_signal.emit(f"[INFO] 检测到远端已存在同名{label}且大小一致，跳过上传: {remote_path}")
                return
            self.log_signal.emit(
                f"[INFO] 检测到远端已存在同名{label}但大小不同，重新上传: {remote_path} (remote={remote_size}, local={local_size})"
            )
        except Exception:
            self.log_signal.emit(f"[INFO] 上传{label}到小脑: {local_path} -> {remote_path}")
            self.ssh.upload(local_path, remote_path)
            self.log_signal.emit(f"[OK] {label}上传完成: {remote_path}")
            return

        self.log_signal.emit(f"[INFO] 重新上传{label}到小脑: {local_path} -> {remote_path}")
        self.ssh.upload(local_path, remote_path)
        self.log_signal.emit(f"[OK] {label}上传完成: {remote_path}")

    def _ensure_docker_on_small_brain(self, offline_pkg: str = ""):
        _, _, docker_exists = self._run_bash_with_exit_code(self.ssh, "command -v docker >/dev/null 2>&1")
        _, _, dockerd_exists = self._run_bash_with_exit_code(self.ssh, "command -v dockerd >/dev/null 2>&1")
        if docker_exists == 0 and dockerd_exists == 0:
            self.log_signal.emit("[OK] 小脑已检测到 Docker")
            return

        self.log_signal.emit("[WARN] 小脑未检测到 Docker，开始自动安装")
        sudo_pwd = self._prompt_small_brain_sudo_password("安装 Docker")
        if not sudo_pwd.strip():
            raise RuntimeError("需要小脑 sudo 密码以安装 Docker，请先填写或重新连接 SSH(小脑)")

        remote_user = self.ssh_user.text().strip() if hasattr(self, "ssh_user") else ""
        remote_user = remote_user or "nav01"

        try:
            install_cmd = (
                "set -e; "
                "apt-get update; "
                "DEBIAN_FRONTEND=noninteractive apt-get install -y docker.io; "
                "groupadd -f docker; "
                f"id -u {shlex.quote(remote_user)} >/dev/null 2>&1 && usermod -aG docker {shlex.quote(remote_user)} || true; "
                "systemctl daemon-reload; "
                "systemctl enable docker; "
                "systemctl restart docker || systemctl start docker; "
                "docker version"
            )
            self.log_signal.emit("[INFO] 尝试在线安装 Docker (apt-get)")
            out, err, exit_code = self._run_sudo_bash_with_exit_code(self.ssh, sudo_pwd, install_cmd)
            merged = ((out or "") + "\n" + (err or "")).strip()
            if exit_code != 0:
                raise RuntimeError(merged or f"exit_code={exit_code}")
            self.log_signal.emit("[OK] 在线安装 Docker 成功")
            if merged:
                self.log_signal.emit(f"[INFO] Docker安装输出:\n{merged}")
            return
        except Exception as install_err:
            self.log_signal.emit(f"[WARN] 在线安装 Docker 失败，尝试离线安装: {install_err}")

        if not offline_pkg:
            raise RuntimeError("在线安装失败，且未选择离线 Docker 包")
        if not os.path.isfile(offline_pkg):
            raise RuntimeError(f"离线 Docker 包不存在: {offline_pkg}")

        remote_tar = f"/home/{remote_user}/{os.path.basename(offline_pkg)}"
        self.log_signal.emit(f"[INFO] 上传离线Docker包到小脑: {offline_pkg} -> {remote_tar}")
        self.ssh.upload(offline_pkg, remote_tar)
        self.log_signal.emit("[OK] 离线Docker包上传完成")

        service_content = "\n".join([
            "[Unit]",
            "Description=Docker Application Container Engine",
            "Documentation=https://docs.docker.com",
            "After=network-online.target firewalld.service",
            "Wants=network-online.target",
            "",
            "[Service]",
            "Type=notify",
            "ExecStart=/usr/bin/dockerd",
            "ExecReload=/bin/kill -s HUP $MAINPID",
            "LimitNOFILE=infinity",
            "LimitNPROC=infinity",
            "LimitCORE=infinity",
            "TimeoutStartSec=0",
            "Delegate=yes",
            "KillMode=process",
            "Restart=on-failure",
            "StartLimitBurst=3",
            "StartLimitInterval=60s",
            "",
            "[Install]",
            "WantedBy=multi-user.target",
            "",
        ])
        service_b64 = base64.b64encode(service_content.encode("utf-8")).decode("ascii")

        offline_install_cmd = (
            "set -e; "
            f"cd /home/{shlex.quote(remote_user)}; "
            f"tar xzvf {shlex.quote(remote_tar)}; "
            "cp docker/* /usr/bin/; "
            f"echo {shlex.quote(service_b64)} | base64 -d > /etc/systemd/system/docker.service; "
            "groupadd -f docker; "
            f"id -u {shlex.quote(remote_user)} >/dev/null 2>&1 && usermod -aG docker {shlex.quote(remote_user)} || true; "
            "systemctl daemon-reload; "
            "systemctl enable docker; "
            "systemctl restart docker || systemctl start docker; "
            "docker version"
        )
        out, err, exit_code = self._run_sudo_bash_with_exit_code(self.ssh, sudo_pwd, offline_install_cmd)
        merged = ((out or "") + "\n" + (err or "")).strip()
        if exit_code != 0:
            raise RuntimeError(f"离线安装 Docker 失败: {merged or ('exit_code=' + str(exit_code))}")
        self.log_signal.emit("[OK] 离线安装 Docker 成功")
        if merged:
            self.log_signal.emit(f"[INFO] Docker离线安装输出:\n{merged}")

    def _load_mpc_image_on_small_brain(self, image_pkg: str, image_name: str = "", docker_offline_pkg: str = ""):
        if not image_pkg:
            raise RuntimeError("未选择镜像包")
        if not os.path.isfile(image_pkg):
            raise RuntimeError(f"镜像包不存在: {image_pkg}")

        self._ensure_docker_on_small_brain(docker_offline_pkg)

        if image_name and self._small_brain_docker_image_exists(image_name):
            self.log_signal.emit(f"[INFO] 小脑已存在镜像，跳过导入: {image_name}")
            return

        pkg_name = os.path.basename(image_pkg)
        remote_tar = f"/tmp/{pkg_name}"
        self._ensure_small_brain_file_uploaded(image_pkg, remote_tar, "镜像包")
        self.log_signal.emit(f"[INFO] 开始导入镜像: {pkg_name}")
        detail = self._run_small_brain_docker_checked(f"docker load -i {shlex.quote(remote_tar)}", "导入镜像")
        if detail:
            self.log_signal.emit(f"[INFO] 导入输出:\n{detail}")
        self.log_signal.emit(f"[OK] 镜像导入完成: {pkg_name}")

    def _prepare_mpc_container_on_small_brain(self, image_name: str, container_name: str, ros_master_uri: str, ros_ip: str, docker_offline_pkg: str = ""):
        if not image_name:
            raise RuntimeError("镜像名不能为空")
        if not container_name:
            raise RuntimeError("容器名不能为空")

        self._ensure_docker_on_small_brain(docker_offline_pkg)

        self._run_bash_checked(self.ssh, "mkdir -p /home/nav01/tele-workspace /home/nav01/tele-workspace/tele-delivery", "创建工作目录")

        check_cmd = f"docker container inspect {shlex.quote(container_name)} >/dev/null 2>&1"
        _, _, exists_code = self._run_small_brain_docker_exit_code(check_cmd)
        if exists_code == 0:
            self.log_signal.emit(f"[INFO] 容器已存在，启动容器: {container_name}")
            self._run_small_brain_docker_checked(f"docker start {shlex.quote(container_name)}", "启动容器")
            self.log_signal.emit(f"[OK] 容器已启动: {container_name}")
            return

        run_cmd = (
            "docker run -d -it "
            f"--name {shlex.quote(container_name)} "
            "--privileged --network host --shm-size 32g --cap-add sys_nice "
            "-e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 "
            "-v /tmp/.X11-unix:/tmp/.X11-unix "
            "-v /dev:/dev "
            "-v /home/nav01/tele-workspace:/root/workspace "
            f"-e ROS_MASTER_URI={shlex.quote(ros_master_uri)} "
            f"-e ROS_IP={shlex.quote(ros_ip)} "
            f"{shlex.quote(image_name)} /bin/bash"
        )
        self.log_signal.emit(f"[INFO] 创建MPC容器: {container_name}")
        self._run_small_brain_docker_checked(run_cmd, "创建容器")
        self.log_signal.emit(f"[OK] 容器创建完成: {container_name}")

    def _deploy_mpc_package_on_small_brain(self, local_path: str, remote_dir: str, auto_extract: bool):
        if not local_path:
            raise RuntimeError("未选择本地 MPC 部署包")
        if not os.path.isfile(local_path):
            raise RuntimeError(f"MPC 部署包不存在: {local_path}")

        self._run_bash_checked(self.ssh, f"mkdir -p {shlex.quote(remote_dir)}", "创建MPC部署目录")
        pkg_name = os.path.basename(local_path)
        remote_path = f"{remote_dir.rstrip('/')}/{pkg_name}"
        self.log_signal.emit(f"[INFO] 上传MPC包: {local_path} -> {remote_path}")
        self.ssh.upload(local_path, remote_path)
        self.log_signal.emit(f"[OK] MPC包上传完成: {remote_path}")

        if auto_extract:
            extract_cmd = self._mpc_extract_command(remote_dir, pkg_name)
            if extract_cmd:
                self.log_signal.emit(f"[INFO] 开始自动解压: {pkg_name}")
                self._run_bash_checked(self.ssh, extract_cmd, "自动解压MPC包")
                self.log_signal.emit(f"[OK] MPC包自动解压完成: {pkg_name}")
            else:
                self.log_signal.emit("[WARN] 当前文件类型不支持自动解压，已仅上传")

    def load_mpc_image_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        if not self._ensure_ssh():
            return
        params = self._mpc_collect_deploy_params()
        image_pkg = params["image_pkg"]
        if not image_pkg:
            self.log_signal.emit("[ERR] 请先选择本地镜像包")
            return

        if hasattr(self, "btn_load_mpc_image"):
            self.btn_load_mpc_image.setEnabled(False)
        if hasattr(self, "btn_browse_mpc_image_pkg"):
            self.btn_browse_mpc_image_pkg.setEnabled(False)
        if hasattr(self, "btn_browse_mpc_docker_offline_pkg"):
            self.btn_browse_mpc_docker_offline_pkg.setEnabled(False)

        def worker():
            try:
                self._load_mpc_image_on_small_brain(image_pkg, params["image_name"], params["docker_offline_pkg"])
            except Exception as e:
                self.log_signal.emit(f"[ERR] 导入MPC镜像失败: {e}")
            finally:
                if hasattr(self, "btn_load_mpc_image"):
                    self.btn_load_mpc_image.setEnabled(True)
                if hasattr(self, "btn_browse_mpc_image_pkg"):
                    self.btn_browse_mpc_image_pkg.setEnabled(True)
                if hasattr(self, "btn_browse_mpc_docker_offline_pkg"):
                    self.btn_browse_mpc_docker_offline_pkg.setEnabled(True)

        self._run_async(worker)

    def prepare_mpc_container(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        if not self._ensure_ssh():
            return
        params = self._mpc_collect_deploy_params()

        if hasattr(self, "btn_prepare_mpc_container"):
            self.btn_prepare_mpc_container.setEnabled(False)

        def worker():
            try:
                self._prepare_mpc_container_on_small_brain(
                    params["image_name"],
                    params["container_name"],
                    params["ros_master_uri"],
                    params["ros_ip"],
                    params["docker_offline_pkg"],
                )
            except Exception as e:
                self.log_signal.emit(f"[ERR] 创建/启动MPC容器失败: {e}")
            finally:
                if hasattr(self, "btn_prepare_mpc_container"):
                    self.btn_prepare_mpc_container.setEnabled(True)

        self._run_async(worker)

    def _mpc_extract_command(self, remote_dir: str, filename: str) -> str:
        low = str(filename or "").lower()
        dir_q = shlex.quote(remote_dir)
        file_q = shlex.quote(filename)
        if low.endswith(".zip"):
            return (
                f"cd {dir_q} && "
                f"if command -v 7z >/dev/null 2>&1; then 7z x -y {file_q}; "
                f"elif command -v unzip >/dev/null 2>&1; then unzip -o {file_q}; "
                "else echo '缺少解压工具(7z/unzip)'; exit 2; fi"
            )
        if low.endswith(".7z"):
            return f"cd {dir_q} && 7z x -y {file_q}"
        if low.endswith(".tar.gz") or low.endswith(".tgz"):
            return f"cd {dir_q} && tar -xzf {file_q}"
        if low.endswith(".tar"):
            return f"cd {dir_q} && tar -xf {file_q}"
        return ""

    def deploy_mpc_package(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        if not self._ensure_ssh():
            return
        params = self._mpc_collect_deploy_params()
        local_path = params["mpc_pkg"]
        if not local_path:
            self.log_signal.emit("[ERR] 请先选择本地 MPC 部署包")
            return

        if hasattr(self, "btn_deploy_mpc_pkg"):
            self.btn_deploy_mpc_pkg.setEnabled(False)
        if hasattr(self, "btn_browse_mpc_pkg"):
            self.btn_browse_mpc_pkg.setEnabled(False)
        if hasattr(self, "mpc_deploy_auto_extract_checkbox"):
            self.mpc_deploy_auto_extract_checkbox.setEnabled(False)

        def worker():
            try:
                pkg_name = os.path.basename(local_path)
                self.log_signal.emit(f"[INFO] 开始部署MPC到小脑: {pkg_name}")
                self._deploy_mpc_package_on_small_brain(local_path, params["remote_dir"], params["auto_extract"])
                self.log_signal.emit(f"[OK] MPC部署完成(小脑): {pkg_name}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] MPC部署失败: {e}")
            finally:
                if hasattr(self, "btn_deploy_mpc_pkg"):
                    self.btn_deploy_mpc_pkg.setEnabled(True)
                if hasattr(self, "btn_browse_mpc_pkg"):
                    self.btn_browse_mpc_pkg.setEnabled(True)
                if hasattr(self, "mpc_deploy_auto_extract_checkbox"):
                    self.mpc_deploy_auto_extract_checkbox.setEnabled(True)

        self._run_async(worker)

    def deploy_mpc_all(self):
        if not self._ensure_feature_enabled("middleware_upgrade"):
            return
        if not self._ensure_ssh():
            return

        params = self._mpc_collect_deploy_params()
        if not params["image_pkg"]:
            self.log_signal.emit("[ERR] 请先选择本地镜像包")
            return
        if not params["mpc_pkg"]:
            self.log_signal.emit("[ERR] 请先选择本地 MPC 部署包")
            return

        for widget_name in [
            "btn_deploy_mpc_all", "btn_load_mpc_image", "btn_prepare_mpc_container", "btn_deploy_mpc_pkg",
            "btn_browse_mpc_image_pkg", "btn_browse_mpc_docker_offline_pkg", "btn_browse_mpc_pkg",
        ]:
            w = getattr(self, widget_name, None)
            if w is not None:
                w.setEnabled(False)

        def worker():
            try:
                self.log_signal.emit("[INFO] 开始MPC全部部署(小脑)")
                self._load_mpc_image_on_small_brain(params["image_pkg"], params["image_name"], params["docker_offline_pkg"])
                self._prepare_mpc_container_on_small_brain(
                    params["image_name"],
                    params["container_name"],
                    params["ros_master_uri"],
                    params["ros_ip"],
                    params["docker_offline_pkg"],
                )
                self._deploy_mpc_package_on_small_brain(params["mpc_pkg"], params["remote_dir"], params["auto_extract"])
                self.log_signal.emit("[OK] MPC全部部署完成(小脑)")
            except Exception as e:
                self.log_signal.emit(f"[ERR] MPC全部部署失败: {e}")
            finally:
                for widget_name in [
                    "btn_deploy_mpc_all", "btn_load_mpc_image", "btn_prepare_mpc_container", "btn_deploy_mpc_pkg",
                    "btn_browse_mpc_image_pkg", "btn_browse_mpc_docker_offline_pkg", "btn_browse_mpc_pkg",
                ]:
                    w = getattr(self, widget_name, None)
                    if w is not None:
                        w.setEnabled(True)

        self._run_async(worker)

    # =================== movej_by_path ===================

    def _huiyang_log(self, text: str):
        self.huiyang_log_item_signal.emit(text)
        self.log_signal.emit(f"[movej_by_path] {text}")

    def _huiyang_append_log(self, text: str):
        if hasattr(self, "huiyang_output"):
            from datetime import datetime as _dt
            ts = _dt.now().strftime("%H:%M:%S")
            self.huiyang_output.appendPlainText(f"[{ts}] {text}")

    def _set_huiyang_controls(self, enter_enabled: bool, exit_enabled: bool):
        if hasattr(self, "btn_huiyang_enter_teach"):
            self.btn_huiyang_enter_teach.setEnabled(bool(enter_enabled))
        if hasattr(self, "btn_huiyang_exit_teach"):
            self.btn_huiyang_exit_teach.setEnabled(bool(exit_enabled))

    def _service_resp_success(self, resp_obj) -> bool:
        if isinstance(resp_obj, dict) and ("success" in resp_obj):
            return bool(resp_obj.get("success"))
        if hasattr(resp_obj, "success"):
            try:
                return bool(getattr(resp_obj, "success"))
            except Exception:
                return True
        return True

    def _service_text_success(self, text: str) -> bool:
        low = (text or "").lower()
        if "success: false" in low or "success=false" in low:
            return False
        if "is not available" in low or "unable to communicate" in low or "error" in low or "exception" in low or "failed" in low:
            return False
        return True

    def _huiyang_call_teach_mode(self, action: str, arm_type: int) -> str:
        action = str(action or "").strip().lower()
        if action not in ("enter", "exit"):
            raise ValueError(f"未知示教动作: {action}")

        last_err = None
        if self.ros and self.ros.check_connection():
            for srv in self._teach_service_candidates(action):
                try:
                    resp = self.ros.request_service(srv, {"arm_type": arm_type})
                    if not self._service_resp_success(resp):
                        raise RuntimeError(json.dumps(resp, ensure_ascii=False))
                    self._huiyang_log(f"[OK] 示教{action}成功 [{srv}] -> {json.dumps(resp, ensure_ascii=False)}")
                    return srv
                except Exception as e:
                    last_err = e
                    self._huiyang_log(f"[WARN] ROS 调用失败 [{srv}]: {e}")

        if not self._ensure_ssh():
            raise RuntimeError(f"示教{action}失败: SSH 未连接")

        last_msg = ""
        for srv in self._teach_service_candidates(action):
            cmd = f"rosservice call {srv} \"arm_type: {arm_type}\""
            out, err = self._run_ros_cli_via_ssh_interactive(cmd)
            text = ((out or "") + "\n" + (err or "")).strip()
            last_msg = text or "调用完成"
            if not self._service_text_success(text):
                self._huiyang_log(f"[WARN] SSH 调用失败 [{srv}]: {last_msg}")
                continue
            self._huiyang_log(f"[OK] 示教{action}成功(SSH) [{srv}] -> {last_msg}")
            return srv

        raise RuntimeError(last_err or last_msg or f"示教{action}服务不可用")

    def _huiyang_teach_arm_type(self) -> int:
        return 15

    def _huiyang_control_arm_type(self) -> int:
        model = str(getattr(self, "robot_model", "")).upper()
        mapping = {
            "WA1": 31,
            "WA2": 15,
            "I2": 15,
        }
        return int(mapping.get(model, 15))

    def _update_huiyang_mode_label(self):
        if not hasattr(self, "huiyang_model_combo"):
            return
        model = str(getattr(self, "robot_model", "WA2")).upper()
        if model == "WA2_LS":
            model = "WA2"
        text = f"{model} 全身(进入15/控制{self._huiyang_control_arm_type()})"
        self.huiyang_model_combo.blockSignals(True)
        if self.huiyang_model_combo.count() <= 0:
            self.huiyang_model_combo.addItem(text)
        else:
            self.huiyang_model_combo.setItemText(0, text)
        self.huiyang_model_combo.setCurrentIndex(0)
        self.huiyang_model_combo.blockSignals(False)

    def _huiyang_use_remote_ssh_recorder(self) -> bool:
        return not (self.ros and self.ros.check_connection())

    def _huiyang_start_remote_recorder(self, joint_names: list[str], interval_sec: float = 0.5):
        if not self._ensure_ssh():
            raise RuntimeError("SSH 未连接，无法启动远端录制")
        if not joint_names:
            raise RuntimeError("关节名为空，无法启动远端录制")

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        remote_dir = "/tmp/huiyang_record"
        script_path = f"{remote_dir}/recorder_{stamp}.py"
        data_path = f"{remote_dir}/frames_{stamp}.jsonl"
        pid_path = f"{remote_dir}/frames_{stamp}.pid"
        log_path = f"{remote_dir}/frames_{stamp}.log"
        script_text = f'''import json\nimport signal\nimport time\nimport rospy\nfrom sensor_msgs.msg import JointState\n\nOUTFILE = {data_path!r}\nINTERVAL = {float(interval_sec)!r}\nJOINT_NAMES = {list(joint_names)!r}\nlatest = {{}}\nrunning = True\n\n\ndef cb(msg):\n    global latest\n    try:\n        latest = {{str(n): float(p) for n, p in zip(list(msg.name), list(msg.position))}}\n    except Exception:\n        latest = latest\n\n\ndef stop(*_args):\n    global running\n    running = False\n\n\nsignal.signal(signal.SIGTERM, stop)\nsignal.signal(signal.SIGINT, stop)\nrospy.init_node("huiyang_remote_recorder", anonymous=True, disable_signals=True)\nrospy.Subscriber("/zj_humanoid/upperlimb/joint_states", JointState, cb, queue_size=1)\nstart_ts = time.time()\nnext_emit = start_ts\nwith open(OUTFILE, "w", encoding="utf-8") as f:\n    while running and not rospy.is_shutdown():\n        now = time.time()\n        if latest and now + 1e-6 >= next_emit:\n            joints = [latest.get(name) for name in JOINT_NAMES]\n            if all(v is not None for v in joints):\n                rec = {{"t": round(now - start_ts, 4), "joints": [float(v) for v in joints]}}\n                f.write(json.dumps(rec, ensure_ascii=False) + "\\n")\n                f.flush()\n            next_emit += INTERVAL\n        time.sleep(0.02)\n'''
        script_b64 = base64.b64encode(script_text.encode("utf-8")).decode("ascii")
        cmd = (
            f"mkdir -p {shlex.quote(remote_dir)}; "
            f"echo {shlex.quote(script_b64)} | base64 -d > {shlex.quote(script_path)}; "
            f"chmod +x {shlex.quote(script_path)}; "
            f"rm -f {shlex.quote(data_path)} {shlex.quote(pid_path)} {shlex.quote(log_path)}; "
            f"nohup python3 {shlex.quote(script_path)} > {shlex.quote(log_path)} 2>&1 & echo $! > {shlex.quote(pid_path)}; "
            f"sleep 0.2; cat {shlex.quote(pid_path)}"
        )
        out, err = self._run_ros_cli_via_ssh_interactive(cmd)
        pid_text = (out or "").strip().splitlines()
        pid_value = pid_text[-1].strip() if pid_text else ""
        if not pid_value.isdigit():
            raise RuntimeError(((out or "") + "\n" + (err or "")).strip() or "远端录制启动失败")

        self._huiyang_remote_recording = True
        self._huiyang_remote_record_path = data_path
        self._huiyang_remote_record_pid_path = pid_path
        self._huiyang_remote_record_log_path = log_path
        self._huiyang_log(f"[INFO] SSH模式已切换为远端录制: pid={pid_value}, file={data_path}")

    def _huiyang_stop_remote_recorder(self) -> list[dict]:
        if not self._huiyang_remote_recording:
            return []

        data_path = self._huiyang_remote_record_path
        pid_path = self._huiyang_remote_record_pid_path
        log_path = self._huiyang_remote_record_log_path
        frames = []
        try:
            stop_cmd = (
                f"if [ -f {shlex.quote(pid_path)} ]; then kill $(cat {shlex.quote(pid_path)}) >/dev/null 2>&1 || true; fi; "
                f"sleep 0.6; "
                f"test -f {shlex.quote(data_path)} && wc -l {shlex.quote(data_path)} || true"
            )
            out, err = self._run_ros_cli_via_ssh_interactive(stop_cmd)
            self._huiyang_log(f"[INFO] 远端录制停止: {((out or '') + (err or '')).strip()}")

            local_path = tempfile.mktemp(prefix="huiyang_remote_", suffix=".jsonl")
            self.ssh.download(data_path, local_path)
            with open(local_path, "r", encoding="utf-8") as f:
                for line in f:
                    raw = line.strip()
                    if not raw:
                        continue
                    try:
                        item = json.loads(raw)
                    except Exception:
                        continue
                    joints = item.get("joints") if isinstance(item, dict) else None
                    if isinstance(joints, list):
                        frames.append({
                            "t": float(item.get("t", 0.0)),
                            "joints": [float(v) for v in joints],
                        })
            try:
                os.remove(local_path)
            except Exception:
                pass
        finally:
            cleanup_cmd = (
                f"rm -f {shlex.quote(pid_path)} {shlex.quote(data_path)} {shlex.quote(log_path)} "
                f"{shlex.quote(data_path.replace('frames_', 'recorder_').replace('.jsonl', '.py'))}"
            )
            try:
                self._run_bash(self.ssh, cleanup_cmd)
            except Exception:
                pass
            self._huiyang_remote_recording = False
            self._huiyang_remote_record_path = ""
            self._huiyang_remote_record_pid_path = ""
            self._huiyang_remote_record_log_path = ""

        return frames

    def _huiyang_update_info(self):
        with self._huiyang_record_lock:
            n = len(self._huiyang_frames)
            n_valid = sum(1 for f in self._huiyang_frames if isinstance(f.get("joints"), list) and len(f.get("joints") or []) > 0)
            dur = self._huiyang_frames[-1]["t"] if self._huiyang_frames else 0.0
        if hasattr(self, "huiyang_frame_count_label"):
            self.huiyang_frame_count_label.setText(f"已记录帧数: {n_valid}/{n}")
        if hasattr(self, "huiyang_duration_label"):
            self.huiyang_duration_label.setText(f"时长: {dur:.2f}s")
        can_play = n_valid >= 2
        if hasattr(self, "btn_huiyang_playback"):
            self.btn_huiyang_playback.setEnabled(can_play and not self._huiyang_playback_running)
        if hasattr(self, "btn_huiyang_export_npz"):
            self.btn_huiyang_export_npz.setEnabled(n >= 1)

    def _huiyang_record_tick(self):
        if not self._huiyang_recording:
            return
        try:
            state = self._fetch_joint_state_once()
            names = self._joint_names_for_arm_type(self._huiyang_control_arm_type())
            joints = [float(state.get(n, 0.0)) for n in names] if state else []
            t = time.monotonic() - self._huiyang_record_start_ts
            with self._huiyang_record_lock:
                self._huiyang_frames.append({"t": round(t, 4), "joints": joints})
            self.huiyang_info_signal.emit()
        except Exception as e:
            self._huiyang_log(f"[WARN] 采样异常: {e}")
        finally:
            if self._huiyang_recording:
                self._huiyang_record_timer = threading.Timer(0.5, self._huiyang_record_tick)
                self._huiyang_record_timer.daemon = True
                self._huiyang_record_timer.start()

    def huiyang_enter_teach(self):
        if self._huiyang_recording:
            self._huiyang_log("[WARN] 已在示教采样中")
            return
        if not self._ensure_ssh():
            return

        if self._huiyang_record_timer is not None:
            self._huiyang_record_timer.cancel()
            self._huiyang_record_timer = None
        if self._huiyang_remote_recording:
            try:
                self._huiyang_stop_remote_recorder()
            except Exception as e:
                self._huiyang_log(f"[WARN] 清理上次远端录制失败: {e}")
        with self._huiyang_record_lock:
            self._huiyang_frames = []
            self._huiyang_initial_frame = []
        self._huiyang_record_start_ts = 0.0
        self.huiyang_info_signal.emit()
        self._huiyang_log("[INFO] 已自动清除上一轮录制数据")

        teach_arm_type = self._huiyang_teach_arm_type()
        control_arm_type = self._huiyang_control_arm_type()
        if hasattr(self, "btn_huiyang_enter_teach"):
            self.btn_huiyang_enter_teach.setEnabled(False)

        def worker():
            try:
                self._huiyang_log(f"[INFO] 进入全身示教 teach_arm_type={teach_arm_type}, control_arm_type={control_arm_type}")
                self._huiyang_call_teach_mode("enter", teach_arm_type)

                names = self._joint_names_for_arm_type(control_arm_type)
                use_remote_recorder = self._huiyang_use_remote_ssh_recorder()
                if use_remote_recorder:
                    self._huiyang_start_remote_recorder(names, 0.5)

                # 记录初始帧
                time.sleep(0.2)
                state = self._fetch_joint_state_once()
                initial = [float(state.get(n, 0.0)) for n in names] if state else []
                if not initial:
                    raise RuntimeError(f"未获取到全身关节状态(control_arm_type={control_arm_type})，请先确认关节监控/ROSBridge是否正常")
                with self._huiyang_record_lock:
                    self._huiyang_initial_frame = initial
                    self._huiyang_frames = [{"t": 0.0, "joints": initial}]
                self._huiyang_log(f"[OK] 已记录初始帧: {len(initial)} 个关节")

                # 启动定时采样
                self._huiyang_recording = True
                self._huiyang_record_start_ts = time.monotonic()
                if not use_remote_recorder:
                    self._huiyang_record_timer = threading.Timer(0.5, self._huiyang_record_tick)
                    self._huiyang_record_timer.daemon = True
                    self._huiyang_record_timer.start()
                else:
                    self._huiyang_log("[INFO] 当前为SSH采样，已切换为小脑本地文件录制，退出示教后自动回传")

                self.huiyang_status_signal.emit("采样中...")
                self.huiyang_info_signal.emit()
                self.huiyang_controls_signal.emit(False, True)
            except Exception as e:
                if self._huiyang_remote_recording:
                    try:
                        self._huiyang_stop_remote_recorder()
                    except Exception:
                        pass
                self._huiyang_log(f"[ERR] 进入示教失败: {e}")
                self.huiyang_controls_signal.emit(True, False)

        self._run_async(worker)

    def huiyang_exit_teach(self):
        teach_arm_type = self._huiyang_teach_arm_type()
        control_arm_type = self._huiyang_control_arm_type()
        if hasattr(self, "btn_huiyang_exit_teach"):
            self.btn_huiyang_exit_teach.setEnabled(False)

        def worker():
            try:
                # 停止采样
                self._huiyang_recording = False
                if self._huiyang_record_timer is not None:
                    self._huiyang_record_timer.cancel()
                    self._huiyang_record_timer = None

                if self._huiyang_remote_recording:
                    remote_frames = self._huiyang_stop_remote_recorder()
                    if remote_frames:
                        with self._huiyang_record_lock:
                            self._huiyang_frames = remote_frames
                            self._huiyang_initial_frame = list(remote_frames[0].get("joints") or [])
                        self._huiyang_log(f"[OK] 已加载远端录制数据: {len(remote_frames)} 帧")
                else:
                    # 记录最后一帧
                    state = self._fetch_joint_state_once()
                    names = self._joint_names_for_arm_type(control_arm_type)
                    last_joints = [float(state.get(n, 0.0)) for n in names] if state else []
                    if last_joints:
                        t = time.monotonic() - self._huiyang_record_start_ts
                        with self._huiyang_record_lock:
                            self._huiyang_frames.append({"t": round(t, 4), "joints": last_joints})
                        self._huiyang_log(f"[OK] 已记录最后一帧 t={t:.2f}s")

                # 退出示教
                self._huiyang_call_teach_mode("exit", teach_arm_type)

                self.huiyang_status_signal.emit("就绪（已停采）")
                self.huiyang_info_signal.emit()
                self.huiyang_controls_signal.emit(True, False)
            except Exception as e:
                self._huiyang_log(f"[ERR] 退出示教失败: {e}")
                self.huiyang_controls_signal.emit(False, True)

        self._run_async(worker)

    def huiyang_playback(self):
        with self._huiyang_record_lock:
            frames = list(self._huiyang_frames)
            initial = list(self._huiyang_initial_frame)
        if not initial:
            self._huiyang_log("[ERR] 初始帧为空，请重新进入示教录制")
            return

        valid_frames = []
        for f in frames:
            joints = f.get("joints") if isinstance(f, dict) else None
            if isinstance(joints, list) and len(joints) == len(initial):
                valid_frames.append(f)

        if len(valid_frames) < 2:
            self._huiyang_log(f"[ERR] 数据不足：有效帧 {len(valid_frames)}，总帧 {len(frames)}，至少需要2帧有效数据")
            return

        arm_type = self._huiyang_control_arm_type()
        service = self._movej_service_for_arm_type(arm_type)
        if not service:
            self._huiyang_log(f"[ERR] 未找到 movej_by_path 服务 arm_type={arm_type}")
            return

        v = float(self.huiyang_movej_v_spin.value()) if hasattr(self, "huiyang_movej_v_spin") else 0.2
        time_scale = float(self.huiyang_time_scale_spin.value()) if hasattr(self, "huiyang_time_scale_spin") else 1.0

        if hasattr(self, "btn_huiyang_playback"):
            self.btn_huiyang_playback.setEnabled(False)
        self._huiyang_playback_running = True

        def worker():
            try:
                transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")
                self._huiyang_log(f"[INFO] 回放开始: 有效{len(valid_frames)}/总{len(frames)} 帧, arm_type={arm_type}, v={v}, 时间缩放={time_scale}, 传输={transport_mode}")

                # Step 1: MoveJ 回到初始位置
                self._huiyang_log("[INFO] Step1: MoveJ 回到初始位置")
                movej_single_srv = self._movej_single_service_for_arm_type(arm_type)
                movej_req = {
                    "joints": [float(x) for x in initial],
                    "v": v,
                    "acc": 1.0,
                    "t": 5.0,
                    "is_async": False,
                    "arm_type": arm_type,
                }
                self._huiyang_log(
                    f"[INFO] MoveJ请求预览: service={movej_single_srv}, payload={json.dumps(movej_req, ensure_ascii=False)}"
                )
                movej_done = False
                if transport_mode == "ros" and self.ros and self.ros.check_connection():
                    try:
                        resp = self.ros.request_service(movej_single_srv, movej_req, timeout=30.0)
                        self._huiyang_log(f"[OK] MoveJ完成: {resp}")
                        movej_done = True
                    except Exception as e:
                        self._huiyang_log(f"[WARN] ROS MoveJ失败，回退SSH: {e}")
                if not movej_done:
                    if not self._ensure_ssh():
                        raise RuntimeError("ROS MoveJ失败且SSH未连接")
                    movej_cmd = f"rosservice call {movej_single_srv} {shlex.quote(json.dumps(movej_req, ensure_ascii=False))}"
                    self._huiyang_log(f"[REQ] {movej_cmd}")
                    out, err = self._run_ros_cli_via_ssh_interactive(movej_cmd)
                    self._huiyang_log(f"[OK] MoveJ(SSH)完成: {((out or '') + (err or '')).strip()[:200]}")

                # Step 2: MoveJByPath 回放整条轨迹
                self._huiyang_log("[INFO] Step2: MoveJByPath 回放轨迹")
                total_time = (valid_frames[-1]["t"] - valid_frames[0]["t"]) * time_scale
                if total_time <= 0:
                    total_time = len(valid_frames) * 0.5 * time_scale
                path = [{"joint": [float(v2) for v2 in f["joints"]]} for f in valid_frames]
                req = {
                    "path": path,
                    "time": round(total_time, 3),
                    "is_async": False,
                    "arm_type": arm_type,
                }
                self._huiyang_log(f"[INFO] MoveJByPath请求预览: service={service}, payload={json.dumps(req, ensure_ascii=False)}")
                bypath_done = False
                if transport_mode == "ros" and self.ros and self.ros.check_connection():
                    try:
                        resp = self.ros.request_service(service, req, timeout=max(60.0, total_time + 10.0))
                        self._huiyang_log(f"[OK] MoveJByPath完成: {resp}")
                        bypath_done = True
                    except Exception as e:
                        self._huiyang_log(f"[WARN] ROS MoveJByPath失败，回退SSH: {e}")
                if not bypath_done:
                    if not self._ensure_ssh():
                        raise RuntimeError("ROS MoveJByPath失败且SSH未连接")
                    cmd = f"rosservice call {service} {shlex.quote(json.dumps(req, ensure_ascii=False))}"
                    self._huiyang_log(f"[REQ] {cmd}")
                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                    self._huiyang_log(f"[OK] MoveJByPath(SSH)完成: {((out or '') + (err or '')).strip()[:200]}")

                self._huiyang_log("[OK] 回放完成")
            except Exception as e:
                self._huiyang_log(f"[ERR] 回放失败: {e}")
            finally:
                self._huiyang_playback_running = False
                self.huiyang_info_signal.emit()

        self._run_async(worker)

    def huiyang_clear_data(self):
        self._huiyang_recording = False
        if self._huiyang_record_timer is not None:
            self._huiyang_record_timer.cancel()
            self._huiyang_record_timer = None
        if self._huiyang_remote_recording:
            try:
                self._huiyang_stop_remote_recorder()
            except Exception as e:
                self._huiyang_log(f"[WARN] 停止远端录制失败: {e}")
        with self._huiyang_record_lock:
            self._huiyang_frames = []
            self._huiyang_initial_frame = []
        self._huiyang_record_start_ts = 0.0
        self._huiyang_log("[INFO] 数据已清除")
        if hasattr(self, "huiyang_status_label"):
            self.huiyang_status_label.setText("状态: 就绪")
        if hasattr(self, "btn_huiyang_enter_teach"):
            self.btn_huiyang_enter_teach.setEnabled(True)
        if hasattr(self, "btn_huiyang_exit_teach"):
            self.btn_huiyang_exit_teach.setEnabled(False)
        self._huiyang_update_info()

    def huiyang_export_npz(self):
        with self._huiyang_record_lock:
            frames = list(self._huiyang_frames)
        if not frames:
            self._huiyang_log("[ERR] 暂无数据可导出")
            return

        save_path, _ = QFileDialog.getSaveFileName(
            self, "导出NPZ", "huiyang_traj.npz", "NumPy Archive (*.npz)"
        )
        if not save_path:
            return
        try:
            import numpy as np
            joints_arr = np.array([f["joints"] for f in frames], dtype=np.float64)
            timestamps = np.array([f["t"] for f in frames], dtype=np.float64)
            arm_type = self._huiyang_arm_type()
            names = self._joint_names_for_arm_type(arm_type)
            np.savez(save_path, joints=joints_arr, timestamps=timestamps, joint_names=names)
            self._huiyang_log(f"[OK] 已导出NPZ: {save_path} ({len(frames)} 帧, shape={joints_arr.shape})")
        except Exception as e:
            self._huiyang_log(f"[ERR] 导出NPZ失败: {e}")

    def _servoj_tool_log(self, text: str):
        self.servoj_tool_log_item_signal.emit(text)
        self.log_signal.emit(f"[ServoJ工具] {text}")

    def _servoj_tool_append_log(self, text: str):
        if hasattr(self, "servoj_tool_output"):
            ts = datetime.now().strftime("%H:%M:%S")
            self.servoj_tool_output.appendPlainText(f"[{ts}] {text}")

    def _servoj_tool_model(self) -> str:
        model = self.servoj_tool_model_combo.currentText().strip().upper() if hasattr(self, "servoj_tool_model_combo") else str(self.robot_model or "WA2").upper()
        return "WA2" if model == "WA2_LS" else model

    def _servoj_tool_joint_names(self, model: str | None = None):
        return list(self._get_joint_names_by_model(model or self._servoj_tool_model()))

    def _servoj_tool_arm_type(self, model: str | None = None) -> int:
        model_name = str(model or self._servoj_tool_model()).upper()
        return 31 if model_name == "WA1" else 15

    def _servoj_tool_mode(self) -> str:
        if hasattr(self, "servoj_tool_mode_combo"):
            data = self.servoj_tool_mode_combo.currentData()
            text = str(data or self.servoj_tool_mode_combo.currentText()).strip().lower()
            if text == "ssh":
                return "ssh"
        return "rosbridge"

    def _servoj_tool_use_remote_ssh_mode(self) -> bool:
        return self._servoj_tool_mode() == "ssh"

    def _servoj_tool_teach_arm_type(self) -> int:
        return 15

    def _servoj_tool_call_teach_mode(self, action: str, arm_type: int) -> str:
        action = str(action or "").strip().lower()
        if action not in ("enter", "exit"):
            raise ValueError(f"未知示教动作: {action}")

        last_err = None
        if self.ros and self.ros.check_connection():
            for srv in self._teach_service_candidates(action):
                try:
                    resp = self.ros.request_service(srv, {"arm_type": arm_type})
                    if not self._service_resp_success(resp):
                        raise RuntimeError(json.dumps(resp, ensure_ascii=False))
                    self._servoj_tool_log(f"[OK] 示教{action}成功 [{srv}] -> {json.dumps(resp, ensure_ascii=False)}")
                    return srv
                except Exception as e:
                    last_err = e
                    self._servoj_tool_log(f"[WARN] ROS 调用失败 [{srv}]: {e}")

        if not self._ensure_ssh():
            raise RuntimeError(f"示教{action}失败: SSH 未连接")

        last_msg = ""
        for srv in self._teach_service_candidates(action):
            cmd = f"rosservice call {srv} \"arm_type: {arm_type}\""
            out, err = self._run_ros_cli_via_ssh_interactive(cmd)
            text = ((out or "") + "\n" + (err or "")).strip()
            last_msg = text or "调用完成"
            if not self._service_text_success(text):
                self._servoj_tool_log(f"[WARN] SSH 调用失败 [{srv}]: {last_msg}")
                continue
            self._servoj_tool_log(f"[OK] 示教{action}成功(SSH) [{srv}] -> {last_msg}")
            return srv

        raise RuntimeError(last_err or last_msg or f"示教{action}服务不可用")

    def _servoj_tool_capture_position_sample(self, position_map: dict, t_value: float):
        model = self._servoj_tool_model()
        joint_names = self._servoj_tool_joint_names(model)
        if not joint_names or not position_map:
            return False

        sample_position = {}
        for name in joint_names:
            if name not in position_map:
                return False
            try:
                sample_position[name] = float(position_map[name])
            except Exception:
                return False

        sample = {
            "t": max(0.0, float(t_value)),
            "position": sample_position,
            "velocity": {},
            "effort": {},
        }
        with self._servoj_tool_lock:
            self._servoj_tool_samples.append(sample)
        return True

    def _servoj_tool_update_status(self):
        with self._servoj_tool_lock:
            count = len(self._servoj_tool_samples)
            duration = self._servoj_tool_samples[-1]["t"] if self._servoj_tool_samples else 0.0
        mode_text = "SSH" if self._servoj_tool_use_remote_ssh_mode() else "ROSBridge"
        if hasattr(self, "servoj_tool_status_label"):
            if self._servoj_tool_playback_running:
                self.servoj_tool_status_label.setText(f"状态: 回放中 ({mode_text})")
            else:
                if self._servoj_tool_recording:
                    self.servoj_tool_status_label.setText(f"状态: 示教录制中 ({mode_text})")
                elif self._servoj_tool_teach_active:
                    self.servoj_tool_status_label.setText(f"状态: 示教中(录制已停) ({mode_text}, {count}帧)")
                else:
                    self.servoj_tool_status_label.setText(f"状态: 就绪 ({mode_text}, {count}帧)")
        if hasattr(self, "servoj_tool_frame_label"):
            self.servoj_tool_frame_label.setText(f"原始帧数: {count}")
        if hasattr(self, "servoj_tool_duration_label"):
            self.servoj_tool_duration_label.setText(f"时长: {duration:.2f}s")
        if hasattr(self, "btn_servoj_tool_start"):
            self.btn_servoj_tool_start.setEnabled((not self._servoj_tool_teach_active) and (not self._servoj_tool_playback_running))
        if hasattr(self, "btn_servoj_tool_stop"):
            self.btn_servoj_tool_stop.setEnabled(self._servoj_tool_teach_active and (not self._servoj_tool_playback_running))
        if hasattr(self, "btn_servoj_tool_clear"):
            self.btn_servoj_tool_clear.setEnabled(not self._servoj_tool_playback_running)
        if hasattr(self, "btn_servoj_tool_export"):
            self.btn_servoj_tool_export.setEnabled((not self._servoj_tool_recording) and (not self._servoj_tool_playback_running) and count >= 2)
        if hasattr(self, "btn_servoj_tool_playback"):
            self.btn_servoj_tool_playback.setEnabled((not self._servoj_tool_recording) and (not self._servoj_tool_playback_running) and count >= 2)
        if hasattr(self, "servoj_tool_mode_combo"):
            self.servoj_tool_mode_combo.setEnabled((not self._servoj_tool_teach_active) and (not self._servoj_tool_recording) and (not self._servoj_tool_playback_running))

    def start_servoj_tool_recording(self):
        if self._servoj_tool_use_remote_ssh_mode():
            self._servoj_tool_log("[INFO] SSH模式请通过“进入全身示教/退出示教”完成录制")
            return
        if self._servoj_tool_recording:
            self._servoj_tool_log("[WARN] 已在录制中")
            return
        if not (self.ros and self.ros.check_connection()):
            self._servoj_tool_log("[ERR] ServoJ录制需使用ROSBridge持续订阅；SSH单次读取不适合高频ServoJ")
            return
        self._stop_servoj_tool_subscription()
        with self._servoj_tool_lock:
            self._servoj_tool_samples = []
        self._servoj_tool_start_ts = time.monotonic()
        self._servoj_tool_recording = True
        try:
            topic = roslibpy.Topic(self.ros.ros, self._joint_monitor_topic, "sensor_msgs/JointState")
            topic.subscribe(self._handle_servoj_tool_joint_msg)
            self._servoj_tool_topic = topic
        except Exception as e:
            self._servoj_tool_recording = False
            self._servoj_tool_topic = None
            self._servoj_tool_log(f"[ERR] ServoJ独立订阅启动失败: {e}")
            return
        self.servoj_tool_refresh_signal.emit()
        self._servoj_tool_log(f"[INFO] 开始ServoJ原始录制: model={self._servoj_tool_model()}，已建立独立joint_states订阅")

    def servoj_tool_enter_teach(self):
        if self._servoj_tool_teach_active:
            self._servoj_tool_log("[WARN] 已在示教录制中")
            return
        if self._servoj_tool_playback_running:
            self._servoj_tool_log("[WARN] 当前正在回放，请稍后再进入示教")
            return
        if (not self._servoj_tool_use_remote_ssh_mode()) and not (self.ros and self.ros.check_connection()):
            self._servoj_tool_log("[ERR] ServoJ示教录制需要 ROSBridge 持续订阅，请先连接 ROSBridge")
            return
        if self._servoj_tool_use_remote_ssh_mode() and not self._ensure_ssh():
            self._servoj_tool_log("[ERR] ServoJ SSH模式需要先连接 SSH(小脑)")
            return

        teach_arm_type = self._servoj_tool_teach_arm_type()
        control_arm_type = self._servoj_tool_arm_type()
        joint_names = self._servoj_tool_joint_names(self._servoj_tool_model())
        self._stop_servoj_tool_subscription()
        self._servoj_tool_teach_active = False
        with self._servoj_tool_lock:
            self._servoj_tool_samples = []
        self._servoj_tool_start_ts = 0.0
        self.servoj_tool_refresh_signal.emit()
        self._servoj_tool_log("[INFO] 已自动清除上一轮ServoJ数据")

        def worker():
            entered = False
            try:
                self._servoj_tool_log(f"[INFO] 进入全身示教 teach_arm_type={teach_arm_type}, control_arm_type={control_arm_type}")
                self._servoj_tool_call_teach_mode("enter", teach_arm_type)
                entered = True
                self._servoj_tool_teach_active = True

                self._servoj_tool_start_ts = time.monotonic()
                state = self._fetch_joint_state_once() or self._get_latest_joint_state_map(self._joint_state_fresh_timeout_sec)
                if not self._servoj_tool_capture_position_sample(state or {}, 0.0):
                    raise RuntimeError(f"未获取到初始全身关节状态(control_arm_type={control_arm_type})")
                self._servoj_tool_log("[OK] 已记录初始帧")

                self._servoj_tool_recording = True
                if self._servoj_tool_use_remote_ssh_mode():
                    self._servoj_tool_start_remote_recorder(joint_names, 1.0 / max(1.0, float(self.servoj_tool_hz_spin.value())))
                else:
                    try:
                        topic = roslibpy.Topic(self.ros.ros, self._joint_monitor_topic, "sensor_msgs/JointState")
                        topic.subscribe(self._handle_servoj_tool_joint_msg)
                        self._servoj_tool_topic = topic
                    except Exception as e:
                        self._servoj_tool_recording = False
                        self._servoj_tool_topic = None
                        raise RuntimeError(f"ServoJ独立订阅启动失败: {e}")

                self.servoj_tool_refresh_signal.emit()
                self._servoj_tool_log("[OK] 已进入示教并开始ServoJ录制")
            except Exception as e:
                self._servoj_tool_recording = False
                self._stop_servoj_tool_subscription()
                if entered:
                    try:
                        self._servoj_tool_call_teach_mode("exit", teach_arm_type)
                        self._servoj_tool_teach_active = False
                    except Exception as exit_err:
                        self._servoj_tool_teach_active = True
                        self._servoj_tool_log(f"[WARN] 回滚退出示教失败: {exit_err}")
                else:
                    self._servoj_tool_teach_active = False
                self.servoj_tool_refresh_signal.emit()
                self._servoj_tool_log(f"[ERR] 进入示教失败: {e}")

        self._run_async(worker)

    def servoj_tool_exit_teach(self):
        if not self._servoj_tool_teach_active:
            self._servoj_tool_log("[INFO] 当前未处于示教状态")
            return

        teach_arm_type = self._servoj_tool_teach_arm_type()
        self._servoj_tool_log(f"[INFO] 正在退出全身示教 teach_arm_type={teach_arm_type}")
        self._servoj_tool_recording = False
        self._stop_servoj_tool_subscription()
        self.servoj_tool_refresh_signal.emit()

        def worker():
            try:
                self._servoj_tool_call_teach_mode("exit", teach_arm_type)
                self._servoj_tool_teach_active = False

                if self._servoj_tool_use_remote_ssh_mode():
                    remote_samples = self._servoj_tool_stop_remote_recorder()
                    if remote_samples:
                        with self._servoj_tool_lock:
                            existing = list(self._servoj_tool_samples)
                            if existing and remote_samples:
                                remote_samples = [s for s in remote_samples if float(s.get("t", 0.0)) > 0.0]
                            self._servoj_tool_samples.extend(remote_samples)

                end_t = max(0.0, time.monotonic() - float(self._servoj_tool_start_ts or 0.0))
                state = self._get_latest_joint_state_map(self._joint_state_fresh_timeout_sec)
                if not state:
                    state = self._fetch_joint_state_once()
                if self._servoj_tool_capture_position_sample(state or {}, end_t):
                    self._servoj_tool_log(f"[OK] 已记录最后一帧 t={end_t:.2f}s")

                self.servoj_tool_refresh_signal.emit()
                with self._servoj_tool_lock:
                    count = len(self._servoj_tool_samples)
                self._servoj_tool_log(f"[OK] 已退出示教并结束录制，共 {count} 帧")
            except Exception as e:
                self._servoj_tool_recording = True
                self._servoj_tool_log(f"[ERR] 退出示教失败: {e}")
                self.servoj_tool_refresh_signal.emit()

        self._run_async(worker)

    def stop_servoj_tool_recording(self):
        if not self._servoj_tool_recording:
            return
        self._servoj_tool_recording = False
        self._stop_servoj_tool_subscription()
        if self._servoj_tool_remote_recording:
            try:
                self._servoj_tool_stop_remote_recorder()
            except Exception as e:
                self._servoj_tool_log(f"[WARN] 停止远端录制失败: {e}")
        self.servoj_tool_refresh_signal.emit()
        with self._servoj_tool_lock:
            count = len(self._servoj_tool_samples)
            duration = self._servoj_tool_samples[-1]["t"] if self._servoj_tool_samples else 0.0
        self._servoj_tool_log(f"[OK] 录制结束: {count} 帧 / {duration:.2f}s")

    def clear_servoj_tool_recording(self):
        self._servoj_tool_recording = False
        self._stop_servoj_tool_subscription()
        self._servoj_tool_teach_active = False
        if self._servoj_tool_remote_recording:
            try:
                self._servoj_tool_stop_remote_recorder()
            except Exception as e:
                self._servoj_tool_log(f"[WARN] 清理远端录制失败: {e}")
        with self._servoj_tool_lock:
            self._servoj_tool_samples = []
        self._servoj_tool_start_ts = 0.0
        self.servoj_tool_refresh_signal.emit()
        self._servoj_tool_log("[INFO] 已清除ServoJ原始数据")

    def _servoj_tool_resample(self, samples: list, joint_names: list[str], hz: float):
        import numpy as np

        valid = []
        for sample in samples:
            if not isinstance(sample, dict):
                continue
            pos_map = sample.get("position") or {}
            if not pos_map:
                continue
            row = []
            ok = True
            for name in joint_names:
                if name not in pos_map:
                    ok = False
                    break
                try:
                    row.append(float(pos_map[name]))
                except Exception:
                    ok = False
                    break
            if ok:
                valid.append((float(sample.get("t", 0.0)), row))

        if len(valid) < 2:
            raise RuntimeError("有效原始样本不足，至少需要2帧")

        times = np.array([item[0] for item in valid], dtype=np.float64)
        values = np.array([item[1] for item in valid], dtype=np.float64)
        duration = float(times[-1] - times[0])
        if duration <= 0:
            raise RuntimeError("样本时长为0，无法重采样")
        dt = 1.0 / max(1.0, float(hz))
        target_times = np.arange(0.0, duration + dt * 0.5, dt, dtype=np.float64)
        shifted_times = times - times[0]
        out = np.empty((len(target_times), len(joint_names)), dtype=np.float64)
        for col in range(values.shape[1]):
            out[:, col] = np.interp(target_times, shifted_times, values[:, col])
        return target_times, out

    def _servoj_tool_npz_payload(self, model: str, target_times, frames, hz: float):
        import numpy as np

        joint_names = self._servoj_tool_joint_names(model)
        payload = {
            "joints": frames,
            "timestamps": target_times,
            "joint_names": np.array(joint_names, dtype=object),
            "model": np.array([model]),
            "hz": np.array([float(hz)], dtype=np.float64),
        }
        if model == "WA1" and frames.shape[1] >= 19:
            payload.update({
                "left": frames[:, 0:7],
                "right": frames[:, 7:14],
                "neck": frames[:, 14:16],
                "waist": frames[:, 16:18],
                "lift": frames[:, 18:19],
            })
        elif model == "WA2" and frames.shape[1] >= 22:
            payload.update({
                "left": frames[:, 0:8],
                "right": frames[:, 8:16],
                "neck": frames[:, 16:18],
                "waist": frames[:, 18:22],
            })
        elif model == "I2" and frames.shape[1] >= 17:
            payload.update({
                "left": frames[:, 0:7],
                "right": frames[:, 7:14],
                "neck": frames[:, 14:16],
                "waist": frames[:, 16:17],
            })
        return payload

    def export_servoj_tool_npz(self):
        with self._servoj_tool_lock:
            samples = list(self._servoj_tool_samples)
        if len(samples) < 2:
            self._servoj_tool_log("[ERR] 原始样本不足，无法导出NPZ")
            return
        model = self._servoj_tool_model()
        joint_names = self._servoj_tool_joint_names(model)
        hz = float(self.servoj_tool_hz_spin.value()) if hasattr(self, "servoj_tool_hz_spin") else 200.0
        try:
            target_times, frames = self._servoj_tool_resample(samples, joint_names, hz)
        except Exception as e:
            self._servoj_tool_log(f"[ERR] 重采样失败: {e}")
            return

        default_name = f"servoj_capture_{model.lower()}_{int(round(hz))}hz.npz"
        save_path, _ = QFileDialog.getSaveFileName(self, "导出ServoJ NPZ", default_name, "NumPy Archive (*.npz)")
        if not save_path:
            return
        try:
            import numpy as np
            payload = self._servoj_tool_npz_payload(model, target_times, frames, hz)
            np.savez(save_path, **payload)
            self._servoj_tool_log(
                f"[OK] 已导出NPZ: {save_path} | model={model} | raw={len(samples)}帧 | resampled={frames.shape[0]}帧 @ {hz:.0f}Hz"
            )
            self._servoj_tool_log("[INFO] ServoJ建议流程: 先固定高频订阅保存原始样本，再按目标频率重采样导出，回放时按同频率发布topic")
        except Exception as e:
            self._servoj_tool_log(f"[ERR] 导出NPZ失败: {e}")

    def playback_servoj_tool(self):
        if self._servoj_tool_recording or self._servoj_tool_playback_running:
            self._servoj_tool_log("[WARN] 当前正在录制或回放")
            return
        if (not self._servoj_tool_use_remote_ssh_mode()) and not (self.ros and self.ros.check_connection()):
            self._servoj_tool_log("[ERR] ServoJ回放需使用ROSBridge")
            return
        if self._servoj_tool_use_remote_ssh_mode() and not self._ensure_ssh():
            self._servoj_tool_log("[ERR] ServoJ SSH模式需要先连接 SSH(小脑)")
            return

        with self._servoj_tool_lock:
            samples = list(self._servoj_tool_samples)
        if len(samples) < 2:
            self._servoj_tool_log("[ERR] 样本不足，无法回放")
            return

        model = self._servoj_tool_model()
        joint_names = self._servoj_tool_joint_names(model)
        hz = float(self.servoj_tool_hz_spin.value()) if hasattr(self, "servoj_tool_hz_spin") else 200.0
        movej_v = float(self.servoj_tool_movej_v_spin.value()) if hasattr(self, "servoj_tool_movej_v_spin") else 0.2
        try:
            _target_times, frames = self._servoj_tool_resample(samples, joint_names, hz)
        except Exception as e:
            self._servoj_tool_log(f"[ERR] 重采样失败，无法回放: {e}")
            return

        if self._servoj_tool_use_remote_ssh_mode():
            self._servoj_tool_playback_running = True
            self.servoj_tool_refresh_signal.emit()

            def worker():
                try:
                    self._servoj_tool_remote_playback(model, joint_names, frames, hz, float(self.servoj_tool_movej_v_spin.value()) if hasattr(self, "servoj_tool_movej_v_spin") else 0.2)
                except Exception as e:
                    self._servoj_tool_log(f"[ERR] ServoJ SSH回放失败: {e}")
                finally:
                    self._servoj_tool_playback_running = False
                    self.servoj_tool_refresh_signal.emit()

            self._run_async(worker)
            return

        arm_type = self._servoj_tool_arm_type(model)
        movej_service = self._movej_single_service_for_arm_type(arm_type)
        servoj_topic_name = "/zj_humanoid/upperlimb/servoj/whole_body"
        self._servoj_tool_playback_running = True
        self.servoj_tool_refresh_signal.emit()

        def worker():
            topic = None
            try:
                first_frame = [float(v) for v in frames[0].tolist()]
                movej_req = {
                    "joints": first_frame,
                    "v": movej_v,
                    "acc": 1.0,
                    "t": 5.0,
                    "is_async": False,
                    "arm_type": arm_type,
                }
                self._servoj_tool_log(f"[REQ] MoveJ回到初始位置: service={movej_service}, payload={json.dumps(movej_req, ensure_ascii=False)}")
                resp = self.ros.request_service(movej_service, movej_req, service_type="upperlimb/MoveJ", timeout=30.0)
                self._servoj_tool_log(f"[OK] 已回到初始位置: {resp}")

                clear_req = {"v": 0.0, "acc": 0.0, "time": 0.0, "lookahead_time": 0.0, "gain": 0, "arm_type": arm_type}
                set_req = {"v": 0.1, "acc": 0.5, "time": 1.0 / hz, "lookahead_time": 0.2, "gain": 100, "arm_type": arm_type}
                self._servoj_tool_log(f"[REQ] clear_servo_params: {json.dumps(clear_req, ensure_ascii=False)}")
                self.ros.request_service("/zj_humanoid/upperlimb/clear_servo_params", clear_req, service_type="upperlimb/Servo", timeout=10.0)
                self._servoj_tool_log(f"[REQ] set_servo_params: {json.dumps(set_req, ensure_ascii=False)}")
                self.ros.request_service("/zj_humanoid/upperlimb/set_servo_params", set_req, service_type="upperlimb/Servo", timeout=10.0)

                topic = roslibpy.Topic(self.ros.ros, servoj_topic_name, "upperlimb/Joints")
                topic.advertise()
                self._servoj_tool_log(f"[INFO] 开始ServoJ回放: frames={frames.shape[0]}, hz={hz:.0f}, arm_type={arm_type}, topic={servoj_topic_name}")
                interval = 1.0 / max(1.0, hz)
                for index, row in enumerate(frames):
                    if not self.ros.check_connection():
                        raise RuntimeError("ROSBridge 已断开")
                    topic.publish(roslibpy.Message({"joint": [float(v) for v in row.tolist()]}))
                    if index == 0 or (index + 1) % max(1, int(hz)) == 0 or index == frames.shape[0] - 1:
                        self._servoj_tool_log(f"[INFO] ServoJ已发布 {index + 1}/{frames.shape[0]} 帧")
                    time.sleep(interval)
                self._servoj_tool_log("[OK] ServoJ回放完成")
            except Exception as e:
                self._servoj_tool_log(f"[ERR] ServoJ回放失败: {e}")
            finally:
                if topic is not None:
                    try:
                        topic.unadvertise()
                    except Exception:
                        pass
                self._servoj_tool_playback_running = False
                self.servoj_tool_refresh_signal.emit()

        self._run_async(worker)

    def _servoj_tool_remote_dir(self) -> str:
        return "/tmp/servoj_tool"

    def _servoj_tool_remote_joint_record_script(self, joint_names: list[str], interval_sec: float) -> str:
        return f'''import json\nimport signal\nimport time\nimport rospy\nfrom sensor_msgs.msg import JointState\n\nOUTFILE = {self._servoj_tool_remote_record_path!r}\nINTERVAL = {float(interval_sec)!r}\nJOINT_NAMES = {list(joint_names)!r}\nlatest = {{}}\nrunning = True\n\n\ndef cb(msg):\n    global latest\n    try:\n        latest = {{str(n): float(p) for n, p in zip(list(msg.name), list(msg.position))}}\n    except Exception:\n        pass\n\n\ndef stop(*_args):\n    global running\n    running = False\n\n\nsignal.signal(signal.SIGTERM, stop)\nsignal.signal(signal.SIGINT, stop)\nrospy.init_node("servoj_remote_recorder", anonymous=True, disable_signals=True)\nrospy.Subscriber("/zj_humanoid/upperlimb/joint_states", JointState, cb, queue_size=1)\nstart_ts = time.time()\nnext_emit = start_ts\nwith open(OUTFILE, "w", encoding="utf-8") as f:\n    while running and not rospy.is_shutdown():\n        now = time.time()\n        if latest and now + 1e-6 >= next_emit:\n            joints = [latest.get(name) for name in JOINT_NAMES]\n            if all(v is not None for v in joints):\n                rec = {{"t": round(now - start_ts, 6), "joints": [float(v) for v in joints]}}\n                f.write(json.dumps(rec, ensure_ascii=False) + "\\n")\n                f.flush()\n            next_emit += INTERVAL\n        time.sleep(min(0.002, max(0.0005, INTERVAL / 4.0)))\n'''

    def _servoj_tool_start_remote_recorder(self, joint_names: list[str], interval_sec: float):
        if not self._ensure_ssh():
            raise RuntimeError("SSH 未连接，无法启动远端录制")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        remote_dir = self._servoj_tool_remote_dir()
        self._servoj_tool_remote_record_script_path = f"{remote_dir}/recorder_{stamp}.py"
        self._servoj_tool_remote_record_path = f"{remote_dir}/frames_{stamp}.jsonl"
        self._servoj_tool_remote_record_pid_path = f"{remote_dir}/frames_{stamp}.pid"
        self._servoj_tool_remote_record_log_path = f"{remote_dir}/frames_{stamp}.log"

        fd, local_script = tempfile.mkstemp(prefix="servoj_remote_recorder_", suffix=".py")
        os.close(fd)
        try:
            self._run_bash(self.ssh, f"mkdir -p {shlex.quote(remote_dir)}")
            with open(local_script, "w", encoding="utf-8") as f:
                f.write(self._servoj_tool_remote_joint_record_script(joint_names, interval_sec))
            self.ssh.upload(local_script, self._servoj_tool_remote_record_script_path)
        finally:
            try:
                os.remove(local_script)
            except Exception:
                pass

        cmd = (
            f"mkdir -p {shlex.quote(remote_dir)}; "
            f"chmod +x {shlex.quote(self._servoj_tool_remote_record_script_path)}; "
            f"rm -f {shlex.quote(self._servoj_tool_remote_record_path)} {shlex.quote(self._servoj_tool_remote_record_pid_path)} {shlex.quote(self._servoj_tool_remote_record_log_path)}; "
            f"nohup python3 {shlex.quote(self._servoj_tool_remote_record_script_path)} > {shlex.quote(self._servoj_tool_remote_record_log_path)} 2>&1 & echo $! > {shlex.quote(self._servoj_tool_remote_record_pid_path)}; "
            f"sleep 0.2; cat {shlex.quote(self._servoj_tool_remote_record_pid_path)}"
        )
        out, err = self._run_ros_cli_via_ssh_interactive(cmd)
        pid_lines = (out or "").strip().splitlines()
        pid_value = pid_lines[-1].strip() if pid_lines else ""
        if not pid_value.isdigit():
            raise RuntimeError(((out or "") + "\n" + (err or "")).strip() or "远端录制启动失败")
        self._servoj_tool_remote_recording = True
        self._servoj_tool_log(f"[INFO] SSH模式已切换为远端录制: pid={pid_value}, file={self._servoj_tool_remote_record_path}")

    def _servoj_tool_stop_remote_recorder(self) -> list[dict]:
        if not self._servoj_tool_remote_recording:
            return []
        joint_names = self._servoj_tool_joint_names(self._servoj_tool_model())
        frames = []
        data_path = self._servoj_tool_remote_record_path
        pid_path = self._servoj_tool_remote_record_pid_path
        log_path = self._servoj_tool_remote_record_log_path
        script_path = self._servoj_tool_remote_record_script_path
        try:
            stop_cmd = (
                f"if [ -f {shlex.quote(pid_path)} ]; then kill $(cat {shlex.quote(pid_path)}) >/dev/null 2>&1 || true; fi; "
                f"sleep 0.6; "
                f"test -f {shlex.quote(data_path)} && wc -l {shlex.quote(data_path)} || true"
            )
            out, err = self._run_ros_cli_via_ssh_interactive(stop_cmd)
            self._servoj_tool_log(f"[INFO] 远端录制停止: {((out or '') + (err or '')).strip()}")

            fd, local_path = tempfile.mkstemp(prefix="servoj_remote_", suffix=".jsonl")
            os.close(fd)
            self.ssh.download(data_path, local_path)
            with open(local_path, "r", encoding="utf-8") as f:
                for line in f:
                    raw = line.strip()
                    if not raw:
                        continue
                    try:
                        item = json.loads(raw)
                    except Exception:
                        continue
                    joints = item.get("joints") if isinstance(item, dict) else None
                    if isinstance(joints, list) and len(joints) == len(joint_names):
                        frames.append({
                            "t": float(item.get("t", 0.0)),
                            "position": {name: float(val) for name, val in zip(joint_names, joints)},
                            "velocity": {},
                            "effort": {},
                        })
            try:
                os.remove(local_path)
            except Exception:
                pass
        finally:
            cleanup_cmd = f"rm -f {shlex.quote(pid_path)} {shlex.quote(data_path)} {shlex.quote(log_path)} {shlex.quote(script_path)}"
            try:
                self._run_bash(self.ssh, cleanup_cmd)
            except Exception:
                pass
            self._servoj_tool_remote_recording = False
            self._servoj_tool_remote_record_path = ""
            self._servoj_tool_remote_record_pid_path = ""
            self._servoj_tool_remote_record_log_path = ""
            self._servoj_tool_remote_record_script_path = ""
        return frames

    def _servoj_tool_remote_playback_script(self, remote_data_path: str, movej_service: str, servoj_topic_name: str, arm_type: int, movej_v: float) -> str:
        return f'''import json\nimport time\nimport rospy\nfrom upperlimb.msg import Joints\nfrom upperlimb.srv import MoveJ, MoveJRequest, Servo, ServoRequest\n\nDATA_PATH = {remote_data_path!r}\nMOVEJ_SERVICE = {movej_service!r}\nTOPIC_NAME = {servoj_topic_name!r}\nARM_TYPE = {int(arm_type)!r}\nMOVEJ_V = {float(movej_v)!r}\nCLEAR_SERVICE = "/zj_humanoid/upperlimb/clear_servo_params"\nSET_SERVICE = "/zj_humanoid/upperlimb/set_servo_params"\n\nwith open(DATA_PATH, "r", encoding="utf-8") as f:\n    payload = json.load(f)\nframes = payload.get("frames") or []\nhz = float(payload.get("hz") or 200.0)\nif not frames:\n    raise RuntimeError("frames为空")\ninterval = 1.0 / max(1.0, hz)\n\nrospy.init_node("servoj_remote_playback", anonymous=True)\nrospy.wait_for_service(MOVEJ_SERVICE, timeout=30.0)\nrospy.wait_for_service(CLEAR_SERVICE, timeout=10.0)\nrospy.wait_for_service(SET_SERVICE, timeout=10.0)\nmovej = rospy.ServiceProxy(MOVEJ_SERVICE, MoveJ)\nclear_srv = rospy.ServiceProxy(CLEAR_SERVICE, Servo)\nset_srv = rospy.ServiceProxy(SET_SERVICE, Servo)\npub = rospy.Publisher(TOPIC_NAME, Joints, queue_size=1)\ntime.sleep(0.2)\n\nmovej_req = MoveJRequest()\nmovej_req.joints = [float(v) for v in frames[0]]\nmovej_req.v = MOVEJ_V\nmovej_req.acc = 1.0\nmovej_req.t = 5.0\nmovej_req.is_async = False\nmovej_req.arm_type = ARM_TYPE\nprint(f"[REQ] MoveJ回到初始位置: service={{MOVEJ_SERVICE}}, joints={{len(movej_req.joints)}}")\nmovej_resp = movej(movej_req)\nprint(f"[OK] 已回到初始位置: {{movej_resp}}")\n\nclear_req = ServoRequest()\nclear_req.v = 0.0\nclear_req.acc = 0.0\nclear_req.time = 0.0\nclear_req.lookahead_time = 0.0\nclear_req.gain = 0\nclear_req.arm_type = ARM_TYPE\nclear_srv(clear_req)\n\nset_req = ServoRequest()\nset_req.v = 0.1\nset_req.acc = 0.5\nset_req.time = interval\nset_req.lookahead_time = 0.2\nset_req.gain = 100\nset_req.arm_type = ARM_TYPE\nset_srv(set_req)\n\nstart_ts = time.monotonic()\nfor index, row in enumerate(frames):\n    target_ts = start_ts + index * interval\n    now = time.monotonic()\n    if target_ts > now:\n        time.sleep(target_ts - now)\n    msg = Joints()\n    msg.joint = [float(v) for v in row]\n    pub.publish(msg)\n    if index == 0 or (index + 1) % max(1, int(hz)) == 0 or index == len(frames) - 1:\n        print(f"[INFO] ServoJ已发布 {{index + 1}}/{{len(frames)}} 帧")\nprint("[OK] ServoJ回放完成")\n'''

    def _servoj_tool_remote_playback(self, model: str, joint_names: list[str], frames, hz: float, movej_v: float):
        if not self._ensure_ssh():
            raise RuntimeError("SSH 未连接")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        remote_dir = self._servoj_tool_remote_dir()
        remote_data_path = f"{remote_dir}/playback_{stamp}.json"
        remote_script_path = f"{remote_dir}/playback_{stamp}.py"
        arm_type = self._servoj_tool_arm_type(model)
        movej_service = self._movej_single_service_for_arm_type(arm_type)
        servoj_topic_name = "/zj_humanoid/upperlimb/servoj/whole_body"

        fd_data, local_data_path = tempfile.mkstemp(prefix="servoj_remote_playback_", suffix=".json")
        os.close(fd_data)
        fd_script, local_script_path = tempfile.mkstemp(prefix="servoj_remote_playback_", suffix=".py")
        os.close(fd_script)
        try:
            payload = {
                "model": model,
                "joint_names": list(joint_names),
                "hz": float(hz),
                "frames": [[float(v) for v in row.tolist()] for row in frames],
            }
            with open(local_data_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False)
            with open(local_script_path, "w", encoding="utf-8") as f:
                f.write(self._servoj_tool_remote_playback_script(remote_data_path, movej_service, servoj_topic_name, arm_type, movej_v))

            self._run_bash(self.ssh, f"mkdir -p {shlex.quote(remote_dir)}")
            self.ssh.upload(local_data_path, remote_data_path)
            self.ssh.upload(local_script_path, remote_script_path)

            cmd = (
                f"chmod +x {shlex.quote(remote_script_path)}; "
                f"source ~/.bashrc >/dev/null 2>&1; "
                f"source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
                f"python3 {shlex.quote(remote_script_path)}"
            )
            self._servoj_tool_log(f"[INFO] 开始SSH ServoJ回放: frames={frames.shape[0]}, hz={hz:.0f}, arm_type={arm_type}, topic={servoj_topic_name}")
            out, err, exit_code = self._run_interactive_bash_with_exit_code(self.ssh, cmd, on_output=self._servoj_tool_log)
            if exit_code != 0:
                raise RuntimeError(((out or "") + "\n" + (err or "")).strip() or f"exit_code={exit_code}")
        finally:
            try:
                os.remove(local_data_path)
            except Exception:
                pass
            try:
                os.remove(local_script_path)
            except Exception:
                pass
            try:
                self._run_bash(self.ssh, f"rm -f {shlex.quote(remote_data_path)} {shlex.quote(remote_script_path)}")
            except Exception:
                pass

    # =================== END movej_by_path ===================

    def _resolve_remote_path_on_little_brain(self, remote_candidates, is_dir: bool):
        if not self._ensure_ssh():
            return None
        if isinstance(remote_candidates, str):
            candidates = [remote_candidates]
        else:
            candidates = [str(x).strip() for x in (remote_candidates or []) if str(x).strip()]
        test_flag = "-d" if is_dir else "-f"
        for p in candidates:
            cmd = f"if [ {test_flag} {shlex.quote(p)} ]; then echo __FOUND__:{p}; fi"
            out, _err = self._run_bash(self.ssh, cmd)
            text = (out or "").strip()
            if "__FOUND__:" in text:
                return text.split("__FOUND__:", 1)[1].strip()
        return None

    def _pull_log_from_little_brain(self, remote_path: str, desc: str, is_dir: bool):
        if not self._ensure_ssh():
            return

        remote_path_value = str(remote_path or "").strip()

        local_parent = QFileDialog.getExistingDirectory(self, "选择本地保存目录")
        if not local_parent:
            return

        def worker():
            try:
                if not remote_path_value:
                    self.log_signal.emit(f"[ERR] 拉取{desc}失败: 远端路径为空")
                    return

                base = os.path.basename(remote_path_value.rstrip("/")) or "log"
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                if is_dir:
                    local_target = os.path.join(local_parent, f"{base}_{stamp}")
                else:
                    stem, ext = os.path.splitext(base)
                    local_target = os.path.join(local_parent, f"{stem}_{stamp}{ext}")

                if is_dir:
                    os.makedirs(local_target, exist_ok=True)
                    remote_tar = f"/tmp/{base}_{stamp}.tar.gz"
                    pack_cmd = f"tar -czf {shlex.quote(remote_tar)} -C {shlex.quote(remote_path_value)} ."
                    out, err = self._run_bash(self.ssh, pack_cmd)
                    if err and "No such file" in err:
                        raise RuntimeError(err.strip())
                    local_tar = os.path.join(local_parent, f"{base}_{stamp}.tar.gz")
                    try:
                        self.ssh.download(remote_tar, local_tar)
                    except Exception as e:
                        # 某些网络环境下，SFTP 大文件传输可能报 Garbage packet received；回退到 SSH 流式拉取。
                        if "Garbage packet received" not in str(e):
                            raise
                        self.log_signal.emit(f"[WARN] SFTP下载失败，改用SSH流式拉取: {e}")
                        self._download_remote_tar_via_ssh_stream(remote_path_value, local_tar)
                    try:
                        with tarfile.open(local_tar, "r:gz") as tf:
                            tf.extractall(local_target)
                    finally:
                        try:
                            os.remove(local_tar)
                        except Exception:
                            pass
                        try:
                            self._run_bash(self.ssh, f"rm -f {shlex.quote(remote_tar)}")
                        except Exception:
                            pass
                else:
                    self.ssh.download(remote_path_value, local_target)
                self.log_signal.emit(f"[OK] 已拉取{desc}: {remote_path_value} -> {local_target}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 拉取{desc}失败: {e}")

        self._run_async(worker)

    def pull_middleware_log(self):
        if not self._ensure_feature_enabled("small_brain_pull_middleware_log"):
            return
        self._pull_log_from_little_brain(
            "/home/nav01/zj_humanoid/logs/run.log",
            "运控算法日志",
            is_dir=False,
        )

    def pull_embedded_logs(self):
        if not self._ensure_feature_enabled("small_brain_pull_embedded_logs"):
            return
        if not self._ensure_ssh():
            return

        def _embedded_archive_filter(path: str) -> bool:
            low = str(path or "").strip().lower()
            return low.endswith((".tar.gz", ".tgz", ".tar", ".zip", ".gz"))

        remote_paths = self._browse_sftp_path_dialog(
            self.ssh,
            "小脑",
            "选择嵌入式日志压缩包",
            start_path="/var/www/html/log/sdk",
            select_kind="file",
            file_filter=_embedded_archive_filter,
            multi_select=True,
        )
        if not remote_paths:
            return

        download_targets = self._choose_local_download_targets(
            remote_paths,
            single_title="选择本地保存路径",
            multi_title="选择本地保存目录",
        )
        if not download_targets:
            return

        def worker():
            failed = []
            for remote_path, local_path in download_targets:
                try:
                    self.ssh.download(remote_path, local_path)
                    self.log_signal.emit(f"[OK] 已拉取嵌入式软件日志压缩包: {remote_path} -> {local_path}")
                except Exception as e:
                    failed.append(f"{remote_path}: {e}")
                    self.log_signal.emit(f"[ERR] 拉取嵌入式软件日志压缩包失败: {remote_path} -> {local_path} | {e}")
            if len(download_targets) > 1:
                ok_count = len(download_targets) - len(failed)
                if failed:
                    self.log_signal.emit(f"[WARN] 批量拉取嵌入式软件日志压缩包完成: 成功 {ok_count}/{len(download_targets)}")
                else:
                    self.log_signal.emit(f"[OK] 批量拉取嵌入式软件日志压缩包完成: 共 {ok_count} 个文件")

        self._run_async(worker)

    def refresh_ros_lists(self):
        def worker():
            try:
                if self.ros and self.ros.check_connection():
                    raw_topics = self.ros.list_topics() or []
                    raw_services = self.ros.list_services() or []
                else:
                    if not self._ensure_ssh():
                        return
                    self.log_signal.emit("[INFO] ROSBridge 未连接，改用 SSH(小脑) 获取话题/服务")
                    out_topics, err_topics = self._run_ros_cli_via_ssh("rostopic list")
                    out_services, err_services = self._run_ros_cli_via_ssh("rosservice list")

                    topic_text = (out_topics or "").strip() or (err_topics or "").strip()
                    service_text = (out_services or "").strip() or (err_services or "").strip()

                    raw_topics = [x.strip() for x in topic_text.splitlines() if x.strip().startswith("/")]
                    raw_services = [x.strip() for x in service_text.splitlines() if x.strip().startswith("/")]

                # 去重 + 排序
                topics = sorted({t.strip() for t in raw_topics if isinstance(t, str) and t.strip()})
                services = sorted({s.strip() for s in raw_services if isinstance(s, str) and s.strip()})

                # 默认过滤系统项，避免数量看起来异常偏大
                display_topics = [t for t in topics if not self._is_system_topic(t)]
                display_services = [s for s in services if not self._is_system_service(s)]
                self.ros_lists_loaded_signal.emit(display_topics, display_services)
            except Exception as e:
                self.log_signal.emit(f"[ERR] 刷新话题/服务失败: {e}")

        self._run_async(worker)

    def start_monitor(self):
        ros_ok = bool(self.ros and self.ros.check_connection())
        if not ros_ok and not self._ensure_ssh():
            return

        def worker():
            try:
                if self.ros and self.ros.check_connection():
                    if self._monitor_started and self._joint_monitor_subscription_alive():
                        self.log_signal.emit("[INFO] 监控已启动，无需重复订阅")
                        return
                    self._monitor_started = True
                    self._joint_monitor_last_recover_ts = 0.0
                    self._joint_monitor_wait_first_sample_until = time.monotonic() + 6.0
                    self.joint_monitor_health_signal.emit("连接中", "-")
                    self._restart_joint_monitor_subscription("启动监控")
                    self._refresh_robot_status_view()
                    return

                self.log_signal.emit("[INFO] ROSBridge 未连接，改用 SSH(小脑) 单次读取关节数据")
                out_js, err_js = self._run_ros_cli_via_ssh("rostopic echo -n 1 /zj_humanoid/upperlimb/joint_states")
                js_text = (out_js or "") + "\n" + (err_js or "")
                js_msg = self._parse_ros_cli_yaml(js_text) or {}
                names = js_msg.get("name") or self._extract_string_list_field(js_text, "name") or []
                pos = js_msg.get("position") or self._extract_float_list_field(js_text, "position") or []
                data = {}
                if isinstance(names, list) and isinstance(pos, list) and names and pos:
                    for n, p in zip(names, pos):
                        data[str(n)] = p
                elif isinstance(pos, list):
                    for i, name in enumerate(self.joint_names):
                        if i < len(pos):
                            data[name] = pos[i]
                self.joints_map_signal.emit(data)
                self.joint_monitor_health_signal.emit("SSH单次刷新", datetime.now().strftime("%H:%M:%S"))
                if self._joint_recording and data:
                    t = max(0.0, time.monotonic() - self._joint_record_start_ts)
                    with self._joint_record_lock:
                        self._joint_record_samples.append({
                            "t": t,
                            "position": {str(k): float(v) for k, v in data.items()},
                            "velocity": {},
                            "effort": {},
                        })

                out_pose_left, err_pose_left = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {self._tcp_pose_left_monitor_topic}")
                pose_left_msg = self._parse_ros_cli_yaml((out_pose_left or "") + "\n" + (err_pose_left or "")) or {}
                out_pose_right, err_pose_right = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {self._tcp_pose_right_monitor_topic}")
                pose_right_msg = self._parse_ros_cli_yaml((out_pose_right or "") + "\n" + (err_pose_right or "")) or {}
                self.tcp_pose_monitor_signal.emit({
                    "left": pose_left_msg if isinstance(pose_left_msg, dict) else {},
                    "right": pose_right_msg if isinstance(pose_right_msg, dict) else {},
                })

                out_speed, err_speed = self._run_ros_cli_via_ssh(f"rostopic echo -n 1 {self._tcp_speed_monitor_topic}")
                speed_msg = self._parse_ros_cli_yaml((out_speed or "") + "\n" + (err_speed or "")) or {}
                self.tcp_speed_monitor_signal.emit(speed_msg if isinstance(speed_msg, dict) else {})

                self.log_signal.emit("[OK] 已通过 SSH 刷新一次关节监控")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 监控失败: {e}")
        self._run_async(worker)

    def _build_joint_state_sample(self, msg: dict):
        if not isinstance(msg, dict):
            return None
        names_raw = msg.get("name")
        pos_raw = msg.get("position")
        vel_raw = msg.get("velocity")
        effort_raw = msg.get("effort")

        names = [str(x) for x in names_raw] if isinstance(names_raw, list) and names_raw else list(self.joint_names)
        if not isinstance(pos_raw, list):
            return None

        def _to_map(values, fallback):
            out = {}
            src = values if isinstance(values, list) else []
            for i, n in enumerate(names):
                if i >= len(src):
                    if fallback is not None:
                        out[n] = fallback
                    continue
                try:
                    out[n] = float(src[i])
                except Exception:
                    if fallback is not None:
                        out[n] = fallback
            return out

        pos_map = _to_map(pos_raw, None)
        if not pos_map:
            return None
        sample = {
            "t": max(0.0, time.monotonic() - self._joint_record_start_ts),
            "position": pos_map,
            "velocity": _to_map(vel_raw, float("nan")),
            "effort": _to_map(effort_raw, float("nan")),
        }
        return sample

    def _capture_joint_state_sample(self, sample: dict):
        if not self._joint_recording:
            return
        with self._joint_record_lock:
            self._joint_record_samples.append(sample)

    def _build_joint_state_sample_at(self, msg: dict, start_ts: float):
        sample = self._build_joint_state_sample(msg)
        if not sample:
            return None
        sample["t"] = max(0.0, time.monotonic() - float(start_ts or 0.0))
        return sample

    def _stop_servoj_tool_subscription(self):
        topic = getattr(self, "_servoj_tool_topic", None)
        self._servoj_tool_topic = None
        if topic is not None:
            try:
                topic.unsubscribe()
            except Exception:
                pass

    def _handle_servoj_tool_joint_msg(self, msg: dict):
        if not self._alive or not self._servoj_tool_recording:
            return
        try:
            sample = self._build_joint_state_sample_at(msg, self._servoj_tool_start_ts)
            if not sample:
                return
            with self._servoj_tool_lock:
                self._servoj_tool_samples.append(sample)
        except RuntimeError:
            pass
        except Exception:
            pass

    def _clear_joint_control_trace(self, trace_kind: str):
        kind = "single" if str(trace_kind).lower() == "single" else "seq"
        with self._joint_ctrl_trace_lock:
            if kind == "single":
                self._joint_ctrl_trace_single = []
            else:
                self._joint_ctrl_trace_seq = []

    def _append_joint_control_trace_sample(self, trace_kind: str, t: float, desired_map: dict, actual_map: dict, tag: str = ""):
        kind = "single" if str(trace_kind).lower() == "single" else "seq"
        item = {
            "t": max(0.0, float(t)),
            "desired": dict(desired_map or {}),
            "actual": dict(actual_map or {}),
            "tag": str(tag or ""),
        }
        with self._joint_ctrl_trace_lock:
            if kind == "single":
                self._joint_ctrl_trace_single.append(item)
            else:
                self._joint_ctrl_trace_seq.append(item)

    def _get_joint_control_trace(self, trace_kind: str):
        kind = "single" if str(trace_kind).lower() == "single" else "seq"
        with self._joint_ctrl_trace_lock:
            if kind == "single":
                return list(self._joint_ctrl_trace_single)
            return list(self._joint_ctrl_trace_seq)

    def _get_latest_joint_state_map(self, max_age_sec: float = None):
        with self._latest_joint_state_lock:
            latest = dict(self._latest_joint_state_map)
            ts = float(self._latest_joint_state_ts or 0.0)
        if not latest:
            return {}
        if max_age_sec is not None and ts > 0.0:
            if (time.monotonic() - ts) > float(max_age_sec):
                return {}
        return latest

    def _handle_joint_monitor_msg(self, msg: dict):
        if not self._alive:
            return
        try:
            sample = self._build_joint_state_sample(msg)
            if sample:
                self.joints_map_signal.emit(sample["position"])
                self._capture_joint_state_sample(sample)
                self._joint_monitor_wait_first_sample_until = 0.0
                self.joint_monitor_health_signal.emit("已连接", datetime.now().strftime("%H:%M:%S"))
            else:
                self.log_signal.emit(json.dumps(msg, ensure_ascii=False)[:120])
        except RuntimeError:
            pass
        except Exception:
            pass

    def _emit_tcp_pose_monitor_payload(self):
        try:
            self.tcp_pose_monitor_signal.emit(dict(self._tcp_pose_monitor_cache))
        except RuntimeError:
            pass
        except Exception:
            pass

    def _handle_tcp_pose_left_monitor_msg(self, msg: dict):
        if not self._alive:
            return
        try:
            self._tcp_pose_monitor_cache["left"] = msg if isinstance(msg, dict) else {}
            self._emit_tcp_pose_monitor_payload()
        except RuntimeError:
            pass
        except Exception:
            pass

    def _handle_tcp_pose_right_monitor_msg(self, msg: dict):
        if not self._alive:
            return
        try:
            self._tcp_pose_monitor_cache["right"] = msg if isinstance(msg, dict) else {}
            self._emit_tcp_pose_monitor_payload()
        except RuntimeError:
            pass
        except Exception:
            pass

    def _handle_tcp_speed_monitor_msg(self, msg: dict):
        if not self._alive:
            return
        try:
            self.tcp_speed_monitor_signal.emit(msg if isinstance(msg, dict) else {})
        except RuntimeError:
            pass
        except Exception:
            pass

    def _joint_monitor_subscription_alive(self) -> bool:
        with self._joint_monitor_client_lock:
            client = self.ros_monitor
        if not client or not client.check_connection():
            return False
        try:
            return bool(getattr(client, "_subscriptions", {}).get(self._joint_monitor_topic))
        except Exception:
            return False

    def _disconnect_joint_monitor_client(self):
        with self._joint_monitor_client_lock:
            client = self.ros_monitor
            self.ros_monitor = None
            self._joint_monitor_recovering = False
            self._joint_monitor_wait_first_sample_until = 0.0
        if client:
            try:
                client.disconnect()
            except Exception:
                pass
        self.joint_monitor_health_signal.emit("未连接", "-")
        self._reset_tcp_monitor_views()

    def _restart_joint_monitor_subscription(self, reason: str = ""):
        if not self._alive or not self._monitor_started:
            return
        if not (self.ros and self.ros.check_connection()):
            return

        with self._joint_monitor_client_lock:
            if self._joint_monitor_recovering:
                return
            self._joint_monitor_recovering = True
            self._joint_monitor_wait_first_sample_until = time.monotonic() + 6.0

        host = self.ros_host.text().strip()
        port_text = self.ros_port.text().strip()

        def worker():
            old_client = None
            try:
                try:
                    port = int(port_text)
                except Exception as e:
                    raise RuntimeError(f"ROSBridge 端口无效: {e}")

                with self._joint_monitor_client_lock:
                    old_client = self.ros_monitor
                    self.ros_monitor = None

                if old_client:
                    try:
                        old_client.disconnect()
                    except Exception:
                        pass

                client = BridgeClient(host, port)
                if not client.connect(timeout=4.0):
                    raise RuntimeError("独立关节监听通道连接失败")
                client.subscribe(self._joint_monitor_topic, self._handle_joint_monitor_msg)
                try:
                    client.subscribe(self._tcp_pose_left_monitor_topic, self._handle_tcp_pose_left_monitor_msg)
                except Exception as e:
                    self.log_signal.emit(f"[WARN] 左臂末端位置话题订阅失败: {self._tcp_pose_left_monitor_topic} ({e})")
                try:
                    client.subscribe(self._tcp_pose_right_monitor_topic, self._handle_tcp_pose_right_monitor_msg)
                except Exception as e:
                    self.log_signal.emit(f"[WARN] 右臂末端位置话题订阅失败: {self._tcp_pose_right_monitor_topic} ({e})")
                try:
                    client.subscribe(self._tcp_speed_monitor_topic, self._handle_tcp_speed_monitor_msg)
                except Exception as e:
                    self.log_signal.emit(f"[WARN] 末端速度话题订阅失败: {self._tcp_speed_monitor_topic} ({e})")

                with self._joint_monitor_client_lock:
                    self.ros_monitor = client

                reason_text = f" ({reason})" if reason else ""
                self.joint_monitor_health_signal.emit(f"已连接{reason_text}", "等待数据")
                self.log_signal.emit(f"[OK] 已建立独立关节监听通道{reason_text}")
            except Exception as e:
                self.joint_monitor_health_signal.emit(f"恢复失败: {e}", "-")
                self.log_signal.emit(f"[WARN] 重建关节监听失败: {e}")
            finally:
                with self._joint_monitor_client_lock:
                    self._joint_monitor_recovering = False

        self._run_async(worker)

    def _check_joint_monitor_health(self):
        if not self._alive or not self._monitor_started:
            return
        if not (self.ros and self.ros.check_connection()):
            return
        if self._joint_monitor_recovering:
            return

        with self._latest_joint_state_lock:
            ts = float(self._latest_joint_state_ts or 0.0)
        now = time.monotonic()
        age = (now - ts) if ts > 0.0 else float("inf")
        subscription_alive = self._joint_monitor_subscription_alive()
        wait_deadline = float(self._joint_monitor_wait_first_sample_until or 0.0)
        if ts <= 0.0 and wait_deadline > now:
            return
        if subscription_alive and age <= float(self._joint_monitor_fresh_timeout_sec):
            return
        if (now - float(self._joint_monitor_last_recover_ts or 0.0)) < max(4.0, float(self._joint_monitor_fresh_timeout_sec)):
            return

        self._joint_monitor_last_recover_ts = now
        reason = []
        if not subscription_alive:
            reason.append("订阅缺失")
        if age > float(self._joint_monitor_fresh_timeout_sec):
            reason.append(f"数据{age:.1f}s未更新")
        reason_text = ", ".join(reason) if reason else "监控异常"
        self.joint_monitor_health_signal.emit(f"恢复中: {reason_text}", datetime.now().strftime("%H:%M:%S"))
        self.log_signal.emit(f"[WARN] 检测到关节监听失活，正在恢复: {reason_text}")
        self._restart_joint_monitor_subscription(reason_text)

    def _sample_control_trace(self, trace_kind: str, start_ts: float, duration: float, desired_fn, stop_event: threading.Event, tag: str = ""):
        duration = max(0.0, float(duration))
        sample_interval = 0.2
        while True:
            if stop_event.is_set():
                break
            elapsed = max(0.0, time.monotonic() - start_ts)
            desired_map = desired_fn(elapsed)
            actual_map = self._get_latest_joint_state_map(self._joint_state_fresh_timeout_sec)
            if not actual_map:
                actual_map = self._fetch_joint_state_once() or self._get_latest_joint_state_map()
            self._append_joint_control_trace_sample(trace_kind, elapsed, desired_map, actual_map, tag=tag)
            if elapsed >= duration:
                break
            time.sleep(sample_interval)

    def _desired_map_for_group(self, arm_type: int, poses: list, durations: list, elapsed: float):
        names = self._joint_names_for_arm_type(int(arm_type))
        if not names or not poses:
            return {}
        idx = 0
        remain = max(0.0, float(elapsed))
        for i, d in enumerate(durations):
            seg_d = max(1e-6, float(d))
            if remain <= seg_d:
                idx = i
                break
            remain -= seg_d
            idx = min(i + 1, len(poses) - 1)
        pose = poses[min(idx, len(poses) - 1)]
        out = {}
        for j, name in enumerate(names):
            if j < len(pose):
                try:
                    out[name] = float(pose[j])
                except Exception:
                    continue
        return out

    def _load_matplotlib_modules(self, need_animation: bool = False):
        try:
            plt = importlib.import_module("matplotlib.pyplot")
            anim_mod = importlib.import_module("matplotlib.animation") if need_animation else None
            return plt, anim_mod
        except Exception as e:
            err_text = f"{type(e).__name__}: {e}"
            self.log_signal.emit(f"[WARN] matplotlib 导入失败: python={sys.executable}, err={err_text}")
            ret = QMessageBox.question(
                self,
                "绘图依赖异常",
                f"当前环境导入 matplotlib 失败。\n"
                f"Python: {sys.executable}\n"
                f"错误: {err_text}\n\n"
                "是否现在自动安装？\n(将执行: python -m pip install matplotlib)",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.Yes,
            )
            if ret != QMessageBox.StandardButton.Yes:
                QMessageBox.warning(
                    self,
                    "绘图依赖异常",
                    f"绘图依赖导入失败。\nPython: {sys.executable}\n错误: {err_text}",
                )
                return None, None
            try:
                cmd = [sys.executable, "-m", "pip", "install", "matplotlib"]
                p = subprocess.run(cmd, capture_output=True, text=True, timeout=300, check=False)
                if p.returncode != 0:
                    err = (p.stderr or p.stdout or "安装失败").strip()
                    QMessageBox.warning(self, "安装失败", f"自动安装 matplotlib 失败:\n{err}")
                    return None, None
                plt = importlib.import_module("matplotlib.pyplot")
                anim_mod = importlib.import_module("matplotlib.animation") if need_animation else None
                self.log_signal.emit("[OK] matplotlib 安装完成")
                return plt, anim_mod
            except Exception as ie:
                QMessageBox.warning(self, "安装失败", f"自动安装 matplotlib 失败:\n{ie}")
                return None, None

    def _show_plot_dialog(self, fig, title: str):
        try:
            backend = importlib.import_module("matplotlib.backends.backend_qtagg")
            FigureCanvas = getattr(backend, "FigureCanvasQTAgg")
            NavigationToolbar = getattr(backend, "NavigationToolbar2QT")
        except Exception:
            # 兜底: 无法嵌入Qt时回退到matplotlib窗口
            try:
                plt = importlib.import_module("matplotlib.pyplot")
                plt.show(block=False)
            except Exception:
                pass
            return None

        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.resize(1080, 760)
        lay = QVBoxLayout(dlg)
        canvas = FigureCanvas(fig)
        toolbar = NavigationToolbar(canvas, dlg)
        lay.addWidget(toolbar)
        lay.addWidget(canvas)
        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        btns.rejected.connect(dlg.close)
        btns.accepted.connect(dlg.close)
        lay.addWidget(btns)

        def _cleanup():
            try:
                if dlg in self._plot_dialog_refs:
                    self._plot_dialog_refs.remove(dlg)
            except Exception:
                pass

        dlg.finished.connect(lambda _=None: _cleanup())
        self._plot_dialog_refs.append(dlg)
        dlg.show()
        return dlg

    def plot_joint_control_trace(self):
        seq_data = self._get_joint_control_trace("seq")
        single_data = self._get_joint_control_trace("single")
        options = []
        if seq_data:
            options.append("动作序列")
        if single_data:
            options.append("单步调试")
        if not options:
            QMessageBox.information(self, "提示", "暂无控制轨迹数据，请先执行动作序列或单步调试。")
            return

        kind_text, ok = QInputDialog.getItem(self, "选择数据", "请选择控制轨迹来源", options, 0, False)
        if not ok or not kind_text:
            return
        trace_kind = "seq" if kind_text == "动作序列" else "single"
        data = seq_data if trace_kind == "seq" else single_data
        if not data:
            QMessageBox.information(self, "提示", "所选来源暂无可绘图数据。")
            return

        names = []
        seen = set()
        for item in data:
            for n in item.get("desired", {}).keys():
                if n not in seen:
                    seen.add(n)
                    names.append(n)
            for n in item.get("actual", {}).keys():
                if n not in seen:
                    seen.add(n)
                    names.append(n)
        if not names:
            QMessageBox.warning(self, "提示", "轨迹中没有关节字段。")
            return

        option_to_joints = {"全部关节": list(names)}
        group_defs = [
            ("左臂关节", 1),
            ("右臂关节", 2),
            ("脖子关节", 4),
            ("腰部关节", 8),
        ]
        for label, arm_type in group_defs:
            group_names = [n for n in self._joint_names_for_arm_type(arm_type) if n in names]
            if group_names:
                option_to_joints[label] = group_names
        for n in names:
            option_to_joints[n] = [n]

        joint_options = list(option_to_joints.keys())
        selected_option, ok = QInputDialog.getItem(self, "选择关节", "选择需要对比的关节", joint_options, 0, False)
        if not ok or not selected_option:
            return
        joint_list = option_to_joints.get(selected_option) or []
        if not joint_list:
            QMessageBox.warning(self, "提示", "所选关节分组无可用数据。")
            return

        plt, _ = self._load_matplotlib_modules(need_animation=False)
        if plt is None:
            return

        t = [float(x.get("t", 0.0)) for x in data]
        if len(joint_list) > 1:
            cols = 2
            rows = max(1, (len(joint_list) + cols - 1) // cols)
            fig, axes = plt.subplots(rows, cols, figsize=(12, max(5, rows * 2.8)), sharex=True)
            axes_flat = axes.flatten() if hasattr(axes, "flatten") else [axes]
            for idx, name in enumerate(joint_list):
                ax = axes_flat[idx]
                desired = [x.get("desired", {}).get(name, float("nan")) for x in data]
                actual = [x.get("actual", {}).get(name, float("nan")) for x in data]
                ax.plot(t, desired, "--", linewidth=1.4, color="#d62728", label="期望")
                ax.plot(t, actual, linewidth=1.4, color="#1f77b4", label="实时")
                ax.set_title(name)
                ax.grid(True, alpha=0.3)
            for j in range(len(joint_list), len(axes_flat)):
                axes_flat[j].axis("off")
            if len(axes_flat) > 0:
                axes_flat[0].legend(loc="best")
            fig.suptitle(f"关节控制对比 - {kind_text} - {selected_option}")
            fig.tight_layout()
            self._show_plot_dialog(fig, f"控制Plot - {kind_text} - {selected_option}")
            return

        joint_name = joint_list[0]
        desired = [x.get("desired", {}).get(joint_name, float("nan")) for x in data]
        actual = [x.get("actual", {}).get(joint_name, float("nan")) for x in data]
        fig, ax = plt.subplots(figsize=(10, 4.5))
        ax.plot(t, desired, "--", linewidth=2.0, color="#d62728", label="期望位置")
        ax.plot(t, actual, linewidth=2.0, color="#1f77b4", label="实时位置")
        ax.set_title(f"关节控制对比 - {kind_text} - {selected_option}")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("position (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        self._show_plot_dialog(fig, f"控制Plot - {kind_text} - {selected_option}")

    def toggle_joint_recording(self):
        if self._joint_recording:
            self.stop_joint_recording()
            return

        if not (self.ros and self.ros.check_connection()):
            QMessageBox.information(self, "提示", "录制需要 ROSBridge 持续订阅。请先连接 ROSBridge 后再录制。")
            return

        if not self._monitor_started:
            self.start_monitor()

        with self._joint_record_lock:
            self._joint_record_samples = []
        self._joint_record_start_ts = time.monotonic()
        self._joint_recording = True
        self.btn_joint_record.setEnabled(False)
        self.btn_joint_record_stop.setEnabled(True)
        self.joint_record_status_value.setText("录制中...")
        self.log_signal.emit("[OK] 开始录制 joint_states(position/velocity/effort)")

    def stop_joint_recording(self):
        if not self._joint_recording:
            self.log_signal.emit("[INFO] 当前未在录制")
            return

        self._joint_recording = False
        self.btn_joint_record.setEnabled(True)
        self.btn_joint_record_stop.setEnabled(False)
        with self._joint_record_lock:
            count = len(self._joint_record_samples)
            duration = self._joint_record_samples[-1]["t"] if self._joint_record_samples else 0.0
        self.joint_record_status_value.setText(f"已停止: {count} 帧 / {duration:.2f}s")
        self.log_signal.emit(f"[OK] 已停止录制 joint_states，共 {count} 帧")

    def plot_recorded_joint_state(self):
        with self._joint_record_lock:
            samples = list(self._joint_record_samples)
        if len(samples) < 2:
            QMessageBox.information(self, "提示", "录制数据不足，请先录制至少 2 帧再绘图。")
            return

        all_names = []
        seen = set()
        for item in samples:
            for n in item.get("position", {}).keys():
                if n not in seen:
                    seen.add(n)
                    all_names.append(n)
        if not all_names:
            QMessageBox.warning(self, "提示", "未找到可绘制的关节数据。")
            return

        option_to_joints = {"全部关节": list(all_names)}
        group_defs = [
            ("左臂关节", 1),
            ("右臂关节", 2),
            ("脖子关节", 4),
            ("腰部关节", 8),
        ]
        for label, arm_type in group_defs:
            group_names = [n for n in self._joint_names_for_arm_type(arm_type) if n in all_names]
            if group_names:
                option_to_joints[label] = group_names
        for n in all_names:
            option_to_joints[n] = [n]

        joint_options = list(option_to_joints.keys())
        selected_option, ok = QInputDialog.getItem(self, "选择关节", "选择需要绘图的关节", joint_options, 0, False)
        if not ok or not selected_option:
            return
        joint_list = option_to_joints.get(selected_option) or []
        if not joint_list:
            QMessageBox.warning(self, "提示", "所选关节分组无可用数据。")
            return

        mode, ok = QInputDialog.getItem(self, "绘图模式", "请选择绘图模式", ["静态", "滚动"], 1 if self._joint_recording else 0, False)
        if not ok or not mode:
            return

        plt, anim_mod = self._load_matplotlib_modules(need_animation=True)
        if plt is None:
            return

        if mode == "滚动":
            self._plot_recorded_joint_state_live(selected_option, joint_list, plt, anim_mod)
            return

        t = [float(x.get("t", 0.0)) for x in samples]
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        for name in joint_list:
            p = [x.get("position", {}).get(name, float("nan")) for x in samples]
            v = [x.get("velocity", {}).get(name, float("nan")) for x in samples]
            e = [x.get("effort", {}).get(name, float("nan")) for x in samples]
            axes[0].plot(t, p, linewidth=1.2, label=name)
            axes[1].plot(t, v, linewidth=1.2, label=name)
            axes[2].plot(t, e, linewidth=1.2, label=name)

        axes[0].set_ylabel("position (rad)")
        axes[0].grid(True, alpha=0.3)
        axes[1].set_ylabel("velocity (rad/s)")
        axes[1].grid(True, alpha=0.3)
        axes[2].set_ylabel("effort (Nm)")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True, alpha=0.3)

        if len(joint_list) > 1:
            axes[0].legend(loc="best", fontsize=8, ncol=2)

        fig.suptitle(f"JointState Plot - {selected_option}")
        fig.tight_layout()
        self._show_plot_dialog(fig, f"JointState Plot - {selected_option}")

    def _plot_recorded_joint_state_live(self, selected_option: str, joint_list: list, plt, anim_mod):
        old = self._joint_live_plot
        if isinstance(old, dict):
            try:
                anim = old.get("anim")
                if anim is not None:
                    anim.event_source.stop()
            except Exception:
                pass
            try:
                fig_old = old.get("fig")
                if fig_old is not None:
                    plt.close(fig_old)
            except Exception:
                pass
            try:
                dlg_old = old.get("dialog")
                if dlg_old is not None:
                    dlg_old.close()
            except Exception:
                pass

        window_sec, ok = QInputDialog.getDouble(self, "滚动窗口", "滚动窗口时长(秒)", 10.0, 1.0, 120.0, 1)
        if not ok:
            return

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        line_p = {n: axes[0].plot([], [], linewidth=1.2, label=n)[0] for n in joint_list}
        line_v = {n: axes[1].plot([], [], linewidth=1.2, label=n)[0] for n in joint_list}
        line_e = {n: axes[2].plot([], [], linewidth=1.2, label=n)[0] for n in joint_list}

        axes[0].set_ylabel("position (rad)")
        axes[1].set_ylabel("velocity (rad/s)")
        axes[2].set_ylabel("effort (Nm)")
        axes[2].set_xlabel("time (s)")
        axes[0].grid(True, alpha=0.3)
        axes[1].grid(True, alpha=0.3)
        axes[2].grid(True, alpha=0.3)
        fig.suptitle(f"JointState Rolling Plot - {selected_option}")
        if len(joint_list) > 1:
            axes[0].legend(loc="best", fontsize=8, ncol=2)

        def _safe_y_limits(y_values):
            finite = [float(v) for v in y_values if isinstance(v, (int, float)) and math.isfinite(v)]
            if not finite:
                return (-1.0, 1.0)
            ymin = min(finite)
            ymax = max(finite)
            if abs(ymax - ymin) < 1e-9:
                pad = max(1e-3, abs(ymax) * 0.1 + 1e-3)
                return (ymin - pad, ymax + pad)
            pad = (ymax - ymin) * 0.1
            return (ymin - pad, ymax + pad)

        def _update(_frame):
            with self._joint_record_lock:
                samples = list(self._joint_record_samples)
            if len(samples) < 1:
                return tuple(list(line_p.values()) + list(line_v.values()) + list(line_e.values()))

            t = [float(x.get("t", 0.0)) for x in samples]

            p_all = {}
            v_all = {}
            e_all = {}
            for n in joint_list:
                p_all[n] = [x.get("position", {}).get(n, float("nan")) for x in samples]
                v_all[n] = [x.get("velocity", {}).get(n, float("nan")) for x in samples]
                e_all[n] = [x.get("effort", {}).get(n, float("nan")) for x in samples]
                line_p[n].set_data(t, p_all[n])
                line_v[n].set_data(t, v_all[n])
                line_e[n].set_data(t, e_all[n])

            t_end = t[-1] if t else 0.0
            t_start = max(0.0, t_end - float(window_sec))
            t_right = max(t_end, t_start + 0.2)
            for ax in axes:
                ax.set_xlim(t_start, t_right)

            p_view = [yy for n in joint_list for tt, yy in zip(t, p_all[n]) if tt >= t_start]
            v_view = [yy for n in joint_list for tt, yy in zip(t, v_all[n]) if tt >= t_start]
            e_view = [yy for n in joint_list for tt, yy in zip(t, e_all[n]) if tt >= t_start]

            axes[0].set_ylim(*_safe_y_limits(p_view))
            axes[1].set_ylim(*_safe_y_limits(v_view))
            axes[2].set_ylim(*_safe_y_limits(e_view))
            return tuple(list(line_p.values()) + list(line_v.values()) + list(line_e.values()))

        anim = anim_mod.FuncAnimation(fig, _update, interval=200, blit=False, cache_frame_data=False)
        fig.tight_layout()
        dlg = self._show_plot_dialog(fig, f"JointState Rolling Plot - {selected_option}")
        self._joint_live_plot = {
            "fig": fig,
            "anim": anim,
            "dialog": dlg,
            "joint_name": selected_option,
            "window_sec": float(window_sec),
        }
        self.log_signal.emit(f"[OK] 已打开滚动绘图: {selected_option}, 窗口 {float(window_sec):.1f}s")

    def run_command(self):
        if self._cmd_echo_topic:
            self.log_signal.emit("[INFO] 当前有 rostopic echo 正在输出，请先点击“停止输出”")
            return
        cmd = self.cmd_edit.text().strip()
        if not cmd:
            return

        def worker():
            try:
                if self.ros and self.ros.check_connection():
                    result = self.ros.execute_command(cmd, self.log_signal.emit)
                    if isinstance(result, dict) and result.get("kind") == "topic_echo":
                        self._cmd_echo_topic = result.get("topic")
                        self.command_btn_signal.emit(True)
                        self._refresh_robot_status_view()
                        self.log_signal.emit("[INFO] 已进入持续输出模式，可点击“停止输出”")
                    return

                # ROSBridge 不可用时，走 SSH(小脑)
                if not self._ensure_ssh():
                    return
                self.log_signal.emit("[INFO] ROSBridge 未连接，改用 SSH(小脑) 执行命令")

                ssh_cmd = cmd
                if "rostopic echo" in cmd and "--once" not in cmd:
                    ssh_cmd = f"{cmd} -n 1"
                    self.log_signal.emit("[INFO] SSH 模式下 rostopic echo 自动追加 -n 1")

                out, err = self._run_ros_cli_via_ssh(ssh_cmd)
                text = (out or "").strip()
                err_text = (err or "").strip()
                if text:
                    self.log_signal.emit(text)
                if err_text:
                    self.log_signal.emit(err_text)
                if not text and not err_text:
                    self.log_signal.emit("[OK] 命令已执行")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 命令执行失败: {e}")
        self._run_async(worker)

    def stop_command_echo(self):
        if not self._ensure_ros():
            return
        topic = self._cmd_echo_topic
        if not topic:
            self.log_signal.emit("[INFO] 当前没有正在持续输出的话题")
            self.command_btn_signal.emit(False)
            return

        def worker():
            try:
                if self.ros:
                    self.ros.unsubscribe(topic)
                self.log_signal.emit(f"[OK] 已停止输出: {topic}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 停止输出失败: {e}")
            finally:
                self._cmd_echo_topic = None
                self.command_btn_signal.emit(False)
                self._refresh_robot_status_view()

        self._run_async(worker)

    def _go_home_for_part(self, arm_type: int, part: str, emit_line, transport_mode: str | None = None):
        service_name = self._home_service_for_part(arm_type)
        transport = str(transport_mode or getattr(self, "_joint_ctrl_transport_mode", "ros") or "ros").strip().lower()
        is_full_body = int(arm_type) in (15, 31)
        if is_full_body:
            home_targets = [
                ("双臂", "/zj_humanoid/upperlimb/go_down/dual_arm"),
                ("脖子", "/zj_humanoid/upperlimb/go_home/neck"),
                ("腰部", "/zj_humanoid/upperlimb/go_home/waist"),
            ]
        else:
            home_targets = [(part, service_name)]

        def worker():
            try:
                emit_line(f"[REQ] 归位({transport}): part={part}")
                if transport == "ros":
                    if not (self.ros and self.ros.check_connection()):
                        raise RuntimeError("ROSBridge 未连接")
                    for target_part, target_service in home_targets:
                        emit_line(f"[REQ] 归位({transport}): part={target_part}, service={target_service}")
                        resp = self.ros.request_service(target_service, {})
                        emit_line(f"[OK] 归位成功: {target_part} [{target_service}] -> {json.dumps(resp, ensure_ascii=False)}")
                    return

                if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
                    raise RuntimeError("SSH(小脑) 未连接")
                emit_line("[INFO] 使用 SSH(小脑) 归位")
                for target_part, target_service in home_targets:
                    cmd = f"rosservice call {target_service} '{{}}'"
                    emit_line(f"[REQ] {cmd}")
                    out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                    text = ((out or "") + "\n" + (err or "")).strip()
                    emit_line(f"[OK] 归位完成: {target_part} [{target_service}] -> {text or '调用完成'}")
            except Exception as e:
                emit_line(f"[ERR] 归位失败: {part} -> {e}")

        self._run_async(worker)

    def go_home(self):
        arm_type = self._current_teach_arm_type()
        part = self.joint_ctrl_part.currentText().strip()
        self._go_home_for_part(arm_type, part, self.log_signal.emit)

    def factory_go_home_dual_arm(self):
        self._go_home_for_part(3, "双臂", self._emit_factory_test, transport_mode="ssh")

    def factory_go_home_waist(self):
        self._go_home_for_part(8, "腰部", self._emit_factory_test, transport_mode="ssh")

    def unlock_upperlimb(self):
        service_name = "/zj_humanoid/upperlimb/unlock"
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

        def worker():
            try:
                self.log_signal.emit(f"[REQ] 解除保护({transport_mode}): service={service_name}")
                if transport_mode == "ros":
                    if not (self.ros and self.ros.check_connection()):
                        raise RuntimeError("ROSBridge 未连接")
                    resp = self.ros.request_service(service_name, {})
                    self.log_signal.emit(f"[OK] 解除保护成功: [{service_name}] -> {json.dumps(resp, ensure_ascii=False)}")
                    return

                if not self._ensure_ssh():
                    return
                self.log_signal.emit("[INFO] 使用 SSH(小脑) 解除保护")
                cmd = f"rosservice call {service_name} '{{}}'"
                self.log_signal.emit(f"[REQ] {cmd}")
                out, err = self._run_ros_cli_via_ssh_interactive(cmd)
                text = ((out or "") + "\n" + (err or "")).strip()
                self.log_signal.emit(f"[OK] 解除保护完成: [{service_name}] -> {text or '调用完成'}")
            except Exception as e:
                self.log_signal.emit(f"[ERR] 解除保护失败: [{service_name}] -> {e}")

        self._run_async(worker)

    def _parse_safety_lock_active(self, resp) -> bool:
        if isinstance(resp, bool):
            return bool(resp)
        if isinstance(resp, (int, float)):
            return bool(resp)
        if isinstance(resp, dict):
            for k in ("in_safety_lock", "safety_lock", "locked", "is_locked", "is_lock", "data", "state"):
                if k in resp:
                    v = resp.get(k)
                    if isinstance(v, str):
                        low = v.strip().lower()
                        if low in ("true", "1", "yes", "on", "locked", "protect", "safety_lock"):
                            return True
                        if low in ("false", "0", "no", "off", "unlock", "unlocked"):
                            return False
                    return bool(v)
            txt = json.dumps(resp, ensure_ascii=False).lower()
        else:
            txt = str(resp or "").lower()
        return any(x in txt for x in ["true", "locked", "safety_lock", "in_safety_lock", "保护", "锁定"])

    def _set_safety_lock_ui(self, active: bool, source: str):
        if active:
            self.safety_lock_view_signal.emit(f"安全保护: 已触发 ({source})", "color: #f85149; font-weight: 800;")
        else:
            self.safety_lock_view_signal.emit(f"安全保护: 正常 ({source})", "color: #3fb950; font-weight: 700;")

    def _poll_safety_lock_state(self):
        transport_mode = getattr(self, "_joint_ctrl_transport_mode", "ros")

        if getattr(self, "_safety_lock_polling", False):
            return
        self._safety_lock_polling = True

        def worker():
            state = None
            source = "-"
            query_available = False
            last_error = ""

            def _call_via_ros():
                nonlocal source, query_available, last_error
                if not (self.ros and self.ros.check_connection()):
                    return None
                query_available = True
                for service_name in self._safety_lock_service_candidates:
                    try:
                        resp = self.ros.request_service(service_name, {}, timeout=2.5)
                        source = f"ROSBridge:{service_name.rsplit('/', 1)[-1]}"
                        return self._parse_safety_lock_active(resp)
                    except Exception as e:
                        last_error = str(e)
                return None

            def _call_via_ssh():
                nonlocal source, query_available, last_error
                if not (self.ssh and self.ssh.ssh and self.ssh.sftp):
                    return None
                query_available = True
                for service_name in self._safety_lock_service_candidates:
                    try:
                        out, err = self._run_ros_cli_via_ssh_interactive(f"rosservice call {service_name} '{{}}'")
                        text = ((out or "") + "\n" + (err or "")).strip()
                        if text:
                            source = f"SSH:{service_name.rsplit('/', 1)[-1]}"
                            return self._parse_safety_lock_active(text)
                    except Exception as e:
                        last_error = str(e)
                return None

            try:
                if transport_mode == "ros" and self.ros and self.ros.check_connection():
                    state = _call_via_ros()
                elif transport_mode == "ssh" and self.ssh and self.ssh.ssh and self.ssh.sftp:
                    state = _call_via_ssh()
                elif self.ros and self.ros.check_connection():
                    state = _call_via_ros()
                elif self.ssh and self.ssh.ssh and self.ssh.sftp:
                    state = _call_via_ssh()
            except Exception as e:
                last_error = str(e)
                state = None
            finally:
                try:
                    if state is None:
                        if query_available:
                            detail = f" ({last_error})" if last_error else ""
                            self.safety_lock_view_signal.emit(
                                f"安全保护: 查询失败{detail}",
                                "color: #d29922; font-weight: 700;",
                            )
                        else:
                            self.safety_lock_view_signal.emit("安全保护: 未连接", "color: #8b949e; font-weight: 600;")
                        return

                    self._set_safety_lock_ui(state, source)
                    if self._safety_lock_last_state is None or self._safety_lock_last_state != state:
                        self._safety_lock_last_state = state
                        if state:
                            self.log_signal.emit("[WARN] 检测到上肢进入安全保护")
                        else:
                            self.log_signal.emit("[OK] 上肢安全保护已解除/未触发")
                finally:
                    self._safety_lock_polling = False

        self._run_async(worker)

    def _download_remote_tar_via_ssh_stream(self, remote_dir: str, local_tar_path: str):
        if not self._ensure_ssh():
            raise RuntimeError("SSH 未连接")

        parent = os.path.dirname(local_tar_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        stream_cmd = f"bash -lc {shlex.quote(f'tar -czf - -C {shlex.quote(remote_dir)} .')}"
        stdin, stdout, stderr = self.ssh.ssh.exec_command(stream_cmd, get_pty=False)
        if stdin is not None:
            try:
                stdin.close()
            except Exception:
                pass

        fd, tmp_path = tempfile.mkstemp(prefix="embed_logs_", suffix=".tar.gz", dir=parent or None)
        os.close(fd)
        try:
            with open(tmp_path, "wb") as f:
                while True:
                    chunk = stdout.read(1024 * 256)
                    if not chunk:
                        break
                    f.write(chunk)

            exit_code = stdout.channel.recv_exit_status()
            err_text = (stderr.read() or b"").decode("utf-8", errors="ignore").strip()
            if exit_code != 0:
                raise RuntimeError(err_text or f"远端打包失败，exit_code={exit_code}")

            os.replace(tmp_path, local_tar_path)
        finally:
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass