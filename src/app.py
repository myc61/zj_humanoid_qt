import sys
import argparse
import os
from pathlib import Path

import yaml

from core.feature_flags import DEFAULT_FEATURE_FLAGS, normalize_feature_flags


def _setup_qt_runtime_env():
    if not sys.platform.startswith("linux"):
        return

    # 诊断崩溃时保留 Python 层栈信息
    os.environ.setdefault("PYTHONFAULTHANDLER", "1")

    # 避免被外部环境注入不兼容 Qt 插件路径导致闪退
    os.environ.pop("QT_PLUGIN_PATH", None)
    os.environ.pop("QML2_IMPORT_PATH", None)

    # 根据会话类型选择平台插件：Wayland 会话优先 wayland，X11 会话用 xcb
    if "QT_QPA_PLATFORM" not in os.environ:
        has_wayland = bool(os.environ.get("WAYLAND_DISPLAY"))
        has_x11 = bool(os.environ.get("DISPLAY"))
        if has_wayland:
            os.environ["QT_QPA_PLATFORM"] = "wayland"
        elif has_x11:
            os.environ["QT_QPA_PLATFORM"] = "xcb"

    # 一些显卡驱动组合下软件渲染更稳定
    os.environ.setdefault("QT_OPENGL", "software")


def _normalize_edition(value: str) -> str:
    text = str(value or "").strip().lower()
    return "customer" if text == "customer" else "internal"


def _runtime_base_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]


def _feature_profile_path_for_edition(edition: str) -> Path:
    return _runtime_base_dir() / "config" / "feature_profiles" / f"{_normalize_edition(edition)}.yaml"


def _resolve_packaged_feature_config() -> Path | None:
    try:
        feature_file = _runtime_base_dir() / "feature_config.yaml"
        if feature_file.is_file():
            return feature_file
    except Exception:
        pass
    return None


def _normalize_feature_flags(raw_flags) -> dict:
    return normalize_feature_flags(raw_flags)


def _load_feature_profile(config_path: Path | None, edition: str) -> tuple[str, dict]:
    default_profile_name = "客户版" if _normalize_edition(edition) == "customer" else "内部版"
    if not config_path:
        return default_profile_name, dict(DEFAULT_FEATURE_FLAGS)

    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        raise RuntimeError(f"读取功能配置失败: {config_path} | {exc}") from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"功能配置格式错误: {config_path}，根节点必须为字典")

    profile_name = str(data.get("profile_name") or default_profile_name).strip() or default_profile_name
    feature_flags = _normalize_feature_flags(data.get("features"))
    return profile_name, feature_flags


def main(argv=None):
    _setup_qt_runtime_env()

    from PyQt6.QtWidgets import QApplication
    from ui.main_window import MainWindow

    parser = argparse.ArgumentParser(description="Humanoid Robot Delivery Toolchain")
    parser.add_argument("--instance-name", default="", help="实例名称（用于区分多开窗口）")
    parser.add_argument("--edition", default="", choices=["internal", "customer"], help="功能版本：internal/customer")
    parser.add_argument("--feature-config", default="", help="功能开关 YAML 路径")
    args, qt_argv = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    edition = _normalize_edition(args.edition or os.environ.get("TOOLCHAIN_EDITION") or "internal")
    feature_config_path = None
    if args.feature_config:
        feature_config_path = Path(args.feature_config).expanduser().resolve()
    elif os.environ.get("TOOLCHAIN_FEATURE_CONFIG"):
        feature_config_path = Path(os.environ["TOOLCHAIN_FEATURE_CONFIG"]).expanduser().resolve()
    else:
        feature_config_path = _resolve_packaged_feature_config() or _feature_profile_path_for_edition(edition)

    profile_name, feature_flags = _load_feature_profile(feature_config_path, edition)

    app = QApplication([sys.argv[0], *qt_argv])
    window = MainWindow(
        instance_name=args.instance_name,
        edition=edition,
        feature_flags=feature_flags,
        feature_profile_name=profile_name,
        feature_config_path=str(feature_config_path) if feature_config_path else "",
    )
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()