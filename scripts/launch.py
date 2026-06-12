import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

src_path = str(SRC_DIR)
# 确保 src 在 sys.path 末尾，避免覆盖标准库同名模块（如 types）
sys.path = [p for p in sys.path if p != src_path]
sys.path.append(src_path)

from app import main


if __name__ == "__main__":
    main()
