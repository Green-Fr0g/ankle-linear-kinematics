"""项目内常用路径（统一从仓库根目录推导，脚本在哪个目录运行都一样）"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / "model"
XML_PATH = MODEL_DIR / "xml" / "calf_actuator.xml"
LUT_PATH = MODEL_DIR / "lut" / "calf_actuator_fk_lut.npz"
LOG_DIR = ROOT / "logs"
