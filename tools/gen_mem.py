"""Write rtl/font8x8.mem (768 bytes, glyph-major, row-major) from model/font8x8.py."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))
from font8x8 import FONT8X8  # noqa: E402


def main():
    lines = [f"{row:02x}" for glyph in FONT8X8 for row in glyph]
    (ROOT / "rtl" / "font8x8.mem").write_text("\n".join(lines) + "\n")
    print("font8x8.mem", len(lines), "bytes")


if __name__ == "__main__":
    main()
