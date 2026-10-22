"""Create BOOT.BIN for SD-card boot (FSBL + bitstream + firmware).

    python sw/make_boot.py            -> sw/boot/BOOT.BIN

Copy BOOT.BIN to the root of a FAT32 SD card. ZedBoard boot jumpers for SD:
MIO6..MIO2 = 0 1 1 0 0 (JP7..JP11: JP7 GND, JP8 GND, JP9 3V3, JP10 3V3, JP11 GND).
"""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "sw" / "ws"
VITIS_BIN = Path(os.environ.get("VITIS_BIN", r"D:\AMDDesignTools\2025.2\Vitis\bin"))


def main():
    fsbl = next((WS / "video_plat" / "export" / "video_plat" / "sw").rglob("fsbl.elf"), None)
    bit = ROOT / "hw" / "build" / "video_top.bit"
    elf = WS / "video_fw" / "build" / "video_fw.elf"
    for f in (fsbl, bit, elf):
        if f is None or not Path(f).exists():
            sys.exit(f"missing: {f}")
    out = ROOT / "sw" / "boot"
    out.mkdir(exist_ok=True)
    bif = out / "boot.bif"
    bif.write_text("the_ROM_image:\n{\n"
                   f"  [bootloader]{Path(fsbl).as_posix()}\n"
                   f"  {bit.as_posix()}\n"
                   f"  {elf.as_posix()}\n"
                   "}\n")
    subprocess.run([str(VITIS_BIN / "bootgen.bat"), "-image", str(bif), "-arch", "zynq",
                    "-o", str(out / "BOOT.BIN"), "-w", "on"], check=True)
    b = out / "BOOT.BIN"
    print(f"{b}  {b.stat().st_size / 1e6:.2f} MB")
    newest = max(p.stat().st_mtime for p in (Path(fsbl), bit, elf))
    if b.stat().st_mtime < newest:
        sys.exit("BOOT.BIN older than its inputs")


if __name__ == "__main__":
    main()
