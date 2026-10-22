"""Vitis 2025.2 build: platform (standalone + lwIP, FSBL) and the firmware app.

    D:/AMDDesignTools/2025.2/Vitis/bin/vitis.bat -s sw/vitis_build.py

Workspace: sw/ws (generated, git-ignored). The app starts from the
lwip_echo_server template (platform/timer glue) with its main.c replaced by
sw/app/src/main.c. Boot image: sw/make_boot.py.
"""
import os
import shutil
import sys
from pathlib import Path

import vitis

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "sw" / "ws"
XSA = ROOT / "hw" / "build" / "video_top.xsa"
PLAT = "video_plat"
DOM = "standalone_ps7_cortexa9_0"
APP = "video_fw"

LWIP = {
    "lwip220_api_mode": "RAW_API",
    "lwip220_dhcp": "false",
    "lwip220_pbuf_pool_size": "4096",
    "lwip220_pbuf_pool_bufsize": "1700",
    "lwip220_mem_size": "524288",
    "lwip220_memp_n_pbuf": "256",
    "lwip220_n_rx_descriptors": "256",
    "lwip220_n_tx_descriptors": "64",
    "lwip220_temac_phy_link_speed": "CONFIG_LINKSPEED_AUTODETECT",
}

fresh = "--fresh" in sys.argv
if fresh and WS.exists():
    shutil.rmtree(WS)
WS.mkdir(parents=True, exist_ok=True)

client = vitis.create_client()
client.set_workspace(path=str(WS))

if not (WS / PLAT).exists():
    plat = client.create_platform_component(name=PLAT, hw_design=str(XSA), os="standalone",
                                            cpu="ps7_cortexa9_0", domain_name=DOM)
    dom = plat.get_domain(name=DOM)
    dom.set_lib(lib_name="lwip220")
    for k, v in LWIP.items():
        dom.set_config(option="lib", param=k, value=v, lib_name="lwip220")
else:
    plat = client.get_component(name=PLAT)
    plat.update_hw(hw_design=str(XSA))
plat.build()

xpfm = WS / PLAT / "export" / PLAT / f"{PLAT}.xpfm"
if not (WS / APP).exists():
    app = client.create_app_component(name=APP, platform=str(xpfm), domain=DOM,
                                      template="lwip_echo_server")
else:
    app = client.get_component(name=APP)

src = WS / APP / "src"
for f in ("echo.c",):
    if (src / f).exists():
        (src / f).unlink()
for f in (ROOT / "sw" / "app" / "src").iterdir():
    shutil.copy(f, src / f.name)

# optimise: the firmware copies ~90 Mbit/s of payload
cfg = src / "UserConfig.cmake"
txt = cfg.read_text()
txt = txt.replace("set(USER_COMPILE_OPTIMIZATION_LEVEL -O0)", "set(USER_COMPILE_OPTIMIZATION_LEVEL -O2)")
cfg.write_text(txt)

app.build()
elf = WS / APP / "build" / f"{APP}.elf"
print("ELF", elf, elf.exists())
vitis.dispose()
