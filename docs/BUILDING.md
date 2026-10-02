# Building and running the Game Bub Mac LC core

Three machines are involved:

| Where | What runs there |
|---|---|
| **Any machine (this Mac included)** | Editing, `scripts/import_maclc.py`, `scripts/lint.sh`, the Verilator simulation `sim/run_tb.sh` |
| **The "Vivado machine"**: Linux, or Windows with WSL2 | The FPGA build. AMD's Vivado only runs on Linux and Windows. |
| **The Game Bub** | Runs the `.bit` from its microSD card |

"Vivado" is AMD/Xilinx's FPGA build tool, the equivalent of Quartus for
Intel/Altera chips. Game Bub uses a Xilinx Artix-7, so it needs Vivado where
MiSTer and the Pocket needed Quartus.

---

## 1. Set up Vivado on WSL2 (one time)

You already run Quartus in WSL, so this follows the same pattern.

1. **Disk and RAM.** Plan for about 40 GB free on the WSL disk for the
   download and install. Vivado needs a few GB of RAM for a chip this size,
   so give WSL at least 8 GB. If WSL is capped lower, raise `memory=` in
   `%UserProfile%\.wslconfig`.
2. **Download.** Create a free AMD account and download the *AMD Unified
   Installer for FPGAs & Adaptive SoCs*, **Linux self-extracting web
   installer** (`FPGAs_AdaptiveSoCs_Unified_<version>_Lin64.bin`), from
   <https://www.xilinx.com/support/download.html>. The framework needs
   **2023.2 or newer**; take the latest.
3. **Install from inside WSL.** On Windows 11, WSLg shows the graphical
   installer:
   ```bash
   chmod +x FPGAs_AdaptiveSoCs_Unified_*_Lin64.bin
   ./FPGAs_AdaptiveSoCs_Unified_*_Lin64.bin
   ```
   * Product: **Vivado**. Edition: **Vivado ML Standard**. This edition is
     free and covers the XC7A100T.
   * Devices: tick only **7 Series → Artix-7**. Untick everything else (Zynq,
     UltraScale, Versal, Vitis, DocNav) to save tens of gigabytes.
   * Install location: the default `/tools/Xilinx` is fine (it needs `sudo`),
     or use `~/Xilinx`.
   If WSLg is unavailable, the installer has a text-only batch mode: run it
   with `--noexec --target ~/xinstall`, then `~/xinstall/xsetup -b ConfigGen`,
   edit the generated config, and run `xsetup -b Install -a XilinxEULA,3rdPartyEULA -c <config>`.
4. **Libraries** (Ubuntu): `sudo apt install libtinfo6 libncurses6 libx11-6 libxrender1 libxtst6 libxi6 make`.
   On 22.04/24.04, if Vivado complains about `libtinfo.so.5`, add the
   compatibility symlink AMD documents for your Vivado version.
5. **Put Vivado on your PATH**, adjusting the version and location:
   ```bash
   echo 'source /tools/Xilinx/Vivado/2025.1/settings64.sh' >> ~/.bashrc
   ```
   Newer releases install to `/tools/Xilinx/<version>/Vivado/settings64.sh`.
   Check with `vivado -version`.
6. **The framework's other needs:** Python 3.12+ (`sudo apt install python3.12
   python3.12-venv`, or use `uv`/`pyenv`), a JDK 17 or 21 for Mill/Chisel
   (`sudo apt install openjdk-21-jdk`), `git`, and `curl`.

## 2. Build the bitstream (on the Vivado machine)

```bash
git clone --recurse-submodules <your repo URL> MacLC_GameBub
cd MacLC_GameBub
./mill root.buildCore --target gamebub_rev4
```

The first run downloads Mill, Scala and Chisel. The command then:
1. elaborates `chisel/src/MacLC.scala` with the framework to SystemVerilog
   (and prints our module's port template);
2. collects every `.v/.sv/.vhdl/.xdc` under `rtl/`;
3. runs Vivado synthesis, place and route, and bitstream generation.

The result is `build/MacLC-gamebub_rev4/MacLC-gamebub_rev4.bit`. Expect tens
of minutes. Keep these reports for review:
* `*.runs/impl_1/*_utilization_placed.rpt`: **block RAM** is the resource to
  watch (see `docs/PORT_PLAN.md`).
* `*.runs/impl_1/*_timing_summary_routed.rpt`: look for `WNS` ≥ 0 and read the
  I/O section for the SDRAM/SRAM constraints in `rtl/gamebub/maclc.xdc`.

## 3. Put it on the Game Bub

On a FAT32 (MBR) microSD card:

```
/cores/danifunker.MacLC/
    core.json                 <- metadata/core.json
    files.json                <- metadata/files.json
    settings.json             <- metadata/settings.json
    MacLC-gamebub_rev4.bit    <- the build output
    MacLC.rom                 <- your Mac LC ROM (512 KiB, same as MiSTer's boot0.rom)
```

Disk images go anywhere on the card; the core asks for them when it starts:
* **Hard Disk**: `.hda`/`.img`/`.vhd`, at most **16 MiB** for now (see
  PORT_PLAN.md §2b). A PRAM file `<diskname>.nvr` is created next to it.
* **Floppy Disk**: `.dsk`/`.image` (raw or DiskCopy 4.2), up to 1.44 MB.
  "Floppy Writes" must be on before the Mac can write to it.

Changes to disks are saved back to the card **when you exit the core
normally**. Turning the device off while the Mac runs loses them.

### Controls (from the Pocket port)
| Button | Mouse mode | Keyboard mode |
|---|---|---|
| D-pad | move the pointer (holds accelerate) | arrow keys |
| A | mouse button | Return |
| B / X / Y | Space / Shift / N | same |
| L / R | Escape / Option | same |
| Start | Command | same |
| Select | switch mouse ↔ keyboard mode | |

## 4. Simulate first (on any machine with Verilator ≥ 5)

```bash
scripts/lint.sh                      # whole design, ~10 s
sim/run_tb.sh +frames=450 +dump=50   # boot the Mac, ~8 min; frames in sim_out/
python3 scripts/ppm2png.py sim_out/frame_0450.ppm
```

The testbench (`sim/tb_gamebub.sv`) stands in for the Game Bub MCU. It loads
the ROM through the real host protocol, sends SetupComplete and CoreRun, and
records what the framework would put on screen. See its header for plusargs
(`+hd=`, `+floppy=`, `+host_be`, ...).

## 5. Debugging on hardware

See `docs/PORT_PLAN.md` §5. In order of cost:
* **PMOD debug beacon.** Connect a 3.3 V USB-serial adapter's RX to PMOD pin 3
  and GND to GND, and open a terminal at 115200 8N1. Every 0.5 s the core
  prints a line of eight hex words, described in `rtl/gamebub/maclc_gamebub.sv`
  above `gb_debug_uart`. The first word starting `4842` means the core is
  alive.
* **Mac serial port.** PMOD pin 1 is the Mac's modem-port TxD and pin 2 its
  RxD (3.3 V levels).
* **JTAG + Vivado ILA** through J702 with a Digilent JTAG-HS2.
