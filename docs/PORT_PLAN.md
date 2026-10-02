# MacLC → Game Bub port: scope and plan

Status (2026-09-30): **phases 0 and 1 done in simulation, nothing built or run
on hardware yet.** See [Progress](#7-progress) at the end.
Source core: `../MacLC_MiSTer` (full-featured MiSTer core).
Reference port: `../MacLC_pocket` (Analogue Pocket, v1.2.0, boots 7.5.5). It
already solved the "no HPS" problems (block device, gamepad input, 48 kHz
audio, SDRAM-only memory), so its glue is the template for this port.
Framework: Game Bub Framework v1.1-beta (this repo, as copied), docs at
https://docs.gamebub.net/developing-cores/overview/

---

## 1. Hardware (rev4, from framework source + gamebub-pcb schematics)

| Resource | Game Bub rev4 | DE10-Nano (MiSTer) | Pocket |
|---|---|---|---|
| FPGA | XC7A100T-1CSG324 | 5CSEBA6 | 5CEBA4 |
| Logic | 63,400 LUT6 / 126,800 FF | 41,910 ALM | 18,480 ALM |
| Block RAM | 135 × BRAM36 = 4.86 Mb | 553 × M10K = 5.5 Mb | 308 × M10K = 3.0 Mb |
| Distributed RAM | ~1.19 Mb (LUTRAM) | — | — |
| SDRAM | 2 × W9825G6KH-6 (32 MiB, 16-bit each) — **only chip 0 wired today** | 32/128 MB + DDR3 | 64 MB |
| SRAM | IS61WV25616 **512 KiB, 16-bit, 10 ns async** | — | — |
| Display | 800×480 panel (720 visible), 24-bit, variable refresh | HDMI scaler | 1600×1440 |
| Input | 12 buttons | USB kbd/mouse | gamepad + (dock) HID |

Logic is not a constraint. The MiSTer `emu` was ~20.9K ALM (roughly 25–32K
LUT6), well under half the chip. **Nothing needs cutting for logic reasons**:
the CD-ROM target, both floppies, both SCSI disks, 10 MB RAM and 16bpp VRAM all
fit.

## 2. Hard constraints imposed by the framework (v1.1-beta)

These cause the compromises. They come from the framework design, not the chip
size, and the framework must not be modified.

### 2a. Video is double-buffered in on-chip BRAM (the binding constraint)
`HandheldTop.scala` allocates `2 × videoWidth × videoHeight × (R+G+B bits)`
of BRAM for the core's video, plus a 360×240×2-bit overlay (~6 BRAM36).

| Output mode | Framework BRAM | Fits? |
|---|---|---|
| 512×384 @ 8–9 bpp (3/3/2 or 3/3/3) | ~96–108 BRAM36 | **Yes**, leaves ~21–33 BRAM36 for the Mac |
| 512×384 @ 12 bpp (4/4/4) | ~144 BRAM36 | No (chip has 135) |
| 640×480 @ 8 bpp | ~150 BRAM36 (4.9 Mb) | No, exceeds the whole chip |
| 640×480 @ 3 bpp (1/1/1, B&W) | ~60 BRAM36 | Yes, as a separate B&W-only bitstream |

Consequences:
- **512×384 (12" RGB monitor) only** for a colour build. Integer scale 1 on the
  800×480 panel.
- The Mac's own 8/16bpp output is **quantized to 8–9 bits** at the framework
  boundary. Add ordered dithering before quantization.
- The Mac's VRAM **cannot** stay in BRAM (MiSTer `vram_bram` = 384 KB). It moves
  to the **512 KiB async SRAM**, which exactly matches the LC's 512 KB VRAM
  option, with a scanline prefetch buffer. The async SRAM is dedicated rather
  than shared, so it avoids the bus-sharing starvation that pushed MiSTer to BRAM.
- The video interface is in the **system clock** domain. `dataEnable` is a
  per-pixel write strobe (acts as `ce_pix`), and `hblank`/`vblank` are
  edge-detected pulses.

### 2b. No random-access file I/O ("no Core → Host commands yet")
Files in `files.json` (max 8) are **copied whole into core memory at setup**
and **copied back whole on core exit** (unless `read_only`). Default
5,000 KB/s.
- HD images must fit in SDRAM alongside RAM and ROM. With one 32 MiB chip:
  10 MB RAM + 512 KB ROM + floppies leaves **~18–20 MB of HD**. With both chips
  it would be ~50 MB.
- Writes persist **only on a clean core exit**. A crash or power loss loses
  the session's disk writes.
- **No mid-session media change.** Workaround: preload up to N floppy images
  as separate files and use a settings list to choose which is "in the drive".
- **CD-ROM is effectively out.** An ISO would have to fit in RAM.
- The upside is a much simpler block device than the Pocket's `apf_blockdev`:
  an `sdram_blockdev` that serves the hps_io `sd_*` interface from
  SDRAM-resident images.

### 2c. Input is 12 buttons, with no keyboard or mouse interface
D-pad drives the mouse and face buttons click. Keys come from chords and
macros (Cmd-Q, Return, Esc, etc.). Port `MacLC_pocket/src/fpga/core/pocket_input.v`.
`adb_device.sv` (ps2_key/ps2_mouse → ADB) is the platform seam and is
unchanged.

### 2d. Other gaps
- No RTC/time from the host. The Egret clock resets each boot unless it is
  seeded from saved PRAM (the Pocket did a Mac-epoch seed).
- No network. Drop the PDS Ethernet card, which needed the MiSTer Main anyway.
- Max 16 settings (MiSTer OSD had ~20; MT32-pi/ethernet/aspect items go away).
- Audio: 48 kHz, 16-bit stereo, system clock domain. ASC is 22.254 kHz, so
  resample (Pocket has this).
- Serial/MIDI: the PMOD header (4 × 3.3 V GPIO) can carry the SCC modem port as
  a UART (MIDI OUT to an MT32-pi, or a serial console).

## 3. Framework feature requests (upstream to Game Bub / Eli)

Each of these removes a compromise in section 2. The plumbing for (1) already
exists: the `commandCore` channel and `coreRequest` MCU interrupt are wired,
but no commands are defined.
1. **Core → Host block read/write** (random-access file I/O). Unlocks large HDs,
   CD-ROM, instant boot and safe writes.
2. **Second SDRAM chip on rev4.** `verilog/handheld/top.sv` has
   `// TODO: support second SDRAM chip (rev 4)` and ties `cs_n[1]` high.
3. **Video path that is not BRAM-limited**: an SDRAM-backed framebuffer, an
   indexed-colour + palette mode, or a direct/pass-through mode. Unlocks
   640×480 and full colour.
4. **Keyboard/mouse input** (dock USB HID pass-through).
5. **Host RTC** and **mid-session file (re)mount**.

## 4. Toolchain

| Step | Where |
|---|---|
| RTL editing, Verilator sim (5.052 installed) | This Mac |
| Chisel elaboration (`./mill`) | This Mac should work. Needs a JDK Scala 2.13.16 supports; JDK 25 is installed and may need 17/21. Not yet verified. |
| Vivado 2023.2+ synthesis/P&R, `./mill root.buildCore --target gamebub_rev4` | **Linux or Windows only** (Vivado has no macOS build). Needs Python ≥ 3.12. The free Vivado ML Standard covers the XC7A100T. |
| Deploy | Copy `.bit` + `core.json`/`files.json`/`settings.json` to `/cores/<Author.Name>/` on a FAT32/MBR microSD |

Repo layout: the official template (`github.com/gamebub/core-example`) is a
**core repo with the framework as a git submodule at `framework/`**, plus
`rtl/`, `chisel/src/`, `metadata/` and `build.mill`. This repo is currently the
bare framework and should be restructured to that layout so the framework stays
pristine and updatable.

## 5. Debugging options

1. **Simulation first (this Mac).** The MacLC core's Verilator harness (boot
   screenshot, CPU trace, MAME diff) carries over. Add a Game Bub-shaped
   testbench around the new top that models the host memory interface (file
   uploads into an SDRAM model, the command handshake) and dumps the VideoV0
   pixel stream to PNG. Most wrapper bugs die here, before Vivado.
2. **JTAG + Vivado ILA** (ChipScope; Xilinx's equivalent of SignalTap). Rev4
   has a **populated 6-pin JTAG header, J702, on the back of the board**.
   Pinout: 1 TMS, 2 TDI, 3 TDO, 4 TCK, 5 GND, 6 3V3. That is the same order
   as Digilent's 6-pin JTAG header (TMS, TDI, TDO, TCK, GND, VDD). J702 is a
   female socket, so use a male-male header or jumpers and wire by signal
   name. Recommended cable: **Digilent JTAG-HS2** (native Vivado support).
   Run Vivado Lab Edition on Windows for JTAG, because WSL needs `usbipd-win`
   for USB pass-through. Xilinx Platform Cable USB II clones also work; a
   generic FT2232H board can program but cannot run Vivado's ILA. Load
   the core normally from SD so the MCU does its file and setup handshake, then
   **attach** Vivado Hardware Manager without reprogramming and arm ILA triggers
   using the build's `.ltx`. JTAG-programming a bitstream directly skips the
   MCU's setup, so the core would get no ROM or files. It may be necessary to
   open the shell to reach J702.
3. **PMOD UART** (J801, top edge, 4 × 3.3 V GPIO): a debug UART or the Mac's
   SCC to a 3.3 V USB-serial adapter gives live logs and a serial console with
   no JTAG cable.
4. **On-screen HUD**: the MiSTer core's `USE_DBG_HUD` sits in the video path
   and works unchanged. The framework can also read the framebuffer back over
   SPI (`0xF3xx_xxxx`).
5. The vibration motor can serve as a crude alive/heartbeat indicator.

## 6. Plan

**Phase 0: setup (this Mac)**
- Restructure the repo to the core-example layout, with the framework as a
  submodule.
- Pick the MiSTer source commit and import RTL. Drop `pds/`, `sys/`, MT32-pi
  and the Toolbox paths that depend on the MiSTer Main.
- Get `./mill` elaborating on macOS so the generated port template can be
  checked.

**Phase 1: the port (this Mac, sim-verified)**
- `chisel/src/MacLC.scala`: interfaces = clocks, video 512×384 (3/3/2 or
  3/3/3), audio, host, input, sdram, sram.
- Clocks: one MMCM from 50 MHz with VCO 650 MHz (M=13). /20 = **32.5 MHz sys**
  (the value `v8_clocks` hard-codes), /10 = **65 MHz SDRAM** (8× bus clock, as
  on the Pocket), /4 = **162.5 MHz SPI** (≥160 required), /20 = **32.5 MHz
  display** (inside ILI9806E's 25.8–35 MHz window).
- `rtl/gamebub/maclc_gamebub.sv` (framework-facing top) and
  `rtl/gamebub/maclc_core.sv` (the machine top, from `MacLC.sv` and the
  Pocket's `mac_lc_pocket.sv`):
  - Host command FSM: GetStatus, SetupComplete, CoreHalt/Run, FileWrite/Read
    Start/End, NotifyFocus.
  - Host memory map: file windows into SDRAM, settings registers.
  - SDRAM controller (from `sdram.v`/`pocket_sdram.v`) with Artix-7 I/O: ODDR
    clock, IOB registers, XDC-level timing. Add a host-write/read port.
  - VRAM in the async SRAM with a line prefetch, replacing `vram_bram`.
  - V8 video → VideoV0 adapter, with ordered dither.
  - `sdram_blockdev` for SCSI and floppy.
  - Gamepad → ps2_key/ps2_mouse.
  - ASC → 48 kHz audio.
- `metadata/`: `core.json`; `files.json` (ROM, HD0, HD1, floppy slots, PRAM);
  `settings.json` (RAM size, CPU speed, input mode, floppy select, reset,
  NMI).
- Verilator testbench of the whole wrapper.

**Phase 2: first build (Vivado box)**
- Check utilization (BRAM is the one to watch) and timing, then fix.
- Package and copy to the SD card.

**Phase 3: hardware bring-up**
- Targets in order: ?-disk screen, then HD boot to Finder.
- Debug over PMOD UART, then ILA.

**In parallel**
- File the section 3 feature requests.

---

## 7. Progress

### Done (2026-09-30)
- The repo uses the official layout: `framework/` is a submodule pinned at
  upstream `db9d3b0` (byte-identical to the copy this repo started as), with
  `build.mill`, `chisel/src/MacLC.scala`, `rtl/`, `metadata/`.
- `scripts/import_maclc.py` copies the machine from MiSTer `master`
  (`045f896`) into `rtl/maclc/`. It applies five checked patches: Egret
  ROM/PRAM inlined, two `` `include``s inlined, V8 VRAM read handshake,
  SDRAM controller I/O for Artix-7 plus a 32 MiB row bit. The VHDL TG68K goes
  to Vivado and the Verilog TG68K to `sim/` only.
- Platform layer in `rtl/gamebub/`:
  - `maclc_core` (MacLC.sv minus MiSTer)
  - `gb_host` (HostV0 commands, settings, SDRAM file windows, ROM byte-order
    detection and fixup)
  - `gb_blockdev` (hps_io block devices from SDRAM)
  - `gb_vram_sram` (VRAM in the async SRAM)
  - `gb_video_out` (512x384 3/3/3 with Bayer dither)
  - `gb_pad_input` (Pocket's gamepad mapping)
  - `gb_clocks` (one MMCM)
  - `gb_debug_uart` (PMOD beacon)
  - `maclc.xdc` (first-pass I/O constraints and multicycles)
- `metadata/` JSONs. The hard disk is file 0 (user-picked, ≤ 16 MiB); the
  PRAM `.nvr` is derived from it; the floppy is user-picked; `MacLC.rom` is
  fixed.
- Verified on this Mac:
  - `scripts/lint.sh` reports 0 errors.
  - Chisel elaborates and prints the expected port template.
  - `sim/run_tb.sh` boots the full core through a model of the MCU host
    protocol (GetStatus → files → SetupComplete → CoreRun), with SDRAM and
    SRAM chip models:
    - The ROM is written as little-endian host words; the byte order is
      detected and the ROM rewritten (SDRAM reads `350E ACF0 0000 002A`).
    - The Egret releases the CPU about 1.4 s in. The RAM test runs, and the
      desktop pattern and arrow cursor appear around frame 500–600 (2 MB).
    - **No disk:** the "?" floppy icon (frame 800).
    - **System 6.0.5 floppy** (`../MacLC_MiSTer/releases/Disk605.dsk`):
      1600 sectors pass through the block device into the floppy loader.
      The ROM boots it and shows the correct alert, "This startup disk
      will not work on this Macintosh model … System 6.0.5 does not work on
      this model" (frame 900). That proves the floppy read path end to end.
    - PRAM is loaded through the block device and flushed back twice by the
      guest (block-device writes).
    - The PMOD beacon UART output decodes correctly in the testbench.
  - Resolved: some black pixels showed as dark grey. The LC ROM's "black"
    is (5,5,5), which the ordered dither faithfully turned into sparse
    dark-grey dots in text and frames. `gb_video_out` now snaps values
    within 16 of black or white to the extreme (found with
    `sim/run_tb.sh +odd_pixels`).

### Next
1. **First Vivado build** on the WSL box (docs/BUILDING.md). Expect to fix
   Vivado-only parse issues in the imported Quartus-era RTL. Then read:
   - **Block RAM.** The framework frame buffer is ~96 of 135 BRAM36. If the
     total overflows, the cheap fixes are, in order: drop SCSI disk targets'
     `RING_LOG` 5→3 (the ring hid MiSTer HPS latency; SDRAM serves a sector
     in ~80 µs); remove the CD-ROM target from `ncr5380.sv` (no CD source on
     Game Bub); shrink the Toolbox buffer (`TB_ADDRW` 12→8); video 3/3/3→3/3/2.
   - **Timing** (WNS), and the I/O report for the SDRAM/SRAM constraints.
2. **Hardware bring-up.** Copy files per BUILDING.md §3 and watch the PMOD
   beacon (word 0 = `4842....`, bit 3 = ROM fixup done, bit 1 = running;
   word 5 = CPU address).
3. **Things only hardware can answer:**
   - the MCU's file byte order (auto-detected, but confirm on the beacon:
     word 0 bits 10:9);
   - whether two `user_selected` files are allowed;
   - whether actions are replayed at setup (guarded anyway);
   - real SDRAM timing margins.
4. Feature requests upstream (§3).
