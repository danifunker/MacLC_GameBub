Resume work on the Mac LC core for Game Bub in `C:\repos\MacLC_GameBub`
(branch `gamebub-port`). Read `docs/PORT_PLAN.md`, `docs/FRAMEWORK_ISSUES.md`
and `scripts/debug/README.md` first; the memory files describe the build
environment. Don't redo work listed as done below.

## Where it stands (2026-10-09)
- **Released and pushed:** `releases/MacLC_GameBub_20261008.zip` (build of
  fb8b802: sound, 19.4 MB hard disk), on `main` and `gamebub-port` at
  github.com/danifunker/MacLC_GameBub. README has user install steps;
  BUILDING.md and PORT_PLAN.md are current. The user publishes the disk
  images themselves.
- **On real hardware** (Game Bub rev 4, firmware 1.1.0-beta2): boots System
  7.1 from floppy and from `mac/Macintosh HD.hda` (19.4 MB, built from
  `C:\repos\snow_validate.hda` with Python `machfs`), chime, colour, load
  and save verified. The user: "everything seemed to work but the framework
  isn't really ready to do much".
- The debug core on the card (`/cores/danifunker.MacLCdebug/`) is an OLDER
  build (old memory map, no sound); rebuild it from the current synth
  checkpoint with `scripts/debug/ila_build.tcl` + `probes_beacon.tcl` if needed.
- Staged, not on the card: `build/sdcard/mac/Empty 800K (boot from HD).dsk`.

## Next steps
1. Larger disks / CD-ROM need random-access file I/O between core and MCU,
   which firmware 1.1.0-beta2 does not have. Options: upstream request
   (`docs/FRAMEWORK_ISSUES.md` #6) or our own firmware fork
   (github.com/elipsitz/gamebub is open source). Discuss before starting
   the fork (custom firmware flash).
2. File the FRAMEWORK_ISSUES.md items upstream (the user decides when).
3. A new release: bump `metadata/core.json` version, zip per BUILDING.md §2
   into `releases/`, never include the ROM.

## SDRAM map (bytes, 32 MiB; keep gb_blockdev.sv, gb_host.sv, files.json in step)
RAM 0x000000-0x9FFFFF (10 MB) | ROM 0xA00000 (512 KiB, fixed) | PRAM 0xA80000 |
floppy file 0xA90000 (<= 0x170000) | floppy controller's copy 0xC00000
(<= 1,474,560) | hard disk 0xD80000-0x1FFFFFF (max 0x1280000).

## How things are done here
- Build (WSL, ~45 min): `wsl.exe -d Ubuntu-24.04 --exec bash -lc 'source ~/atrixdev/2025.2/Vivado/settings64.sh && cd /mnt/c/repos/MacLC_GameBub && ./mill root.buildCore --target gamebub_rev4 > build/buildCore-gamebub_rev4.log 2>&1; echo BUILD_EXIT=$? >> build/buildCore-gamebub_rev4.log'`
  Use `--exec` (else the outer shell expands `$?`). Never leave a shell or
  Monitor with its cwd inside `build/`: Windows then can't delete the folder
  and the build fails on `rmtree ... generated`.
- Simulation (Verilator 5.020 in WSL): `SIM_OBJ=/home/owner/vobj sim/run_tb.sh`
  with `+paced +readback +stop_after_load` (real MCU transfer pacing),
  `+floppy=../MacOS71-boot.dsk +ram=2 +frames=N`, `+trace_rd`. The frame line
  shows `audio_peak` and ASC cycles.
- SD card: stage in `build/sdcard/`, copy with PowerShell to the FAT32 USB
  drive (check BusType USB, not system, FAT32 first), MD5-verify every file,
  then **eject** (the user uses the eject as the "done" signal). Restage
  before ejecting. `Remove-Item` on the card is blocked by the harness.
- Commits use `git -c user.name="Dani Sarfati" -c user.email="dani@funkervogt.com"`
  (no global identity on this PC) and end with the Co-Authored-By line.
- JTAG: HS2 on J702, Vivado Lab 2025.2 on Windows; see `scripts/debug/README.md`.

## Hard-won facts
- Firmware SPI rates are 40/20/16/10 MHz: `max_transfer_speed` < 5000 panics it.
  The core must take a 32-bit word per 800 ns and answer reads within ~2 words.
- Framework ILI9806E: display clock must keep the line <= 612 clocks (sign bug).
- `common.xdc` needs the core's display MMCM net named `clk_dpi`.
- The firmware forces a pick for file 0 (hard disk); `mac/Blank HD 400K.hda`
  is the placeholder for floppy-only use. A bootable floppy boots before the HD.
- Apple HD SC Setup refuses the emulated drive ("MiSTer  VIRTUAL DISK"); use
  Lido (on Macintosh HD) or prebuilt images.
- `C:\repos\MacAtrium-7.1.hda` has a damaged primary HFS header (`0x229A`
  pattern, predates this work); its alternate MDB is intact.
- The user is concise and direct; give plain answers, avoid long option lists,
  and don't change files on the card beyond what was asked.
