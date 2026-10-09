# Macintosh LC for Game Bub

A port of the [MiSTer Macintosh LC core](https://github.com/danifunker/MacLC_MiSTer) to
[Game Bub](https://gamebub.net/), using the Game Bub Core Framework
(v1.1-beta, in `framework/` as a git submodule — never edited here).

**Status: early alpha, runs on hardware.** On a Game Bub rev 4 the core
boots System 7.1 from a floppy or a hard disk image, with colour, the
startup chime and sound, and saves disk and PRAM changes back to the card.
It is held back mostly by what the framework and firmware can do today
(see [Limits](#limits)). See [docs/PORT_PLAN.md](docs/PORT_PLAN.md) for the
details and [docs/BUILDING.md](docs/BUILDING.md) to build it yourself.

## Try it

You need:
* a **Game Bub rev 4** with firmware **v1.1-beta2 or later** (older firmware
  does not list cores on the SD card);
* a **FAT32 microSD card with an MBR** partition table;
* a **Mac LC ROM** (512 KiB, the same file as MiSTer's `boot0.rom`). It is
  Apple's copyrighted code and is not included;
* disk images (`.hda` hard disk, `.dsk` floppy).

1. Unzip the newest `releases/MacLC_GameBub_*.zip` at the root of the card.
   It creates `/cores/danifunker.MacLC/`.
2. Copy your ROM into that folder as `MacLC.rom`.
3. Put disk images anywhere on the card (a `mac/` folder works).
4. Start "Macintosh LC" from the Game Bub menu. It asks for a hard disk
   first, then a floppy. A bootable floppy boots before the hard disk.

Disks:
* **Hard disk** (`.hda`/`.img`/`.vhd`): at most **19,398,656 bytes**
  (18.5 MiB). The firmware insists on a hard disk, so for floppy-only use
  pick any small placeholder image. A PRAM file `<diskname>.nvr` is created
  next to it.
* **Floppy** (`.dsk`/`.image`, raw or DiskCopy 4.2): up to 1.44 MB. Turn on
  **Floppy Writes** in the settings before the Mac writes to it.
* Changes are saved back to the card **when you exit the core normally**.
  Turning the Game Bub off while the Mac runs loses them.
* Apple HD SC Setup refuses the emulated drive. Format new disks with Lido,
  or start from a prepared image.

Controls:

| Button | Mouse mode | Keyboard mode |
|---|---|---|
| D-pad | move the pointer (hold to accelerate) | arrow keys |
| A | mouse button | Return |
| B / X / Y | Space / Shift / N | same |
| L / R | Escape / Option | same |
| Start | Command | same |
| Select | switch mouse ↔ keyboard mode | |

Settings: memory (2 or 10 MB, applied on restart), start in mouse or
keyboard mode, Floppy Writes, Restart Mac, Interrupt (MacsBug), Reset PRAM.

## Limits

These come from the framework v1.1-beta and firmware v1.1-beta2, not from
the FPGA:
* Every file is copied whole into the 32 MiB SDRAM at start and back at
  exit; there is no random-access file I/O. That caps the hard disk at
  18.5 MiB, rules out CD-ROM, and means no disk swaps while the Mac runs.
* Only one of rev 4's two SDRAM chips is usable.
* Video is 512×384 (the 12" RGB monitor) at 9 bits per pixel, dithered,
  because the framework frame buffer lives in block RAM.
* No keyboard or mouse: the 12 buttons are mapped as above.
* No clock from the host: the Mac's date resets unless PRAM was saved.

[docs/FRAMEWORK_ISSUES.md](docs/FRAMEWORK_ISSUES.md) lists these and the bugs
found along the way, written to be filed upstream.

## Layout

| Path | What |
|---|---|
| `framework/` | Game Bub framework (submodule, pinned) |
| `chisel/src/MacLC.scala` | Declares the framework interfaces the core uses |
| `rtl/gamebub/` | Game Bub platform layer: host protocol, block devices, VRAM in SRAM, video, input, clocks, constraints |
| `rtl/maclc/` | The Mac itself, **imported** from the MiSTer core by `scripts/import_maclc.py`; never edit by hand |
| `metadata/` | `core.json`, `files.json`, `settings.json` for the SD card |
| `releases/` | Built cores, ready to unzip onto the SD card |
| `sim/` | Verilator testbench (`tb_gamebub.sv`), Xilinx primitive stubs, Verilog CPU for simulation |
| `scripts/` | `import_maclc.py`, `lint.sh`, `ppm2png.py`; `debug/` has the JTAG/ILA tooling |

## Quick start (developers)

```bash
git submodule update --init
scripts/lint.sh                         # Verilator lint, any OS
sim/run_tb.sh +frames=450 +dump=50      # boot in simulation (sim_out/)
./mill root.buildCore --target gamebub_rev4   # bitstream; needs Vivado (Linux/WSL)
```

## License

The machine RTL comes from the MiSTer Macintosh LC core (GPL; based on
Sorgelig's MacPlus core and Plus Too). TG68K is LGPL. The Game Bub framework
is used unmodified under CERN-OHL-W (see `framework/LICENSE`).
