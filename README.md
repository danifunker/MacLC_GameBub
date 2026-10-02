# Macintosh LC for Game Bub

A port of the [MiSTer Macintosh LC core](../MacLC_MiSTer) to
[Game Bub](https://gamebub.net/), using the Game Bub Core Framework
(v1.1-beta, in `framework/` as a git submodule — never edited here).

**Status: pre-alpha.** The core builds and boots in simulation; it has not
yet been built with Vivado or run on hardware. See
[docs/PORT_PLAN.md](docs/PORT_PLAN.md) for scope, limits, and the plan, and
[docs/BUILDING.md](docs/BUILDING.md) to build, simulate, and install it.

## Layout

| Path | What |
|---|---|
| `framework/` | Game Bub framework (submodule, pinned) |
| `chisel/src/MacLC.scala` | Declares the framework interfaces the core uses |
| `rtl/gamebub/` | Game Bub platform layer: host protocol, block devices, VRAM in SRAM, video, input, clocks, constraints |
| `rtl/maclc/` | The Mac itself, **imported** from the MiSTer core by `scripts/import_maclc.py`; never edit by hand |
| `metadata/` | `core.json`, `files.json`, `settings.json` for the SD card |
| `sim/` | Verilator testbench (`tb_gamebub.sv`), Xilinx primitive stubs, Verilog CPU for simulation |
| `scripts/` | `import_maclc.py`, `lint.sh`, `ppm2png.py` |

## Quick start

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
