# Hardware debug tooling (JTAG + ILA)

Used to find the LCD timing bug, the dropped file transfers and the silent
ASC on a real Game Bub (rev 4) with a Digilent JTAG-HS2 on J702.

| File | Runs in | What |
|---|---|---|
| `jtag_status.tcl` | Vivado Lab (Windows) | Read-only: JTAG chain, FPGA DONE/config status, USERCODE |
| `ila_build.tcl` | Vivado (WSL) | Insert an ILA into `build/MacLC-gamebub_rev4/.../synth_1/top_handheld.dcp`, implement, write `.bit` + `.ltx`. Reads the same three XDCs as the real impl run and **refuses to write a bitstream unless every port has a pin and USERID is B0100004** |
| `probes_display.tcl` | (input to ila_build) | LCD hsync/vsync/DE/data, hdmi_enable, MCU SPI CS/IRQ, core clocks locked, video vblank, host cmd state |
| `probes_beacon.tcl` | (input to ila_build) | The PMOD beacon's 8 status words (`beacon/snap_reg`) + live signals |
| `ila_capture.tcl` | Vivado Lab (Windows) | Program (or attach with `-`), capture immediate / vsync-fall / SPI-CS triggers to CSV |
| `ila_summary.py` | Python | Toggle counts per probe from the CSVs |
| `beacon_from_ila.py` | Python | Rebuild and decode the beacon words from a `probes_beacon` capture |
| `run_xsim.sh` | WSL | Run `sim/tb_gamebub.sv` in Vivado xsim (slow; prefer Verilator `sim/run_tb.sh`) |

```bash
# WSL: debug bitstream from the last synthesized design
vivado -mode batch -source scripts/debug/ila_build.tcl -tclargs \
  /mnt/c/repos/MacLC_GameBub/build/MacLC-gamebub_rev4/MacLC-gamebub_rev4.runs/synth_1/top_handheld.dcp \
  /mnt/c/repos/MacLC_GameBub/build/debug-beacon scripts/debug/probes_beacon.tcl
```

```bash
# Windows: attach to the core the MCU loaded (debug core from SD), capture
C:\AMDDesignTools\2025.2\Vivado_Lab\bin\vivado_lab.bat -mode batch -source scripts/debug/ila_capture.tcl -tclargs - C:/repos/MacLC_GameBub/build/debug-beacon/MacLC-gamebub_rev4.ltx C:/repos/MacLC_GameBub/build/cap
```

Notes: JTAG-programming a bitstream skips the MCU's loader (no ROM, no files),
so to debug a real boot put the debug `.bit` on the card as a separate core
(`/cores/danifunker.MacLCdebug/`, its own `core.json` id) and attach without
reprogramming. A long Power press restores the device after a JTAG load.
The scripts contain this machine's paths (`/mnt/c/repos/MacLC_GameBub`,
the Vivado Lab install); adjust them elsewhere.
