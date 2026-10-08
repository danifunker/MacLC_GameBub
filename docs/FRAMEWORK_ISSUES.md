# Game Bub framework and firmware issues found while porting the Mac LC

Each entry is written to be filed upstream as is. Versions: framework
`gamebub/framework` @ `db9d3b0` (v1.1-beta), handheld firmware
**1.1.0-beta2** on rev 4 hardware (HW 1.4.1.0), Vivado 2025.2. Paths are
relative to the framework repository unless they start with `firmware/`
(`elipsitz/gamebub`).

| # | Severity | Area | Summary |
|---|---|---|---|
| 1 | **Data loss** | Framework SPI + firmware | File loads and saves silently drop or corrupt data when the core is slower than the SPI stream |
| 2 | Crash | Firmware | `max_transfer_speed` below 5000 KB/s panics the firmware |
| 3 | Black screen | Framework display | ILI9806E back-porch clamp has the wrong sign; DE never falls |
| 4 | Timing failure | Framework constraints | `common.xdc` silently requires the core's display clock to be named `clk_dpi` |
| 5 | Build crash | Framework build | `build_core.py` "skips" unknown file types but passes them to edalize anyway |
| 6 | Feature | Framework + firmware | No core-to-host random-access file I/O (disks limited to SDRAM, no CD-ROM) |
| 7 | Feature | Framework | Second SDRAM chip on rev 4 is not usable |
| 8 | Usability | Firmware | An optional user-selected file cannot be skipped |
| 9 | Docs | Firmware releases | The firmware that runs SD-card cores (v1.1-beta2) is not on the releases page |
| 10 | Docs | Developer docs | Undocumented requirements that cost us days |
| 11 | Feature | Framework build | A core cannot pass Verilog macros or include paths to the build |

---

## 1. File transfers silently lose data when the core is slower than the SPI stream (data loss)

**Affects:** `src/main/scala/platform/handheld/spi/SpiReceiverFifo.scala`;
firmware `firmware/handheld/src/core/mod.rs` (`load_files`, `persist_files`).

**What happens.** The MCU streams file data over quad SPI at a fixed rate
(at least 10 MHz, its slowest SPI clock: one 32-bit word every 800 ns). The
framework passes it to the core through `SpiReceiverFifo`, which has no
flow control:

- **Writes (file load):** when the 512-entry request FIFO is full, the rest
  of the SPI transaction is discarded (`fifoRequestOverflow`, "push only if
  no overflow in this transaction").
- **Reads (file save):** a word whose response has not crossed back yet is
  sent as `0xFFFFFFFF` (`fifoResponseUnderflow`); the late response is then
  consumed by a later word, so everything after it is shifted.

Neither condition reaches the MCU (`debugRequestOverflow` and
`debugResponseUnderflow` are outputs of the module but not readable), and the
firmware does not verify transfers. The result is silent:

- the core gets a ROM/disk image with holes (our Mac crashed before its boot
  chime: the CPU ran into garbage);
- **on core exit, the saved files are overwritten with corrupted data**. Our
  test floppy came back with 773,500 of 819,200 bytes changed (mostly `0xFF`),
  in the same pattern in every 16 KiB chunk: the first 4 words right, about a
  third right after that, then nothing.

**Reproduce.** Any core whose `HostV0.mem` takes more than ~26 system clocks
(at 32.5 MHz) per 32-bit word, sustained. Ours took 28 per write and 30-35
per read (two SDRAM accesses per word behind three registered arbiters).

**Suggested fixes** (any of these would have turned silent corruption into
a visible error):

1. Latch overflow/underflow per transaction in a status register the MCU
   reads after each transfer; fail the load (error message) and, for a save,
   **do not write the file**.
2. Flow control on the core side (a busy/wait-state the MCU honours).
3. Document the requirement: the core must accept one 32-bit write per
   800 ns sustained (more at faster SPI clocks), and return read data within
   ~2 word times (the 8 dummy bytes).

**Our workaround:** posted writes into a 512-word queue, sequential
read-ahead for reads, and a zero-latency arbiter (`rtl/gamebub/gb_host.sv`,
`rtl/gamebub/gb_eth_arb.sv`), verified with a testbench that streams at the
real rate through a model of these FIFOs (`sim/tb_gamebub.sv +paced`).

## 2. `max_transfer_speed` below 5000 KB/s panics the firmware

**Affects:** firmware `firmware/handheld/src/device/drivers/fpga/mod.rs:226`
and the `files.json` parser.

**What happens.** `files.json` documents `max_transfer_speed` (KB/s, default
5000). The firmware turns it into an SPI clock of `speed * 2 kHz` and looks
for an SPI driver at or below that rate; the drivers are 40, 20, 16 and
10 MHz. Any value below 5000 has no match, and the firmware panics while
loading the core:

```
Crash Report
  FW: 1.1.0-beta2
  HW: 1.4.1.0
PANIC: thread 'Worker' at src/device/drivers/fpga/mod.rs:226:21:
No suitable spi for max clock Some(2000000Hz)
```

The device shows a black screen for a moment, then reboots to the menu.

**Reproduce.** Set `"max_transfer_speed": 1000` on any file in `files.json`.

**Suggested fix:** validate the value when parsing `files.json` (reject or
clamp to the slowest SPI rate, with a message), and/or fall back to the
slowest driver instead of panicking.

## 3. ILI9806E: back-porch clamp has the wrong sign; DE never falls (black screen)

**Affects:** `src/main/scala/platform/handheld/display/ILI9806E.scala`.

**What happens.** The line length is
`totalWidth = floor(sourceFramePeriod * clockHz / totalHeightMin)`. When the
back porch that leaves exceeds `hBackPorchMax` (126):

```scala
var hBackPorch = (totalWidth - hActive - hSync - hFrontPorchMin)
if (hBackPorch > hBackPorchMax) {
  val amount = hBackPorchMax - hBackPorch   // negative
  hBackPorch -= amount                      // grows instead of shrinking
  hFrontPorch += amount                     // goes negative
}
```

DE is cleared at `x == hSync + hBackPorch + hActive - 1`, which is now past
the end of the line, so **DE is set once and never cleared**. The panel shows
nothing, and neither the core's picture nor the firmware's overlay (Home
menu, "Loading...", errors) is visible, so the device looks hung. None of the
asserts fire.

**Example.** Display clock 32.5 MHz, source frame 640x407 at 15.6672 MHz
(16.626 ms): `totalWidth` = 668, raw back porch 182, clamped to **238**,
front porch **-54**; DE falls at x = 721 but the line wraps at 667. Generated
`ILI9806E.sv`: `x == 10'h29B` (wrap), DE set at `10'hF1`, cleared at
`10'h2D1`. Confirmed on hardware with an ILA (`lcd_data_en` stuck high).

It triggers whenever `totalWidth > 612`, i.e. for display clocks above about
29.7 MHz at 60 Hz, although `getClockDisplayHz` advertises 25.8-35 MHz.

**Suggested fix:** `val amount = hBackPorch - hBackPorchMax`.

**Our workaround:** display clock 29.55 MHz (line 607 clocks).

## 4. `common.xdc` silently requires the core's display clock to be named `clk_dpi`

**Affects:** `verilog/handheld/common.xdc:38`.

```
set_clock_groups -name exclusive_dpi_hdmi -physically_exclusive -group clk_dpi -group clk_hdmi_clk_wiz_hdmi
```

`clk_dpi` is the name of the clock Vivado derives from the core's display
MMCM output, which it takes from **the core's own net name**. With any other
name (ours was `clk_display_u`) the constraint matches nothing (two critical
warnings that are easy to miss), and every path between the core's display
clock and the HDMI clock through `bufgmux_av` is timed as synchronous: ~3,000
paths at a 0.18 ns requirement, WNS -13.5 ns, and the router grinds for hours.

**Suggested fix:** derive the clock from the mux pin instead of a name the
core must guess, e.g.
`-group [get_clocks -of_objects [get_pins bufgmux_av/I0]]`, or document that
the net must be named `clk_dpi`.

## 5. `build_core.py` passes "skipped" files to edalize anyway

**Affects:** `scripts/build_core.py`, in `build()`.

```python
file_type = {...}.get(file_extension)
if not file_type:
    print("WARNING: Skipping unknown file type:", file)
files.append(dict(name=file, file_type=file_type))   # still appended
```

Any non-RTL file under the core's `rtl/` (a README, a `.txt`) is appended
with `file_type=None`, and edalize then fails with
`AttributeError: 'NoneType' object has no attribute 'startswith'`
(`edalize/tools/vivado.py`, `setup`).

**Suggested fix:** `continue` after the warning.

## 6. Feature request: core-to-host random-access file I/O

**Affects:** `HostV0` (the `commandCore` channel and the `coreRequest`
interrupt are wired, but no commands are defined) and the firmware.

Files can only be copied whole into core memory at setup and back at exit.
For a computer core this means:

- a hard disk must fit in SDRAM next to RAM and ROM: **16-20 MiB** on the
  Mac LC, where real disks of the era were 40-160 MB and users' images are
  hundreds of MB;
- **CD-ROM is impossible** (images are 100-700 MB);
- disk writes reach the card only on a clean exit; a crash or power loss
  loses the session.

**Proposal.** Two core-to-host commands on the existing channel, parameters
in the `0xF000_1000` registers:

- `FileRead(file_id, byte_offset, length, core_address)`: the MCU reads from
  the selected file and writes `length` bytes to `core_address` through the
  normal host memory path, then completes the command.
- `FileWrite(file_id, byte_offset, length, core_address)`: the reverse.

Sector-sized requests (512 B to a few KiB) with a few ms of latency are fine
for SCSI and CD emulation. `files.json` would mark such files as
`"access": "random"` (not loaded at setup), with no size limit beyond FAT32.

## 7. Feature request: the second SDRAM chip on rev 4

**Affects:** `verilog/handheld/top.sv` (`// TODO: support second SDRAM chip
(rev 4)`, `sdram_cs_n[1]` tied high, `sdram_cke[1]` low).

Rev 4 has two 32 MiB chips but cores can only use one. Exposing the second
(as `SdramV0` with `chips = 2`, or a second interface) would double the space
for whole-file images until #6 exists.

## 8. An optional user-selected file cannot be skipped

**Affects:** firmware core loader / file browser.

Our hard disk is file 0, `"user_selected": true, "optional": true`. At launch
the browser insists on a file; with no matching file on the card it shows an
empty list and offers no way to start the core without one (we ship a
400 KiB placeholder image). The only skip is "Run Cartridge" (`cartridge_selectable`), which is
meant for cartridge cores.

**Suggested fix:** a "None" entry in the browser for optional files.

## 9. The firmware that runs SD-card cores is not published

SD-card cores need firmware **v1.1-beta2** or later (v1.0.2 does not list
`/cores/`, v1.1-beta1 had a bug that hid them). The official releases page
(`elipsitz/gamebub/releases`) stops at v1.0.2; we found beta2 attached to a
third-party core's release. Please publish the beta (or note in the core
docs where developers should get it, and which version is required).

## 10. Undocumented requirements

Things that are true, matter, and are not in the core documentation:

- **The whole framework runs on the core's `clockOutSystem`** and is held in
  reset until `clocks.locked`: the MCU's register access, the button IRQ
  and the overlay all depend on the core's clocks.
- **Host memory throughput and latency** (see #1).
- **The display clock net name** (see #4) and the **usable display clock
  range** (see #3).
- **How files are packed**: 32-bit words, and in which byte order. We detect
  the order from the ROM because it is not stated.
- **Card format**: FAT32 and MBR are required (stated in the user guide's
  "Playing games" page, not in the developer docs where a core author looks).
- **`max_transfer_speed`** cannot go below 5000 (see #2).

## 11. A core cannot pass Verilog macros or include paths to the build

**Affects:** `scripts/build_core.py` (`verilog_defines` comes only from the
target, `BOARD_REV_n`; no include directories are passed to edalize).

Cores ported from other platforms often rely on both. Ours (from MiSTer)
needed:

- the macro `USE_ASC_AUDIO=1`, which MiSTer sets in its Quartus project.
  Without it the sound chip compiles to a stub with its samples tied to zero:
  the core builds cleanly and is silent, with no warning;
- two `` `include `` files, which fail to resolve without an include path.

We patch both into the imported sources (`scripts/import_maclc.py`).

**Suggested fix:** let the core's `build.mill` (e.g. a `verilogDefines` and
`includeDirs` override on `FrameworkModule`) add to what `build_core.py`
hands edalize.
