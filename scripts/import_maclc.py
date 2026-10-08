#!/usr/bin/env python3
"""
Import the Macintosh LC machine RTL from the MiSTer core into rtl/maclc/.

The MiSTer repository (../MacLC_MiSTer by default) stays the source of truth
for the machine itself. This script copies the files the Game Bub build needs
and applies the small number of platform patches listed in PATCHES below.
Every patch is an exact text match: if upstream changes the text a patch
targets, the import stops with an error instead of silently producing a
different design. Re-run it after pulling MiSTer changes, review the diff,
and commit.

    python3 scripts/import_maclc.py [--src ../MacLC_MiSTer]

What goes where:
  rtl/maclc/        machine RTL, built by Vivado (every .v/.sv/.vhdl under
                    rtl/ is picked up automatically by the framework build;
                    any other file type there makes the build crash). MiSTer
                    .v files are written as .sv so that Vivado parses them
                    as SystemVerilog, as Verilator does.
  scripts/import_maclc.txt
                    the MiSTer commit the RTL was imported from
  sim/tg68k_v/      the GHDL-converted Verilog TG68K, for Verilator only.
                    Vivado builds the VHDL originals, as Quartus does on
                    MiSTer; having both under rtl/ would define the CPU twice.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Machine RTL copied verbatim (or patched below), relative to the MiSTer rtl/.
FILES = [
    "adb.sv",
    "adb_device.sv",
    "addrController_top.v",
    "addrDecoder.v",
    "ariel_ramdac.sv",
    "asc.sv",
    "cd_audio.sv",
    "dataController_top.sv",
    "fetch_cache.sv",
    "floppy.v",
    "floppy_sd.v",
    "floppy_track_decoder.v",
    "floppy_track_encoder.v",
    "maclc_v8_video.sv",
    "mfm_track_encoder.v",
    "mfm_write_decoder.v",
    "ncr5380.sv",
    "ps2_kbd.sv",
    "ps2_mouse.v",
    "pseudovia.sv",
    "scc.v",
    "scsi.v",
    "sdram.v",
    "swim.v",
    "v8_clocks.sv",
    "via6522.sv",
    "egret/egret_wrapper.sv",
    "egret/m68hc05_alu.sv",
    "egret/m68hc05_core.sv",
    "uart/rxuart.v",
    "uart/txuart.v",
    "tg68k/tg68k.v",
]

# VHDL CPU kernel: the framework build only recognises the .vhdl extension.
VHDL_FILES = [
    "tg68k/TG68K_Pack.vhd",
    "tg68k/TG68K_ALU.vhd",
    "tg68k/TG68KdotC_Kernel.vhd",
]

# Verilator-only CPU kernel (kept in sync with the VHDL upstream).
SIM_FILES = [
    "tg68k/TG68K_ALU.v",
    "tg68k/TG68K_Pack.sv",
    "tg68k/TG68KdotC_Kernel.v",
]

# Deliberately NOT imported:
#   pll.v, pll_video.v, pll/   Altera PLLs (rtl/gamebub/gb_clocks.sv instead)
#   vram_bram.sv               VRAM lives in the external SRAM (gb_vram_sram.sv)
#   pds/pds_enet.sv            Ethernet needs the MiSTer Main; no network here
#   dbg_probes.sv              Altera In-System Sources & Probes
#   egret_behavioral.sv        opt-in debug fallback, not instantiated


class PatchError(Exception):
    pass


def replace_once(text, old, new, what):
    count = text.count(old)
    if count != 1:
        raise PatchError(f"{what}: expected exactly 1 match, found {count}")
    return text.replace(old, new)


def hex_bytes(path):
    values = []
    for line in path.read_text(encoding="latin-1").split():
        values.append(int(line, 16))
    return values


def patch_egret(text, src_rtl):
    """Inline the Egret ROM/PRAM images.

    $readmemh resolves relative paths against the tool's run directory,
    which differs between Quartus, Vivado project runs and Verilator. Inline
    constant assignments work identically in all of them.
    """
    old = (
        "`ifdef SIMULATION\n"
        "    $readmemh(\"../rtl/egret/egret_rom.hex\", rom);\n"
        "    $display(\"EGRET ROM: Loaded %0d bytes from ../rtl/egret/egret_rom.hex\", ROM_SIZE);\n"
        "    $readmemh(\"../rtl/egret/egret.pram\", pram);\n"
        "    $display(\"EGRET PRAM: Loaded 256 bytes from ../rtl/egret/egret.pram\");\n"
        "`else\n"
        "    $readmemh(\"rtl/egret/egret_rom.hex\", rom);\n"
        "    $readmemh(\"rtl/egret/egret.pram\", pram);\n"
        "`endif\n"
    )
    rom = hex_bytes(src_rtl / "egret" / "egret_rom.hex")
    pram = hex_bytes(src_rtl / "egret" / "egret.pram")
    if len(pram) != 256:
        raise PatchError(f"egret.pram: expected 256 bytes, found {len(pram)}")
    lines = ["    // [gamebub] egret_rom.hex and egret.pram, inlined by scripts/import_maclc.py\n"]
    lines += [f"    rom[{i}] = 8'h{v:02X};\n" for i, v in enumerate(rom)]
    lines += [f"    pram[{i}] = 8'h{v:02X};\n" for i, v in enumerate(pram)]
    return replace_once(text, old, "".join(lines), "egret_wrapper.sv $readmemh block")


def patch_include(include_name):
    """Inline a `include: the framework build passes no include paths."""

    def patch(text, src_rtl):
        body = (src_rtl / include_name).read_text(encoding="latin-1")
        new = (
            f"// [gamebub] begin inlined {include_name}\n"
            f"{body.rstrip()}\n"
            f"// [gamebub] end inlined {include_name}\n"
        )
        return replace_once(text, f'`include "{include_name}"\n', new, f"`include {include_name}")

    return patch


def patch_asc(text, src_rtl):
    """Enable the ASC sound model, as MacLC.qsf does.

    MiSTer sets VERILOG_MACRO "USE_ASC_AUDIO=1" in MacLC.qsf. The framework
    build passes no Verilog macros, and without it asc.sv compiles a register
    stub whose samples are tied to zero: the ROM programs the chip and fills
    its FIFO, and nothing is heard (not even the startup chime).
    """
    return replace_once(
        text,
        "`ifdef USE_ASC_AUDIO\n",
        '// [gamebub] MacLC.qsf: VERILOG_MACRO "USE_ASC_AUDIO=1"\n'
        "`define USE_ASC_AUDIO\n"
        "`ifdef USE_ASC_AUDIO\n",
        "asc.sv USE_ASC_AUDIO",
    )


def patch_v8_video(text, src_rtl):
    """Replace the 1-cycle BRAM VRAM read with a request/valid handshake.

    On MiSTer the scanline prefetch read an on-chip framebuffer with a fixed
    one-cycle latency, one word per clock. On Game Bub VRAM is the external
    async SRAM, shared with CPU VRAM writes, so a read is a request that may
    wait (vram_rready low) and whose data arrives later (vram_rvalid).
    Reads complete in order, so the write index simply counts returns.
    """
    text = replace_once(
        text,
        "    output [17:0] vram_raddr,\n"
        "    input  [15:0] vram_rdata\n",
        "    output [17:0] vram_raddr,\n"
        "    // [gamebub] VRAM is external SRAM: request/grant + in-order valid\n"
        "    output        vram_rd,      // a read of vram_raddr is wanted\n"
        "    input         vram_rready,  // ...and was accepted this cycle\n"
        "    input         vram_rvalid,  // vram_rdata holds the next word\n"
        "    input  [15:0] vram_rdata\n",
        "maclc_v8_video ports",
    )
    old_fetch = text[
        text.index("reg [9:0] fetch_idx;      // address-phase word index"):
        text.index("// --- Display side:")
    ]
    new_fetch = (
        "reg [9:0] fetch_idx;      // next word to request\n"
        "reg [9:0] fetch_wr_idx;   // next word to land in the line buffer\n"
        "\n"
        "assign vram_raddr = fetch_packed_base + {8'd0, fetch_idx};\n"
        "// [gamebub] Never request on the line-restart cycle: its address belongs\n"
        "// to the line that is ending.\n"
        "wire   fetch_restart = pix_en && h_count == h_total - 1;\n"
        "assign vram_rd = !reset && !fetch_restart && (fetch_idx < words_per_line);\n"
        "\n"
        "always @(posedge clk_sys) begin\n"
        "    if (reset || fetch_restart) begin\n"
        "        fetch_idx    <= 10'd0;   // restart prefetch each scanline\n"
        "        fetch_wr_idx <= 10'd0;\n"
        "    end else begin\n"
        "        if (vram_rd && vram_rready)\n"
        "            fetch_idx <= fetch_idx + 1'b1;\n"
        "        if (vram_rvalid) begin\n"
        "            linebuf[{fetch_buf, fetch_wr_idx[8:0]}] <= vram_rdata;\n"
        "            fetch_wr_idx <= fetch_wr_idx + 1'b1;\n"
        "        end\n"
        "    end\n"
        "end\n"
        "\n"
    )
    return replace_once(text, old_fetch, new_fetch, "maclc_v8_video fetch block")


def patch_sdram(text, src_rtl):
    """Artix-7 I/O for the SDRAM controller.

    - The framework splits the data bus into dataOut/dataIn/dataDir, so the
      controller drives an explicit output enable instead of a tristate.
    - The inverted SDRAM clock comes from an ODDR (was altddio_out).
    - One more row-address bit: Game Bub's W9825G6KH has 8192 rows, so word
      address bit 23 reaches the upper 16 MiB (disk images live there).
    - I/O-block packing of the capture and output registers is requested in
      rtl/gamebub/maclc.xdc, where their timing stops depending on placement
      (the lesson of rtl/sdram.v's 2026-09-12 read-capture note).
    """
    start = text.index("`ifdef TB_NO_TRISTATE\n\t// TB-ONLY pin split")
    end = text.index("`endif\n", start) + len("`endif\n")
    text = text[:start] + (
        "\t// [gamebub] split data bus: the framework owns the pad tristate\n"
        "\toutput reg [15:0]   sd_data,    // controller -> chip\n"
        "\tinput      [15:0]   sd_data_in, // chip -> controller\n"
        "\toutput reg          sd_data_oe, // 1 = drive sd_data\n"
    ) + text[end:]
    text = replace_once(
        text,
        "`ifdef TB_NO_TRISTATE\nwire [15:0] sd_data_rd = sd_data_in;\n`else\nwire [15:0] sd_data_rd = sd_data;\n`endif\n",
        "wire [15:0] sd_data_rd = sd_data_in;\n",
        "sd_data_rd",
    )
    text = replace_once(
        text,
        "`ifndef TB_NO_TRISTATE\n\tsd_data <= 16'bZZZZZZZZZZZZZZZZ;\n`endif\n",
        "\tsd_data_oe <= 1'b0;\n",
        "tristate default",
    )
    text = replace_once(
        text,
        "\t\t\t\tif (we_latch) sd_data <= din_q;\n",
        "\t\t\t\tif (we_latch) begin sd_data <= din_q; sd_data_oe <= 1'b1; end\n",
        "write data drive",
    )
    text = replace_once(
        text,
        "\t\t\tsd_addr <= req_flp ? { 1'b0, flp_addr[19:8] } :\n"
        "\t\t\t           req_dl  ? { 1'b0, dl_addr[19:8]  } :\n"
        "\t\t\t           req_cpu ? { 1'b0, addr[19:8] }     : { 1'b0, eth_addr[19:8] };\n",
        "\t\t\t// [gamebub] row bit 12 = word address bit 23 (32 MiB part)\n"
        "\t\t\tsd_addr <= req_flp ? { flp_addr[23], flp_addr[19:8] } :\n"
        "\t\t\t           req_dl  ? { dl_addr[23],  dl_addr[19:8]  } :\n"
        "\t\t\t           req_cpu ? { addr[23],     addr[19:8] }     : { eth_addr[23], eth_addr[19:8] };\n",
        "row address",
    )
    start = text.index("altddio_out\n#(")
    end = text.index(");\n", start) + len(");\n")
    text = text[:start] + (
        "// [gamebub] SDRAM_CLK = inverted clk_64 (the chip's rising edge is a\n"
        "// clk_64 falling edge), produced in the output DDR register.\n"
        "ODDR #(\n"
        "\t.DDR_CLK_EDGE(\"SAME_EDGE\"),\n"
        "\t.INIT(1'b0),\n"
        "\t.SRTYPE(\"SYNC\")\n"
        ") sdramclk_ddr (\n"
        "\t.Q(sd_clk),\n"
        "\t.C(clk_64),\n"
        "\t.CE(1'b1),\n"
        "\t.D1(1'b0),\n"
        "\t.D2(1'b1),\n"
        "\t.R(1'b0),\n"
        "\t.S(1'b0)\n"
        ");\n"
    ) + text[end:]
    return text


PATCHES = {
    "egret/egret_wrapper.sv": patch_egret,
    "scsi.v": patch_include("scsi_vendor.vh"),
    "cd_audio.sv": patch_include("cd_vol_lut.vh"),
    "asc.sv": patch_asc,
    "maclc_v8_video.sv": patch_v8_video,
    "sdram.v": patch_sdram,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--src", type=Path, default=ROOT.parent / "MacLC_MiSTer")
    args = parser.parse_args()

    src_rtl = args.src / "rtl"
    if not src_rtl.is_dir():
        sys.exit(f"MiSTer source not found at {args.src}")

    try:
        commit = subprocess.check_output(
            ["git", "-C", str(args.src), "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(
            ["git", "-C", str(args.src), "status", "--porcelain", "--", "rtl"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        commit, dirty = "unknown", ""

    dst = ROOT / "rtl" / "maclc"
    sim_dst = ROOT / "sim" / "tg68k_v"
    for d in (dst, sim_dst):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)

    try:
        for name in FILES:
            text = (src_rtl / name).read_text(encoding="latin-1")
            if name in PATCHES:
                text = PATCHES[name](text, src_rtl)
            # .v -> .sv: Vivado reads .v as strict Verilog-2001 and rejects what
            # Quartus lets through (declarations in unnamed blocks in scc.v).
            # Verilator, used by the lint and the sim, parses everything as
            # SystemVerilog, so this makes Vivado build what was verified.
            out = dst / (name[: -len(".v")] + ".sv" if name.endswith(".v") else name)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(text, encoding="latin-1")
        for name in VHDL_FILES:
            out = dst / (name[: -len(".vhd")] + ".vhdl")
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src_rtl / name, out)
        for name in SIM_FILES:
            shutil.copyfile(src_rtl / name, sim_dst / Path(name).name)
    except PatchError as e:
        sys.exit(f"Patch failed (upstream changed?): {e}")

    # Not in rtl/: the framework's build_core.py passes every file under rtl/
    # to edalize, which crashes on types other than .v/.sv/.vhdl/.xdc.
    (ROOT / "scripts" / "import_maclc.txt").write_text(
        "rtl/maclc/ is imported by scripts/import_maclc.py - do not edit it by hand.\n"
        "Change the MiSTer source (or the patches in the script) and re-import.\n\n"
        f"source: {os.path.relpath(args.src.resolve(), ROOT)}\n"
        f"commit: {commit}{' (rtl/ had uncommitted changes)' if dirty else ''}\n"
    )
    print(f"Imported {len(FILES) + len(VHDL_FILES)} files from {commit[:12]} into {dst}")


if __name__ == "__main__":
    main()
