#!/usr/bin/env bash
#
# Build and run the whole-core Verilator testbench (sim/tb_gamebub.sv).
#
#   sim/run_tb.sh                       # boot with the default ROM, 400 frames
#   sim/run_tb.sh +frames=60 +dump=20   # quick check
#   sim/run_tb.sh +hd=path/to/disk.hda  # with a hard disk image
#   sim/run_tb.sh +host_be              # MCU packs file bytes big-endian
#
# Frames land in sim_out/frame_NNNN.ppm; scripts/ppm2png.py converts them.
# The build is cached in sim/obj_dir (or $SIM_OBJ, so two runs can coexist);
# it is redone when sources change.
set -euo pipefail
cd "$(dirname "$0")/.."

# macOS: the Command Line Tools may ship a newer SDK than the installed linker
# understands ("tapi error: unknown architecture arm64e.x1" at link time).
# Prefer the SDK inside Xcode.app, which matches its own linker.
if [[ "$(uname)" == "Darwin" && -z "${SDKROOT:-}" ]]; then
	xsdk=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
	[[ -d "$xsdk" ]] && export SDKROOT="$xsdk"
fi

files=(sim/xilinx_stubs.v sim/tg68k_v/TG68K_Pack.sv sim/tg68k_v/TG68K_ALU.v sim/tg68k_v/TG68KdotC_Kernel.v)
while IFS= read -r f; do files+=("$f"); done < <(find rtl -name '*.v' -o -name '*.sv' | sort)
files+=(sim/tb_gamebub.sv)

obj=${SIM_OBJ:-sim/obj_dir}
verilator --binary --timing -j 0 -O3 --x-assign fast --x-initial fast \
	-Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
	--top-module tb_gamebub --Mdir "$obj" -o Vtb_gamebub \
	"${files[@]}" > "$obj.build.log" 2>&1 || { tail -40 "$obj.build.log"; exit 1; }

mkdir -p sim_out
exec "$obj/Vtb_gamebub" "$@"
