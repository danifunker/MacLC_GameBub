#!/usr/bin/env bash
#
# Verilator lint of the whole core, no Vivado needed (runs on macOS).
#
#   scripts/lint.sh            # errors + warnings in rtl/gamebub/
#   scripts/lint.sh --all      # every warning, including imported MiSTer RTL
#
# Verilator cannot read VHDL, so the CPU comes from the GHDL-converted
# Verilog copy in sim/tg68k_v/ (scripts/import_maclc.py keeps it in sync),
# and the Xilinx primitives from sim/xilinx_stubs.v.
set -euo pipefail
cd "$(dirname "$0")/.."

files=(sim/xilinx_stubs.v sim/tg68k_v/TG68K_Pack.sv sim/tg68k_v/TG68K_ALU.v sim/tg68k_v/TG68KdotC_Kernel.v)
while IFS= read -r f; do files+=("$f"); done < <(find rtl -name '*.v' -o -name '*.sv' | sort)

# The imported MiSTer RTL is lint-noisy by design (Quartus-era idioms); keep
# the categories that matter for the platform layer.
waivers=(-Wno-WIDTH -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME
         -Wno-CASEINCOMPLETE -Wno-UNDRIVEN -Wno-SYNCASYNCNET -Wno-MULTIDRIVEN
         -Wno-BLKSEQ -Wno-PINMISSING -Wno-UNOPTFLAT -Wno-GENUNNAMED -Wno-VARHIDDEN
         -Wno-IMPORTSTAR -Wno-LATCH -Wno-COMBDLY -Wno-CASEOVERLAP -Wno-ASCRANGE
         -Wno-PROCASSINIT)

log=$(mktemp)
set +e
verilator --lint-only --timing -Wall -Wno-fatal "${waivers[@]}" \
	--top-module maclc_gamebub "${files[@]}" >"$log" 2>&1
status=$?
set -e

if [[ "${1:-}" == "--all" ]]; then
	cat "$log"
else
	grep -E '^%Error' "$log" || true
	grep -E '^%Warning' "$log" | grep -E 'rtl/gamebub/' || true
fi
errors=$(grep -c '^%Error' "$log" || true)
echo "lint: ${errors} error(s) (verilator exit ${status})"
rm -f "$log"
[[ "$errors" == "0" ]]
