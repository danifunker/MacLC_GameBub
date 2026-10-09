#!/usr/bin/env bash
# Build and run sim/tb_gamebub.sv in Vivado's xsim (no Verilator needed).
#   build/xsim/run_xsim.sh [plusargs without the leading +]
set -euo pipefail
source ~/atrixdev/2025.2/Vivado/settings64.sh
repo=/mnt/c/repos/MacLC_GameBub
work=~/xsim_maclc            # on ext4: xsim is slow on /mnt/c
mkdir -p "$work" && cd "$work"
files=($repo/sim/xilinx_stubs.v $repo/sim/tg68k_v/TG68K_Pack.sv $repo/sim/tg68k_v/TG68K_ALU.v $repo/sim/tg68k_v/TG68KdotC_Kernel.v)
while IFS= read -r f; do files+=("$f"); done < <(find $repo/rtl -name '*.v' -o -name '*.sv' | sort)
files+=($repo/sim/tb_gamebub.sv)
xvlog -sv --relax "${files[@]}" > xvlog.log 2>&1 || { grep -E "ERROR" xvlog.log | head -30; exit 1; }
xelab tb_gamebub -s tb_sim --relax --timescale 1ns/1ps -O3 --debug off > xelab.log 2>&1 || { grep -E "ERROR" xelab.log | head -30; exit 1; }
args=()
for a in "$@"; do args+=(-testplusarg "$a"); done
mkdir -p sim_out
xsim tb_sim -R "${args[@]}"
