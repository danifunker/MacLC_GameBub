# Insert an ILA into a synthesized checkpoint and implement it (non-project).
#   vivado -mode batch -source ila_build.tcl -tclargs <synth.dcp> <outdir> <probes.tcl>
# <probes.tcl> sets `probes` to a list of {name {net patterns...}}.
set dcp    [lindex $argv 0]
set outdir [lindex $argv 1]
source     [lindex $argv 2]
file mkdir $outdir

# Same as the framework's impl_1 run: the synthesized netlist plus the three
# constraint files, in the same order. (The synth checkpoint alone carries no
# pin assignments or timing.)
set repo /mnt/c/repos/MacLC_GameBub
create_project -in_memory -part xc7a100tcsg324-1
set_property design_mode GateLvl [current_fileset]
set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO} [current_project]
add_files -quiet $dcp
read_xdc $repo/framework/verilog/handheld/rev_4.xdc
read_xdc $repo/framework/verilog/handheld/common.xdc
read_xdc $repo/rtl/gamebub/maclc.xdc
link_design -top top_handheld -part xc7a100tcsg324-1

# Hard safety checks before anything goes near hardware.
set unplaced [get_ports -quiet -filter {PACKAGE_PIN == "" || IOSTANDARD == "DEFAULT"}]
set userid [get_property BITSTREAM.CONFIG.USERID [current_design]]
puts "CHECK clocks: [lsort [get_clocks]]"
puts "CHECK ports=[llength [get_ports]] unassigned_or_default_iostd=[llength $unplaced] USERID=$userid"
if {[llength $unplaced] != 0 || ![regexp -nocase {^(0x|32'h)B0100004$} $userid] || [llength [get_clocks -quiet sys_clk_pin]] != 1 || [llength [get_clocks -quiet sdram_clk]] != 1} {
    puts "SAFETY_ABORT unassigned/default ports: [lrange $unplaced 0 20]"
    exit 1
}

# Clocked by the board's 50 MHz oscillator: independent of the core's MMCM.
set ila_clk [get_nets clk_in_50mhz_BUFG]
create_debug_core u_ila_0 ila
if {![info exists ila_depth]} { set ila_depth 4096 }
set_property C_DATA_DEPTH         $ila_depth [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES  2     [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN          false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN         false [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER        false [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU    true  [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 2    [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk $ila_clk
set_property C_CLK_INPUT_FREQ_HZ 50000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false   [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1          [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk $ila_clk

set i 0
foreach p $probes {
    lassign $p name pats ptype
    if {$ptype eq ""} { set ptype DATA_AND_TRIGGER }
    set nets {}
    foreach pat $pats {
        set n [get_nets -quiet $pat]
        if {[llength $n] == 0} { puts "PROBE_MISSING $name: $pat" } else { lappend nets {*}$n }
    }
    if {[llength $nets] == 0} continue
    if {$i > 0} { create_debug_port u_ila_0 probe }
    set port u_ila_0/probe$i
    set_property PROBE_TYPE $ptype [get_debug_ports $port]
    set_property port_width [llength $nets] [get_debug_ports $port]
    connect_debug_port $port $nets
    puts "PROBE $i $name width=[llength $nets]: [lrange $nets 0 3]"
    incr i
}

implement_debug_core
# Every probed signal is asynchronous to the 50 MHz sampling clock.
set_false_path -to [get_cells -hierarchical -filter {NAME =~ u_ila_0/*}] \
    -from [get_clocks -quiet {clk_dpi clk_sys_u clk_mem_u clk_spi_u clk_hdmi_clk_wiz_hdmi}]

opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 5 -file $outdir/timing_summary.rpt
report_utilization -file $outdir/utilization.rpt
puts "TIMING [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] setup, [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]] hold"
write_bitstream -force $outdir/MacLC-gamebub_rev4.bit
write_debug_probes -force $outdir/MacLC-gamebub_rev4.ltx
puts "ILA_BUILD_DONE"
