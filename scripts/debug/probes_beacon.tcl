# Beacon snapshot (all 8 status words, refreshed every 0.5 s) plus live signals.
# snap bit layout: word0 [31:0] host state ... word7 [255:224] status.
set core handheld_top/core/extModule
set ila_depth 1024
set probes [list \
    [list lcd_hsync     {lcd_hsync_OBUF}] \
    [list lcd_vsync     {lcd_vsync_OBUF}] \
    [list lcd_de        {lcd_data_en_OBUF}] \
    [list mcu_spi_cs_n  {mcu_spi_cs_n_IBUF}] \
    [list mcu_irq_n     {mcu_irq_n_OBUF}] \
    [list core_locked   [list $core/pll_locked_s]] \
    [list core_vblank   [list $core/video/vblank]] \
    [list host_state    [list "$core/host/cmd_state_reg_n_0_\[0\]" "$core/host/cmd_state_reg_n_0_\[1\]"]] \
]
foreach i {0 1 2 3 4 5 6 7 8 9 10 11 12 13 14} { lappend probes [list snap$i [list "$core/beacon/snap_reg_n_0_\[$i\]"] DATA] }
for {set i 32} {$i < 256} {incr i} { lappend probes [list snap$i [list "$core/beacon/snap_reg_n_0_\[$i\]"] DATA] }
