# Display-path probes (net names from the synthesized checkpoint).
set core handheld_top/core/extModule
set probes [list \
    [list lcd_hsync     {lcd_hsync_OBUF}] \
    [list lcd_vsync     {lcd_vsync_OBUF}] \
    [list lcd_de        {lcd_data_en_OBUF}] \
    [list lcd_msb_rgb   {{lcd_data_r_OBUF[7]} {lcd_data_g_OBUF[7]} {lcd_data_b_OBUF[7]}}] \
    [list hdmi_enable   {hdmi_enable}] \
    [list mcu_spi_cs_n  {mcu_spi_cs_n_IBUF}] \
    [list mcu_irq_n     {mcu_irq_n_OBUF}] \
    [list core_locked   [list $core/pll_locked_s]] \
    [list core_vblank   [list $core/video/vblank]] \
    [list host_state    [list "$core/host/cmd_state_reg_n_0_\[0\]" "$core/host/cmd_state_reg_n_0_\[1\]"]] \
]
