// ============================================================================
// gb_clocks.sv — every clock the core uses, from one MMCM.
//
//   50 MHz x 13 = 650 MHz VCO (Artix-7 -1 range: 600..1200 MHz)
//     /20  32.5  MHz  clk_sys      the machine; v8_clocks/VIA/Egret assume it
//     /10  65.0  MHz  clk_mem      SDRAM controller ("clk_64"): must be exactly
//                                  2x clk_sys with rising edges aligned, which
//                                  sdram.v's t[0] start-parity rule relies on
//     /4  162.5  MHz  clk_spi      framework MCU SPI sampler (>= 160 MHz)
//     /20  32.5  MHz  clk_display  LCD dot clock (ILI9806E: 25.8..35 MHz @60Hz)
//
// clk_sys and clk_mem go through BUFGs from the same MMCM, so their phase
// relationship is fixed and fully timed by Vivado (no CDC between them).
// The frequencies are mirrored in chisel/src/MacLC.scala; keep them in sync.
// ============================================================================
`default_nettype none

module gb_clocks (
	input  wire clk_in_50mhz,
	output wire clk_sys,
	output wire clk_mem,
	output wire clk_spi,
	output wire clk_display,
	output wire locked
);

	wire clk_fb;
	wire clk_sys_u, clk_mem_u, clk_spi_u, clk_display_u;

	MMCME2_BASE #(
		.BANDWIDTH          ("OPTIMIZED"),
		.CLKIN1_PERIOD      (20.000),
		.CLKFBOUT_MULT_F    (13.000),
		.DIVCLK_DIVIDE      (1),
		.CLKOUT0_DIVIDE_F   (20.000),
		.CLKOUT1_DIVIDE     (10),
		.CLKOUT2_DIVIDE     (4),
		.CLKOUT3_DIVIDE     (20),
		.CLKOUT0_PHASE      (0.0),
		.CLKOUT1_PHASE      (0.0),
		.CLKOUT2_PHASE      (0.0),
		.CLKOUT3_PHASE      (0.0),
		.CLKOUT0_DUTY_CYCLE (0.5),
		.CLKOUT1_DUTY_CYCLE (0.5),
		.CLKOUT2_DUTY_CYCLE (0.5),
		.CLKOUT3_DUTY_CYCLE (0.5),
		.STARTUP_WAIT       ("FALSE")
	) mmcm (
		.CLKIN1   (clk_in_50mhz),
		.CLKFBIN  (clk_fb),
		.CLKFBOUT (clk_fb),
		.CLKFBOUTB(),
		.CLKOUT0  (clk_sys_u),
		.CLKOUT0B (),
		.CLKOUT1  (clk_mem_u),
		.CLKOUT1B (),
		.CLKOUT2  (clk_spi_u),
		.CLKOUT2B (),
		.CLKOUT3  (clk_display_u),
		.CLKOUT3B (),
		.CLKOUT4  (),
		.CLKOUT5  (),
		.CLKOUT6  (),
		.LOCKED   (locked),
		.PWRDWN   (1'b0),
		.RST      (1'b0)
	);

	BUFG bufg_sys     (.I(clk_sys_u),     .O(clk_sys));
	BUFG bufg_mem     (.I(clk_mem_u),     .O(clk_mem));
	BUFG bufg_spi     (.I(clk_spi_u),     .O(clk_spi));
	BUFG bufg_display (.I(clk_display_u), .O(clk_display));

endmodule

`default_nettype wire
