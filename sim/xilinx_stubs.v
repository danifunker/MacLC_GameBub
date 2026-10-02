// ============================================================================
// Minimal behavioural stand-ins for the Xilinx primitives the core uses, so
// that lint and simulation can run without Vivado. Never part of the FPGA build
// (sim/ is outside rtl/, which is all the framework build picks up).
// The MMCM model needs --timing; it produces clk_sys/clk_mem
// (32.5 / 65 MHz) with aligned rising edges.
// ============================================================================
`timescale 1ps/1ps

module MMCME2_BASE #(
	parameter BANDWIDTH = "OPTIMIZED",
	parameter real CLKIN1_PERIOD = 20.0,
	parameter real CLKFBOUT_MULT_F = 13.0,
	parameter integer DIVCLK_DIVIDE = 1,
	parameter real CLKOUT0_DIVIDE_F = 20.0,
	parameter integer CLKOUT1_DIVIDE = 10,
	parameter integer CLKOUT2_DIVIDE = 4,
	parameter integer CLKOUT3_DIVIDE = 20,
	parameter integer CLKOUT4_DIVIDE = 1,
	parameter integer CLKOUT5_DIVIDE = 1,
	parameter integer CLKOUT6_DIVIDE = 1,
	parameter real CLKOUT0_PHASE = 0.0, parameter real CLKOUT1_PHASE = 0.0,
	parameter real CLKOUT2_PHASE = 0.0, parameter real CLKOUT3_PHASE = 0.0,
	parameter real CLKFBOUT_PHASE = 0.0,
	parameter real CLKOUT0_DUTY_CYCLE = 0.5, parameter real CLKOUT1_DUTY_CYCLE = 0.5,
	parameter real CLKOUT2_DUTY_CYCLE = 0.5, parameter real CLKOUT3_DUTY_CYCLE = 0.5,
	parameter STARTUP_WAIT = "FALSE"
)(
	input  CLKIN1, CLKFBIN, PWRDWN, RST,
	output CLKFBOUT, CLKFBOUTB,
	output reg CLKOUT0 = 0, output CLKOUT0B,
	output reg CLKOUT1 = 1, output CLKOUT1B,  // starts high: rises with CLKOUT0
	output reg CLKOUT2 = 0, output CLKOUT2B,
	output reg CLKOUT3 = 0, output CLKOUT3B,
	output CLKOUT4, CLKOUT5, CLKOUT6,
	output reg LOCKED = 0
);
	assign CLKFBOUT = 1'b0; assign CLKFBOUTB = 1'b1;
	assign CLKOUT0B = ~CLKOUT0; assign CLKOUT1B = ~CLKOUT1;
	assign CLKOUT2B = ~CLKOUT2; assign CLKOUT3B = ~CLKOUT3;
	assign CLKOUT4 = 1'b0; assign CLKOUT5 = 1'b0; assign CLKOUT6 = 1'b0;
	// 32.5 MHz / 65 MHz with rising edges aligned (CLKOUT1 starts high, so
	// both rise at 15380 ps). 30760 ps is the 650/20 period rounded to 1 ps.
	// CLKOUT2/CLKOUT3 (SPI, LCD) only feed the framework, which is not in
	// the simulation, so they are left still to save events.
	initial begin
		#100000 LOCKED = 1;
	end
	always #15380 CLKOUT0 = ~CLKOUT0;
	always #7690  CLKOUT1 = ~CLKOUT1;
endmodule

module BUFG (input I, output O);
	assign O = I;
endmodule

module ODDR #(
	parameter DDR_CLK_EDGE = "SAME_EDGE",
	parameter INIT = 1'b0,
	parameter SRTYPE = "SYNC"
)(
	input C, CE, D1, D2, R, S,
	output Q
);
	// Clock-forwarding use only (D1/D2 constant): Q = C or ~C.
	assign Q = C ? D1 : D2;
endmodule
