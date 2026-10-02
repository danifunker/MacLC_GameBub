// ============================================================================
// gb_video_out.sv — V8 scanout -> framework VideoV0 (512x384, 3/3/3).
//
// The framework writes one pixel into its frame buffer for every system
// clock that video_dataEnable is high, advances a line on each hblank pulse
// and closes the frame on vblank. The V8 runs on clk_sys with a fractional
// pixel enable, so its RGB/DE hold for 2-3 clocks per pixel; maclc_core's
// vid_pix_stb marks exactly one clock inside each pixel.
//
// COLOUR. The framework stores the frame on chip, and only 9 bits per
// pixel fit (see docs/PORT_PLAN.md). An ordered 4x4 Bayer dither spreads the
// quantisation error so 8-bit palette greys and 16-bit photos degrade into
// fine texture instead of banding: level = number of thresholds k*255 that
// v*7 + bayer*16 reaches, k = 1..7.
// Values within 16 of either end snap to black/white instead: the LC ROM's
// "black" is (5,5,5), which a faithful dither turns into a speckle of dark
// grey dots in text and window frames (seen in simulation, 2026-10-01).
// ============================================================================
`default_nettype none

module gb_video_out #(
	parameter bit DITHER = 1'b1
)(
	input  wire       clk,
	input  wire       reset,           // video reset: hold the framework idle

	input  wire [7:0] in_r,
	input  wire [7:0] in_g,
	input  wire [7:0] in_b,
	input  wire       in_de,
	input  wire       in_vblank,
	input  wire       in_pix_stb,

	output reg  [2:0] out_r,
	output reg  [2:0] out_g,
	output reg  [2:0] out_b,
	output reg        out_de,          // framework video_dataEnable
	output reg        out_hblank,      // one-cycle pulse after each line
	output reg        out_vblank       // level, high between frames
);

	// Position within the frame, for the dither matrix
	reg [1:0] x4 = 2'd0, y4 = 2'd0;
	reg       de_seen = 1'b0;          // the current line had pixels
	// Stream nothing until the first vblank after reset: the V8's output
	// registers are not reset and keep showing pixel 0 meanwhile, which
	// added a 513th pixel to the first row (seen in simulation).
	reg       synced  = 1'b0;

	function [3:0] bayer(input [1:0] x, input [1:0] y);
		case ({y, x})
			4'b00_00: bayer = 4'd0;  4'b00_01: bayer = 4'd8;  4'b00_10: bayer = 4'd2;  4'b00_11: bayer = 4'd10;
			4'b01_00: bayer = 4'd12; 4'b01_01: bayer = 4'd4;  4'b01_10: bayer = 4'd14; 4'b01_11: bayer = 4'd6;
			4'b10_00: bayer = 4'd3;  4'b10_01: bayer = 4'd11; 4'b10_10: bayer = 4'd1;  4'b10_11: bayer = 4'd9;
			default:  bayer = (x == 2'd0) ? 4'd15 : (x == 2'd1) ? 4'd7 : (x == 2'd2) ? 4'd13 : 4'd5;
		endcase
	endfunction

	function [2:0] quant(input [7:0] v, input [3:0] d);
		reg [11:0] s;
		begin
			s = v * 12'd7 + (DITHER ? {4'd0, d, 4'd0} : 12'd127);
			quant = (v < 8'd16)     ? 3'd0 :
			        (v > 8'd239)    ? 3'd7 :
			        (s >= 12'd1785) ? 3'd7 :
			        (s >= 12'd1530) ? 3'd6 :
			        (s >= 12'd1275) ? 3'd5 :
			        (s >= 12'd1020) ? 3'd4 :
			        (s >= 12'd765)  ? 3'd3 :
			        (s >= 12'd510)  ? 3'd2 :
			        (s >= 12'd255)  ? 3'd1 : 3'd0;
		end
	endfunction

	wire [3:0] d = bayer(x4, y4);

	always @(posedge clk) begin
		out_de     <= 1'b0;
		out_hblank <= 1'b0;
		if (reset) begin
			out_vblank <= 1'b1;
			x4         <= 2'd0;
			y4         <= 2'd0;
			de_seen    <= 1'b0;
			synced     <= 1'b0;
		end else if (!synced) begin
			out_vblank <= 1'b1;
			synced     <= in_vblank;
		end else begin
			out_vblank <= in_vblank;
			if (in_vblank) begin
				y4      <= 2'd0;
				x4      <= 2'd0;
				de_seen <= 1'b0;
			end else if (in_pix_stb) begin
				if (in_de) begin
					out_r   <= quant(in_r, d);
					out_g   <= quant(in_g, d);
					out_b   <= quant(in_b, d);
					out_de  <= 1'b1;
					x4      <= x4 + 2'd1;
					de_seen <= 1'b1;
				end else if (de_seen) begin
					// first blank pixel after a line of pixels: end of line
					out_hblank <= 1'b1;
					de_seen    <= 1'b0;
					x4         <= 2'd0;
					y4         <= y4 + 2'd1;
				end
			end
		end
	end

endmodule

`default_nettype wire
