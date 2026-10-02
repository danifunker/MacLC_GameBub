// ============================================================================
// gb_debug_uart.sv — status beacon on a 3.3 V UART pin (PMOD header).
//
// Every PERIOD_MS it snapshots NWORDS 32-bit words and prints them as hex:
//     MLC 0000484B 35F00EAC 00000123 ...\r\n
// at 115200 8N1. Plug any 3.3 V USB-serial adapter's RX into the pin and
// watch with a terminal; no JTAG cable or Vivado needed. What each word
// means is listed where maclc_gamebub.sv wires it up.
// ============================================================================
`default_nettype none

module gb_debug_uart #(
	parameter integer CLK_HZ    = 32_500_000,
	parameter integer BAUD      = 115_200,
	parameter integer NWORDS    = 8,
	parameter integer PERIOD_MS = 500
)(
	input  wire                 clk,
	input  wire                 reset,
	input  wire [NWORDS*32-1:0] words,
	output reg                  txd
);

	localparam integer BIT_DIV   = CLK_HZ / BAUD;
	localparam integer PERIOD    = (CLK_HZ / 1000) * PERIOD_MS;
	localparam integer NCHARS    = 3 + NWORDS * 9 + 2;

	reg [NWORDS*32-1:0] snap;
	reg [31:0]          period_cnt = 32'd0;
	reg                 sending    = 1'b0;
	reg [8:0]           char_idx;          // which character of the line
	reg [3:0]           bit_idx;           // 0 start, 1..8 data, 9 stop
	reg [15:0]          bit_cnt;
	reg [7:0]           cur;

	function [7:0] hexchar(input [3:0] n);
		hexchar = (n < 4'd10) ? (8'h30 + n) : (8'h41 + n - 4'd10);
	endfunction

	// Character `i` of the line.
	function [7:0] line_char(input [8:0] i, input [NWORDS*32-1:0] w);
		integer word, pos;
		reg [31:0] v;
		begin
			if (i == 0)      line_char = "M";
			else if (i == 1) line_char = "L";
			else if (i == 2) line_char = "C";
			else if (i >= NCHARS - 2) line_char = (i == NCHARS - 2) ? 8'h0D : 8'h0A;
			else begin
				word = (i - 3) / 9;
				pos  = (i - 3) % 9;
				v    = w[word*32 +: 32];
				if (pos == 0) line_char = " ";
				else          line_char = hexchar(v[31 - (pos - 1) * 4 -: 4]);
			end
		end
	endfunction

	always @(posedge clk) begin
		if (reset) begin
			txd        <= 1'b1;
			sending    <= 1'b0;
			period_cnt <= 32'd0;
		end else begin
			if (!sending) begin
				txd <= 1'b1;
				if (period_cnt >= PERIOD - 1) begin
					period_cnt <= 32'd0;
					snap       <= words;
					sending    <= 1'b1;
					char_idx   <= 9'd0;
					bit_idx    <= 4'd0;
					bit_cnt    <= 16'd0;
					cur        <= "M";
				end else begin
					period_cnt <= period_cnt + 32'd1;
				end
			end else begin
				period_cnt <= period_cnt + 32'd1;
				// shift out the current character
				case (bit_idx)
					4'd0:    txd <= 1'b0;                 // start
					4'd9:    txd <= 1'b1;                 // stop
					default: txd <= cur[bit_idx - 4'd1];  // data, LSB first
				endcase
				if (bit_cnt == BIT_DIV - 1) begin
					bit_cnt <= 16'd0;
					if (bit_idx == 4'd9) begin
						bit_idx <= 4'd0;
						if (char_idx == NCHARS - 1) begin
							sending <= 1'b0;
						end else begin
							char_idx <= char_idx + 9'd1;
							cur      <= line_char(char_idx + 9'd1, snap);
						end
					end else begin
						bit_idx <= bit_idx + 4'd1;
					end
				end else begin
					bit_cnt <= bit_cnt + 16'd1;
				end
			end
		end
	end

endmodule

`default_nettype wire
