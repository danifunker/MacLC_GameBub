// ============================================================================
// gb_vram_sram.sv — Mac LC VRAM in Game Bub's external SRAM.
//
// The board's IS61WV25616 (256K x 16, 10 ns) is exactly the LC's 512 KB VRAM
// option. On MiSTer VRAM was on-chip RAM (rtl/vram_bram.sv); here the chip's
// block RAM is taken by the framework's double-buffered frame, so the V8's
// scanline prefetch and the CPU's VRAM write mirror share this SRAM instead.
//
// TIMING (clk_sys = 32.5 MHz, 30.8 ns). Every pin is driven from a register
// (packed into the I/O blocks by rtl/gamebub/maclc.xdc) and the read data is
// captured by an input register:
//   read   address out on edge N; data valid ~5 ns (out) + 10 ns (tAA)
//          + ~2 ns (in) later; captured on edge N+1. One read per cycle,
//          rvalid two cycles after the request was accepted.
//   write  3 cycles: [addr, OE high] [data driven, WE low] [WE high, data
//          held]. Address and data are stable around the whole WE pulse,
//          so the SRAM's zero-ns setup/hold specs never come into play.
//   bus turnaround: one idle cycle (OE high, not driving) before a read that
//          follows a write, so the FPGA and SRAM never drive together.
//
// ARBITRATION. Video reads win while the write queue is less than half full;
// writes win otherwise. A CPU VRAM write needs at least ~6 clk_sys, so the
// 3-cycle writes leave the video at least half the cycles even during a
// sustained fill — enough for the worst case (16 bpp, 512 reads per 1328-cycle
// line = 39%). Queue overflow cannot happen at that rate; it is still
// counted for the debug beacon.
// ============================================================================
`default_nettype none

module gb_vram_sram (
	input  wire        clk,
	input  wire        reset,

	// ---- V8 scanline prefetch ----
	input  wire [17:0] rd_addr,
	input  wire        rd,
	output wire        rd_ready,
	output reg         rd_valid,
	output reg  [15:0] rd_data,

	// ---- CPU VRAM write mirror (1-cycle strobe) ----
	input  wire [17:0] wr_addr,
	input  wire [15:0] wr_data,
	input  wire  [1:0] wr_be,          // {upper, lower} byte enables
	input  wire        wr,

	// ---- SRAM pins (framework SramV0) ----
	output reg         sram_ce_n,
	output reg         sram_we_n,
	output reg         sram_oe_n,
	output reg   [1:0] sram_mask_n,    // {UB#, LB#}
	output reg  [17:0] sram_a,
	input  wire [15:0] sram_dq_in,
	output reg  [15:0] sram_dq_out,
	output reg         sram_dq_oe,

	output wire  [7:0] dbg_overflows
);

	// ---------------------------------------------------------------------
	// Write queue (16 deep, distributed RAM)
	// ---------------------------------------------------------------------
	reg  [35:0] q_mem [0:15];          // {be, data, addr}
	reg  [4:0]  q_wp = 5'd0, q_rp = 5'd0;
	wire [4:0]  q_count = q_wp - q_rp;
	wire        q_empty = (q_count == 5'd0);
	wire        q_full  = (q_count == 5'd16);
	wire        q_half  = q_count[4] | q_count[3];
	wire [35:0] q_head  = q_mem[q_rp[3:0]];
	reg  [7:0]  overflows = 8'd0;
	assign dbg_overflows = overflows;

	// ---------------------------------------------------------------------
	// Pin sequencer
	// ---------------------------------------------------------------------
	localparam [1:0] P_IDLE = 2'd0, P_W1 = 2'd1, P_W2 = 2'd2, P_W3 = 2'd3;
	reg  [1:0] ph = P_IDLE;
	reg        after_write = 1'b0;      // last cycle ended a write (turnaround)
	reg        rd_issued   = 1'b0;      // a read address is on the pins now

	wire       want_write = !q_empty && (q_half || !rd);
	// A read can go out when the pins are free, no write is due, and the
	// bus has turned around.
	assign rd_ready = (ph == P_IDLE) && !want_write && !after_write && !reset;
	wire       rd_go    = rd && rd_ready;
	wire       wr_go    = (ph == P_IDLE) && want_write;

	always @(posedge clk) begin
		// queue push (independent of the sequencer)
		if (reset) begin
			q_wp <= 5'd0;
		end else if (wr) begin
			if (q_full) begin
				if (overflows != 8'hFF) overflows <= overflows + 8'd1;
			end else begin
				q_mem[q_wp[3:0]] <= {wr_be, wr_data, wr_addr};
				q_wp <= q_wp + 5'd1;
			end
		end
	end

	always @(posedge clk) begin
		// read data: the address issued last cycle is answered now
		rd_data  <= sram_dq_in;
		rd_valid <= rd_issued;

		if (reset) begin
			ph          <= P_IDLE;
			q_rp        <= 5'd0;
			after_write <= 1'b0;
			rd_issued   <= 1'b0;
			rd_valid    <= 1'b0;
			sram_ce_n   <= 1'b1;
			sram_we_n   <= 1'b1;
			sram_oe_n   <= 1'b1;
			sram_mask_n <= 2'b11;
			sram_dq_oe  <= 1'b0;
		end else begin
			sram_ce_n   <= 1'b0;
			rd_issued   <= 1'b0;
			after_write <= 1'b0;
			case (ph)
			P_IDLE: begin
				sram_we_n  <= 1'b1;
				sram_dq_oe <= 1'b0;
				if (wr_go) begin
					// W1: address + byte lanes out, outputs disabled
					sram_a      <= q_head[17:0];
					sram_dq_out <= q_head[33:18];
					sram_mask_n <= ~q_head[35:34];
					sram_oe_n   <= 1'b1;
					ph          <= P_W1;
				end else if (rd_go) begin
					sram_a      <= rd_addr;
					sram_mask_n <= 2'b00;
					sram_oe_n   <= 1'b0;
					rd_issued   <= 1'b1;
				end else begin
					sram_oe_n   <= 1'b1;
				end
			end
			P_W1: begin
				// W2: drive data, WE low
				sram_dq_oe <= 1'b1;
				sram_we_n  <= 1'b0;
				ph         <= P_W2;
			end
			P_W2: begin
				// W3: WE high, data and address still held
				sram_we_n  <= 1'b1;
				q_rp       <= q_rp + 5'd1;
				ph         <= P_W3;
			end
			default: begin
				// release the bus; a following read waits one cycle
				sram_dq_oe  <= 1'b0;
				after_write <= 1'b1;
				ph          <= P_IDLE;
			end
			endcase
		end
	end

endmodule

`default_nettype wire
