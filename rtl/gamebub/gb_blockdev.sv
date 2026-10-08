// ============================================================================
// gb_blockdev.sv — MiSTer hps_io block devices, served from SDRAM.
//
// On MiSTer the ARM side answers sd_rd/sd_wr by moving 512-byte sectors
// between the SD card and the core. Game Bub's host cannot do random access
// (framework v1.1 has no core->host commands): it copies whole files into
// core memory at setup and back at exit. So the disk images live in SDRAM
// (placed by files.json, see gb_host.sv) and this module answers the same
// requests from there. Everything upstream — scsi.v, floppy_sd.v, the PRAM
// FSM — is unchanged.
//
// SLOTS (maclc_core numbering)      FILE          SDRAM word base
//   0  SCSI ID 0 hard disk          0 Hard disk   $6C0000  (<= 18.5 MiB, to the top)
//   2  PRAM save image              1 PRAM        $540000  (always 512 B)
//   6  internal floppy              2 Floppy      $548000  (<= 1.44 MiB, in the gap
//                                                           below the floppy
//                                                           controller's copy)
//   1, 3, 4, 5 (SCSI 1, Toolbox, CD, CD changer) are never mounted.
// The floppy controller keeps its own copy of the disk at word $600000
// (floppy_sd's loader, header stripped): <= 1,474,560 bytes, so it ends below
// the hard disk at $6C0000. Keep these, gb_host.sv and files.json in step.
//
// TRANSFER SHAPE, copied from sys/hps_io.sv (WIDE=1):
//   read  : sd_ack[n]=1, sd_buff_addr=0; per word: dout, then a one-cycle
//           sd_buff_wr, then sd_buff_addr+1; finally sd_ack[n]=0
//   write : sd_ack[n]=1, sd_buff_addr=0; per word: wait, sample
//           sd_buff_din[n], sd_buff_addr+1; finally sd_ack[n]=0
//   mount : one-cycle img_mounted[n] with img_size/img_readonly valid
// Data is {odd byte, even byte} like hps_io; see gb_host.sv for how the
// verbatim host words are unpacked (host_le).
// ============================================================================
`default_nettype none

module gb_blockdev #(
	parameter integer VDNUM = 7
)(
	input  wire        clk,
	input  wire        reset,

	// ---- file table (gb_host) ----
	input  wire  [3:0] file_present,
	input  wire [127:0] file_size_flat,
	input  wire        host_le,
	input  wire        mount_req,

	// ---- hps_io-shaped block devices (maclc_core) ----
	input  wire [VDNUM*32-1:0] sd_lba_flat,
	input  wire [VDNUM-1:0]    sd_rd,
	input  wire [VDNUM-1:0]    sd_wr,
	output reg  [VDNUM-1:0]    sd_ack,
	output reg  [12:0]         sd_buff_addr,
	output reg  [15:0]         sd_buff_dout,
	input  wire [VDNUM*16-1:0] sd_buff_din_flat,
	output reg                 sd_buff_wr,
	output reg  [VDNUM-1:0]    img_mounted,
	output reg  [63:0]         img_size,
	output reg                 img_readonly,

	// ---- SDRAM client (eth_port_arb protocol) ----
	output reg         mem_req,
	output reg         mem_we,
	output reg  [23:0] mem_addr,
	output reg  [15:0] mem_din,
	input  wire        mem_ack,
	input  wire [15:0] mem_dout,

	output wire [31:0] dbg
);

	localparam [2:0] SLOT_HD = 3'd0, SLOT_PRAM = 3'd2, SLOT_FLOPPY = 3'd6;
	localparam [1:0] FILE_HD = 2'd0, FILE_PRAM = 2'd1, FILE_FLOPPY = 2'd2;

	wire [31:0] file_size [0:3];
	assign file_size[0] = file_size_flat[31:0];
	assign file_size[1] = file_size_flat[63:32];
	assign file_size[2] = file_size_flat[95:64];
	assign file_size[3] = file_size_flat[127:96];

	// Per-slot image description
	function [23:0] slot_base(input [2:0] s);
		case (s)
			SLOT_HD:     slot_base = 24'h6C0000;
			SLOT_PRAM:   slot_base = 24'h540000;
			SLOT_FLOPPY: slot_base = 24'h548000;
			default:     slot_base = 24'h000000;
		endcase
	endfunction

	function [31:0] slot_bytes(input [2:0] s);
		case (s)
			SLOT_HD:     slot_bytes = file_present[FILE_HD]     ? file_size[FILE_HD]     : 32'd0;
			SLOT_PRAM:   slot_bytes = file_present[FILE_PRAM]   ? 32'd512                : 32'd0;
			SLOT_FLOPPY: slot_bytes = file_present[FILE_FLOPPY] ? file_size[FILE_FLOPPY] : 32'd0;
			default:     slot_bytes = 32'd0;
		endcase
	endfunction

	function [15:0] bswap(input [15:0] w);
		bswap = {w[7:0], w[15:8]};
	endfunction

	// ---------------------------------------------------------------------
	// Mount sequencer: present HD, PRAM, floppy, one at a time. PRAM is
	// presented even when absent (size 0) so the PRAM FSM releases the boot
	// at once instead of waiting out its ~6 s backstop.
	// ---------------------------------------------------------------------
	reg        mount_pending = 1'b0;
	reg  [1:0] mount_idx;
	reg  [4:0] mount_gap;

	function [2:0] mount_slot(input [1:0] k);
		case (k)
			2'd0:    mount_slot = SLOT_HD;
			2'd1:    mount_slot = SLOT_PRAM;
			default: mount_slot = SLOT_FLOPPY;
		endcase
	endfunction

	// ---------------------------------------------------------------------
	// Transfer engine
	// ---------------------------------------------------------------------
	localparam [3:0] S_IDLE = 4'd0, S_MOUNT = 4'd1, S_START = 4'd2,
	                 S_RD_MEM = 4'd3, S_RD_PUT = 4'd4, S_RD_WR = 4'd5, S_RD_INC = 4'd6,
	                 S_WR_WAIT = 4'd7, S_WR_MEM = 4'd8, S_WR_INC = 4'd9,
	                 S_MEM_DROP = 4'd10, S_MEM_GAP = 4'd11, S_END = 4'd12;
	reg  [3:0]  st = S_IDLE;
	reg  [3:0]  st_after_mem;         // where to go once the SDRAM access retires
	reg  [2:0]  slot;
	reg         is_write;
	reg  [31:0] lba;
	reg         in_range;
	reg  [8:0]  word;                 // 0..256
	reg  [2:0]  wait_cnt;
	reg  [15:0] rd_word;
	reg  [2:0]  scan;                 // round-robin start
	reg  [15:0] cnt_rd, cnt_wr;

	wire [31:0] sd_lba    [0:VDNUM-1];
	wire [15:0] sd_buff_din [0:VDNUM-1];
	genvar gi;
	generate
		for (gi = 0; gi < VDNUM; gi = gi + 1) begin : g_unflat
			assign sd_lba[gi]      = sd_lba_flat[gi*32 +: 32];
			assign sd_buff_din[gi] = sd_buff_din_flat[gi*16 +: 16];
		end
	endgenerate

	// Pick the first requesting slot at or after `scan`.
	reg        pick_valid;
	reg  [2:0] pick;
	integer k;
	always @(*) begin
		pick_valid = 1'b0;
		pick       = 3'd0;
		for (k = VDNUM - 1; k >= 0; k = k - 1) begin
			if ((sd_rd[(scan + k) % VDNUM] || sd_wr[(scan + k) % VDNUM])) begin
				pick_valid = 1'b1;
				pick       = (scan + k) % VDNUM;
			end
		end
	end

	wire [31:0] cur_bytes  = slot_bytes(slot);
	wire [31:0] cur_blocks = (cur_bytes + 32'd511) >> 9;
	wire [23:0] cur_addr   = slot_base(slot) + {lba[15:0], 8'd0} + {15'd0, word[7:0] ^ {7'd0, host_le}};

	always @(posedge clk) begin
		if (reset) begin
			st            <= S_IDLE;
			sd_ack        <= {VDNUM{1'b0}};
			sd_buff_addr  <= 13'd0;
			sd_buff_wr    <= 1'b0;
			img_mounted   <= {VDNUM{1'b0}};
			img_size      <= 64'd0;
			img_readonly  <= 1'b0;
			mem_req       <= 1'b0;
			mount_pending <= 1'b0;
			scan          <= 3'd0;
			cnt_rd        <= 16'd0;
			cnt_wr        <= 16'd0;
		end else begin
			img_mounted <= {VDNUM{1'b0}};
			sd_buff_wr  <= 1'b0;
			if (mount_req) mount_pending <= 1'b1;

			case (st)
			S_IDLE: begin
				if (mount_pending) begin
					mount_pending <= 1'b0;
					mount_idx     <= 2'd0;
					mount_gap     <= 5'd0;
					st            <= S_MOUNT;
				end else if (pick_valid) begin
					slot     <= pick;
					is_write <= !sd_rd[pick];
					lba      <= sd_lba[pick];
					scan     <= (pick == VDNUM - 1) ? 3'd0 : pick + 3'd1;
					st       <= S_START;
				end
			end

			S_MOUNT: begin
				mount_gap <= mount_gap + 5'd1;
				if (mount_gap == 5'd0) begin
					if (slot_bytes(mount_slot(mount_idx)) != 0 || mount_slot(mount_idx) == SLOT_PRAM) begin
						img_mounted[mount_slot(mount_idx)] <= 1'b1;
						img_size     <= {32'd0, slot_bytes(mount_slot(mount_idx))};
						img_readonly <= 1'b0;
					end
				end else if (mount_gap == 5'd31) begin
					if (mount_idx == 2'd2) st <= S_IDLE;
					else mount_idx <= mount_idx + 2'd1;
				end
			end

			S_START: begin
				in_range         <= (lba < cur_blocks);
				sd_ack[slot]     <= 1'b1;
				sd_buff_addr     <= 13'd0;
				word             <= 9'd0;
				wait_cnt         <= 3'd0;
				if (is_write) cnt_wr <= cnt_wr + 16'd1;
				else          cnt_rd <= cnt_rd + 16'd1;
				st               <= is_write ? S_WR_WAIT : S_RD_MEM;
			end

			// ---- read: SDRAM -> sd_buff ----
			S_RD_MEM: begin
				if (!in_range) begin
					rd_word <= 16'd0;
					st      <= S_RD_PUT;
				end else if (!mem_req) begin
					mem_req  <= 1'b1;
					mem_we   <= 1'b0;
					mem_addr <= cur_addr;
				end else if (mem_ack) begin
					mem_req      <= 1'b0;
					rd_word      <= host_le ? mem_dout : bswap(mem_dout);
					st_after_mem <= S_RD_PUT;
					st           <= S_MEM_DROP;
				end
			end
			S_RD_PUT: begin
				sd_buff_dout <= rd_word;
				st           <= S_RD_WR;
			end
			S_RD_WR: begin
				sd_buff_wr <= 1'b1;
				st         <= S_RD_INC;
			end
			S_RD_INC: begin
				sd_buff_addr <= sd_buff_addr + 13'd1;
				word         <= word + 9'd1;
				st           <= (word == 9'd255) ? S_END : S_RD_MEM;
			end

			// ---- write: sd_buff -> SDRAM ----
			S_WR_WAIT: begin
				// give the consumer's (possibly registered) sd_buff_din time
				// to follow sd_buff_addr, as the slow HPS link always did
				wait_cnt <= wait_cnt + 3'd1;
				if (wait_cnt == 3'd4) st <= S_WR_MEM;
			end
			S_WR_MEM: begin
				if (!in_range) begin
					st <= S_WR_INC;
				end else if (!mem_req) begin
					mem_req  <= 1'b1;
					mem_we   <= 1'b1;
					mem_addr <= cur_addr;
					mem_din  <= host_le ? sd_buff_din[slot] : bswap(sd_buff_din[slot]);
				end else if (mem_ack) begin
					mem_req      <= 1'b0;
					st_after_mem <= S_WR_INC;
					st           <= S_MEM_DROP;
				end
			end
			S_WR_INC: begin
				sd_buff_addr <= sd_buff_addr + 13'd1;
				word         <= word + 9'd1;
				wait_cnt     <= 3'd0;
				st           <= (word == 9'd255) ? S_END : S_WR_WAIT;
			end

			// ---- SDRAM handshake tail (eth_port_arb wants ack low + 1) ----
			S_MEM_DROP: if (!mem_ack) st <= S_MEM_GAP;
			S_MEM_GAP:  st <= st_after_mem;

			S_END: begin
				sd_ack <= {VDNUM{1'b0}};
				// a few idle cycles so the consumer sees ack fall before we
				// look at its request lines again
				wait_cnt <= wait_cnt + 3'd1;
				if (wait_cnt == 3'd7) st <= S_IDLE;
			end

			default: st <= S_IDLE;
			endcase
		end
	end

	assign dbg = {cnt_wr, cnt_rd};

endmodule

`default_nettype wire
