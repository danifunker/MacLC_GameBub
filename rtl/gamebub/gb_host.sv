// ============================================================================
// gb_host.sv — the Game Bub host protocol (HostV0) for the Mac LC.
//
// Owns everything the MCU talks to:
//   * the command channel (GetStatus, SetupComplete, CoreRun/Halt,
//     NotifyFocus, FileWrite/ReadStart/End) and its parameter/result
//     registers at 0xF000_0000;
//   * the settings registers written from settings.json (0x0000_0xxx);
//   * a window onto the SDRAM through which the MCU loads every file from
//     files.json and reads the writable ones back at core exit.
//
// HOST MEMORY MAP (byte addresses, 32-bit accesses)
//   0x0000_0100..0x0000_02FF  settings (see SETTINGS below)
//   0x0000_0800..0x0000_08FF  read-only status/debug words
//   0x1000_0000 + n           SDRAM byte n (32 MiB). files.json addresses:
//       0x10A0_0000  ROM       512 KiB   (Mac SDRAM word $500000, fixed by
//                                         addrController_top)
//       0x10A8_0000  PRAM      512 B     (word $540000, a gap in the map)
//       0x10E0_0000  Floppy    <= 2 MiB  (word $700000, the removed second
//                                         floppy's region)
//       0x1100_0000  Hard disk <= 16 MiB (word $800000, the upper half)
//   0xF000_0000..0xF000_001C  host->core command opcode/params/results
//   0xF000_1000..             core->host command registers (unused: v1.1
//                             defines no core->host commands)
//
// FILE BYTE ORDER. The MCU packs four file bytes into each 32-bit word, in
// an order the documentation does not state. The window therefore stores
// words verbatim (word[31:16] at the lower SDRAM word, word[15:0] at the
// next), so every file round-trips exactly whatever the order is, and the
// order is DETECTED from the ROM: every LC-family ROM checksum starts with
// byte 0x35. Only two consumers care about the order:
//   * the CPU reads the ROM straight from SDRAM as big-endian words, so a
//     little-endian host's ROM is rewritten in place once after setup
//     (the "ROM fixup", ~130 ms);
//   * gb_blockdev unpacks disk images in the detected order (host_le).
// ============================================================================
`default_nettype none

module gb_host #(
	// Used if the ROM's first word gives no verdict (not an LC-family ROM).
	parameter bit HOST_LE_DEFAULT = 1'b1
)(
	input  wire        clk,
	input  wire        reset,

	// ---- framework host memory interface (HostV0.mem) ----
	input  wire        mem_enable,
	input  wire        mem_write,
	input  wire [31:0] mem_address,
	input  wire [31:0] mem_dataWrite,
	output reg  [31:0] mem_dataRead,
	output reg         mem_done,

	// ---- framework host->core command channel ----
	input  wire        cmd_request,
	output wire        cmd_busy,
	output wire        cmd_done,
	output wire        cmd_error,

	// ---- SDRAM client (eth_port_arb protocol) ----
	output wire        sd_req,
	output wire        sd_we,
	output wire [23:0] sd_addr,
	output wire [15:0] sd_din,
	input  wire        sd_ack,
	input  wire [15:0] sd_dout,

	// ---- machine control ----
	output wire        core_reset,       // hold the Mac in reset
	output reg         rom_loading,      // the ROM file is being written
	output reg         rom_loaded,
	output wire [31:0] status,           // MiSTer status[] bit layout
	output reg         mount_req,        // pulse: present the disk images
	output reg         host_le,          // valid once setup is complete
	output reg         focus,

	// ---- settings for the input layer ----
	output wire        ptr_default,      // start in pointer (mouse) mode
	output wire  [1:0] ptr_speed,

	// ---- file table (for gb_blockdev) ----
	output reg   [3:0] file_present,
	output wire [127:0] file_size_flat,  // {size[3], size[2], size[1], size[0]}

	// ---- debug ----
	input  wire [31:0] dbg_blockdev,
	output wire [31:0] dbg_state,
	output wire [31:0] dbg_rom_word
);

	// ---------------------------------------------------------------------
	// File IDs (files.json) and their SDRAM placement
	// ---------------------------------------------------------------------
	localparam [1:0] FILE_HD     = 2'd0;
	localparam [1:0] FILE_PRAM   = 2'd1;
	localparam [1:0] FILE_FLOPPY = 2'd2;
	localparam [1:0] FILE_ROM    = 2'd3;

	localparam [31:0] ROM_HOST_ADDR = 32'h10A0_0000;
	localparam [23:0] ROM_WORD_BASE = 24'h500000;
	localparam [18:0] ROM_WORDS     = 19'h40000;     // 512 KiB

	// ---------------------------------------------------------------------
	// HostV0 command opcodes and status codes
	// ---------------------------------------------------------------------
	localparam [15:0] CMD_GET_STATUS     = 16'h0000;
	localparam [15:0] CMD_CORE_RUN       = 16'h0100;
	localparam [15:0] CMD_CORE_HALT      = 16'h0101;
	localparam [15:0] CMD_SETUP_COMPLETE = 16'h0102;
	localparam [15:0] CMD_NOTIFY_FOCUS   = 16'h0200;
	localparam [15:0] CMD_FILE_WR_START  = 16'h0300;
	localparam [15:0] CMD_FILE_WR_END    = 16'h0301;
	localparam [15:0] CMD_FILE_RD_START  = 16'h0302;
	localparam [15:0] CMD_FILE_RD_END    = 16'h0303;

	localparam [31:0] STATUS_SETUP     = 32'd2;
	localparam [31:0] STATUS_CORE_HALT = 32'd3;
	localparam [31:0] STATUS_CORE_RUN  = 32'd4;

	// ---------------------------------------------------------------------
	// Settings registers (settings.json writes these)
	// ---------------------------------------------------------------------
	localparam [11:0] SET_ACT_RESET     = 12'h100;  // action: reset the Mac
	localparam [11:0] SET_ACT_NMI       = 12'h104;  // action: programmer's interrupt
	localparam [11:0] SET_ACT_WIPE_PRAM = 12'h108;  // action: zero PRAM + reset
	localparam [11:0] SET_MEMORY        = 12'h200;  // 0 = 2 MB, 1 = 10 MB
	localparam [11:0] SET_FLOPPY_WRITE  = 12'h204;  // 1 = guest may write floppies
	localparam [11:0] SET_PTR_DEFAULT   = 12'h208;  // 1 = start in mouse mode
	localparam [11:0] SET_PTR_SPEED     = 12'h20C;  // 0 slow, 1 normal, 2 fast
	localparam [11:0] SET_ICACHE_OFF    = 12'h210;  // hidden: keep 0

	reg  [31:0] set_memory       = 32'd1;
	reg  [31:0] set_floppy_write = 32'd0;
	reg  [31:0] set_ptr_default  = 32'd1;
	reg  [31:0] set_ptr_speed    = 32'd1;
	reg  [31:0] set_icache_off   = 32'd0;

	// Actions become pulses long enough for the clk8_en_p-sampled reset
	// logic and the edge detectors in maclc_core to see.
	reg  [7:0]  act_reset_cnt = 8'd0, act_nmi_cnt = 8'd0, act_wipe_cnt = 8'd0;

	assign status = {
		12'd0,
		1'b1,                     // [19] Ethernet card: off (no network)
		1'b1,                     // [18] CD-ROM drive: removed from the bus
		3'd0,                     // [17:15]
		set_floppy_write[0],      // [14] floppy write enable
		2'd0,                     // [13:12]
		set_icache_off[0],        // [11] fetch cache disable (non-constant 0)
		4'd0,                     // [10:7]
		act_wipe_cnt != 0,        // [6]  wipe PRAM
		act_nmi_cnt != 0,         // [5]  NMI
		set_memory[0],            // [4]  10 MB
		3'd0,                     // [3:1]
		act_reset_cnt != 0        // [0]  reset
	};
	assign ptr_default = set_ptr_default[0];
	assign ptr_speed   = set_ptr_speed[1:0];

	// ---------------------------------------------------------------------
	// Core state
	// ---------------------------------------------------------------------
	reg         setup_done = 1'b0;   // SetupComplete received
	reg         run        = 1'b0;   // CoreRun (vs CoreHalt)
	reg         fixup_done = 1'b0;   // ROM byte order settled
	reg  [31:0] file_size [0:3];
	reg  [31:0] rom_first_word = 32'd0;
	reg         rom_first_seen = 1'b0;

	assign file_size_flat = {file_size[3], file_size[2], file_size[1], file_size[0]};
	assign core_reset     = !(run && fixup_done && rom_loaded);

	// ---------------------------------------------------------------------
	// Command channel
	// ---------------------------------------------------------------------
	reg  [31:0] cmd_reg [0:7];       // 0xF000_0000.. opcode/params, then results
	localparam [1:0] CS_IDLE = 2'd0, CS_DONE = 2'd2, CS_ERROR = 2'd3;
	reg  [1:0]  cmd_state = CS_IDLE;
	assign cmd_busy  = 1'b0;
	assign cmd_done  = (cmd_state == CS_DONE);
	assign cmd_error = (cmd_state == CS_ERROR);

	wire [15:0] opcode = cmd_reg[0][15:0];
	wire [1:0]  arg_id = cmd_reg[1][1:0];
	wire        arg_id_ok = (cmd_reg[1] < 32'd4);

	reg         run_d = 1'b0;

	// ---------------------------------------------------------------------
	// SDRAM requesters: the host window and the ROM fixup, merged by an
	// eth_port_arb (window first) into this module's single client port.
	// ---------------------------------------------------------------------
	reg         win_req = 1'b0, win_we = 1'b0;
	reg  [23:0] win_addr;
	reg  [15:0] win_din;
	wire        win_ack;
	wire [15:0] win_dout;

	reg         fix_req = 1'b0, fix_we = 1'b0;
	reg  [23:0] fix_addr;
	reg  [15:0] fix_din;
	wire        fix_ack;
	wire [15:0] fix_dout;

	eth_port_arb host_arb (
		.clk    ( clk ),
		.reset  ( reset ),
		.a_req  ( win_req ), .a_we ( win_we ), .a_addr ( win_addr ),
		.a_din  ( win_din ), .a_ack ( win_ack ), .a_dout ( win_dout ),
		.b_req  ( fix_req ), .b_we ( fix_we ), .b_addr ( fix_addr ),
		.b_din  ( fix_din ), .b_ack ( fix_ack ), .b_dout ( fix_dout ),
		.m_req  ( sd_req ),  .m_we ( sd_we ),  .m_addr ( sd_addr ),
		.m_din  ( sd_din ),  .m_ack ( sd_ack ), .m_dout ( sd_dout )
	);

	// ---------------------------------------------------------------------
	// Host memory transactions
	// ---------------------------------------------------------------------
	localparam [2:0] MS_IDLE = 3'd0, MS_SD_REQ = 3'd1, MS_SD_DROP = 3'd2,
	                 MS_SD_GAP = 3'd3, MS_DONE = 3'd4;
	reg  [2:0]  ms = MS_IDLE;
	reg         ms_half;             // 0 = upper 16 bits (lower SDRAM word), 1 = lower
	reg         ms_write;
	reg  [31:0] ms_wdata;
	reg  [31:0] ms_rdata;

	wire        is_sdram = (mem_address[31:28] == 4'h1);
	wire        is_cmd   = (mem_address[31:12] == 20'hF0000) && (mem_address[11:5] == 7'd0);
	wire        is_local = (mem_address[31:12] == 20'h00000);
	wire [11:0] loc      = mem_address[11:0];

	function [31:0] local_read(input [11:0] a);
		case (a)
			SET_MEMORY:       local_read = set_memory;
			SET_FLOPPY_WRITE: local_read = set_floppy_write;
			SET_PTR_DEFAULT:  local_read = set_ptr_default;
			SET_PTR_SPEED:    local_read = set_ptr_speed;
			SET_ICACHE_OFF:   local_read = set_icache_off;
			12'h800:          local_read = 32'h4D4C4331;   // "MLC1"
			12'h804:          local_read = dbg_state;
			12'h808:          local_read = rom_first_word;
			12'h810:          local_read = file_size[0];
			12'h814:          local_read = file_size[1];
			12'h818:          local_read = file_size[2];
			12'h81C:          local_read = file_size[3];
			12'h820:          local_read = dbg_blockdev;
			default:          local_read = 32'd0;
		endcase
	endfunction

	// ---------------------------------------------------------------------
	// ROM fixup: for a little-endian host, swap each word pair and the
	// bytes within it, turning {b3,b2},{b1,b0} into {b0,b1},{b2,b3}.
	// ---------------------------------------------------------------------
	localparam [2:0] FX_IDLE = 3'd0, FX_RD0 = 3'd1, FX_RD1 = 3'd2,
	                 FX_WR0 = 3'd3, FX_WR1 = 3'd4, FX_DONE = 3'd5;
	reg  [2:0]  fx = FX_IDLE;
	reg  [1:0]  fx_ph;               // 0 req, 1 wait ack low, 2 gap
	reg  [17:0] fx_pair;             // word-pair index
	reg  [15:0] fx_w0, fx_w1;
	wire        rom_says_be = (rom_first_word[31:24] == 8'h35);
	wire        rom_says_le = (rom_first_word[7:0]   == 8'h35);
	wire        le_verdict  = rom_says_be ? 1'b0 : rom_says_le ? 1'b1 : HOST_LE_DEFAULT;

	function [15:0] bswap(input [15:0] w);
		bswap = {w[7:0], w[15:8]};
	endfunction

	integer i;
	always @(posedge clk) begin
		if (reset) begin
			mem_done       <= 1'b0;
			mem_dataRead   <= 32'd0;
			ms             <= MS_IDLE;
			win_req        <= 1'b0;
			cmd_state      <= CS_IDLE;
			setup_done     <= 1'b0;
			run            <= 1'b0;
			run_d          <= 1'b0;
			fixup_done     <= 1'b0;
			rom_loading    <= 1'b0;
			rom_loaded     <= 1'b0;
			rom_first_seen <= 1'b0;
			rom_first_word <= 32'd0;
			mount_req      <= 1'b0;
			host_le        <= HOST_LE_DEFAULT;
			focus          <= 1'b0;
			file_present   <= 4'd0;
			fx             <= FX_IDLE;
			fix_req        <= 1'b0;
			act_reset_cnt  <= 8'd0;
			act_nmi_cnt    <= 8'd0;
			act_wipe_cnt   <= 8'd0;
			for (i = 0; i < 4; i = i + 1) file_size[i] <= 32'd0;
			for (i = 0; i < 8; i = i + 1) cmd_reg[i]   <= 32'd0;
		end else begin
			mem_done  <= 1'b0;
			mount_req <= 1'b0;
			if (act_reset_cnt != 0) act_reset_cnt <= act_reset_cnt - 8'd1;
			if (act_nmi_cnt   != 0) act_nmi_cnt   <= act_nmi_cnt   - 8'd1;
			if (act_wipe_cnt  != 0) act_wipe_cnt  <= act_wipe_cnt  - 8'd1;

			// ---- host memory transactions ----
			// !mem_done: the framework's SPI receiver keeps `enable` high for
			// the cycle in which it sees `done` (it pops the request on that
			// edge), so that cycle still shows the FINISHED transaction.
			case (ms)
			MS_IDLE: if (mem_enable && !mem_done) begin
				if (is_sdram) begin
					// Verbatim 32-bit word -> two SDRAM words (see header).
					ms_write <= mem_write;
					ms_wdata <= mem_dataWrite;
					ms_half  <= 1'b0;
					win_addr <= {mem_address[24:2], 1'b0};
					win_we   <= mem_write;
					win_din  <= mem_dataWrite[31:16];
					win_req  <= 1'b1;
					ms       <= MS_SD_REQ;
					if (mem_write && mem_address == ROM_HOST_ADDR && !rom_first_seen) begin
						rom_first_word <= mem_dataWrite;
						rom_first_seen <= 1'b1;
					end
				end else begin
					if (mem_write) begin
						if (is_cmd)
							cmd_reg[mem_address[4:2]] <= mem_dataWrite;
						else if (is_local) case (loc)
							// Actions count only while the Mac runs: settings are
							// also written during setup, and a replayed "Reset
							// PRAM" there would wipe PRAM on every launch.
							SET_ACT_RESET:     if (mem_dataWrite != 0 && !core_reset) act_reset_cnt <= 8'hFF;
							SET_ACT_NMI:       if (mem_dataWrite != 0 && !core_reset) act_nmi_cnt   <= 8'hFF;
							SET_ACT_WIPE_PRAM: if (mem_dataWrite != 0 && !core_reset) act_wipe_cnt  <= 8'hFF;
							SET_MEMORY:        set_memory       <= mem_dataWrite;
							SET_FLOPPY_WRITE:  set_floppy_write <= mem_dataWrite;
							SET_PTR_DEFAULT:   set_ptr_default  <= mem_dataWrite;
							SET_PTR_SPEED:     set_ptr_speed    <= mem_dataWrite;
							SET_ICACHE_OFF:    set_icache_off   <= mem_dataWrite;
							default: ;
						endcase
					end else begin
						mem_dataRead <= is_cmd   ? cmd_reg[mem_address[4:2]] :
						                is_local ? local_read(loc) : 32'd0;
					end
					ms <= MS_DONE;
				end
			end
			MS_SD_REQ: if (win_ack) begin
				win_req <= 1'b0;
				if (!ms_half) ms_rdata[31:16] <= win_dout;
				else          ms_rdata[15:0]  <= win_dout;
				ms <= MS_SD_DROP;
			end
			MS_SD_DROP: if (!win_ack) ms <= MS_SD_GAP;
			MS_SD_GAP: begin
				if (!ms_half) begin
					ms_half  <= 1'b1;
					win_addr <= win_addr + 24'd1;
					win_din  <= ms_wdata[15:0];
					win_req  <= 1'b1;
					ms       <= MS_SD_REQ;
				end else begin
					mem_dataRead <= ms_rdata;
					ms           <= MS_DONE;
				end
			end
			MS_DONE: begin
				mem_done <= 1'b1;
				ms       <= MS_IDLE;
			end
			default: ms <= MS_IDLE;
			endcase

			// ---- command channel ----
			if (cmd_request) begin
				if (cmd_state == CS_IDLE) begin
					cmd_state  <= CS_DONE;
					cmd_reg[0] <= 32'd0;
					case (opcode)
					CMD_GET_STATUS:
						cmd_reg[0] <= !setup_done ? STATUS_SETUP :
						              run         ? STATUS_CORE_RUN : STATUS_CORE_HALT;
					CMD_SETUP_COMPLETE:
						setup_done <= 1'b1;
					CMD_CORE_RUN:
						run <= 1'b1;
					CMD_CORE_HALT:
						run <= 1'b0;
					CMD_NOTIFY_FOCUS:
						focus <= cmd_reg[1][0];
					CMD_FILE_WR_START:
						if (!arg_id_ok) cmd_state <= CS_ERROR;
						else if (arg_id == FILE_ROM) rom_loading <= 1'b1;
					CMD_FILE_WR_END:
						if (!arg_id_ok) cmd_state <= CS_ERROR;
						else begin
							file_present[arg_id] <= 1'b1;
							file_size[arg_id]    <= cmd_reg[2];
							if (arg_id == FILE_ROM) begin
								rom_loading <= 1'b0;
								rom_loaded  <= 1'b1;
							end
						end
					CMD_FILE_RD_START:
						if (!arg_id_ok) cmd_state <= CS_ERROR;
						// The PRAM image is always one 512-byte sector,
						// whatever the host created it as.
						else cmd_reg[0] <= (arg_id == FILE_PRAM) ? 32'd512 : file_size[arg_id];
					CMD_FILE_RD_END:
						if (!arg_id_ok) cmd_state <= CS_ERROR;
					default:
						cmd_state <= CS_ERROR;
					endcase
				end
			end else begin
				cmd_state <= CS_IDLE;
			end

			// Present the disk images every time the Mac is released: a halt
			// and re-run reboots the Mac, and a re-mount is how the machine
			// learns about media (MiSTer's img_mounted semantics).
			run_d <= run && fixup_done;
			if (run && fixup_done && !run_d) mount_req <= 1'b1;

			// ---- ROM fixup (after setup, once) ----
			case (fx)
			FX_IDLE: if (setup_done && !fixup_done) begin
				host_le <= le_verdict;
				if (!rom_loaded || !le_verdict) begin
					fx <= FX_DONE;               // nothing to rewrite
				end else begin
					fx_pair <= 18'd0;
					fx_ph   <= 2'd0;
					fx      <= FX_RD0;
				end
			end
			FX_RD0, FX_RD1, FX_WR0, FX_WR1: begin
				case (fx_ph)
				2'd0: begin
					if (!fix_req) begin
						fix_req  <= 1'b1;
						fix_we   <= (fx == FX_WR0 || fx == FX_WR1);
						fix_addr <= ROM_WORD_BASE + {5'd0, fx_pair, (fx == FX_RD1 || fx == FX_WR1)};
						fix_din  <= (fx == FX_WR0) ? bswap(fx_w1) : bswap(fx_w0);
					end else if (fix_ack) begin
						fix_req <= 1'b0;
						if (fx == FX_RD0) fx_w0 <= fix_dout;
						if (fx == FX_RD1) fx_w1 <= fix_dout;
						fx_ph <= 2'd1;
					end
				end
				2'd1: if (!fix_ack) fx_ph <= 2'd2;
				default: begin
					fx_ph <= 2'd0;
					case (fx)
					FX_RD0: fx <= FX_RD1;
					FX_RD1: fx <= FX_WR0;
					FX_WR0: fx <= FX_WR1;
					default: begin
						if (fx_pair == ROM_WORDS[18:1] - 18'd1) fx <= FX_DONE;
						else begin
							fx_pair <= fx_pair + 18'd1;
							fx      <= FX_RD0;
						end
					end
					endcase
				end
				endcase
			end
			FX_DONE: fixup_done <= 1'b1;
			default: fx <= FX_IDLE;
			endcase
		end
	end

	assign dbg_state = {
		16'h4842,                   // "HB" marker
		1'b0, fx,                   // [15:12] fixup state
		focus,                      // [11]
		rom_says_le, rom_says_be,   // [10:9]
		host_le,                    // [8]
		file_present,               // [7:4]
		fixup_done,                 // [3]
		rom_loaded,                 // [2]
		run,                        // [1]
		setup_done                  // [0]
	};
	assign dbg_rom_word = rom_first_word;

endmodule

`default_nettype wire
