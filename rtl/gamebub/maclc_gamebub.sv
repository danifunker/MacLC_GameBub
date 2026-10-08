// ============================================================================
// maclc_gamebub.sv — Macintosh LC for Game Bub: the framework-facing top.
//
// chisel/src/MacLC.scala declares the framework interfaces; the framework
// binds them to this module (port names are generated from that bundle —
// rebuild prints the template if they ever change). This file only wires
// the platform layer around the machine:
//
//   gb_clocks      one MMCM: clk_sys 32.5, clk_mem 65, SPI 162.5, LCD 32.5
//   gb_host        HostV0 commands, settings, file windows into SDRAM
//   gb_blockdev    MiSTer hps_io block devices served from SDRAM images
//   gb_vram_sram   LC VRAM in the external 512 KiB SRAM
//   gb_video_out   V8 scanout -> 512x384 3/3/3 frame stream (dithered)
//   gb_pad_input   12 buttons -> PS/2 keyboard + mouse (-> ADB)
//   gb_debug_uart  status beacon on the PMOD header
//   maclc_core     the Macintosh itself (from the MiSTer core)
//
// PMOD header (3.3 V, J801):  pin 1 (pmod[0]) out  Mac modem port TxD
//                             pin 2 (pmod[1]) in   Mac modem port RxD
//                             pin 3 (pmod[2]) out  debug beacon, 115200 8N1
//                             pin 4 (pmod[3]) in   unused
// ============================================================================
`default_nettype none

module maclc_gamebub #(
	parameter int BUILD_UNIX_TIME = 0
)(
	input  wire         clock,
	input  wire         reset,
	input  wire         clocks_clockIn50M,
	output logic        clocks_clockOutSystem,
	output logic        clocks_clockOutDisplay,
	output logic        clocks_clockOutSpi,
	output logic        clocks_locked,
	output logic [2:0]  video_data_r,
	output logic [2:0]  video_data_g,
	output logic [2:0]  video_data_b,
	output logic        video_dataEnable,
	output logic        video_vblank,
	output logic        video_hblank,
	output logic [15:0] audio_left,
	output logic [15:0] audio_right,
	input  wire         host_mem_enable,
	input  wire         host_mem_write,
	output logic        host_mem_done,
	input  wire  [31:0] host_mem_address,
	output logic [31:0] host_mem_dataRead,
	input  wire  [31:0] host_mem_dataWrite,
	input  wire         host_commandHost_request,
	output logic        host_commandHost_busy,
	output logic        host_commandHost_done,
	output logic        host_commandHost_error,
	output logic        host_commandCore_request,
	input  wire         host_commandCore_busy,
	input  wire         host_commandCore_done,
	input  wire         host_commandCore_error,
	input  wire         input_buttons_a,
	input  wire         input_buttons_b,
	input  wire         input_buttons_x,
	input  wire         input_buttons_y,
	input  wire         input_buttons_up,
	input  wire         input_buttons_down,
	input  wire         input_buttons_left,
	input  wire         input_buttons_right,
	input  wire         input_buttons_l,
	input  wire         input_buttons_r,
	input  wire         input_buttons_start,
	input  wire         input_buttons_select,
	output logic        sdram_clock,
	output logic        sdram_cke,
	output logic        sdram_cs,
	output logic        sdram_ras,
	output logic        sdram_cas,
	output logic        sdram_we,
	output logic [1:0]  sdram_dqm,
	output logic [1:0]  sdram_bank,
	output logic [12:0] sdram_address,
	input  wire  [15:0] sdram_dataIn,
	output logic [15:0] sdram_dataOut,
	output logic        sdram_dataDir,
	output logic        sram_ceN,
	output logic        sram_weN,
	output logic        sram_oeN,
	output logic [1:0]  sram_writeMaskN,
	output logic [17:0] sram_address,
	input  wire  [15:0] sram_dataIn,
	output logic [15:0] sram_dataOut,
	output logic        sram_dataDir,
	input  wire  [3:0]  pmod_in,
	output logic [3:0]  pmod_out,
	output logic [3:0]  pmod_dir
);

	localparam integer VDNUM = 7;

	// ---------------------------------------------------------------------
	// Clocks
	// ---------------------------------------------------------------------
	wire clk_sys, clk_mem, pll_locked;
	gb_clocks clocks (
		.clk_in_50mhz (clocks_clockIn50M),
		.clk_sys      (clk_sys),
		.clk_mem      (clk_mem),
		.clk_spi      (clocks_clockOutSpi),
		.clk_display  (clocks_clockOutDisplay),
		.locked       (pll_locked)
	);
	assign clocks_clockOutSystem = clk_sys;
	assign clocks_locked         = pll_locked;
	// `clock` is clk_sys handed back by the framework; logic here uses
	// clk_sys directly (same net) and the framework's synchronous `reset`.

	// ---------------------------------------------------------------------
	// Host protocol
	// ---------------------------------------------------------------------
	wire        core_reset, rom_loading, rom_loaded, mount_req, host_le, focus;
	wire [31:0] status;
	wire        ptr_default;
	wire  [1:0] ptr_speed;
	wire  [3:0] file_present;
	wire [127:0] file_size_flat;
	wire [31:0] dbg_blockdev, dbg_host_state, dbg_rom_word;

	wire        host_sd_req, host_sd_we, host_sd_ack;
	wire [23:0] host_sd_addr;
	wire [15:0] host_sd_din, host_sd_dout;

	gb_host host (
		.clk            (clk_sys),
		.reset          (reset),
		.mem_enable     (host_mem_enable),
		.mem_write      (host_mem_write),
		.mem_address    (host_mem_address),
		.mem_dataWrite  (host_mem_dataWrite),
		.mem_dataRead   (host_mem_dataRead),
		.mem_done       (host_mem_done),
		.cmd_request    (host_commandHost_request),
		.cmd_busy       (host_commandHost_busy),
		.cmd_done       (host_commandHost_done),
		.cmd_error      (host_commandHost_error),
		.sd_req         (host_sd_req),
		.sd_we          (host_sd_we),
		.sd_addr        (host_sd_addr),
		.sd_din         (host_sd_din),
		.sd_ack         (host_sd_ack),
		.sd_dout        (host_sd_dout),
		.core_reset     (core_reset),
		.rom_loading    (rom_loading),
		.rom_loaded     (rom_loaded),
		.status         (status),
		.mount_req      (mount_req),
		.host_le        (host_le),
		.focus          (focus),
		.ptr_default    (ptr_default),
		.ptr_speed      (ptr_speed),
		.file_present   (file_present),
		.file_size_flat (file_size_flat),
		.dbg_blockdev   (dbg_blockdev),
		.dbg_state      (dbg_host_state),
		.dbg_rom_word   (dbg_rom_word)
	);
	// No core->host commands exist in framework v1.1.
	assign host_commandCore_request = 1'b0;

	// ---------------------------------------------------------------------
	// Block devices
	// ---------------------------------------------------------------------
	wire [VDNUM*32-1:0] sd_lba_flat;
	wire [VDNUM-1:0]    sd_rd, sd_wr, sd_ack, img_mounted;
	wire [12:0]         sd_buff_addr;
	wire [15:0]         sd_buff_dout;
	wire [VDNUM*16-1:0] sd_buff_din_flat;
	wire                sd_buff_wr, img_readonly;
	wire [63:0]         img_size;

	wire        bd_sd_req, bd_sd_we, bd_sd_ack;
	wire [23:0] bd_sd_addr;
	wire [15:0] bd_sd_din, bd_sd_dout;

	gb_blockdev #(.VDNUM(VDNUM)) blockdev (
		.clk              (clk_sys),
		.reset            (reset),
		.file_present     (file_present),
		.file_size_flat   (file_size_flat),
		.host_le          (host_le),
		.mount_req        (mount_req),
		.sd_lba_flat      (sd_lba_flat),
		.sd_rd            (sd_rd),
		.sd_wr            (sd_wr),
		.sd_ack           (sd_ack),
		.sd_buff_addr     (sd_buff_addr),
		.sd_buff_dout     (sd_buff_dout),
		.sd_buff_din_flat (sd_buff_din_flat),
		.sd_buff_wr       (sd_buff_wr),
		.img_mounted      (img_mounted),
		.img_size         (img_size),
		.img_readonly     (img_readonly),
		.mem_req          (bd_sd_req),
		.mem_we           (bd_sd_we),
		.mem_addr         (bd_sd_addr),
		.mem_din          (bd_sd_din),
		.mem_ack          (bd_sd_ack),
		.mem_dout         (bd_sd_dout),
		.dbg              (dbg_blockdev)
	);

	// Host window (files) and block devices share the core's platform port.
	wire        pm_req, pm_we, pm_ack;
	wire [23:0] pm_addr;
	wire [15:0] pm_din, pm_dout;
	gb_eth_arb pm_arb (
		.clk    ( clk_sys ),
		.reset  ( reset ),
		.a_req  ( host_sd_req ), .a_we ( host_sd_we ), .a_addr ( host_sd_addr ),
		.a_din  ( host_sd_din ), .a_ack ( host_sd_ack ), .a_dout ( host_sd_dout ),
		.b_req  ( bd_sd_req ),   .b_we ( bd_sd_we ),   .b_addr ( bd_sd_addr ),
		.b_din  ( bd_sd_din ),   .b_ack ( bd_sd_ack ), .b_dout ( bd_sd_dout ),
		.m_req  ( pm_req ),      .m_we ( pm_we ),      .m_addr ( pm_addr ),
		.m_din  ( pm_din ),      .m_ack ( pm_ack ),    .m_dout ( pm_dout )
	);

	// ---------------------------------------------------------------------
	// Input: Game Bub buttons in the Pocket cont1_key layout
	// ---------------------------------------------------------------------
	wire [15:0] cont1_key = {
		input_buttons_start,   // 15
		input_buttons_select,  // 14
		4'd0,                  // 13:10
		input_buttons_r,       // 9  (R1)
		input_buttons_l,       // 8  (L1)
		input_buttons_y,       // 7
		input_buttons_x,       // 6
		input_buttons_b,       // 5
		input_buttons_a,       // 4
		input_buttons_right,   // 3
		input_buttons_left,    // 2
		input_buttons_down,    // 1
		input_buttons_up       // 0
	};
	wire [10:0] ps2_key;
	wire [24:0] ps2_mouse;
	gb_pad_input pad (
		.clk         (clk_sys),
		// re-sampled each time the Mac is released, so the "start in mouse
		// mode" setting written during setup takes effect
		.reset       (reset || core_reset),
		.cont1_key   (cont1_key),
		// PS/2 Set 2 scancodes (bit 8 = E0), translated to ADB by adb_device
		.map_a       (9'h05A),   // Return  (mouse button in pointer mode)
		.map_b       (9'h029),   // Space
		.map_x       (9'h012),   // Shift
		.map_y       (9'h031),   // N
		.map_l       (9'h076),   // Escape
		.map_r       (9'h11F),   // Option
		.map_start   (9'h011),   // Command
		.ps2_key     (ps2_key),
		.ps2_mouse   (ps2_mouse),
		.ptr_default (ptr_default),
		.ptr_mode    ()
	);

	// ---------------------------------------------------------------------
	// The Macintosh
	// ---------------------------------------------------------------------
	wire [7:0]  vid_r, vid_g, vid_b;
	wire        vid_de, vid_vblank, vid_pix_stb, vid_reset;
	wire [17:0] vram_raddr, vram_waddr;
	wire        vram_rd, vram_rready, vram_rvalid, vram_we;
	wire [15:0] vram_rdata, vram_wdata;
	wire  [1:0] vram_wbe;
	wire        sdram_dq_oe;
	wire [31:0] dbg_cpu_addr;
	wire        dbg_cpu_reset_n;
	wire  [1:0] dbg_disk_act;
	wire        mac_txd;

	maclc_core #(
		.BUILD_UNIX_TIME (BUILD_UNIX_TIME),
		.VDNUM           (VDNUM)
	) mac (
		.clk_sys          (clk_sys),
		.clk_mem          (clk_mem),
		.pll_locked       (pll_locked),
		.host_reset       (core_reset),
		.rom_loading      (rom_loading),
		.rom_loaded       (rom_loaded),
		.status           (status),
		.sd_lba_flat      (sd_lba_flat),
		.sd_rd            (sd_rd),
		.sd_wr            (sd_wr),
		.sd_ack           (sd_ack),
		.sd_buff_addr     (sd_buff_addr),
		.sd_buff_dout     (sd_buff_dout),
		.sd_buff_din_flat (sd_buff_din_flat),
		.sd_buff_wr       (sd_buff_wr),
		.img_mounted      (img_mounted),
		.img_size         (img_size),
		.img_readonly     (img_readonly),
		.ps2_key          (ps2_key),
		.ps2_mouse        (ps2_mouse),
		.uart_txd         (mac_txd),
		.uart_rxd         (pmod_in[1]),
		.pm_req           (pm_req),
		.pm_we            (pm_we),
		.pm_addr          (pm_addr),
		.pm_din           (pm_din),
		.pm_ack           (pm_ack),
		.pm_dout          (pm_dout),
		.vid_r            (vid_r),
		.vid_g            (vid_g),
		.vid_b            (vid_b),
		.vid_de           (vid_de),
		.vid_vblank       (vid_vblank),
		.vid_pix_stb      (vid_pix_stb),
		.vid_reset        (vid_reset),
		.vram_raddr       (vram_raddr),
		.vram_rd          (vram_rd),
		.vram_rready      (vram_rready),
		.vram_rvalid      (vram_rvalid),
		.vram_rdata       (vram_rdata),
		.vram_waddr       (vram_waddr),
		.vram_wdata       (vram_wdata),
		.vram_wbe         (vram_wbe),
		.vram_we          (vram_we),
		.audio_l          (audio_left),
		.audio_r          (audio_right),
		.sdram_clk        (sdram_clock),
		.sdram_dq_out     (sdram_dataOut),
		.sdram_dq_in      (sdram_dataIn),
		.sdram_dq_oe      (sdram_dq_oe),
		.sdram_a          (sdram_address),
		.sdram_dqm        (sdram_dqm),
		.sdram_ba         (sdram_bank),
		.sdram_cs_n       (sdram_cs),
		.sdram_we_n       (sdram_we),
		.sdram_ras_n      (sdram_ras),
		.sdram_cas_n      (sdram_cas),
		.dbg_cpu_addr     (dbg_cpu_addr),
		.dbg_cpu_reset_n  (dbg_cpu_reset_n),
		.dbg_disk_act     (dbg_disk_act)
	);
	assign sdram_cke     = 1'b1;
	assign sdram_dataDir = sdram_dq_oe;

	// ---------------------------------------------------------------------
	// VRAM and video
	// ---------------------------------------------------------------------
	wire [7:0] vram_overflows;
	gb_vram_sram vram (
		.clk           (clk_sys),
		.reset         (reset),
		.rd_addr       (vram_raddr),
		.rd            (vram_rd),
		.rd_ready      (vram_rready),
		.rd_valid      (vram_rvalid),
		.rd_data       (vram_rdata),
		.wr_addr       (vram_waddr),
		.wr_data       (vram_wdata),
		.wr_be         (vram_wbe),
		.wr            (vram_we),
		.sram_ce_n     (sram_ceN),
		.sram_we_n     (sram_weN),
		.sram_oe_n     (sram_oeN),
		.sram_mask_n   (sram_writeMaskN),
		.sram_a        (sram_address),
		.sram_dq_in    (sram_dataIn),
		.sram_dq_out   (sram_dataOut),
		.sram_dq_oe    (sram_dataDir),
		.dbg_overflows (vram_overflows)
	);

	gb_video_out video (
		.clk        (clk_sys),
		.reset      (reset || vid_reset),
		.in_r       (vid_r),
		.in_g       (vid_g),
		.in_b       (vid_b),
		.in_de      (vid_de),
		.in_vblank  (vid_vblank),
		.in_pix_stb (vid_pix_stb),
		.out_r      (video_data_r),
		.out_g      (video_data_g),
		.out_b      (video_data_b),
		.out_de     (video_dataEnable),
		.out_hblank (video_hblank),
		.out_vblank (video_vblank)
	);

	// ---------------------------------------------------------------------
	// Debug beacon (PMOD pin 3). Words, in order:
	//   0  gb_host state: "HB" | fixup | focus, rom LE/BE verdict, host_le,
	//      file_present[3:0], fixup_done, rom_loaded, run, setup_done
	//   1  first 32-bit word the host wrote to the ROM
	//   2  hard disk size (bytes)      3  floppy size (bytes)
	//   4  block device {writes, reads}
	//   5  CPU address bus
	//   6  {frames[15:0], vram queue overflows[7:0], cpu_reset_n, disk act[1:0], ...}
	//   7  status[] bits handed to the machine
	// ---------------------------------------------------------------------
	reg [15:0] frames = 16'd0;
	reg        vbl_d  = 1'b0;
	always @(posedge clk_sys) begin
		vbl_d <= video_vblank;
		if (video_vblank && !vbl_d) frames <= frames + 16'd1;
	end
	wire dbg_txd;
	gb_debug_uart #(.NWORDS(8)) beacon (
		.clk   (clk_sys),
		.reset (reset),
		.words ({
			status,
			{frames, vram_overflows, dbg_cpu_reset_n, dbg_disk_act, 3'd0, focus, core_reset},
			dbg_cpu_addr,
			dbg_blockdev,
			file_size_flat[95:64],
			file_size_flat[31:0],
			dbg_rom_word,
			dbg_host_state
		}),
		.txd   (dbg_txd)
	);

	assign pmod_out = {1'b0, dbg_txd, 1'b1, mac_txd};
	assign pmod_dir = 4'b0101;

endmodule

`default_nettype wire
