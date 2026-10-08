//============================================================================
//  Macintosh LC — machine top for Game Bub
//
//  Derived from MacLC.sv (module emu) of the MiSTer core, ../MacLC_MiSTer,
//  at commit 045f896 (master, 2026-09-20). Everything that IS the Macintosh
//  is kept as-is: reset sequencing, PRAM persistence, CPU bus glue, the
//  fetch cache, floppy loader/writer, the SDRAM request pipeline.
//  Everything that was the MiSTer platform is replaced by ports that
//  rtl/gamebub/maclc_gamebub.sv serves:
//
//    MiSTer                         here
//    hps_io status/buttons/RESET    status, host_reset (from gb_host)
//    hps_io ioctl ROM download      rom_loading / rom_loaded (the ROM is
//                                   written into SDRAM by gb_host directly)
//    hps_io sd_* block devices      same signals, served by gb_blockdev
//    hps_io ps2_key / ps2_mouse     same buses, from gb_input (buttons)
//    pll / pll_video (+reconfig)    clk_sys / clk_mem from gb_clocks; the V8
//                                   scans out on clk_sys with a fractional
//                                   pixel enable (the framework re-buffers
//                                   every frame, so enable jitter is harmless)
//    vram_bram (on-chip)            vram_* ports -> gb_vram_sram (ext. SRAM)
//    pds_enet SDRAM DMA port        pm_* ports (gb_host + gb_blockdev)
//    removed: video_freak, mt32pi, pds_enet, Altera probes/anchors, the HUD
//
//  When re-importing a newer MiSTer core, diff MacLC.sv against 045f896 and
//  carry machine-side changes over by hand.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//============================================================================
`default_nettype none

module maclc_core #(
	parameter [31:0] BUILD_UNIX_TIME = 32'd0,
	parameter integer VDNUM = 7
)(
	input  wire        clk_sys,          // 32.5 MHz
	input  wire        clk_mem,          // 65 MHz, 2x clk_sys, edges aligned
	input  wire        pll_locked,

	// ---- platform control (gb_host) ----
	input  wire        host_reset,       // hold the Mac in reset (core halted)
	input  wire        rom_loading,      // ROM image is being written to SDRAM
	input  wire        rom_loaded,       // a ROM image is in SDRAM
	input  wire [31:0] status,           // MiSTer status[] bit layout

	// ---- block devices, hps_io shape (gb_blockdev) ----
	output wire [VDNUM*32-1:0] sd_lba_flat,
	output wire [VDNUM-1:0]    sd_rd,
	output wire [VDNUM-1:0]    sd_wr,
	input  wire [VDNUM-1:0]    sd_ack,
	input  wire [12:0]         sd_buff_addr,
	input  wire [15:0]         sd_buff_dout,
	output wire [VDNUM*16-1:0] sd_buff_din_flat,
	input  wire                sd_buff_wr,
	input  wire [VDNUM-1:0]    img_mounted,
	input  wire [63:0]         img_size,
	input  wire                img_readonly,

	// ---- input (gb_input) ----
	input  wire [10:0] ps2_key,
	input  wire [24:0] ps2_mouse,

	// ---- SCC channel A (modem port) ----
	output wire        uart_txd,
	input  wire        uart_rxd,

	// ---- platform SDRAM client: shares the controller's DMA port with the
	//      floppy writer (eth_port_arb client protocol) ----
	input  wire        pm_req,
	input  wire        pm_we,
	input  wire [23:0] pm_addr,
	input  wire [15:0] pm_din,
	output wire        pm_ack,
	output wire [15:0] pm_dout,

	// ---- video, clk_sys domain ----
	output wire [7:0]  vid_r,
	output wire [7:0]  vid_g,
	output wire [7:0]  vid_b,
	output wire        vid_de,           // aligned with vid_r/g/b
	output wire        vid_vblank,       // aligned with vid_de
	output wire        vid_pix_stb,      // one clk_sys per pixel, aligned
	output wire        vid_reset,

	// ---- VRAM in external SRAM (gb_vram_sram) ----
	output wire [17:0] vram_raddr,
	output wire        vram_rd,
	input  wire        vram_rready,
	input  wire        vram_rvalid,
	input  wire [15:0] vram_rdata,
	output wire [17:0] vram_waddr,
	output wire [15:0] vram_wdata,
	output wire  [1:0] vram_wbe,
	output wire        vram_we,

	// ---- audio, clk_sys domain ----
	output wire [15:0] audio_l,
	output wire [15:0] audio_r,

	// ---- SDRAM pins ----
	output wire        sdram_clk,
	output wire [15:0] sdram_dq_out,
	input  wire [15:0] sdram_dq_in,
	output wire        sdram_dq_oe,
	output wire [12:0] sdram_a,
	output wire  [1:0] sdram_dqm,
	output wire  [1:0] sdram_ba,
	output wire        sdram_cs_n,
	output wire        sdram_we_n,
	output wire        sdram_ras_n,
	output wire        sdram_cas_n,

	// ---- debug (gb_debug_uart beacon) ----
	output wire [31:0] dbg_cpu_addr,
	output wire        dbg_cpu_reset_n,
	output wire  [1:0] dbg_disk_act
);

	////////////////////   CLOCKS / RESET   ///////////////////

	// pll_locked is asynchronous to clk_sys — synchronize it before the reset
	// logic / PRAM FSM consume it. (The sdram controller keeps the raw signal:
	// a glitchy init reload there is harmless, it just re-runs the ladder.)
	reg [1:0] pll_locked_sync = 2'b00;
	always @(posedge clk_sys) pll_locked_sync <= {pll_locked_sync[0], pll_locked};
	wire pll_locked_s = pll_locked_sync[1];

	// MiSTer's ioctl ROM download maps onto the host's ROM file write: the
	// machine is held in reset while it is written, and the fetch cache is
	// flushed by it (see icache.flush_bits).
	wire       dio_download = rom_loading;

	reg       status_mem = 1'b1;
	localparam [1:0] status_cpu = 2'b10; // 68020
	reg       n_reset = 0;
	reg       pram_force_reset = 1'b0;  // "Reset PRAM & Core" -> system reset pulse
	wire      egret_reset_680x0_w;      // Egret HC05 holding 68k in reset
	wire      clk8_en_p, clk8_en_n;     // from v8_clocks below (declared before first use)
	// Mac LC always runs at C15M (~15.67 MHz) - use 16 MHz clock enables
	always @(posedge clk_sys) begin
		reg [15:0] rst_cnt;

		if (clk8_en_p) begin
			// NOTE: Do NOT include ~_cpuReset_o here — the CPU executes the RESET
			// instruction during boot to reset peripherals, which would cause an
			// infinite reset loop if fed back to the system reset.
			if(~pll_locked_s || !rom_loaded || status[0] || host_reset || pram_force_reset || dio_download) begin
				rst_cnt <= '1;
				n_reset <= 0;
			end
			else if(rst_cnt) begin
				rst_cnt    <= rst_cnt - 1'd1;
				status_mem <= status[4];
			end
			else begin
				n_reset <= 1;
			end
		end
	end

	///////////////////////////////////////////////////

	localparam SCSI_DEVS = 2;          // SCSI block devices -> slots 0,1
	localparam VD_PRAM    = 2;         // PRAM NVRAM save image -> slot 2
	localparam VD_TOOLBOX = 3;         // BlueSCSI Toolbox shared folder -> slot 3 (never mounted here)
	localparam VD_CDROM   = 4;         // CD-ROM image (SCSI ID 3) -> slot 4 (never mounted here)
	localparam VD_CD_TOOLBOX = 5;      // BlueSCSI Toolbox CD Changer control -> slot 5 (never mounted)
	localparam VD_FLOPPY_INT = 6;      // floppy image -> slot 6

	// block-device buses (all VDNUM devices)
	wire [31:0] sd_lba[VDNUM];
	wire [15:0] sd_buff_din[VDNUM];
	genvar gi;
	generate
		for (gi = 0; gi < VDNUM; gi = gi + 1) begin : g_flat
			assign sd_lba_flat[gi*32 +: 32]      = sd_lba[gi];
			assign sd_buff_din_flat[gi*16 +: 16] = sd_buff_din[gi];
		end
	endgenerate

	// SCSI side (slots 0,1): separate buses driven by dataController, stitched into
	// the shared buses so the PRAM save image (slot 2) can coexist.
	wire [31:0] scsi_lba[SCSI_DEVS];
	wire  [SCSI_DEVS-1:0] scsi_rd, scsi_wr;
	wire  [SCSI_DEVS-1:0] scsi_ack = sd_ack[SCSI_DEVS-1:0];
	wire           [15:0] scsi_buff_din[SCSI_DEVS];
	assign sd_lba[0]      = scsi_lba[0];
	assign sd_lba[1]      = scsi_lba[1];
	assign sd_rd[1:0]     = scsi_rd;
	assign sd_wr[1:0]     = scsi_wr;
	assign sd_buff_din[0] = scsi_buff_din[0];
	assign sd_buff_din[1] = scsi_buff_din[1];

	// BlueSCSI Toolbox dedicated slot (3). Inert: nothing mounts it on Game Bub.
	wire [31:0] tb_lba;
	wire        tb_rd, tb_wr;
	wire [15:0] tb_buff_din;
	assign sd_lba[VD_TOOLBOX]      = tb_lba;
	assign sd_rd [VD_TOOLBOX]      = tb_rd;
	assign sd_wr [VD_TOOLBOX]      = tb_wr;
	assign sd_buff_din[VD_TOOLBOX] = tb_buff_din;
	wire        tb_ack     = sd_ack[VD_TOOLBOX];
	wire        tb_mounted = img_mounted[VD_TOOLBOX];

	// BlueSCSI Toolbox CD Changer control slot (5). Inert.
	wire [31:0] cdtb_lba;
	wire        cdtb_rd, cdtb_wr;
	wire [15:0] cdtb_buff_din;
	assign sd_lba[VD_CD_TOOLBOX]      = cdtb_lba;
	assign sd_rd [VD_CD_TOOLBOX]      = cdtb_rd;
	assign sd_wr [VD_CD_TOOLBOX]      = cdtb_wr;
	assign sd_buff_din[VD_CD_TOOLBOX] = cdtb_buff_din;
	wire        cdtb_ack     = sd_ack[VD_CD_TOOLBOX];
	wire        cdtb_mounted = img_mounted[VD_CD_TOOLBOX];

	// CD-ROM (SCSI ID 3) slot (4): read-only. Nothing mounts it on Game Bub
	// (a CD image cannot be loaded whole into memory), so the drive reports
	// no media; status[18] can still remove it from the bus entirely.
	wire [31:0] cd_lba;
	wire        cd_rd;
	wire [15:0] cd_buff_din;
	assign sd_lba[VD_CDROM]      = cd_lba;
	assign sd_rd [VD_CDROM]      = cd_rd;
	assign sd_wr [VD_CDROM]      = 1'b0;
	assign sd_buff_din[VD_CDROM] = cd_buff_din;
	wire        cd_ack     = sd_ack[VD_CDROM];
	wire        cd_mounted = img_mounted[VD_CDROM];
	wire        cd_enable  = ~status[18];

	// RTC seed: the host has no clock to give us, so the Mac starts at the
	// time the bitstream was built (TIMESTAMP[31:0] is read once, at the
	// Egret's PRAM boot-copy; see egret_wrapper.sv mac_seconds).
	wire [32:0] TIMESTAMP = {1'b0, BUILD_UNIX_TIME};

	// =====================================================================
	// PRAM persistence (NVRAM) — autosave to the mounted save image (slot 2).
	//   load  : when the PRAM image mounts (img_mounted[VD_PRAM], size>0)
	//   flush : when PRAM changed and writes settled (eager persistence)
	//   R6    : "Reset PRAM & Core" — zero PRAM, flush zeros, reboot the machine
	// One 512-byte sector at LBA 0 holds the 256 PRAM bytes (rest padded).
	// On Game Bub the "SD card" behind slot 2 is the PRAM file's image in
	// SDRAM, which the host writes back to the microSD card at core exit.
	// =====================================================================
	reg        pram_load_wr;
	reg  [7:0] pram_load_addr, pram_load_data, pram_save_addr;
	wire [7:0] pram_save_data;
	wire       pram_wr_stb;

	reg        pram_rd, pram_wr_req;
	wire       pram_ack = sd_ack[VD_PRAM];
	assign sd_lba[VD_PRAM] = 32'd0;             // single 512B sector at LBA 0
	assign sd_rd [VD_PRAM] = pram_rd;
	assign sd_wr [VD_PRAM] = pram_wr_req;

	reg  [7:0] pram_buf[0:255];                 // staging buffer <-> SD sector
	// FPGA->HPS readback during save: 16-bit word = {odd byte, even byte}; pad.
	assign sd_buff_din[VD_PRAM] = (sd_buff_addr < 8'd128)
	        ? {pram_buf[{sd_buff_addr[6:0],1'b1}], pram_buf[{sd_buff_addr[6:0],1'b0}]}
	        : 16'h0000;

	reg        pram_ena;                        // a save image is mounted (size>0)
	reg        pram_dirty;                      // PRAM changed since last save
	reg        pram_rst_after;                  // pulse reset after the current save
	reg        pram_load_pending, pram_flush_pending, pram_clr_pending;
	reg [26:0] pram_settle;          // eager-flush settle timer (restarts on each PRAM write)
	reg        old_pack, old_mnt2, old_rstpram;
	reg        pram_ready;        // -> Egret: pram[] loaded (or no image / timed out)
	reg [31:0] pram_rdy_cnt;      // ready backstop so a missing image never hangs boot
	reg        pram_restart_after_load; // load landed after CPU release -> clean restart
	reg [26:0] pram_ld_wd;        // load watchdog: re-kick a stalled SD read
	reg  [1:0] pram_ld_try;       // retries before giving up (boot with defaults)

	localparam [3:0] P_IDLE=0, P_LD_RD=1, P_LD_DAT=2, P_LD_CPY=3,
	                 P_FILL=4, P_SV_WR=5, P_SV_DAT=6, P_CLR=7, P_RST=8, P_LD_KICK=9;
	localparam [26:0] PRAM_LD_WD_MAX = 27'd65_000_000;
	reg  [3:0] pst;
	reg  [8:0] pcnt;
	reg  [6:0] rst_hold;

	always @(posedge clk_sys) begin
		if (~pll_locked_s) begin
			pst <= P_IDLE; pram_rd <= 0; pram_wr_req <= 0; pram_load_wr <= 0;
			pram_ena <= 0; pram_dirty <= 0; pram_force_reset <= 0; pram_rst_after <= 0;
			pram_load_pending <= 0; pram_flush_pending <= 0; pram_clr_pending <= 0;
			pram_settle <= 0;
			old_pack <= 0; old_mnt2 <= 0; old_rstpram <= 0; rst_hold <= 0;
			pram_ready <= 0; pram_rdy_cnt <= 0;
			pram_restart_after_load <= 0; pram_ld_wd <= 0; pram_ld_try <= 0;
		end else begin
			old_pack    <= pram_ack;
			old_mnt2    <= img_mounted[VD_PRAM];
			old_rstpram <= status[6];
			pram_load_wr <= 1'b0;                  // default low; pulsed in copy/clear

			// PRAM SD-read capture (only while the slot is being serviced)
			if (pram_ack && sd_buff_wr && sd_buff_addr < 8'd128) begin
				pram_buf[{sd_buff_addr[6:0],1'b0}] <= sd_buff_dout[7:0];
				pram_buf[{sd_buff_addr[6:0],1'b1}] <= sd_buff_dout[15:8];
			end

			// firmware PRAM writes mark the image dirty
			if (pram_wr_stb) pram_dirty <= 1'b1;

			// event latches
			if (img_mounted[VD_PRAM] && !old_mnt2) begin
				pram_ena <= (img_size != 0);
				if (img_size != 0) pram_load_pending <= 1'b1;  // load runs -> P_LD_CPY sets pram_ready
				else               pram_ready        <= 1'b1;  // no image: release the boot-copy now
			end
			// Eager persistence: flush whenever PRAM changed and the writes
			// settled (~2 s). (MiSTer also flushed on OSD-open; there is no
			// OSD here, and the host saves the image at core exit.)
			if (pram_wr_stb)                pram_settle <= 27'd65_000_000;  // ~2 s at 32.5 MHz clk_sys
			else if (pram_settle > 27'd1)   pram_settle <= pram_settle - 1'b1;
			else if (pram_settle == 27'd1) begin
				pram_settle <= 27'd0;
				if (pram_dirty && pram_ena) pram_flush_pending <= 1'b1;
			end
			if (status[6] && !old_rstpram) pram_clr_pending <= 1'b1;

			// PRAM-ready gate (see MacLC.sv): released by the load FSM, by a
			// size-0 mount, or by this backstop (~6 s at 32.5 MHz).
			if (!pram_ready && pst != P_LD_CPY) begin
				if (pram_rdy_cnt >= 32'd200_000_000) pram_ready <= 1'b1;
				else pram_rdy_cnt <= pram_rdy_cnt + 1'b1;
			end

			// hold the reset pulse long enough for the clk8_en_p reset block to latch
			if (pram_force_reset) begin
				if (rst_hold == 0) pram_force_reset <= 1'b0;
				else rst_hold <= rst_hold - 1'b1;
			end

			case (pst)
			P_IDLE: begin
				if (pram_clr_pending) begin
					pram_clr_pending <= 0; pcnt <= 0; pst <= P_CLR;
				end else if (pram_load_pending) begin
					pram_load_pending <= 0; pram_rd <= 1'b1;
					pram_ld_wd <= 0; pram_ld_try <= 0; pst <= P_LD_RD;
				end else if (pram_flush_pending) begin
					pram_flush_pending <= 0; pram_rst_after <= 0; pcnt <= 0; pst <= P_FILL;
				end
			end

			// ---- LOAD: SD sector -> pram_buf -> Egret pram[] ----
			P_LD_RD:
				if (pram_ack) begin pram_rd <= 1'b0; pram_ld_wd <= 0; pst <= P_LD_DAT; end
				else if (pram_ld_wd == PRAM_LD_WD_MAX) begin
					pram_ld_wd <= 0;
					if (pram_ld_try == 2'd3) begin  // give up: release the boot
						pram_rd <= 1'b0; pram_ready <= 1'b1; pst <= P_IDLE;
					end else begin                  // drop + re-arm the request
						pram_ld_try <= pram_ld_try + 1'b1;
						pram_rd <= 1'b0; pst <= P_LD_KICK;
					end
				end
				else pram_ld_wd <= pram_ld_wd + 1'b1;
			P_LD_KICK: begin pram_rd <= 1'b1; pst <= P_LD_RD; end
			P_LD_DAT:
				if (old_pack && !pram_ack) begin
					pcnt <= 0;
					pram_restart_after_load <= pram_ready;
					pst <= P_LD_CPY;
				end
				else if (pram_ld_wd == PRAM_LD_WD_MAX) begin
					pram_ld_wd <= 0; pram_ready <= 1'b1; pst <= P_IDLE;  // wedged ack: boot as-is
				end
				else pram_ld_wd <= pram_ld_wd + 1'b1;
			P_LD_CPY: begin
				pram_load_wr   <= 1'b1;
				pram_load_addr <= pcnt[7:0];
				pram_load_data <= pram_buf[pcnt[7:0]];
				if (pcnt == 9'd255) begin
					pram_dirty <= 0; pram_ena <= 1; pram_ready <= 1'b1;
					if (pram_restart_after_load) begin pram_restart_after_load <= 0; pst <= P_RST; end
					else pst <= P_IDLE;
				end
				else pcnt <= pcnt + 1'b1;
			end

			// ---- SAVE: Egret pram[] -> pram_buf -> SD sector ----
			P_FILL: begin
				pram_save_addr <= pcnt[7:0];               // addr for capture next cycle
				if (pcnt != 0) pram_buf[pcnt[7:0] - 8'd1] <= pram_save_data;
				if (pcnt == 9'd256) pst <= P_SV_WR;
				else pcnt <= pcnt + 1'b1;
			end
			P_SV_WR: begin
				pram_wr_req <= 1'b1;
				if (pram_ack) begin pram_wr_req <= 1'b0; pst <= P_SV_DAT; end
			end
			P_SV_DAT: if (old_pack && !pram_ack) begin
				pram_dirty <= 0;
				if (pram_rst_after) begin pram_rst_after <= 0; pst <= P_RST; end
				else pst <= P_IDLE;
			end

			// ---- Reset PRAM & Core ----
			P_CLR: begin                                   // zero Egret pram[] + pram_buf
				pram_load_wr   <= 1'b1;
				pram_load_addr <= pcnt[7:0];
				pram_load_data <= 8'h00;
				pram_buf[pcnt[7:0]] <= 8'h00;
				if (pcnt == 9'd255) begin
					if (pram_ena) begin pram_rst_after <= 1; pst <= P_SV_WR; end
					else pst <= P_RST;
				end else pcnt <= pcnt + 1'b1;
			end
			P_RST: begin
				pram_force_reset <= 1'b1; rst_hold <= 7'd127; pst <= P_IDLE;
			end
			default: pst <= P_IDLE;
			endcase
		end
	end

	////////////////////   VIDEO TIMING   ///////////////////

	// Pixel enable: the LC 12" RGB dot clock (15.6672 MHz) as a fractional
	// enable of clk_sys (32.5 MHz), Bresenham in units of 100 Hz. On MiSTer a
	// fractional enable made the scaler shake; Game Bub's framework stores
	// each frame in its own buffer first, so it does not care.
	localparam integer PIX_NUM = 156672;   // 15.6672 MHz / 100
	localparam integer PIX_DEN = 325000;   // 32.5    MHz / 100
	reg [18:0] pix_acc = 19'd0;
	reg        pix_ce  = 1'b0;
	always @(posedge clk_sys) begin
		if (pix_acc + PIX_NUM >= PIX_DEN) begin
			pix_acc <= pix_acc + PIX_NUM - PIX_DEN;
			pix_ce  <= 1'b1;
		end else begin
			pix_acc <= pix_acc + PIX_NUM;
			pix_ce  <= 1'b0;
		end
	end

	// Video-domain reset: scanout follows the machine reset (2FF kept from
	// the MiSTer design, where scanout had its own clock).
	reg vidrst_meta = 1'b1, vidrst_s = 1'b1;
	always @(posedge clk_sys) begin
		vidrst_meta <= ~n_reset;
		vidrst_s    <= vidrst_meta;
	end
	assign vid_reset = vidrst_s;

	// VBL/HBL levels for the guest-facing consumers (pseudovia VBL IRQ,
	// VIA PB7). Same clock domain now; the 2FF stages are kept so the
	// guest-visible timing matches MiSTer.
	reg vbl_meta, v8_vblank_s, hbl_meta, v8_hblank_s;
	always @(posedge clk_sys) begin
		vbl_meta    <= v8_vblank;
		v8_vblank_s <= vbl_meta;
		hbl_meta    <= v8_hblank;
		v8_hblank_s <= hbl_meta;
	end

	////////////////////   AUDIO   ///////////////////

	// ASC samples + CD audio at unity gain, saturating (see MacLC.sv).
	wire signed [15:0] cd_snd_l, cd_snd_r;
	wire signed [17:0] audio_mix_l = {{2{asc_sample_l[15]}}, asc_sample_l}
	                               + {{2{cd_snd_l[15]}}, cd_snd_l};
	wire signed [17:0] audio_mix_r = {{2{asc_sample_r[15]}}, asc_sample_r}
	                               + {{2{cd_snd_r[15]}}, cd_snd_r};
	assign audio_l = (audio_mix_l > 18'sd32767)  ? 16'sd32767 :
	                 (audio_mix_l < -18'sd32768) ? -16'sd32768 : audio_mix_l[15:0];
	assign audio_r = (audio_mix_r > 18'sd32767)  ? 16'sd32767 :
	                 (audio_mix_r < -18'sd32768) ? -16'sd32768 : audio_mix_r[15:0];

	// Mac LC memory configuration (V8 RAM config byte, MAME encoding):
	//   2MB  = $24  (2MB board, no SIMMs)
	//   10MB = $E4  (2MB board + 4MB + 4MB SIMMs => 8MB bank A)
	wire [7:0] configRAMSize = status[4] ? 8'hE4 : 8'h24;
	wire [7:0] pvia_ram_config_out;   // Active RAM config from pseudovia
	wire       pvia_ram_configured;   // ROM has programmed V8 RAM config ($0 mirror enable)

	// Serial Ports (SCC channel A = modem port, on the PMOD header)
	wire serialOut;
	wire serialIn  = uart_rxd;
	wire serialCTS = 1'b1; // Idle/deasserted when no serial device connected
	wire serialRTS;
	assign uart_txd = serialOut;

	// V8 Video system wires
	wire v8_hsync, v8_vsync, v8_hblank, v8_vblank, v8_de;
	wire v8_ce_pix;
	wire [7:0] v8_vga_r, v8_vga_g, v8_vga_b;
	wire [7:0] ariel_pixel_addr;
	wire [23:0] ariel_palette_data;
	wire [7:0] ariel_reg_dout;
	wire selectAriel;      // From address decoder
	wire selectPseudoVIA;  // From address decoder
	wire selectVRAM;       // From address decoder
	wire [7:0] pseudovia_dout;
	wire pseudovia_irq;

	// interconnects
	// CPU
	wire clk8, _cpuReset, _cpuReset_o, _cpuUDS, _cpuLDS, _cpuRW, _cpuAS;
	wire clk16_en_p, clk16_en_n;
	// V8 SCSI_PCLK / SCC RTxC source — see v8_clocks.sv.
	wire scsi_pclk_en;
	v8_clocks v8_clocks_inst (
		.clk_sys     (clk_sys),
		.reset       (~n_reset),
		.scsi_pclk_en(scsi_pclk_en)
	);
	wire _cpuVMA, _cpuVPA, _cpuDTACK;
	wire E_rising, E_falling;
	wire [2:0] _cpuIPL;       // final IPL to CPU (programmer's-switch NMI applied below)
	wire [2:0] _cpuIPL_dc;    // raw IPL from dataController (VIA1 / PseudoVIA / SCC)
	wire [2:0] cpuFC;
	wire [31:0] cpuAddr;
	assign cpuAddr[0] = 1'b0;
	wire [7:0]  cpuAddrFullHi = cpuAddr[31:24];
	wire [15:0] cpuDataOut;

	// RAM/ROM
	wire _romOE;
	wire _ramOE, _ramWE;
	wire _memoryUDS, _memoryLDS;
	wire dioBusControl;
	wire cpuBusControl;
	wire flp_guard;
	wire [22:0] memoryAddr;  // 23-bit SDRAM word address from address controller
	wire [15:0] memoryDataOut;
	wire memoryLatch;
	// peripherals
	wire pds_slot_irq = 1'b0;     // no PDS card on Game Bub
	wire pds_card_sel = 1'b0;
	wire pds_card_ack = 1'b0;
	wire [15:0] pds_dout = 16'hFFFF;
	wire vid_alt;
	wire memoryOverlayOn, selectSCSI, selectSCC, selectIWM, selectVIA, selectRAM, selectROM, selectASC, selectUnmapped;
	wire selectSCSIDMA;   // SCSI pseudo-DMA window (DACK) from address decoder
	wire scsiDREQ;        // SCSI pseudo-DMA request → gates CPU DTACK on DMA cycles
	wire scsiIRQ;         // NCR5380 latched IRQ (level)
	wire [23:0] overlay_trigger_addr;
	wire [15:0] dataControllerDataOut;

	// floppy disk image interface
	wire dskReadAckInt;
	wire [21:0] dskReadAddrInt;
	wire dskReadAckExt;
	wire [21:0] dskReadAddrExt;

	// DTACK for the immediate (non-SDRAM) paths: peripheral/unmapped space and
	// ROM-region WRITES (ack-and-discard). See MacLC.sv for the history.
	reg  dtack_en;
	always @(posedge clk_sys) begin
		if (!_cpuReset) begin
			dtack_en <= 0;
		end
		else begin
			if (_cpuAS) dtack_en <= 0;
			if (!_cpuAS & ( (!selectROM & !selectRAM & !selectVRAM)
			              | (selectROM & !_cpuRW) )) dtack_en <= 1;
		end
	end

	// FC=7 CPU space: IACK autovectors via VPA; anything else must bus-error
	// (the boot ROM's `moves.w $22000,D1` probe relies on it).
	wire        fc7_iack = (cpuFC == 3'b111) && (cpuAddr[19:16] == 4'hF);
	wire        fc7_berr = (cpuFC == 3'b111) && !fc7_iack;
	// NuBus/PDS slot space ($F1000000-$FEFFFFFF): ack with open-bus $FFFF
	// (the LBMacTwo hardware-validated empty-slot path).
	wire        slot_space = (cpuAddrFullHi >= 8'hF1) && (cpuAddrFullHi <= 8'hFE);
	// SCSI pseudo-DMA: async DTACK gated by DREQ, bus error after ~250 ms.
	localparam SDMA_TIMEOUT = 23'd8125000;  // ~250 ms @ 32.5 MHz
	reg [22:0] sdma_stall_ctr = 23'd0;
	reg        sdma_berr      = 1'b0;
	always @(posedge clk_sys) begin
		if (!_cpuReset) begin
			sdma_stall_ctr <= 0;
			sdma_berr      <= 0;
		end else if (_cpuAS) begin
			sdma_stall_ctr <= 0;
			sdma_berr      <= 0;
		end else if (selectSCSIDMA && !scsiDREQ && !sdma_berr) begin
			sdma_stall_ctr <= sdma_stall_ctr + 23'd1;
			if (sdma_stall_ctr == SDMA_TIMEOUT) sdma_berr <= 1'b1;   // held until AS deasserts
		end else if (selectSCSIDMA)
			sdma_stall_ctr <= 0;     // DREQ arrived
	end

	assign      _cpuVPA = fc7_iack ? 1'b0 : ((fc7_berr || slot_space || pds_card_sel) ? 1'b1 : ~(!_cpuAS && cpuAddr[23:21] == 3'b111 && !selectVRAM && !selectSCSIDMA));
	assign      _cpuDTACK = fc7_berr ? 1'b1 :
	                        icache_hit ? 1'b0 :        // fetch-cache hit answers now
	                        pds_card_sel ? ~pds_card_ack :
	                        (slot_space && !_cpuAS) ? 1'b0 :
	                        selectSCSIDMA ? ~scsiDREQ :
	                        // SDRAM-backed targets ack via the demand handshake
	                        (!_cpuAS && (selectRAM || selectVRAM || (selectROM && _cpuRW))) ? ~sdram_cpu_done :
	                        (~(!_cpuAS && cpuAddr[23:21] != 3'b111) | !dtack_en);

	// ── Programmer's switch / Level-7 NMI (status[5], "Interrupt") ─────────
	reg        nmi_req   = 1'b0;
	reg        nmi_btn_d = 1'b0;
	reg [15:0] nmi_to    = 16'd0;
	always @(posedge clk_sys) begin
		nmi_btn_d <= status[5];
		if (status[5] && !nmi_btn_d) begin
			nmi_req <= 1'b1;
			nmi_to  <= 16'hFFFF;
		end else if (nmi_req) begin
			if ((fc7_iack && cpuAddr[3:1] == 3'b111) || nmi_to == 16'd0)
				nmi_req <= 1'b0;
			else
				nmi_to <= nmi_to - 1'b1;
		end
	end
	assign _cpuIPL = nmi_req ? 3'b000 : _cpuIPL_dc;
	wire        cpu_en_p      = clk16_en_p;
	wire        cpu_en_n      = clk16_en_n;
	assign      _cpuReset_o   = tg68_reset_n;

	// RESET-instruction soft peripheral reset (2026-08-08 warm-restart fix).
	reg [3:0] softrst_cnt = 4'd0;
	always @(posedge clk_sys) begin
		if (!_cpuReset_o)           softrst_cnt <= 4'hF;
		else if (softrst_cnt != 0)  softrst_cnt <= softrst_cnt - 1'd1;
	end
	wire soft_periph_rst = (softrst_cnt != 0);
	assign      _cpuRW        = tg68_rw;
	assign      _cpuAS        = tg68_as_n;
	assign      _cpuUDS       = tg68_uds_n;
	assign      _cpuLDS       = tg68_lds_n;
	assign      E_falling     = tg68_E_falling;
	assign      E_rising      = tg68_E_rising;
	assign      _cpuVMA       = tg68_vma_n;
	assign      cpuFC[0]      = tg68_fc0;
	assign      cpuFC[1]      = tg68_fc1;
	assign      cpuFC[2]      = tg68_fc2;
	assign      cpuAddr[31:1] = tg68_a[31:1];
	assign      cpuDataOut    = tg68_dout;

	wire        tg68_rw;
	wire        tg68_as_n;
	wire        tg68_uds_n;
	wire        tg68_lds_n;
	wire        tg68_E_rising;
	wire        tg68_E_falling;
	wire        tg68_vma_n;
	wire        tg68_fc0;
	wire        tg68_fc1;
	wire        tg68_fc2;
	wire [15:0] tg68_dout;
	wire [31:0] tg68_a;
	wire [31:0] tg68_a_early;   // pre-AS address for the fetch cache
	wire        tg68_reset_n;
	wire        tg68_longword;   // 32-bit access flag — drives SCSI pseudo-DMA byte packing

	wire cpu_berr = (fc7_berr && !_cpuAS) || sdma_berr;

	// ── Fetch cache ─────────────────────────────────────────────────────────
	// ★ .enable MUST be a non-constant net that evaluates to 1 (see MacLC.sv
	// and MacLC_pocket docs/RESUME.md §-7): status[11] is a register bit in
	// gb_host that stays 0.
	wire        icache_hit;
	wire [15:0] icache_data;
	wire        icache_hit_now;   // per-access request-suppression verdict
	fetch_cache #(.LOG2_WORDS(9)) icache (
		.clk        ( clk_sys ),
		.reset      ( ~_cpuReset ),
		.flush_bits ( {memoryOverlayOn, dio_download} ),
		.enable     ( ~status[11] ),
		.cpuAddr    ( tg68_a_early[23:0] ),
		.as_n       ( _cpuAS ),
		.rw         ( _cpuRW ),
		.fc         ( cpuFC ),
		.cacheable  ( selectRAM || selectROM ),
		.snoopable  ( selectRAM ),
		.mem_din    ( dataControllerDataOut ),
		.hit        ( icache_hit ),
		.hit_data   ( icache_data ),
		.hit_now    ( icache_hit_now )
	);

	// Peripheral reads complete via the E-paced VPA cycle; register the read
	// data one clk_sys stage (see MacLC.sv "SCSI / peripheral read-path").
	wire vpa_periph_read = !fc7_iack && !fc7_berr && !slot_space && !_cpuAS &&
	                       (cpuAddr[23:21] == 3'b111) && !selectVRAM && !selectSCSIDMA;
	reg [15:0] periph_din_reg;
	always @(posedge clk_sys) periph_din_reg <= dataControllerDataOut;
	wire [15:0] cpu_din_muxed = pds_card_sel   ? pds_dout :
	                            slot_space     ? 16'hFFFF :
	                            icache_hit     ? icache_data :
	                            vpa_periph_read ? periph_din_reg :
	                                              dataControllerDataOut;

	tg68k tg68k (
		.clk        ( clk_sys      ),
		.reset      ( !_cpuReset ),
		.phi1       ( cpu_en_p  ),
		.phi2       ( cpu_en_n  ),
		.cpu        ( {status_cpu[1], |status_cpu} ),

		.dtack_n    ( _cpuDTACK  ),
		.rw_n       ( tg68_rw    ),
		.as_n       ( tg68_as_n  ),
		.uds_n      ( tg68_uds_n ),
		.lds_n      ( tg68_lds_n ),
		.fc         ( { tg68_fc2, tg68_fc1, tg68_fc0 } ),
		.reset_n    ( tg68_reset_n ),

		.E          (  ),
		.E_div      ( 1'b1 ),
		.E_PosClkEn ( tg68_E_falling ),
		.E_NegClkEn ( tg68_E_rising  ),
		.vma_n      ( tg68_vma_n ),
		.vpa_n      ( _cpuVPA ),

		.br_n       ( 1'b1    ),
		.bg_n       (  ),
		.bgack_n    ( 1'b1 ),
		.ipl        ( _cpuIPL ),
		.berr       ( cpu_berr ),
		.din        ( cpu_din_muxed ),
		.dout       ( tg68_dout ),
		.longword   ( tg68_longword ),
		.addr       ( tg68_a ),
		.addr_early ( tg68_a_early )
	);

	// VRAM write mirror: CPU VRAM writes are packed (stride gap removed) and
	// sent to the external SRAM; CPU VRAM reads come from the SDRAM shadow.
	wire [10:0] v8_words_per_line;
	wire [17:0] vram_bram_waddr;
	wire        vram_bram_we;
	assign vram_waddr = vram_bram_waddr;
	assign vram_wdata = memoryDataOut;
	assign vram_wbe   = {~_cpuUDS, ~_cpuLDS};
	assign vram_we    = vram_bram_we;

	wire dsk_int_ins;
	addrController_top ac0
	(
		.flp_present(dsk_int_ins),
		.clk(clk_sys),
		.clk8(clk8),
		.clk8_en_p(clk8_en_p),
		.clk8_en_n(clk8_en_n),
		.clk16_en_p(clk16_en_p),
		.clk16_en_n(clk16_en_n),
		._cpuReset(_cpuReset),
		.cpuAddr(cpuAddr),
		._cpuUDS(_cpuUDS),
		._cpuLDS(_cpuLDS),
		._cpuRW(_cpuRW),
		._cpuAS(_cpuAS),
		.cpuFC(cpuFC),
		.pds_claim(pds_card_sel),
		.ram_config(pvia_ram_config_out),
		.ram_config_phys(configRAMSize),
		.ram_configured(pvia_ram_configured),
		.memoryAddr(memoryAddr),
		.memoryLatch(memoryLatch),
		._memoryUDS(_memoryUDS),
		._memoryLDS(_memoryLDS),
		._romOE(_romOE),
		._ramOE(_ramOE),
		._ramWE(_ramWE),
		.dioBusControl(dioBusControl),
		.cpuBusControl(cpuBusControl),
		.flp_guard(flp_guard),
		.cpu_wr_ack(sdram_cpu_done),
		.dio_download(dio_download),
		.selectSCSI(selectSCSI),
		.selectSCSIDMA(selectSCSIDMA),
		.selectSCC(selectSCC),
		.selectIWM(selectIWM),
		.selectVIA(selectVIA),
		.selectASC(selectASC),
		.selectRAM(selectRAM),
		.selectROM(selectROM),
		.selectAriel(selectAriel),
		.selectPseudoVIA(selectPseudoVIA),
		.selectVRAM(selectVRAM),
		.selectUnmapped(selectUnmapped),
		.words_per_line(v8_words_per_line),
		.vram_waddr(vram_bram_waddr),
		.vram_we(vram_bram_we),
		.memoryOverlayOn(memoryOverlayOn),
		.overlay_trigger_addr(overlay_trigger_addr),

		.dskReadAddrInt(dskReadAddrInt),
		.dskReadAckInt(dskReadAckInt),
		.dskReadAddrExt(dskReadAddrExt),
		.dskReadAckExt(dskReadAckExt)
	);

	wire [1:0] diskEject;
	wire [1:0] diskMotor, diskAct;

	// 0=1bpp, 1=2bpp, 2=4bpp, 3=8bpp, 4=16bpp — set by the guest via PseudoVIA
	wire [2:0] v8_video_mode = pvia_video_config[2:0];

	// Monitor sense: 512x384 12" RGB only. The framework double-buffers the
	// frame in on-chip RAM, which has room for 512x384 but not 640x480.
	wire [3:0] v8_monitor_id = 4'h2;

	ariel_ramdac ariel(
		.clk_sys(clk_sys),
		.clk_pix(clk_sys),
		.reset(~n_reset),
		.reg_addr(cpuAddr[10:0]),
		.uds_n(_cpuUDS),
		.lds_n(_cpuLDS),
		.data_in(cpuDataOut[7:0]),
		.data_out(ariel_reg_dout),
		.we(selectAriel && !_cpuRW && cpuBusControl),
		.req(selectAriel && cpuBusControl),
		.mem_latch(memoryLatch),
		.cpu_as_n(_cpuAS),
		.pixel_index(ariel_pixel_addr),
		.rgb_out(ariel_palette_data),
		.ariel_written(ariel_written)
	);
	wire ariel_written;

	wire [7:0] pvia_video_config;
	wire [7:0] asc_data_out;
	wire asc_irq;

	pseudovia pvia(
		.clk_sys(clk_sys),
		.reset(~n_reset),
		.soft_rst(soft_periph_rst),
		.addr({cpuAddr[12:1], tg68_a[0]}),
		.data_in(cpuDataOut[7:0]),
		.data_out(pseudovia_dout),
		.we(selectPseudoVIA && !_cpuRW && cpuBusControl),
		.req(selectPseudoVIA && cpuBusControl),
		.vblank_irq(v8_vblank_s),
		.slot_irq(pds_slot_irq),
		.asc_irq(asc_irq),
		// SCSI flags tied off: MAME's LC does not connect the 5380 IRQ and
		// wiring it crashed System 7 (see MacLC.sv for the full history).
		.scsi_irq(1'b0),
		.scsi_drq(1'b0),
		.irq_out(pseudovia_irq),
		.ram_config(configRAMSize),
		.monitor_id(v8_monitor_id),
		.video_config(pvia_video_config),
		.ram_config_out(pvia_ram_config_out),
		.ram_configured(pvia_ram_configured)
	);

	maclc_v8_video v8_video(
		.clk_sys(clk_sys),
		.clk8_en_p(clk8_en_p),
		.pix_ce(pix_ce),
		.reset(vidrst_s),

		.video_mode(v8_video_mode),
		.monitor_id(v8_monitor_id),

		.test_bypass_vram(1'b0),
		.test_pattern_sel(2'b00),

		.hsync(v8_hsync),
		.vsync(v8_vsync),
		.hblank(v8_hblank),
		.vblank(v8_vblank),
		.vga_r(v8_vga_r),
		.vga_g(v8_vga_g),
		.vga_b(v8_vga_b),
		.de(v8_de),
		.ce_pix(v8_ce_pix),

		.palette_addr(ariel_pixel_addr),
		.palette_data(ariel_palette_data),

		.words_per_line(v8_words_per_line),
		.vram_raddr(vram_raddr),
		.vram_rd(vram_rd),
		.vram_rready(vram_rready),
		.vram_rvalid(vram_rvalid),
		.vram_rdata(vram_rdata)
	);

	// Pixel strobe aligned with vga_r/g/b and de: pix_ce advances h_count on
	// edge E0; the palette read lands at E1 and the RGB/DE output registers
	// at E2, so the pixel is valid in the cycle after E2 — pix_ce delayed by
	// three registers. pix_ce never repeats within two cycles, so each pixel
	// stays valid for at least two cycles and the strobe sits inside it.
	reg [2:0] pix_ce_d = 3'b000;
	always @(posedge clk_sys) pix_ce_d <= {pix_ce_d[1:0], pix_ce};
	assign vid_r       = v8_vga_r;
	assign vid_g       = v8_vga_g;
	assign vid_b       = v8_vga_b;
	assign vid_de      = v8_de;
	assign vid_vblank  = v8_vblank;
	assign vid_pix_stb = pix_ce_d[2];

	// ASC sample outputs
	wire signed [15:0] asc_sample_l;
	wire signed [15:0] asc_sample_r;
	wire               asc_sample_tick;

	asc asc_inst(
		.clk(clk_sys),
		.reset(~n_reset || soft_periph_rst),
		.cs(selectASC),
		.addr({cpuAddr[11:1], tg68_a[0]}),
		.data_in(cpuDataOut),
		.data_out(asc_data_out),
		.we(!_cpuRW && cpuBusControl),
		.cpu_as_n(_cpuAS),
		.uds_n(_cpuUDS),
		.lds_n(_cpuLDS),
		.sample_l(asc_sample_l),
		.sample_r(asc_sample_r),
		.sample_tick(asc_sample_tick),
		.irq(asc_irq)
	);

	// Forward declarations (assigned further down).
	wire [15:0] extra_rom_data_demux;
	wire flp_int_wp;
	wire [21:0] wc_addr;      // image BYTE offset from the committer
	wire [15:0] wc_data;
	wire        wc_req, wc_done;
	wire        wc_ack;
	wire [21:0] wc_commit_addr;
	wire  [7:0] wc_buf_addr;
	wire [15:0] wc_buf_data;
	wire        wc_buf_wr;

	dataController_top dataController (
		.clk32(clk_sys),
		.clk8_en_p(clk8_en_p),
		.clk8_en_n(clk8_en_n),
		.scsi_pclk_en(scsi_pclk_en),
		.E_rising(E_rising),
		.E_falling(E_falling),
		._systemReset(n_reset),
		.softRst(soft_periph_rst),
		.pseudovia_irq(pseudovia_irq),
		._cpuReset(_cpuReset),
		._cpuIPL(_cpuIPL_dc),
		._cpuUDS(_cpuUDS),
		._cpuLDS(_cpuLDS),
		._cpuRW(_cpuRW),
		._cpuVMA(_cpuVMA),
		.cpuDataIn(cpuDataOut),
		.cpuDataOut(dataControllerDataOut),
		.cpuAddrRegHi(cpuAddr[12:9]),
		.cpuAddrRegMid(cpuAddr[6:4]),  // for SCSI register select (A6-A4)
		.cpuAddrRegLo(cpuAddr[2:1]),
		.cpuLongword(tg68_longword),
		.selectSCSI(selectSCSI),
		.selectSCSIDMA(selectSCSIDMA),
		.scsiDREQ(scsiDREQ),
		.scsiIRQ(scsiIRQ),
		.dbg_scsi(),
		.dbg_scsi2(),
		.dbg_scsi4(),
		.dbg_scsi5(),
		.dbg_ncr(),
		.dbg_cda0(),
		.dbg_cda1(),
		.dbg_cda2(),
		.dbg_cda3(),
		.dbg_cda4(),
		.dbg_cdur(),
		.dbg_ncr2(),
		.dbg_wr(),
		.dbg_wrfb(),
		.dbg_ism_flpe(),
		.dbg_ring0(),
		.dbg_ring1(),
		.selectSCC(selectSCC),
		.selectIWM(selectIWM),
		.selectVIA(selectVIA),
		.selectASC(selectASC),
		.asc_data_in(asc_data_out),
		.cpuBusControl(cpuBusControl),
		.memoryDataOut(memoryDataOut),
		.memoryDataIn(sdram_do),
		.dskReadDataIn(extra_rom_data_demux[7:0]),
		.memoryLatch(memoryLatch),
		.selectAriel(selectAriel),
		.ariel_data_in(ariel_reg_dout),
		.selectPseudoVIA(selectPseudoVIA),
		.pseudovia_data_in(pseudovia_dout),
		.selectUnmapped(selectUnmapped),

		// peripherals
		.ps2_key(ps2_key),
		.capslock(),
		.ps2_mouse(ps2_mouse),
		// serial uart
		.serialIn(serialIn),
		.serialOut(serialOut),
		.serialCTS(serialCTS),
		.serialRTS(serialRTS),

		// rtc unix ticks
		.timestamp(TIMESTAMP),

		// video
		._hblank(~v8_hblank_s),
		._vblank(~v8_vblank_s),
		.vid_alt(vid_alt),

		// floppy disk interface. Drive 1 (external) never has media.
		.insertDisk({1'b0, dsk_int_ins}),
		.writeProtect({1'b1, flp_int_wp}),
		.wrSdAddr(wc_addr), .wrSdData(wc_data),
		.wrSdReq(wc_req),   .wrSdAck(wc_ack),
		.wrCommitDone(wc_done),
		.wrCommitAddr(wc_commit_addr),
		.wrSdBufAddr(wc_buf_addr), .wrSdBufData(wc_buf_data), .wrSdBufWr(wc_buf_wr),
		.diskSides({1'b0, dsk_int_ds}),
		.mediaSides({1'b1, flp_int_media_ds}),
		.diskMFM({1'b0, dsk_int_mfm}),
		.diskHD({1'b0, dsk_int_hd}),
		.diskEject(diskEject),
		.dskReadAddrInt(dskReadAddrInt),
		.dskReadAckInt(dskReadAckInt),
		.dskReadAddrExt(dskReadAddrExt),
		.dskReadAckExt(dskReadAckExt),
		.diskMotor(diskMotor),
		.diskAct(diskAct),

		// block device interface for scsi disk (slots 0,1)
		.img_mounted(img_mounted[SCSI_DEVS-1:0]),
		.img_size(img_size[40:9]),
		.io_lba(scsi_lba),
		.io_rd(scsi_rd),
		.io_wr(scsi_wr),
		.io_ack(scsi_ack),

		.sd_buff_addr(sd_buff_addr[7:0]),
		.sd_buff_addr_hi(sd_buff_addr[12:8]),
		.sd_buff_dout(sd_buff_dout),
		.sd_buff_din(scsi_buff_din),
		.sd_buff_wr(sd_buff_wr),

		// BlueSCSI Toolbox dedicated transport (slot VD_TOOLBOX).
		.tb_mounted(tb_mounted),
		.tb_lba(tb_lba),
		.tb_rd(tb_rd),
		.tb_wr(tb_wr),
		.tb_ack(tb_ack),
		.tb_buff_din(tb_buff_din),

		// BlueSCSI Toolbox CD Changer transport (slot VD_CD_TOOLBOX).
		.cdtb_mounted(cdtb_mounted),
		.cdtb_lba(cdtb_lba),
		.cdtb_rd(cdtb_rd),
		.cdtb_wr(cdtb_wr),
		.cdtb_ack(cdtb_ack),
		.cdtb_buff_din(cdtb_buff_din),
		.cd_snd_l(cd_snd_l),
		.cd_snd_r(cd_snd_r),

		// CD-ROM target (SCSI ID 3) block interface (slot VD_CDROM).
		.cd_enable(cd_enable),
		.cd_img_mounted(cd_mounted),
		.cd_io_lba(cd_lba),
		.cd_io_rd(cd_rd),
		.cd_io_wr(),           // read-only target: never writes
		.cd_io_ack(cd_ack),
		.cd_sd_buff_din(cd_buff_din),

		// PRAM persistence (NVRAM) — driven by the FSM above
		.pram_load_wr(pram_load_wr),
		.pram_load_addr(pram_load_addr),
		.pram_load_data(pram_load_data),
		.pram_save_addr(pram_save_addr),
		.pram_save_data(pram_save_data),
		.pram_wr_stb(pram_wr_stb),
		.pram_ready(pram_ready),
		.egret_dbg_reset_680x0(egret_reset_680x0_w),
		.dbg_flp_byte_cnt(),
		.dbg_flp_miss_cnt(),
		.dbg_flp_disk_data(),
		.dbg_flp_track(),
		.dbg_flp_side(),
		.dbg_flp_step_cnt(),
		.dbg_iwm_latch(),
		.dbg_flp_byte_stb(),
		.dbg_flp_raw(),
		.dbg_flp_gcr_addr(),
		.dbg_ism_verdict(),
		.dbg_ism_unrlatch(),
		.dbg_ism_scan(),
		.dbg_mfm_stall(),
		.dbg_ism_state(),
		.dbg_flp_strb_cnt(),
		.dbg_flp_strb_en_cnt(),
		.dbg_flp_strb_last(),
		.dbg_flp_rej_step(),
		.dbg_flp_status(),
		.dbg_flp_media()
	);

	//////////////////////// FLOPPY ///////////////////////////

	reg dsk_int_ds;
	reg dsk_int_ss;   // single sided image inserted
	reg dsk_int_mfm;  // MFM-format image (ISM/SWIM path): 720K or 1.44MB
	reg dsk_int_hd;   // 1.44MB HD (vs 720K DD)

	// Disk CHANGE must be presented as a TRANSITION: hold the drive EMPTY
	// from mount until DSK_EMPTY_CY (2.06 s) after the load completes.
	localparam [25:0] DSK_EMPTY_CY = 26'h3FFFFFF;
	reg [25:0] dsk_int_empty_cy;
	wire dsk_int_empty = (dsk_int_empty_cy != DSK_EMPTY_CY);
	assign dsk_int_ins = !dsk_int_empty && (dsk_int_ds || dsk_int_ss || dsk_int_mfm);

	wire        flp_int_loading, flp_int_done, flp_int_dc42, flp_int_raw, flp_int_ro;
	wire        flp_int_media_ds;
	wire [63:0] flp_int_size;
	wire  [7:0] flp_int_fmt;
	wire [31:0] flp_ldr_lba, flp_sdw_lba;
	wire        flp_ldr_rd;
	wire  [5:0] flp_hdr_addr;
	wire [15:0] flp_hdr_data;
	reg  flp_eject_d;
	wire flp_eject_pulse = diskEject[0] && !flp_eject_d;
	always @(posedge clk_sys) flp_eject_d <= diskEject[0];
	wire [63:0] flp_file_bytes  = flp_int_size + (flp_int_dc42 ? 64'd84 : 64'd0);
	wire [12:0] flp_file_blocks = flp_file_bytes[21:9];
	wire        flp_file_tail   = |flp_file_bytes[8:0];
	wire [23:0] flp_int_wr_addr;
	wire [15:0] flp_int_wr_data;
	wire        flp_int_wr_req;
	wire        flp_int_wr_ack;
	wire        flp_loading = flp_int_loading;

	floppy_loader floppy_loader_int
	(
		.clk_sys(clk_sys), .reset(!pll_locked_s),
		.img_mounted (img_mounted[VD_FLOPPY_INT]),
		.img_size    (img_size),
		.img_readonly(img_readonly),
		.sd_lba(flp_ldr_lba), .sd_rd(flp_ldr_rd),
		.sd_ack(sd_ack[VD_FLOPPY_INT]),
		.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_wr(sd_buff_wr),
		.base_addr(24'h600000),
		.wr_addr(flp_int_wr_addr), .wr_data(flp_int_wr_data),
		.wr_req(flp_int_wr_req),   .wr_ack(flp_int_wr_ack),
		.loading(flp_int_loading), .done(flp_int_done), .size(flp_int_size),
		.readonly(flp_int_ro), .raw_img(flp_int_raw),
		.is_dc42(flp_int_dc42),
		.media_ds(flp_int_media_ds),
		.hdr_addr(flp_hdr_addr), .hdr_data(flp_hdr_data),
		.dc42_fmt(flp_int_fmt)
	);

	// Write-protect: the "Floppy Write" option is off, or the image is read-only.
	assign flp_int_wp = ~status[14] || flp_int_ro;

	wire        flp_int_sdw_busy;
	wire        flp_sdw_wr;
	wire        flp_mem_req, flp_mem_ack;
	wire [23:0] flp_mem_addr;
	wire [15:0] flp_mem_dout;
	floppy_sd_writer floppy_sd_writer_int
	(
		.clk             ( clk_sys ),
		.reset           ( !pll_locked_s ),
		.img_mounted     ( img_mounted[VD_FLOPPY_INT] ),

		.commit_done     ( wc_done ),
		.commit_addr     ( wc_commit_addr ),

		.write_ok        ( ~flp_int_wp ),
		.loader_busy     ( flp_int_loading ),
		.dc42            ( flp_int_dc42 ),
		.flush_req       ( flp_eject_pulse ),
		.file_blocks     ( flp_file_blocks ),
		.file_tail       ( flp_file_tail ),

		.img_base        ( 24'h600000 ),
		.hdr_addr        ( flp_hdr_addr ),
		.hdr_data        ( flp_hdr_data ),

		.mem_req         ( flp_mem_req ),
		.mem_addr        ( flp_mem_addr ),
		.mem_ack         ( flp_mem_ack ),
		.mem_dout        ( flp_mem_dout ),

		.sd_lba          ( flp_sdw_lba ),
		.sd_wr           ( flp_sdw_wr ),
		.sd_ack          ( sd_ack[VD_FLOPPY_INT] ),
		.sd_buff_addr_i  ( sd_buff_addr ),
		.sd_buff_din     ( sd_buff_din[VD_FLOPPY_INT] ),
		.busy            ( flp_int_sdw_busy )
	);

	// The controller's DMA ("eth") port, shared: the platform client (host
	// file transfers + block devices, priority — it took pds_enet's place)
	// and the floppy writer.
	wire        mem_eth_req, mem_eth_we, mem_eth_ack;
	wire [23:0] mem_eth_addr;
	wire [15:0] mem_eth_din, mem_eth_dout;
	gb_eth_arb eth_port_arb_i
	(
		.clk    ( clk_sys ),
		.reset  ( !pll_locked_s ),
		.a_req  ( pm_req  ), .a_we ( pm_we ),   .a_addr ( pm_addr ),
		.a_din  ( pm_din  ), .a_ack ( pm_ack ), .a_dout ( pm_dout ),
		.b_req  ( flp_mem_req  ), .b_we ( 1'b0 ),       .b_addr ( flp_mem_addr ),
		.b_din  ( 16'd0 ),        .b_ack ( flp_mem_ack ), .b_dout ( flp_mem_dout ),
		.m_req  ( mem_eth_req  ), .m_we ( mem_eth_we ), .m_addr ( mem_eth_addr ),
		.m_din  ( mem_eth_din  ), .m_ack ( mem_eth_ack ), .m_dout ( mem_eth_dout )
	);

	assign sd_lba[VD_FLOPPY_INT] = flp_int_loading ? flp_ldr_lba : flp_sdw_lba;
	assign sd_rd [VD_FLOPPY_INT] = flp_ldr_rd;
	assign sd_wr [VD_FLOPPY_INT] = flp_int_loading ? 1'b0        : flp_sdw_wr;

	always @(posedge clk_sys) begin
		if(img_mounted[VD_FLOPPY_INT]) begin
			dsk_int_ds  <= 0;
			dsk_int_ss  <= 0;
			dsk_int_mfm <= 0;
			dsk_int_hd  <= 0;
			dsk_int_empty_cy <= 26'd0;
		end
		else if(flp_int_loading)
			dsk_int_empty_cy <= 26'd0;
		else if(dsk_int_empty_cy != DSK_EMPTY_CY)
			dsk_int_empty_cy <= dsk_int_empty_cy + 26'd1;

		if(flp_int_done) begin
			dsk_int_ds  <= flp_int_dc42 ? (flp_int_fmt == 8'd1) : (flp_int_size == 64'd819200);
			dsk_int_ss  <= flp_int_dc42 ? (flp_int_fmt == 8'd0) : (flp_int_size == 64'd409600);
			dsk_int_mfm <= flp_int_dc42 ? (flp_int_fmt == 8'd2 || flp_int_fmt == 8'd3)
			                            : (flp_int_size == 64'd737280 || flp_int_size == 64'd1474560);
			dsk_int_hd  <= flp_int_dc42 ? (flp_int_fmt == 8'd3) : (flp_int_size == 64'd1474560);
		end

		if(diskEject[0]) begin
			dsk_int_ds <= 0;
			dsk_int_ss <= 0;
			dsk_int_mfm <= 0;
			dsk_int_hd <= 0;
		end
	end

	// -- SDRAM download-port arbitration (floppy loader vs. write committer).
	// The ROM no longer uses this port (gb_host writes it through pm_*), so
	// during dio_download the port simply idles.
	localparam DLG_LDR = 1'b0;   // the mount-time loader
	localparam DLG_WC  = 1'b1;   // the write committer
	reg  dl_grant;
	reg  dl_locked;
	always @(posedge clk_sys) begin
		if (!pll_locked_s) begin
			dl_grant  <= DLG_LDR;
			dl_locked <= 1'b0;
		end else if (dio_download) begin
			dl_locked <= 1'b0;
		end else if (dl_locked) begin
			if ((dl_grant == DLG_LDR && !flp_int_wr_req) ||
			    (dl_grant == DLG_WC  && !wc_req))
				dl_locked <= 1'b0;
		end else if (flp_int_wr_req) begin
			dl_grant <= DLG_LDR; dl_locked <= 1'b1;
		end else if (wc_req) begin
			dl_grant <= DLG_WC;  dl_locked <= 1'b1;
		end
	end

	wire dl_ldr_sel = dl_locked && (dl_grant == DLG_LDR) && !dio_download;
	wire dl_wc_sel  = dl_locked && (dl_grant == DLG_WC)  && !dio_download;

	wire [23:0] wc_word_addr = 24'h600000 + {3'd0, wc_addr[21:1]};

	wire        dl_req_mux  = dio_download ? 1'b0 :
	                          dl_wc_sel    ? wc_req     : flp_int_wr_req;
	wire [23:0] dl_addr_mux = dl_wc_sel    ? wc_word_addr        : flp_int_wr_addr;
	wire [15:0] dl_data_mux = dl_wc_sel    ? wc_data  : flp_int_wr_data;

	wire        sdram_dl_ack;
	assign flp_int_wr_ack = sdram_dl_ack && dl_ldr_sel;
	assign wc_ack         = sdram_dl_ack && dl_wc_sel;

	////////////////////////// SDRAM /////////////////////////////////

	wire [24:0] sdram_addr = {2'b00, memoryAddr[22:0]};
	wire [15:0] sdram_din  = memoryDataOut;
	wire  [1:0] sdram_ds   = { !_memoryUDS, !_memoryLDS };
	wire        sdram_we   = !_ramWE;
	// oe is PURE CPU read intent; a fetch-cache hit never starts a transaction.
	wire        sdram_oe   = (!_ramOE || !_romOE) && !icache_hit_now;
	wire [15:0] sdram_do   = cpu_dout_patched;
	// Floppy image byte demux (2 bytes per SDRAM word, even byte high).
	wire dsk_byte_odd = dskReadAckExt ? dskReadAddrExt[0] : dskReadAddrInt[0];
	reg  sdram_dskodd_q;
	always @(posedge clk_sys) sdram_dskodd_q <= dsk_byte_odd;
	assign extra_rom_data_demux = sdram_dskodd_q?
							 {sdram_out[7:0],sdram_out[7:0]}:{sdram_out[15:8],sdram_out[15:8]};
	wire [15:0] sdram_out;

	wire [15:0] sdram_cpu_dout;
	wire        sdram_cpu_done;
	wire [15:0] cpu_dout_patched = sdram_cpu_dout;

	// Pipeline the SDRAM request bundle one clk_sys stage (see MacLC.sv
	// "Phase C fix (2026-08-18)").
	reg [24:0] sdram_addr_q;
	reg [15:0] sdram_din_q;
	reg  [1:0] sdram_ds_q;
	reg        sdram_we_q, sdram_oe_q;
	reg        sdram_flpwin_q, sdram_flpguard_q;
	reg        sdram_dlreq_q, sdram_dlslot_q;
	reg [23:0] sdram_dladdr_q;
	reg [15:0] sdram_dldin_q;
	always @(posedge clk_sys) begin
		sdram_addr_q     <= sdram_addr;
		sdram_din_q      <= sdram_din;
		sdram_ds_q       <= sdram_ds;
		sdram_we_q       <= sdram_we;
		sdram_oe_q       <= sdram_oe;
		sdram_flpwin_q   <= (dskReadAckInt || dskReadAckExt) && !dio_download && !flp_loading;
		sdram_flpguard_q <= flp_guard && !dio_download && !flp_loading;
		sdram_dlreq_q    <= dl_req_mux;
		sdram_dlslot_q   <= dioBusControl;
		sdram_dladdr_q   <= dl_addr_mux;
		sdram_dldin_q    <= dl_data_mux;
	end

	// Content-preserving SDRAM re-init on explicit user resets (see MacLC.sv).
	reg  [3:0] sdram_reinit_cnt = 4'd0;
	reg        user_reset_d = 1'b0;
	wire       user_reset_now = status[0] | pram_force_reset;
	always @(posedge clk_sys) begin
		user_reset_d <= user_reset_now;
		if (user_reset_now && !user_reset_d && rom_loaded && !dio_download)
			sdram_reinit_cnt <= 4'd15;
		else if (sdram_reinit_cnt != 0)
			sdram_reinit_cnt <= sdram_reinit_cnt - 4'd1;
	end
	wire sdram_reinit = (sdram_reinit_cnt != 0);

	sdram sdram
	(
		.init           ( !pll_locked || sdram_reinit ),
		.clk_64         ( clk_mem                  ),
		.clk_8          ( clk8                     ),

		.sd_clk         ( sdram_clk                ),
		.sd_data        ( sdram_dq_out             ),
		.sd_data_in     ( sdram_dq_in              ),
		.sd_data_oe     ( sdram_dq_oe              ),
		.sd_addr        ( sdram_a                  ),
		.sd_dqm         ( sdram_dqm                ),
		.sd_cs          ( sdram_cs_n               ),
		.sd_ba          ( sdram_ba                 ),
		.sd_we          ( sdram_we_n               ),
		.sd_ras         ( sdram_ras_n              ),
		.sd_cas         ( sdram_cas_n              ),

		.din            ( sdram_din_q              ),
		.addr           ( sdram_addr_q[23:0]       ),
		.ds             ( sdram_ds_q               ),
		.we             ( sdram_we_q               ),
		.oe             ( sdram_oe_q               ),
		.dout           ( sdram_out                ),

		.flp_win        ( (dskReadAckInt || dskReadAckExt) && !dio_download ),
		.flp_addr       ( sdram_addr[23:0] ),
		.flp_guard      ( sdram_flpguard_q         ),

		.dl_req         ( sdram_dlreq_q            ),
		.dl_slot        ( sdram_dlslot_q           ),
		.dl_addr        ( sdram_dladdr_q           ),
		.dl_din         ( sdram_dldin_q            ),
		.dl_ack         ( sdram_dl_ack             ),

		.eth_req        ( mem_eth_req              ),
		.eth_we         ( mem_eth_we               ),
		.eth_addr       ( mem_eth_addr             ),
		.eth_din        ( mem_eth_din              ),
		.eth_ack        ( mem_eth_ack              ),
		.eth_dout       ( mem_eth_dout             ),

		.cpu_done       ( sdram_cpu_done           ),
		.cpu_dout       ( sdram_cpu_dout           )
	);

	//////////////////////// DEBUG ///////////////////////////

	assign dbg_cpu_addr    = cpuAddr;
	assign dbg_cpu_reset_n = _cpuReset;
	assign dbg_disk_act    = diskAct;

endmodule

`default_nettype wire
