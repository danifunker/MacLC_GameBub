// ============================================================================
// tb_gamebub.sv — boot the whole Game Bub core in Verilator, with a model of
// the Game Bub MCU on the host interface.
//
//   sim/run_tb.sh [plusargs]           (builds, then runs; see that script)
//
// What it models:
//   * the framework's HostV0 memory bus exactly as framework/.../
//     SpiReceiverFifo.scala drives it (enable held until done; the request
//     is popped on the edge that sees done) and the command handshake;
//   * the MCU's core-load sequence from docs.gamebub.net: FileWriteStart /
//     words / FileWriteEnd per file, SetupComplete, CoreRun;
//   * a W9825G6KH SDRAM (CAS latency 2, single-word bursts) and the
//     IS61WV25616 async SRAM;
//   * the framework's frame capture: pixels on video_dataEnable, rows on
//     video_hblank, frames on video_vblank -> sim_out/frame_NNNN.ppm.
//
// Plusargs:
//   +rom=<file>     Mac LC ROM (default ../MacLC_MiSTer/releases/boot0.rom)
//   +hd=<file>      hard disk image (optional)
//   +floppy=<file>  floppy image (optional)
//   +nvr=<file>     PRAM image (optional; otherwise 512 x 0xFF like the MCU)
//   +host_be        pack file bytes big-endian (default: little-endian)
//   +ram=2|10       memory setting (default: the core's default, 10 MB)
//   +floppy_write   enable guest floppy writes
//   +kbd_mode       start the pad in keyboard mode
//   +frames=<n>     stop after n frames (default 400)
//   +dump=<k>       write every k-th frame (default 50)
//   +out=<dir>      where frames go (default sim_out; must exist)
//   +odd_pixels     report 1bpp pixels that are neither black nor white
// ============================================================================
`timescale 1ps/1ps
`default_nettype none

module tb_gamebub;

	// -------------------------------------------------------------------
	// DUT
	// -------------------------------------------------------------------
	reg  clk50 = 1'b0;
	always #10000 clk50 = ~clk50;

	wire clk_sys, locked;
	reg  fw_reset = 1'b1;           // framework reset (pll_reset_generator)

	reg         h_en = 1'b0, h_wr = 1'b0;
	reg  [31:0] h_addr = 32'd0, h_wdata = 32'd0;
	wire [31:0] h_rdata;
	wire        h_done;
	reg         c_req = 1'b0;
	wire        c_busy, c_done, c_error;

	wire [2:0]  v_r, v_g, v_b;
	wire        v_de, v_vblank, v_hblank;
	wire [15:0] a_left, a_right;

	wire        sd_clk, sd_cke, sd_cs, sd_ras, sd_cas, sd_we, sd_dir;
	wire [1:0]  sd_dqm, sd_ba;
	wire [12:0] sd_a;
	wire [15:0] sd_dout;
	reg  [15:0] sd_din = 16'd0;

	wire        sr_ce_n, sr_we_n, sr_oe_n, sr_dir;
	wire [1:0]  sr_mask_n;
	wire [17:0] sr_a;
	wire [15:0] sr_dout;
	wire [15:0] sr_din;

	wire [3:0]  pmod_out, pmod_dir;

	maclc_gamebub #(.BUILD_UNIX_TIME(32'd1790000000)) dut (
		.clock                    (clk_sys),
		.reset                    (fw_reset),
		.clocks_clockIn50M        (clk50),
		.clocks_clockOutSystem    (clk_sys),
		.clocks_clockOutDisplay   (),
		.clocks_clockOutSpi       (),
		.clocks_locked            (locked),
		.video_data_r             (v_r),
		.video_data_g             (v_g),
		.video_data_b             (v_b),
		.video_dataEnable         (v_de),
		.video_vblank             (v_vblank),
		.video_hblank             (v_hblank),
		.audio_left               (a_left),
		.audio_right              (a_right),
		.host_mem_enable          (h_en),
		.host_mem_write           (h_wr),
		.host_mem_done            (h_done),
		.host_mem_address         (h_addr),
		.host_mem_dataRead        (h_rdata),
		.host_mem_dataWrite       (h_wdata),
		.host_commandHost_request (c_req),
		.host_commandHost_busy    (c_busy),
		.host_commandHost_done    (c_done),
		.host_commandHost_error   (c_error),
		.host_commandCore_request (),
		.host_commandCore_busy    (1'b0),
		.host_commandCore_done    (1'b0),
		.host_commandCore_error   (1'b0),
		.input_buttons_a          (1'b0),
		.input_buttons_b          (1'b0),
		.input_buttons_x          (1'b0),
		.input_buttons_y          (1'b0),
		.input_buttons_up         (1'b0),
		.input_buttons_down       (1'b0),
		.input_buttons_left       (1'b0),
		.input_buttons_right      (1'b0),
		.input_buttons_l          (1'b0),
		.input_buttons_r          (1'b0),
		.input_buttons_start      (1'b0),
		.input_buttons_select     (1'b0),
		.sdram_clock              (sd_clk),
		.sdram_cke                (sd_cke),
		.sdram_cs                 (sd_cs),
		.sdram_ras                (sd_ras),
		.sdram_cas                (sd_cas),
		.sdram_we                 (sd_we),
		.sdram_dqm                (sd_dqm),
		.sdram_bank               (sd_ba),
		.sdram_address            (sd_a),
		.sdram_dataIn             (sd_din),
		.sdram_dataOut            (sd_dout),
		.sdram_dataDir            (sd_dir),
		.sram_ceN                 (sr_ce_n),
		.sram_weN                 (sr_we_n),
		.sram_oeN                 (sr_oe_n),
		.sram_writeMaskN          (sr_mask_n),
		.sram_address             (sr_a),
		.sram_dataIn              (sr_din),
		.sram_dataOut             (sr_dout),
		.sram_dataDir             (sr_dir),
		.pmod_in                  (4'b0010),   // modem RxD idles high
		.pmod_out                 (pmod_out),
		.pmod_dir                 (pmod_dir)
	);

	// Framework reset: held until the core's MMCM locks, released in clk_sys.
	reg [3:0] lock_cnt = 4'd0;
	always @(posedge clk_sys) begin
		if (!locked) lock_cnt <= 4'd0;
		else if (lock_cnt != 4'hF) lock_cnt <= lock_cnt + 4'd1;
		fw_reset <= (lock_cnt != 4'hF);
	end

	// -------------------------------------------------------------------
	// SDRAM: W9825G6KH, 4 banks x 8192 rows x 512 columns x 16 bits.
	// The array is indexed by the controller's own word address
	// {row12, col8, bank, row11:0, col7:0} so it can be peeked directly.
	// -------------------------------------------------------------------
	reg [15:0] sdram [0:(1<<24)-1];
	reg [12:0] open_row [0:3];
	reg [23:0] rd_addr_q;
	reg [1:0]  rd_lat = 2'd0;
	wire [3:0] sd_cmd = {sd_cs, sd_ras, sd_cas, sd_we};
	function [23:0] sd_word(input [1:0] ba, input [12:0] row, input [8:0] col);
		sd_word = {row[12], col[8], ba, row[11:0], col[7:0]};
	endfunction
	always @(posedge sd_clk) begin
		case (sd_cmd)
			4'b0011: open_row[sd_ba] <= sd_a;                                  // ACTIVE
			4'b0101: begin rd_addr_q <= sd_word(sd_ba, open_row[sd_ba], sd_a[8:0]); rd_lat <= 2'd2; end  // READ
			4'b0100: begin                                                     // WRITE
				if (!sd_dqm[1]) sdram[sd_word(sd_ba, open_row[sd_ba], sd_a[8:0])][15:8] <= sd_dout[15:8];
				if (!sd_dqm[0]) sdram[sd_word(sd_ba, open_row[sd_ba], sd_a[8:0])][7:0]  <= sd_dout[7:0];
				if (!sd_dir) $display("TB ERROR: SDRAM WRITE with data bus not driven @%0t", $time);
			end
			default: ;
		endcase
		if (rd_lat != 2'd0) begin
			rd_lat <= rd_lat - 2'd1;
			if (rd_lat == 2'd1) sd_din <= sdram[rd_addr_q];   // CL2: valid after the 2nd edge
		end
	end

	// -------------------------------------------------------------------
	// Async SRAM: IS61WV25616, 256K x 16
	// -------------------------------------------------------------------
	reg [15:0] sram [0:(1<<18)-1];
	assign sr_din = (!sr_ce_n && !sr_oe_n && sr_we_n) ? sram[sr_a] : 16'hDEAD;
	always @(posedge sr_we_n) if (!sr_ce_n) begin                          // write ends on WE rise
		if (!sr_dir) $display("TB ERROR: SRAM write with data bus not driven @%0t", $time);
		if (!sr_mask_n[1]) sram[sr_a][15:8] <= sr_dout[15:8];
		if (!sr_mask_n[0]) sram[sr_a][7:0]  <= sr_dout[7:0];
	end
	integer sram_writes = 0;
	always @(posedge sr_we_n) if (!sr_ce_n) sram_writes = sram_writes + 1;
	integer palette_writes = 0;
	always @(posedge clk_sys) if (dut.mac.selectAriel && !dut.mac._cpuRW && !dut.mac._cpuAS && dut.mac.cpuBusControl)
		palette_writes = palette_writes + 1;
	always @(posedge clk_sys) if (sr_dir && !sr_oe_n)
		$display("TB ERROR: SRAM bus contention (FPGA driving with OE low) @%0t", $time);

	// -------------------------------------------------------------------
	// Host (MCU) model
	// -------------------------------------------------------------------
	reg  [7:0]  filebuf [0:(1<<24)-1];
	bit         host_be = 1'b0;
	integer     xfers = 0;

	// One HostV0 memory transaction, shaped like SpiReceiverFifo: enable is
	// held, the first cycle is never a completion, and the transaction ends
	// on the edge that samples done high.
	task automatic mem_xfer(input bit wr, input [31:0] a, input [31:0] d, output [31:0] q);
		h_en = 1'b1; h_wr = wr; h_addr = a; h_wdata = d;
		@(posedge clk_sys);
		forever begin
			@(posedge clk_sys);
			if (h_done) break;
		end
		q    = h_rdata;
		h_en = 1'b0;
		xfers = xfers + 1;
	endtask

	task automatic mem_write(input [31:0] a, input [31:0] d);
		reg [31:0] dummy;
		mem_xfer(1'b1, a, d, dummy);
	endtask

	task automatic mem_read(input [31:0] a, output [31:0] q);
		mem_xfer(1'b0, a, 32'd0, q);
	endtask

	task automatic host_cmd(input [15:0] op, input [31:0] p1, input [31:0] p2,
	                        input [31:0] p3, output [31:0] r0);
		mem_write(32'hF000_0000, {16'd0, op});
		mem_write(32'hF000_0004, p1);
		mem_write(32'hF000_0008, p2);
		mem_write(32'hF000_000C, p3);
		c_req = 1'b1;
		forever begin
			@(posedge clk_sys);
			if (c_done || c_error) break;
		end
		if (c_error) $display("TB: command %04h returned ERROR", op);
		mem_read(32'hF000_0000, r0);
		c_req = 1'b0;
		forever begin
			@(posedge clk_sys);
			if (!c_done && !c_error) break;
		end
	endtask

	// Write a file the way the MCU does: 4 bytes per 32-bit word.
	task automatic load_file(input [15:0] id, input string path, input [31:0] addr,
	                         input integer fill_ff, input integer fill_len, output integer len);
		integer fd, n, i;
		reg [31:0] r, w;
		len = 0;
		if (path != "") begin
			fd = $fopen(path, "rb");
			if (fd == 0) begin
				$display("TB: cannot open %s", path);
				$finish;
			end
			len = $fread(filebuf, fd, 0, 1 << 24);
			$fclose(fd);
		end else if (fill_ff) begin
			len = fill_len;
			for (i = 0; i < len; i = i + 1) filebuf[i] = 8'hFF;
		end
		$display("TB: file %0d: %0d bytes -> host 0x%08h (%s)", id, len, addr, path == "" ? "0xFF fill" : path);
		host_cmd(16'h0300, {16'd0, id}, 32'd0, 32'd0, r);
		for (n = 0; n < len; n = n + 4) begin
			if (host_be) w = {filebuf[n], filebuf[n+1], filebuf[n+2], filebuf[n+3]};
			else         w = {filebuf[n+3], filebuf[n+2], filebuf[n+1], filebuf[n]};
			mem_write(addr + n, w);
		end
		host_cmd(16'h0301, {16'd0, id}, len, 32'd0, r);
	endtask

	// -------------------------------------------------------------------
	// Frame capture (what the framework stores)
	// -------------------------------------------------------------------
	reg  [8:0]  fb [0:512*384-1];
	integer     fx = 0, fy = 0, frame = 0, max_frames = 400, dump_every = 50;
	integer     line_pixels_bad = 0;
	string      out_dir = "sim_out";
	reg         vbl_d = 1'b1, hbl_d = 1'b0;

	task automatic dump_frame(input integer n);
		integer fd, i;
		string name;
		reg [8:0] p;
		name = $sformatf("%s/frame_%04d.ppm", out_dir, n);
		fd = $fopen(name, "wb");
		$fwrite(fd, "P6\n512 384\n255\n");
		for (i = 0; i < 512*384; i = i + 1) begin
			p = fb[i];
			$fwrite(fd, "%c%c%c", {p[8:6], p[8:6], p[8:7]}, {p[5:3], p[5:3], p[5:4]}, {p[2:0], p[2:0], p[2:1]});
		end
		$fclose(fd);
		$display("TB: wrote %s", name);
	endtask

	// Raw V8 output one cycle back = what gb_video_out quantised into the
	// pixel the framework is storing now (+odd_pixels prints the unusual ones).
	reg  [23:0] raw_rgb_d;
	reg  [7:0]  raw_idx_d, raw_idx_dd;
	reg  [2:0]  raw_mode_d;
	integer     odd_reports = 0;
	bit         report_odd = 1'b0;
	initial report_odd = $test$plusargs("odd_pixels");
	always @(posedge clk_sys) begin
		raw_rgb_d  <= {dut.vid_r, dut.vid_g, dut.vid_b};
		raw_idx_dd <= dut.mac.ariel_pixel_addr;
		raw_idx_d  <= raw_idx_dd;
		raw_mode_d <= dut.mac.v8_video_mode;
		if (report_odd && v_de && odd_reports < 40 && frame >= 400 &&
		    {v_r, v_g, v_b} == 9'o111 && raw_mode_d == 3'd0) begin
			odd_reports <= odd_reports + 1;
			$display("TB ODD: frame %0d x=%0d y=%0d out=%03o raw=%06h idx(-2)=%02h mode=%0d pal7F=%06h palFF=%06h",
			         frame, fx, fy, {v_r, v_g, v_b}, raw_rgb_d, raw_idx_d, raw_mode_d,
			         dut.mac.ariel.palette[8'h7F], dut.mac.ariel.palette[8'hFF]);
		end
	end

	always @(posedge clk_sys) begin
		vbl_d <= v_vblank;
		hbl_d <= v_hblank;
		if (v_vblank) begin
			fx <= 0; fy <= 0;
		end else if (v_hblank && !hbl_d) begin
			if (fx != 512) begin
				line_pixels_bad <= line_pixels_bad + 1;
				if (line_pixels_bad < 5)
					$display("TB WARNING: frame %0d row %0d had %0d pixels (vid_reset=%0d)",
					         frame, fy, fx, dut.vid_reset);
			end
			fx <= 0; fy <= fy + 1;
		end else if (v_de) begin
			if (fx < 512 && fy < 384) fb[fy*512 + fx] <= {v_r, v_g, v_b};
			fx <= fx + 1;
		end
		if (v_vblank && !vbl_d) begin
			frame <= frame + 1;
			if (fy != 384 && frame > 0)
				$display("TB WARNING: frame %0d had %0d rows", frame, fy);
			if (frame % 10 == 0) begin
				$display("TB: frame %0d host=%08h cpu=%08h bd=%08h bad_lines=%0d vram_wr=%0d pal_wr=%0d vmode=%0d",
				         frame, dut.dbg_host_state, dut.dbg_cpu_addr, dut.dbg_blockdev,
				         line_pixels_bad, sram_writes, palette_writes, dut.mac.v8_video_mode);
				$fflush;
			end
			if (dump_every > 0 && frame > 0 && frame % dump_every == 0) dump_frame(frame);
			if (frame >= max_frames) begin
				dump_frame(frame);
				$display("TB: done after %0d frames", frame);
				$finish;
			end
		end
	end

	// -------------------------------------------------------------------
	// The MCU's core-load sequence
	// -------------------------------------------------------------------
	string rom_path, hd_path, floppy_path, nvr_path;
	integer len, ram_mb;
	reg [31:0] r;
	initial begin
		if (!$value$plusargs("rom=%s", rom_path)) rom_path = "../MacLC_MiSTer/releases/boot0.rom";
		if (!$value$plusargs("hd=%s", hd_path)) hd_path = "";
		if (!$value$plusargs("floppy=%s", floppy_path)) floppy_path = "";
		if (!$value$plusargs("nvr=%s", nvr_path)) nvr_path = "";
		if ($test$plusargs("host_be")) host_be = 1'b1;
		if (!$value$plusargs("frames=%d", max_frames)) max_frames = 400;
		if (!$value$plusargs("dump=%d", dump_every)) dump_every = 50;
		if (!$value$plusargs("out=%s", out_dir)) out_dir = "sim_out";

		wait (!fw_reset);
		repeat (10) @(posedge clk_sys);
		host_cmd(16'h0000, 0, 0, 0, r);
		$display("TB: GetStatus = %0d (expect 2 = setup)", r);
		mem_read(32'h0000_0800, r);
		$display("TB: magic = %08h (expect 4D4C4331)", r);

		// Settings, written during setup like the MCU does (settings.json)
		if ($value$plusargs("ram=%d", ram_mb)) mem_write(32'h0000_0200, (ram_mb == 2) ? 32'd0 : 32'd1);
		if ($test$plusargs("floppy_write"))     mem_write(32'h0000_0204, 32'd1);
		if ($test$plusargs("kbd_mode"))         mem_write(32'h0000_0208, 32'd0);

		load_file(3, rom_path, 32'h10A0_0000, 0, 0, len);
		load_file(1, nvr_path, 32'h10A8_0000, 1, 512, len);
		if (hd_path != "")     load_file(0, hd_path, 32'h1100_0000, 0, 0, len);
		if (floppy_path != "") load_file(2, floppy_path, 32'h10E0_0000, 0, 0, len);

		host_cmd(16'h0102, 0, 0, 0, r);           // SetupComplete
		host_cmd(16'h0000, 0, 0, 0, r);
		$display("TB: GetStatus = %0d (expect 3 = halted)", r);
		host_cmd(16'h0200, 1, 0, 0, r);           // NotifyFocus(1)
		host_cmd(16'h0100, 0, 0, 0, r);           // CoreRun
		host_cmd(16'h0000, 0, 0, 0, r);
		$display("TB: GetStatus = %0d (expect 4 = running) host=%08h rom=%08h",
		         r, dut.dbg_host_state, dut.dbg_rom_word);
	end

	// -------------------------------------------------------------------
	// PMOD debug beacon receiver (pin 3, 115200 8N1): prints each line, so
	// the UART that will be watched on hardware is itself verified here.
	// -------------------------------------------------------------------
	localparam integer BAUD_PS = 1_000_000_000 / 115_200 * 1000;  // ps per bit
	string beacon_line = "";
	initial begin : beacon_rx
		reg [7:0] ch;
		integer b;
		forever begin
			@(negedge pmod_out[2]);                 // start bit
			#(BAUD_PS + BAUD_PS / 2);               // middle of bit 0
			for (b = 0; b < 8; b = b + 1) begin
				ch[b] = pmod_out[2];
				#(BAUD_PS);
			end
			if (ch == 8'h0A) begin
				$display("TB BEACON: %s", beacon_line);
				$fflush;
				beacon_line = "";
			end else if (ch != 8'h0D) begin
				beacon_line = {beacon_line, string'(ch)};
			end
		end
	end

	// Sanity: after the fixup the ROM must read big-endian in SDRAM.
	initial begin
		wait (dut.host.fixup_done);
		@(posedge clk_sys);
		$display("TB: ROM fixup done (host_le=%0d): SDRAM[$500000..3] = %04h %04h %04h %04h (expect 350E ACF0 0000 002A)",
		         dut.host.host_le, sdram[24'h500000], sdram[24'h500001], sdram[24'h500002], sdram[24'h500003]);
	end

endmodule

`default_nettype wire
