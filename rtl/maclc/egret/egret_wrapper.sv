// egret_wrapper.sv - Egret microcontroller for Mac LC
// Uses m68hc05_core CPU with real Egret ROM (341S0851)
//
// Based on MAME's egret.cpp by R. Belmont
// m68hc05_core converted from VHDL by Ulrich Riedel
//
// This is a drop-in replacement for egret.sv with the exact same interface

`default_nettype none

module egret_wrapper (
    input  wire        clk,
    input  wire        clk8_en,
    input  wire        reset,

    // RTC timestamp initialization (Unix time)
    input  wire [32:0] timestamp,

    // Direct VIA Port B connections
    input  wire        via_tip,          // VIA Port B bit 5 - Transaction In Progress (active low)
    input  wire        via_byteack_in,   // VIA Port B bit 4 - from VIA
    output wire        cuda_treq,        // Port B bit 3 - Transfer Request (active LOW)
    output wire        cuda_byteack,     // Port B bit 4 - Byte Acknowledge

    // VIA Shift Register interface (CB1/CB2)
    output wire        cuda_cb1,         // CB1 - Shift clock (Egret drives in external mode)
    input  wire        via_cb2_in,       // CB2 - Data from VIA (when VIA sending)
    output wire        cuda_cb2,         // CB2 - Data to VIA (when Egret sending)
    output wire        cuda_cb2_oe,      // CB2 output enable

    // VIA SR control signals
    input  wire        via_sr_read,      // VIA is reading SR (shift in mode)
    input  wire        via_sr_write,     // VIA has written SR (shift out mode)
    input  wire        via_sr_ext_clk,   // VIA is in external clock mode
    input  wire        via_sr_dir,       // VIA shift direction: 0=in, 1=out
    output reg         cuda_sr_irq,      // Request SR interrupt

    // Full port B for completeness
    output wire [7:0]  cuda_portb,       // Complete Port B output
    output wire [7:0]  cuda_portb_oe,    // Port B output enables

    // ADB signals (simplified)
    input  wire        adb_data_in,
    output reg         adb_data_out,

    // System control
    output reg         reset_680x0,
    output reg         nmi_680x0,

    // ---- PRAM persistence (additive: NVRAM save/restore via top-level SD DMA) ----
    // pram[] (below) is the canonical 256-byte PRAM: seeded into the HC05 RAM at
    // boot and kept in sync with firmware PRAM-region writes, so the top level can
    // snapshot/restore it. The Egret boot/SR logic is unchanged by these ports.
    input  wire        pram_load_wr,    // SD -> pram[]: write one byte
    input  wire  [7:0] pram_load_addr,
    input  wire  [7:0] pram_load_data,
    input  wire  [7:0] pram_save_addr,  // pram[] -> SD: read one byte (async)
    output wire  [7:0] pram_save_data,
    output wire        pram_wr_stb,      // 1-cyc strobe: firmware wrote a PRAM byte
    input  wire        pram_ready,       // top: SD load of pram[] complete (or no image / timeout)

    // Debug outputs for on-screen indicators
    output wire        dbg_cen,              // HC05 clock enable (pulse)
    output wire        dbg_port_test_done,   // Port test phase complete
    output wire        dbg_handshake_done,   // Handshake init complete
    output wire        dbg_treq,             // TREQ output (1=asserting)
    output wire        dbg_tip_in,           // TIP input from VIA (synced)
    output wire        dbg_byteack_in,       // BYTEACK input from VIA (synced)
    output wire [7:0]  dbg_pb_out,           // Egret Port B output register
    output wire [7:0]  dbg_pc_out,           // Egret Port C output register
    output wire        dbg_cpu_running       // HC05 is executing (not in reset)
);

// ============================================================================
// Clock generation for 68HC05
// MAME: M68HC05E1 runs at XTAL(32'768)*128 = 4.194304 MHz
// From 32 MHz system clock, divide by 8 gives 4 MHz (close to 4.19 MHz)
// ============================================================================
reg [2:0] clk_div;
wire cen = (clk_div == 3'b000);  // Pulse once every 8 cycles = 4 MHz

always @(posedge clk) begin
    if (reset)
        clk_div <= 3'b000;
    else
        clk_div <= clk_div + 3'b001;
end

// ============================================================================
// Memory map for Egret (68HC05E1 with 13-bit address space)
// ============================================================================
// 0x0000-0x001F: I/O registers (Ports A, B, C, DDR, Timer, etc.)
// 0x0090-0x01FF: Internal RAM (368 bytes for PRAM, RTC, stack, variables)
// 0x0F00-0x1FFF: ROM (4352 bytes = 0x1100)
//
// ROM file is 4352 bytes and maps directly:
// - CPU 0x0F00 → ROM offset 0x000 (first 256 bytes are copyright notice)
// - CPU 0x1FFF → ROM offset 0x10FF (last byte)
// - Reset vector at CPU 0x1FFE-0x1FFF → ROM offset 0x10FE-0x10FF

localparam ROM_SIZE = 4352;  // 0x1100 bytes - maps to CPU 0x0F00-0x1FFF

// CPU signals (from m68hc05_core)
wire [15:0] cpu_addr;
wire        cpu_wr;
wire [7:0]  cpu_din;
wire [7:0]  cpu_dout;
wire [3:0]  cpu_state;

// Port registers (68HC05 style)
reg  [7:0] pa_ddr, pb_ddr;
reg  [7:0] pa_latch, pb_latch;
reg  [7:0] pc_ddr, pc_latch;  // Full 8 bits for port test compatibility

// Port I/O
reg  [7:0] pa_out, pb_out;
reg  [7:0] pc_out;  // 8 bits for port test (only lower 4 bits used for actual I/O)

// Memory
// MLAB (ported 2026-08-08 from MacIIvi 9f5d0d3): as plain registers these two
// arrays cost ~1.5-2k ALMs (368:1 and 256:1 async byte read muxes + write
// decode) — the bulk of the wrapper's ~4.3k self-ALMs. MLAB keeps the
// async-read semantics the HC05's combinational RAM path needs (M10K could
// not). Inference additionally required the PRAM boot-copy below to become a
// sequential one-byte-per-cycle engine — a 256-parallel-write burst is not a
// RAM port. Same recipe as m10k-repack tier 1 (2026-07-18).
// ★ no_rw_check is LOAD-BEARING (2026-08-08 map audit): plain "MLAB" leaves
// intram UNINFERRED — Info 276009 "unsupported read-during-write behavior"
// (the merged priority write block defeats the prover). The waiver is safe:
// the HC05 never reads and writes the same address in one cycle (one bus op
// per cen), the CPU is frozen (cen gated) for the whole boot-copy, and the
// mirror block never reads intram. The MacIIvi parent commit shipped plain
// "MLAB" and its intram silently stayed fabric — same 276009 in their map.
(* ramstyle = "MLAB, no_rw_check" *) reg  [7:0] intram[0:367];    // Internal RAM: intram[x] = CPU addr 0x90+x (RAM at 0x90-0x1FF)
reg  [7:0] ram_dout;

// ROM
reg  [7:0] rom[0:8191];  // 2^13 to match 13-bit rom_addr width (only 4352 bytes used)
reg  [7:0] rom_dout;

// PRAM storage (256 bytes loaded from disk)
// no_rw_check here too: with plain "MLAB", synthesis inferred pram as TWO
// altsyncram copies (one per async read port) with absorbed/registered
// reads — legal only via an exotic retime, and AUTO block type risks two
// M10Ks out of the 91%-full budget. The waiver keeps the literal async-read
// MLAB form (read-during-write is a non-case: pram_load_wr only fires
// before pram_ready, the mirror only after pram_loaded, and neither
// coincides with the copy engine's read).
(* ramstyle = "MLAB, no_rw_check" *) reg  [7:0] pram[0:255];
reg        pram_loaded;
reg        pc_bit3_prev;        // (legacy; PC3 edge no longer gates the boot-copy)

// Initialize ROM and PRAM from hex files
integer init_i;
initial begin
    // [gamebub] egret_rom.hex and egret.pram, inlined by scripts/import_maclc.py
    rom[0] = 8'h43;
    rom[1] = 8'h6F;
    rom[2] = 8'h70;
    rom[3] = 8'h79;
    rom[4] = 8'h72;
    rom[5] = 8'h69;
    rom[6] = 8'h67;
    rom[7] = 8'h68;
    rom[8] = 8'h74;
    rom[9] = 8'h20;
    rom[10] = 8'hA9;
    rom[11] = 8'h20;
    rom[12] = 8'h31;
    rom[13] = 8'h39;
    rom[14] = 8'h38;
    rom[15] = 8'h39;
    rom[16] = 8'h2C;
    rom[17] = 8'h20;
    rom[18] = 8'h31;
    rom[19] = 8'h39;
    rom[20] = 8'h39;
    rom[21] = 8'h30;
    rom[22] = 8'h0D;
    rom[23] = 8'h41;
    rom[24] = 8'h70;
    rom[25] = 8'h70;
    rom[26] = 8'h6C;
    rom[27] = 8'h65;
    rom[28] = 8'h20;
    rom[29] = 8'h43;
    rom[30] = 8'h6F;
    rom[31] = 8'h6D;
    rom[32] = 8'h70;
    rom[33] = 8'h75;
    rom[34] = 8'h74;
    rom[35] = 8'h65;
    rom[36] = 8'h72;
    rom[37] = 8'h2C;
    rom[38] = 8'h20;
    rom[39] = 8'h49;
    rom[40] = 8'h6E;
    rom[41] = 8'h63;
    rom[42] = 8'h2E;
    rom[43] = 8'h0D;
    rom[44] = 8'h41;
    rom[45] = 8'h6C;
    rom[46] = 8'h6C;
    rom[47] = 8'h20;
    rom[48] = 8'h72;
    rom[49] = 8'h69;
    rom[50] = 8'h67;
    rom[51] = 8'h68;
    rom[52] = 8'h74;
    rom[53] = 8'h73;
    rom[54] = 8'h20;
    rom[55] = 8'h72;
    rom[56] = 8'h65;
    rom[57] = 8'h73;
    rom[58] = 8'h65;
    rom[59] = 8'h72;
    rom[60] = 8'h76;
    rom[61] = 8'h65;
    rom[62] = 8'h64;
    rom[63] = 8'h2E;
    rom[64] = 8'h0D;
    rom[65] = 8'h57;
    rom[66] = 8'h72;
    rom[67] = 8'h69;
    rom[68] = 8'h74;
    rom[69] = 8'h74;
    rom[70] = 8'h65;
    rom[71] = 8'h6E;
    rom[72] = 8'h20;
    rom[73] = 8'h62;
    rom[74] = 8'h79;
    rom[75] = 8'h3A;
    rom[76] = 8'h20;
    rom[77] = 8'h52;
    rom[78] = 8'h61;
    rom[79] = 8'h79;
    rom[80] = 8'h20;
    rom[81] = 8'h4D;
    rom[82] = 8'h6F;
    rom[83] = 8'h6E;
    rom[84] = 8'h74;
    rom[85] = 8'h61;
    rom[86] = 8'h67;
    rom[87] = 8'h6E;
    rom[88] = 8'h65;
    rom[89] = 8'h00;
    rom[90] = 8'h00;
    rom[91] = 8'h10;
    rom[92] = 8'h01;
    rom[93] = 8'h01;
    rom[94] = 8'h00;
    rom[95] = 8'hFF;
    rom[96] = 8'h01;
    rom[97] = 8'h08;
    rom[98] = 8'h00;
    rom[99] = 8'hA4;
    rom[100] = 8'h00;
    rom[101] = 8'h95;
    rom[102] = 8'h00;
    rom[103] = 8'hAF;
    rom[104] = 8'h00;
    rom[105] = 8'hAB;
    rom[106] = 8'h00;
    rom[107] = 8'h00;
    rom[108] = 8'h00;
    rom[109] = 8'h00;
    rom[110] = 8'h99;
    rom[111] = 8'h92;
    rom[112] = 8'h08;
    rom[113] = 8'h9C;
    rom[114] = 8'hA6;
    rom[115] = 8'h06;
    rom[116] = 8'hB7;
    rom[117] = 8'h07;
    rom[118] = 8'hA6;
    rom[119] = 8'h10;
    rom[120] = 8'hB7;
    rom[121] = 8'h12;
    rom[122] = 8'hAE;
    rom[123] = 8'h06;
    rom[124] = 8'hD6;
    rom[125] = 8'h0F;
    rom[126] = 8'h6A;
    rom[127] = 8'hF7;
    rom[128] = 8'h5A;
    rom[129] = 8'h2A;
    rom[130] = 8'hF9;
    rom[131] = 8'h0A;
    rom[132] = 8'h00;
    rom[133] = 8'h04;
    rom[134] = 8'h11;
    rom[135] = 8'h04;
    rom[136] = 8'h12;
    rom[137] = 8'h04;
    rom[138] = 8'hAE;
    rom[139] = 8'h46;
    rom[140] = 8'h4F;
    rom[141] = 8'hE7;
    rom[142] = 8'h90;
    rom[143] = 8'h5A;
    rom[144] = 8'h2A;
    rom[145] = 8'hFB;
    rom[146] = 8'h5F;
    rom[147] = 8'hD7;
    rom[148] = 8'h01;
    rom[149] = 8'h00;
    rom[150] = 8'h5C;
    rom[151] = 8'h26;
    rom[152] = 8'hFA;
    rom[153] = 8'hA6;
    rom[154] = 8'h63;
    rom[155] = 8'hB7;
    rom[156] = 8'hAB;
    rom[157] = 8'hA6;
    rom[158] = 8'h0B;
    rom[159] = 8'hB7;
    rom[160] = 8'hAC;
    rom[161] = 8'hA6;
    rom[162] = 8'h60;
    rom[163] = 8'hB7;
    rom[164] = 8'hAD;
    rom[165] = 8'hA6;
    rom[166] = 8'h6F;
    rom[167] = 8'hB7;
    rom[168] = 8'hAE;
    rom[169] = 8'hA6;
    rom[170] = 8'h02;
    rom[171] = 8'hB7;
    rom[172] = 8'hA6;
    rom[173] = 8'h16;
    rom[174] = 8'hA3;
    rom[175] = 8'h9C;
    rom[176] = 8'h9A;
    rom[177] = 8'h1D;
    rom[178] = 8'h07;
    rom[179] = 8'h17;
    rom[180] = 8'h07;
    rom[181] = 8'h0A;
    rom[182] = 8'h00;
    rom[183] = 8'h02;
    rom[184] = 8'h13;
    rom[185] = 8'h00;
    rom[186] = 8'hA6;
    rom[187] = 8'h0B;
    rom[188] = 8'hB7;
    rom[189] = 8'hA4;
    rom[190] = 8'hA6;
    rom[191] = 8'h0F;
    rom[192] = 8'hB7;
    rom[193] = 8'hA5;
    rom[194] = 8'hA6;
    rom[195] = 8'h0C;
    rom[196] = 8'hB7;
    rom[197] = 8'hCA;
    rom[198] = 8'h3F;
    rom[199] = 8'hC9;
    rom[200] = 8'hB6;
    rom[201] = 8'hA1;
    rom[202] = 8'hA4;
    rom[203] = 8'hC0;
    rom[204] = 8'h00;
    rom[205] = 8'h01;
    rom[206] = 8'h02;
    rom[207] = 8'hA4;
    rom[208] = 8'h40;
    rom[209] = 8'hB7;
    rom[210] = 8'hA1;
    rom[211] = 8'hB6;
    rom[212] = 8'hA2;
    rom[213] = 8'hA4;
    rom[214] = 8'h04;
    rom[215] = 8'hB7;
    rom[216] = 8'hA2;
    rom[217] = 8'hB6;
    rom[218] = 8'hA3;
    rom[219] = 8'hA4;
    rom[220] = 8'hCC;
    rom[221] = 8'hB7;
    rom[222] = 8'hA3;
    rom[223] = 8'h00;
    rom[224] = 8'h01;
    rom[225] = 8'h02;
    rom[226] = 8'h1F;
    rom[227] = 8'hA3;
    rom[228] = 8'h01;
    rom[229] = 8'h01;
    rom[230] = 8'h03;
    rom[231] = 8'h0D;
    rom[232] = 8'h01;
    rom[233] = 8'hFA;
    rom[234] = 8'h0A;
    rom[235] = 8'h00;
    rom[236] = 8'h19;
    rom[237] = 8'hCD;
    rom[238] = 8'h12;
    rom[239] = 8'hB6;
    rom[240] = 8'hCD;
    rom[241] = 8'h1E;
    rom[242] = 8'h4E;
    rom[243] = 8'hCD;
    rom[244] = 8'h12;
    rom[245] = 8'h0A;
    rom[246] = 8'h25;
    rom[247] = 8'h06;
    rom[248] = 8'h1D;
    rom[249] = 8'h07;
    rom[250] = 8'h17;
    rom[251] = 8'h07;
    rom[252] = 8'h20;
    rom[253] = 8'hF2;
    rom[254] = 8'hCD;
    rom[255] = 8'h12;
    rom[256] = 8'h12;
    rom[257] = 8'hCD;
    rom[258] = 8'h12;
    rom[259] = 8'h3E;
    rom[260] = 8'h20;
    rom[261] = 8'h2E;
    rom[262] = 8'hCD;
    rom[263] = 8'h12;
    rom[264] = 8'hB6;
    rom[265] = 8'h16;
    rom[266] = 8'hA3;
    rom[267] = 8'hCD;
    rom[268] = 8'h1E;
    rom[269] = 8'h4E;
    rom[270] = 8'h03;
    rom[271] = 8'h02;
    rom[272] = 8'hF8;
    rom[273] = 8'hCD;
    rom[274] = 8'h1E;
    rom[275] = 8'h01;
    rom[276] = 8'h03;
    rom[277] = 8'h02;
    rom[278] = 8'hF2;
    rom[279] = 8'hCD;
    rom[280] = 8'h11;
    rom[281] = 8'hA0;
    rom[282] = 8'h25;
    rom[283] = 8'h10;
    rom[284] = 8'h05;
    rom[285] = 8'h00;
    rom[286] = 8'h0D;
    rom[287] = 8'h04;
    rom[288] = 8'hA3;
    rom[289] = 8'h0A;
    rom[290] = 8'h0C;
    rom[291] = 8'hA3;
    rom[292] = 8'h07;
    rom[293] = 8'hCD;
    rom[294] = 8'h12;
    rom[295] = 8'h0A;
    rom[296] = 8'h25;
    rom[297] = 8'h02;
    rom[298] = 8'h20;
    rom[299] = 8'hDA;
    rom[300] = 8'hCD;
    rom[301] = 8'h11;
    rom[302] = 8'hE7;
    rom[303] = 8'h24;
    rom[304] = 8'hD5;
    rom[305] = 8'hCD;
    rom[306] = 8'h12;
    rom[307] = 8'h3E;
    rom[308] = 8'hB6;
    rom[309] = 8'hA4;
    rom[310] = 8'hA0;
    rom[311] = 8'h02;
    rom[312] = 8'hB7;
    rom[313] = 8'h90;
    rom[314] = 8'h0A;
    rom[315] = 8'h00;
    rom[316] = 8'h06;
    rom[317] = 8'hA6;
    rom[318] = 8'h09;
    rom[319] = 8'hB7;
    rom[320] = 8'h91;
    rom[321] = 8'h20;
    rom[322] = 8'h04;
    rom[323] = 8'hA6;
    rom[324] = 8'h08;
    rom[325] = 8'hB7;
    rom[326] = 8'h91;
    rom[327] = 8'hCD;
    rom[328] = 8'h11;
    rom[329] = 8'h98;
    rom[330] = 8'h11;
    rom[331] = 8'hA3;
    rom[332] = 8'hCD;
    rom[333] = 8'h1A;
    rom[334] = 8'hCE;
    rom[335] = 8'h24;
    rom[336] = 8'hE3;
    rom[337] = 8'hCD;
    rom[338] = 8'h11;
    rom[339] = 8'h38;
    rom[340] = 8'h25;
    rom[341] = 8'h03;
    rom[342] = 8'hCC;
    rom[343] = 8'h0F;
    rom[344] = 8'hAF;
    rom[345] = 8'h1D;
    rom[346] = 8'h95;
    rom[347] = 8'hCD;
    rom[348] = 8'h12;
    rom[349] = 8'hC2;
    rom[350] = 8'h24;
    rom[351] = 8'hD4;
    rom[352] = 8'h3A;
    rom[353] = 8'h91;
    rom[354] = 8'h26;
    rom[355] = 8'hE3;
    rom[356] = 8'h3A;
    rom[357] = 8'h90;
    rom[358] = 8'h26;
    rom[359] = 8'hD2;
    rom[360] = 8'h0E;
    rom[361] = 8'hA2;
    rom[362] = 8'h07;
    rom[363] = 8'h10;
    rom[364] = 8'hA3;
    rom[365] = 8'hCD;
    rom[366] = 8'h11;
    rom[367] = 8'h49;
    rom[368] = 8'h20;
    rom[369] = 8'hC2;
    rom[370] = 8'h3F;
    rom[371] = 8'h95;
    rom[372] = 8'h1C;
    rom[373] = 8'h95;
    rom[374] = 8'hB6;
    rom[375] = 8'hB3;
    rom[376] = 8'hAA;
    rom[377] = 8'h0C;
    rom[378] = 8'hCD;
    rom[379] = 8'h1C;
    rom[380] = 8'h71;
    rom[381] = 8'hB6;
    rom[382] = 8'h95;
    rom[383] = 8'hA4;
    rom[384] = 8'h03;
    rom[385] = 8'hA8;
    rom[386] = 8'h02;
    rom[387] = 8'h27;
    rom[388] = 8'hAF;
    rom[389] = 8'hA4;
    rom[390] = 8'h02;
    rom[391] = 8'h27;
    rom[392] = 8'h06;
    rom[393] = 8'hCD;
    rom[394] = 8'h14;
    rom[395] = 8'h3B;
    rom[396] = 8'h01;
    rom[397] = 8'h95;
    rom[398] = 8'hA5;
    rom[399] = 8'hCD;
    rom[400] = 8'h1D;
    rom[401] = 8'hD7;
    rom[402] = 8'hB6;
    rom[403] = 8'hB3;
    rom[404] = 8'hB7;
    rom[405] = 8'hCE;
    rom[406] = 8'hB6;
    rom[407] = 8'hB4;
    rom[408] = 8'hB7;
    rom[409] = 8'hCD;
    rom[410] = 8'hAA;
    rom[411] = 8'h0C;
    rom[412] = 8'hCD;
    rom[413] = 8'h1C;
    rom[414] = 8'h71;
    rom[415] = 8'h02;
    rom[416] = 8'h95;
    rom[417] = 8'h19;
    rom[418] = 8'hB6;
    rom[419] = 8'hCD;
    rom[420] = 8'hB7;
    rom[421] = 8'hCE;
    rom[422] = 8'hCD;
    rom[423] = 8'h14;
    rom[424] = 8'h3B;
    rom[425] = 8'h00;
    rom[426] = 8'h95;
    rom[427] = 8'h0F;
    rom[428] = 8'hB6;
    rom[429] = 8'hB3;
    rom[430] = 8'hB1;
    rom[431] = 8'hCE;
    rom[432] = 8'h27;
    rom[433] = 8'h06;
    rom[434] = 8'hBE;
    rom[435] = 8'hCE;
    rom[436] = 8'hB7;
    rom[437] = 8'hB4;
    rom[438] = 8'hBF;
    rom[439] = 8'hB3;
    rom[440] = 8'hCC;
    rom[441] = 8'h10;
    rom[442] = 8'h34;
    rom[443] = 8'hCD;
    rom[444] = 8'h1D;
    rom[445] = 8'hD7;
    rom[446] = 8'hB6;
    rom[447] = 8'hCD;
    rom[448] = 8'hA0;
    rom[449] = 8'h10;
    rom[450] = 8'hB7;
    rom[451] = 8'hCD;
    rom[452] = 8'hB1;
    rom[453] = 8'hB3;
    rom[454] = 8'h27;
    rom[455] = 8'hF6;
    rom[456] = 8'hB1;
    rom[457] = 8'hB4;
    rom[458] = 8'h27;
    rom[459] = 8'h18;
    rom[460] = 8'hCD;
    rom[461] = 8'h11;
    rom[462] = 8'h21;
    rom[463] = 8'h24;
    rom[464] = 8'hED;
    rom[465] = 8'hB6;
    rom[466] = 8'hCD;
    rom[467] = 8'hAA;
    rom[468] = 8'h0C;
    rom[469] = 8'hCD;
    rom[470] = 8'h1C;
    rom[471] = 8'h71;
    rom[472] = 8'h02;
    rom[473] = 8'h95;
    rom[474] = 8'hE0;
    rom[475] = 8'hB6;
    rom[476] = 8'hCD;
    rom[477] = 8'hB7;
    rom[478] = 8'hCE;
    rom[479] = 8'hCD;
    rom[480] = 8'h14;
    rom[481] = 8'h3B;
    rom[482] = 8'h20;
    rom[483] = 8'hC8;
    rom[484] = 8'hB6;
    rom[485] = 8'hB3;
    rom[486] = 8'hB7;
    rom[487] = 8'hCD;
    rom[488] = 8'hAA;
    rom[489] = 8'h0C;
    rom[490] = 8'hCD;
    rom[491] = 8'h1C;
    rom[492] = 8'h71;
    rom[493] = 8'h02;
    rom[494] = 8'h95;
    rom[495] = 8'h09;
    rom[496] = 8'hB6;
    rom[497] = 8'hCD;
    rom[498] = 8'hB7;
    rom[499] = 8'hCE;
    rom[500] = 8'hCD;
    rom[501] = 8'h14;
    rom[502] = 8'h3B;
    rom[503] = 8'h20;
    rom[504] = 8'hB3;
    rom[505] = 8'hB6;
    rom[506] = 8'hB4;
    rom[507] = 8'hB7;
    rom[508] = 8'hCD;
    rom[509] = 8'hCD;
    rom[510] = 8'h1D;
    rom[511] = 8'hD7;
    rom[512] = 8'hB6;
    rom[513] = 8'hCD;
    rom[514] = 8'hA0;
    rom[515] = 8'h10;
    rom[516] = 8'hB7;
    rom[517] = 8'hCD;
    rom[518] = 8'hB1;
    rom[519] = 8'hB4;
    rom[520] = 8'h27;
    rom[521] = 8'hA2;
    rom[522] = 8'hCD;
    rom[523] = 8'h11;
    rom[524] = 8'h21;
    rom[525] = 8'h25;
    rom[526] = 8'hF1;
    rom[527] = 8'hB6;
    rom[528] = 8'hCD;
    rom[529] = 8'hAA;
    rom[530] = 8'h0C;
    rom[531] = 8'hCD;
    rom[532] = 8'h1C;
    rom[533] = 8'h71;
    rom[534] = 8'h02;
    rom[535] = 8'h95;
    rom[536] = 8'hE4;
    rom[537] = 8'hCD;
    rom[538] = 8'h14;
    rom[539] = 8'h3B;
    rom[540] = 8'h01;
    rom[541] = 8'h95;
    rom[542] = 8'h8D;
    rom[543] = 8'h20;
    rom[544] = 8'hDC;
    rom[545] = 8'h44;
    rom[546] = 8'h44;
    rom[547] = 8'h44;
    rom[548] = 8'h44;
    rom[549] = 8'hA1;
    rom[550] = 8'h08;
    rom[551] = 8'h25;
    rom[552] = 8'h07;
    rom[553] = 8'hA4;
    rom[554] = 8'h07;
    rom[555] = 8'h97;
    rom[556] = 8'hB6;
    rom[557] = 8'hC9;
    rom[558] = 8'h20;
    rom[559] = 8'h03;
    rom[560] = 8'h97;
    rom[561] = 8'hB6;
    rom[562] = 8'hCA;
    rom[563] = 8'h46;
    rom[564] = 8'h5A;
    rom[565] = 8'h2A;
    rom[566] = 8'hFC;
    rom[567] = 8'h81;
    rom[568] = 8'hCD;
    rom[569] = 8'h12;
    rom[570] = 8'h0A;
    rom[571] = 8'h25;
    rom[572] = 8'h05;
    rom[573] = 8'h1D;
    rom[574] = 8'h07;
    rom[575] = 8'h17;
    rom[576] = 8'h07;
    rom[577] = 8'h81;
    rom[578] = 8'hCD;
    rom[579] = 8'h12;
    rom[580] = 8'h12;
    rom[581] = 8'hCD;
    rom[582] = 8'h11;
    rom[583] = 8'h49;
    rom[584] = 8'h81;
    rom[585] = 8'hB7;
    rom[586] = 8'hD5;
    rom[587] = 8'hBF;
    rom[588] = 8'hD6;
    rom[589] = 8'h04;
    rom[590] = 8'h00;
    rom[591] = 8'h2B;
    rom[592] = 8'hB6;
    rom[593] = 8'hCC;
    rom[594] = 8'hB1;
    rom[595] = 8'hA6;
    rom[596] = 8'h24;
    rom[597] = 8'h03;
    rom[598] = 8'h01;
    rom[599] = 8'hA3;
    rom[600] = 8'h22;
    rom[601] = 8'h07;
    rom[602] = 8'h02;
    rom[603] = 8'h1F;
    rom[604] = 8'hAE;
    rom[605] = 8'h05;
    rom[606] = 8'hE6;
    rom[607] = 8'h95;
    rom[608] = 8'hE7;
    rom[609] = 8'hCF;
    rom[610] = 8'h5A;
    rom[611] = 8'h2A;
    rom[612] = 8'hF9;
    rom[613] = 8'hCD;
    rom[614] = 8'h1D;
    rom[615] = 8'hD7;
    rom[616] = 8'hA6;
    rom[617] = 8'h2E;
    rom[618] = 8'hCD;
    rom[619] = 8'h1C;
    rom[620] = 8'h71;
    rom[621] = 8'h02;
    rom[622] = 8'h95;
    rom[623] = 8'h00;
    rom[624] = 8'hAE;
    rom[625] = 8'h05;
    rom[626] = 8'hE6;
    rom[627] = 8'hCF;
    rom[628] = 8'hE7;
    rom[629] = 8'h95;
    rom[630] = 8'h5A;
    rom[631] = 8'h2A;
    rom[632] = 8'hF9;
    rom[633] = 8'h25;
    rom[634] = 8'h00;
    rom[635] = 8'h0B;
    rom[636] = 8'h00;
    rom[637] = 8'h1A;
    rom[638] = 8'hCD;
    rom[639] = 8'h11;
    rom[640] = 8'hA0;
    rom[641] = 8'h24;
    rom[642] = 8'h15;
    rom[643] = 8'hCD;
    rom[644] = 8'h11;
    rom[645] = 8'hC1;
    rom[646] = 8'h25;
    rom[647] = 8'h10;
    rom[648] = 8'h03;
    rom[649] = 8'h00;
    rom[650] = 8'hFD;
    rom[651] = 8'hCD;
    rom[652] = 8'h1D;
    rom[653] = 8'hF2;
    rom[654] = 8'h03;
    rom[655] = 8'h00;
    rom[656] = 8'hF7;
    rom[657] = 8'h16;
    rom[658] = 8'hA3;
    rom[659] = 8'h15;
    rom[660] = 8'hA3;
    rom[661] = 8'hCC;
    rom[662] = 8'h0F;
    rom[663] = 8'hAF;
    rom[664] = 8'h0D;
    rom[665] = 8'h01;
    rom[666] = 8'hFA;
    rom[667] = 8'hBE;
    rom[668] = 8'hD6;
    rom[669] = 8'hB6;
    rom[670] = 8'hD5;
    rom[671] = 8'h81;
    rom[672] = 8'h03;
    rom[673] = 8'h00;
    rom[674] = 8'h0F;
    rom[675] = 8'h07;
    rom[676] = 8'hA3;
    rom[677] = 8'h02;
    rom[678] = 8'h20;
    rom[679] = 8'h08;
    rom[680] = 8'hCD;
    rom[681] = 8'h1E;
    rom[682] = 8'h01;
    rom[683] = 8'h03;
    rom[684] = 8'h00;
    rom[685] = 8'h02;
    rom[686] = 8'h16;
    rom[687] = 8'hA3;
    rom[688] = 8'h98;
    rom[689] = 8'h81;
    rom[690] = 8'h07;
    rom[691] = 8'hA3;
    rom[692] = 8'hFB;
    rom[693] = 8'hCD;
    rom[694] = 8'h1E;
    rom[695] = 8'h01;
    rom[696] = 8'h03;
    rom[697] = 8'h00;
    rom[698] = 8'h02;
    rom[699] = 8'h20;
    rom[700] = 8'hF3;
    rom[701] = 8'h17;
    rom[702] = 8'hA3;
    rom[703] = 8'h99;
    rom[704] = 8'h81;
    rom[705] = 8'h1D;
    rom[706] = 8'h07;
    rom[707] = 8'h17;
    rom[708] = 8'h07;
    rom[709] = 8'h11;
    rom[710] = 8'h00;
    rom[711] = 8'h10;
    rom[712] = 8'h04;
    rom[713] = 8'hA6;
    rom[714] = 8'h96;
    rom[715] = 8'hB7;
    rom[716] = 8'h93;
    rom[717] = 8'hCD;
    rom[718] = 8'h1D;
    rom[719] = 8'hF2;
    rom[720] = 8'h01;
    rom[721] = 8'h01;
    rom[722] = 8'h0A;
    rom[723] = 8'h3A;
    rom[724] = 8'h93;
    rom[725] = 8'h26;
    rom[726] = 8'hF6;
    rom[727] = 8'h11;
    rom[728] = 8'h04;
    rom[729] = 8'hCD;
    rom[730] = 8'h12;
    rom[731] = 8'h12;
    rom[732] = 8'h81;
    rom[733] = 8'h11;
    rom[734] = 8'h04;
    rom[735] = 8'h1F;
    rom[736] = 8'hA3;
    rom[737] = 8'hCD;
    rom[738] = 8'h12;
    rom[739] = 8'hB6;
    rom[740] = 8'h1F;
    rom[741] = 8'hA1;
    rom[742] = 8'h81;
    rom[743] = 8'h10;
    rom[744] = 8'h00;
    rom[745] = 8'h10;
    rom[746] = 8'h04;
    rom[747] = 8'hA6;
    rom[748] = 8'h96;
    rom[749] = 8'hB7;
    rom[750] = 8'h93;
    rom[751] = 8'hCD;
    rom[752] = 8'h1D;
    rom[753] = 8'hF2;
    rom[754] = 8'h00;
    rom[755] = 8'h01;
    rom[756] = 8'h0A;
    rom[757] = 8'h4A;
    rom[758] = 8'h26;
    rom[759] = 8'hF7;
    rom[760] = 8'h11;
    rom[761] = 8'h04;
    rom[762] = 8'h1D;
    rom[763] = 8'h07;
    rom[764] = 8'h17;
    rom[765] = 8'h07;
    rom[766] = 8'h81;
    rom[767] = 8'hCD;
    rom[768] = 8'h12;
    rom[769] = 8'h0A;
    rom[770] = 8'h24;
    rom[771] = 8'hF4;
    rom[772] = 8'h11;
    rom[773] = 8'h04;
    rom[774] = 8'hCD;
    rom[775] = 8'h12;
    rom[776] = 8'h12;
    rom[777] = 8'h81;
    rom[778] = 8'h0F;
    rom[779] = 8'hA3;
    rom[780] = 8'h1B;
    rom[781] = 8'h01;
    rom[782] = 8'h01;
    rom[783] = 8'h2B;
    rom[784] = 8'h1D;
    rom[785] = 8'hA3;
    rom[786] = 8'h0C;
    rom[787] = 8'h07;
    rom[788] = 8'h11;
    rom[789] = 8'h18;
    rom[790] = 8'h07;
    rom[791] = 8'h16;
    rom[792] = 8'h07;
    rom[793] = 8'hCD;
    rom[794] = 8'h1E;
    rom[795] = 8'h01;
    rom[796] = 8'h19;
    rom[797] = 8'h07;
    rom[798] = 8'hCD;
    rom[799] = 8'h1E;
    rom[800] = 8'h01;
    rom[801] = 8'h01;
    rom[802] = 8'h01;
    rom[803] = 8'h15;
    rom[804] = 8'h1C;
    rom[805] = 8'h07;
    rom[806] = 8'h99;
    rom[807] = 8'h81;
    rom[808] = 8'hAE;
    rom[809] = 8'h32;
    rom[810] = 8'hBF;
    rom[811] = 8'h93;
    rom[812] = 8'hCD;
    rom[813] = 8'h1D;
    rom[814] = 8'hF2;
    rom[815] = 8'h01;
    rom[816] = 8'h01;
    rom[817] = 8'h09;
    rom[818] = 8'h3A;
    rom[819] = 8'h93;
    rom[820] = 8'h26;
    rom[821] = 8'hF6;
    rom[822] = 8'h1E;
    rom[823] = 8'hA3;
    rom[824] = 8'h81;
    rom[825] = 8'h17;
    rom[826] = 8'h07;
    rom[827] = 8'h1F;
    rom[828] = 8'hA3;
    rom[829] = 8'h81;
    rom[830] = 8'hA6;
    rom[831] = 8'hF7;
    rom[832] = 8'hB7;
    rom[833] = 8'h02;
    rom[834] = 8'hA6;
    rom[835] = 8'h92;
    rom[836] = 8'hB7;
    rom[837] = 8'h01;
    rom[838] = 8'h3F;
    rom[839] = 8'h00;
    rom[840] = 8'h17;
    rom[841] = 8'h00;
    rom[842] = 8'h17;
    rom[843] = 8'h02;
    rom[844] = 8'h16;
    rom[845] = 8'h06;
    rom[846] = 8'h0F;
    rom[847] = 8'hA1;
    rom[848] = 8'h20;
    rom[849] = 8'hCD;
    rom[850] = 8'h12;
    rom[851] = 8'h0A;
    rom[852] = 8'h24;
    rom[853] = 8'h5D;
    rom[854] = 8'hCD;
    rom[855] = 8'h1E;
    rom[856] = 8'h4E;
    rom[857] = 8'hCD;
    rom[858] = 8'h11;
    rom[859] = 8'h49;
    rom[860] = 8'hA6;
    rom[861] = 8'h2E;
    rom[862] = 8'hCD;
    rom[863] = 8'h1C;
    rom[864] = 8'h71;
    rom[865] = 8'h02;
    rom[866] = 8'h95;
    rom[867] = 8'h0D;
    rom[868] = 8'hB6;
    rom[869] = 8'h99;
    rom[870] = 8'hA4;
    rom[871] = 8'h19;
    rom[872] = 8'hA8;
    rom[873] = 8'h19;
    rom[874] = 8'h27;
    rom[875] = 8'h05;
    rom[876] = 8'hCD;
    rom[877] = 8'h1D;
    rom[878] = 8'hD7;
    rom[879] = 8'h20;
    rom[880] = 8'hE0;
    rom[881] = 8'hAE;
    rom[882] = 8'h05;
    rom[883] = 8'hBF;
    rom[884] = 8'h93;
    rom[885] = 8'hCD;
    rom[886] = 8'h1D;
    rom[887] = 8'hF2;
    rom[888] = 8'h01;
    rom[889] = 8'h01;
    rom[890] = 8'h38;
    rom[891] = 8'h3A;
    rom[892] = 8'h93;
    rom[893] = 8'h26;
    rom[894] = 8'hF6;
    rom[895] = 8'h16;
    rom[896] = 8'h00;
    rom[897] = 8'hAE;
    rom[898] = 8'h14;
    rom[899] = 8'hBF;
    rom[900] = 8'h93;
    rom[901] = 8'hCD;
    rom[902] = 8'h1D;
    rom[903] = 8'hF2;
    rom[904] = 8'h01;
    rom[905] = 8'h01;
    rom[906] = 8'h28;
    rom[907] = 8'h3A;
    rom[908] = 8'h93;
    rom[909] = 8'h26;
    rom[910] = 8'hF6;
    rom[911] = 8'h16;
    rom[912] = 8'h02;
    rom[913] = 8'h17;
    rom[914] = 8'h06;
    rom[915] = 8'h1E;
    rom[916] = 8'h00;
    rom[917] = 8'hAE;
    rom[918] = 8'h03;
    rom[919] = 8'hCD;
    rom[920] = 8'h1D;
    rom[921] = 8'hE2;
    rom[922] = 8'h1F;
    rom[923] = 8'h00;
    rom[924] = 8'hAE;
    rom[925] = 8'h1C;
    rom[926] = 8'hCD;
    rom[927] = 8'h1D;
    rom[928] = 8'hD1;
    rom[929] = 8'hCD;
    rom[930] = 8'h12;
    rom[931] = 8'h0A;
    rom[932] = 8'h24;
    rom[933] = 8'h0D;
    rom[934] = 8'hCD;
    rom[935] = 8'h1E;
    rom[936] = 8'h4E;
    rom[937] = 8'hCD;
    rom[938] = 8'h11;
    rom[939] = 8'h49;
    rom[940] = 8'h06;
    rom[941] = 8'h01;
    rom[942] = 8'hF2;
    rom[943] = 8'h04;
    rom[944] = 8'h01;
    rom[945] = 8'hEF;
    rom[946] = 8'h81;
    rom[947] = 8'hCC;
    rom[948] = 8'h0F;
    rom[949] = 8'hAF;
    rom[950] = 8'h00;
    rom[951] = 8'h01;
    rom[952] = 8'h08;
    rom[953] = 8'h16;
    rom[954] = 8'h06;
    rom[955] = 8'h3F;
    rom[956] = 8'h02;
    rom[957] = 8'h3F;
    rom[958] = 8'h01;
    rom[959] = 8'h3F;
    rom[960] = 8'h00;
    rom[961] = 8'h81;
    rom[962] = 8'h07;
    rom[963] = 8'h01;
    rom[964] = 8'h66;
    rom[965] = 8'h3F;
    rom[966] = 8'hB5;
    rom[967] = 8'hCD;
    rom[968] = 8'h14;
    rom[969] = 8'hC8;
    rom[970] = 8'h25;
    rom[971] = 8'h5F;
    rom[972] = 8'hCD;
    rom[973] = 8'h14;
    rom[974] = 8'hC8;
    rom[975] = 8'h25;
    rom[976] = 8'h5A;
    rom[977] = 8'hB6;
    rom[978] = 8'hBA;
    rom[979] = 8'hB7;
    rom[980] = 8'hB8;
    rom[981] = 8'hB6;
    rom[982] = 8'hB9;
    rom[983] = 8'hB7;
    rom[984] = 8'hB7;
    rom[985] = 8'h26;
    rom[986] = 8'h03;
    rom[987] = 8'hCC;
    rom[988] = 8'h13;
    rom[989] = 8'h8D;
    rom[990] = 8'hA1;
    rom[991] = 8'h01;
    rom[992] = 8'h27;
    rom[993] = 8'h2E;
    rom[994] = 8'hA6;
    rom[995] = 8'h01;
    rom[996] = 8'h20;
    rom[997] = 8'h0F;
    rom[998] = 8'hB6;
    rom[999] = 8'hB7;
    rom[1000] = 8'hB7;
    rom[1001] = 8'hB9;
    rom[1002] = 8'hB6;
    rom[1003] = 8'hB8;
    rom[1004] = 8'hB7;
    rom[1005] = 8'hBA;
    rom[1006] = 8'hA6;
    rom[1007] = 8'h03;
    rom[1008] = 8'hCD;
    rom[1009] = 8'h12;
    rom[1010] = 8'hF5;
    rom[1011] = 8'h99;
    rom[1012] = 8'h81;
    rom[1013] = 8'hB7;
    rom[1014] = 8'h93;
    rom[1015] = 8'hB6;
    rom[1016] = 8'hB7;
    rom[1017] = 8'hB7;
    rom[1018] = 8'hBB;
    rom[1019] = 8'hB6;
    rom[1020] = 8'hB8;
    rom[1021] = 8'hB7;
    rom[1022] = 8'h99;
    rom[1023] = 8'hA6;
    rom[1024] = 8'h02;
    rom[1025] = 8'hB7;
    rom[1026] = 8'hB9;
    rom[1027] = 8'hB6;
    rom[1028] = 8'h93;
    rom[1029] = 8'hB7;
    rom[1030] = 8'hBA;
    rom[1031] = 8'hA6;
    rom[1032] = 8'h01;
    rom[1033] = 8'hB7;
    rom[1034] = 8'h97;
    rom[1035] = 8'hB7;
    rom[1036] = 8'h96;
    rom[1037] = 8'hCC;
    rom[1038] = 8'h14;
    rom[1039] = 8'h45;
    rom[1040] = 8'hB6;
    rom[1041] = 8'hBA;
    rom[1042] = 8'hA1;
    rom[1043] = 8'h20;
    rom[1044] = 8'h24;
    rom[1045] = 8'h09;
    rom[1046] = 8'hB7;
    rom[1047] = 8'h93;
    rom[1048] = 8'h48;
    rom[1049] = 8'hBB;
    rom[1050] = 8'h93;
    rom[1051] = 8'h97;
    rom[1052] = 8'hDC;
    rom[1053] = 8'h13;
    rom[1054] = 8'h2D;
    rom[1055] = 8'h3F;
    rom[1056] = 8'hB5;
    rom[1057] = 8'hCD;
    rom[1058] = 8'h14;
    rom[1059] = 8'hC8;
    rom[1060] = 8'h24;
    rom[1061] = 8'hF9;
    rom[1062] = 8'hA6;
    rom[1063] = 8'h02;
    rom[1064] = 8'hCC;
    rom[1065] = 8'h12;
    rom[1066] = 8'hF5;
    rom[1067] = 8'h99;
    rom[1068] = 8'h81;
    rom[1069] = 8'hCC;
    rom[1070] = 8'h16;
    rom[1071] = 8'h77;
    rom[1072] = 8'hCC;
    rom[1073] = 8'h16;
    rom[1074] = 8'h84;
    rom[1075] = 8'hCC;
    rom[1076] = 8'h16;
    rom[1077] = 8'hA1;
    rom[1078] = 8'hCC;
    rom[1079] = 8'h16;
    rom[1080] = 8'hD2;
    rom[1081] = 8'hCC;
    rom[1082] = 8'h17;
    rom[1083] = 8'h0A;
    rom[1084] = 8'hCC;
    rom[1085] = 8'h17;
    rom[1086] = 8'h26;
    rom[1087] = 8'hCC;
    rom[1088] = 8'h17;
    rom[1089] = 8'h43;
    rom[1090] = 8'hCC;
    rom[1091] = 8'h17;
    rom[1092] = 8'h60;
    rom[1093] = 8'hCC;
    rom[1094] = 8'h17;
    rom[1095] = 8'hCF;
    rom[1096] = 8'hCC;
    rom[1097] = 8'h17;
    rom[1098] = 8'hF7;
    rom[1099] = 8'hCC;
    rom[1100] = 8'h18;
    rom[1101] = 8'h0F;
    rom[1102] = 8'hCC;
    rom[1103] = 8'h18;
    rom[1104] = 8'h2A;
    rom[1105] = 8'hCC;
    rom[1106] = 8'h18;
    rom[1107] = 8'h4A;
    rom[1108] = 8'hCC;
    rom[1109] = 8'h18;
    rom[1110] = 8'h89;
    rom[1111] = 8'hCC;
    rom[1112] = 8'h18;
    rom[1113] = 8'h9E;
    rom[1114] = 8'hCC;
    rom[1115] = 8'h18;
    rom[1116] = 8'hDB;
    rom[1117] = 8'hCC;
    rom[1118] = 8'h19;
    rom[1119] = 8'hA7;
    rom[1120] = 8'hCC;
    rom[1121] = 8'h19;
    rom[1122] = 8'hCA;
    rom[1123] = 8'hCC;
    rom[1124] = 8'h19;
    rom[1125] = 8'hD8;
    rom[1126] = 8'hCC;
    rom[1127] = 8'h19;
    rom[1128] = 8'hF5;
    rom[1129] = 8'hCC;
    rom[1130] = 8'h1A;
    rom[1131] = 8'h12;
    rom[1132] = 8'hCC;
    rom[1133] = 8'h1A;
    rom[1134] = 8'h23;
    rom[1135] = 8'hCC;
    rom[1136] = 8'h1A;
    rom[1137] = 8'h40;
    rom[1138] = 8'hCC;
    rom[1139] = 8'h1A;
    rom[1140] = 8'h56;
    rom[1141] = 8'hCC;
    rom[1142] = 8'h1A;
    rom[1143] = 8'h67;
    rom[1144] = 8'hCC;
    rom[1145] = 8'h1A;
    rom[1146] = 8'h7D;
    rom[1147] = 8'hCC;
    rom[1148] = 8'h1A;
    rom[1149] = 8'h92;
    rom[1150] = 8'hCC;
    rom[1151] = 8'h1A;
    rom[1152] = 8'hAF;
    rom[1153] = 8'hCC;
    rom[1154] = 8'h1B;
    rom[1155] = 8'h11;
    rom[1156] = 8'hCC;
    rom[1157] = 8'h1B;
    rom[1158] = 8'h26;
    rom[1159] = 8'hCC;
    rom[1160] = 8'h1B;
    rom[1161] = 8'h3B;
    rom[1162] = 8'hCC;
    rom[1163] = 8'h1B;
    rom[1164] = 8'h4F;
    rom[1165] = 8'hB6;
    rom[1166] = 8'hBA;
    rom[1167] = 8'hA4;
    rom[1168] = 8'h0F;
    rom[1169] = 8'h27;
    rom[1170] = 8'h3E;
    rom[1171] = 8'hA1;
    rom[1172] = 8'h01;
    rom[1173] = 8'h27;
    rom[1174] = 8'h66;
    rom[1175] = 8'hA1;
    rom[1176] = 8'h08;
    rom[1177] = 8'h25;
    rom[1178] = 8'h84;
    rom[1179] = 8'hA1;
    rom[1180] = 8'h0C;
    rom[1181] = 8'h24;
    rom[1182] = 8'h7C;
    rom[1183] = 8'hBE;
    rom[1184] = 8'hB5;
    rom[1185] = 8'hA3;
    rom[1186] = 8'h0B;
    rom[1187] = 8'h25;
    rom[1188] = 8'h02;
    rom[1189] = 8'h3A;
    rom[1190] = 8'hB5;
    rom[1191] = 8'hCD;
    rom[1192] = 8'h14;
    rom[1193] = 8'hC8;
    rom[1194] = 8'h24;
    rom[1195] = 8'hF3;
    rom[1196] = 8'hCD;
    rom[1197] = 8'h14;
    rom[1198] = 8'h8B;
    rom[1199] = 8'hBE;
    rom[1200] = 8'hB5;
    rom[1201] = 8'hA3;
    rom[1202] = 8'h04;
    rom[1203] = 8'h25;
    rom[1204] = 8'h17;
    rom[1205] = 8'h5A;
    rom[1206] = 8'hBF;
    rom[1207] = 8'h96;
    rom[1208] = 8'h5A;
    rom[1209] = 8'hA3;
    rom[1210] = 8'h09;
    rom[1211] = 8'h24;
    rom[1212] = 8'h0F;
    rom[1213] = 8'hE6;
    rom[1214] = 8'hBB;
    rom[1215] = 8'hE7;
    rom[1216] = 8'h99;
    rom[1217] = 8'h5A;
    rom[1218] = 8'h2A;
    rom[1219] = 8'hF9;
    rom[1220] = 8'hB6;
    rom[1221] = 8'hBA;
    rom[1222] = 8'hCD;
    rom[1223] = 8'h1C;
    rom[1224] = 8'h71;
    rom[1225] = 8'hCC;
    rom[1226] = 8'h16;
    rom[1227] = 8'h40;
    rom[1228] = 8'hA6;
    rom[1229] = 8'h03;
    rom[1230] = 8'hCC;
    rom[1231] = 8'h12;
    rom[1232] = 8'hF5;
    rom[1233] = 8'hCD;
    rom[1234] = 8'h14;
    rom[1235] = 8'hC8;
    rom[1236] = 8'h24;
    rom[1237] = 8'h19;
    rom[1238] = 8'hA6;
    rom[1239] = 8'h30;
    rom[1240] = 8'hB7;
    rom[1241] = 8'hB3;
    rom[1242] = 8'hA6;
    rom[1243] = 8'h20;
    rom[1244] = 8'hB7;
    rom[1245] = 8'hB4;
    rom[1246] = 8'h1E;
    rom[1247] = 8'h00;
    rom[1248] = 8'hAE;
    rom[1249] = 8'h03;
    rom[1250] = 8'hCD;
    rom[1251] = 8'h1D;
    rom[1252] = 8'hE2;
    rom[1253] = 8'h1F;
    rom[1254] = 8'h00;
    rom[1255] = 8'hAE;
    rom[1256] = 8'h1B;
    rom[1257] = 8'hCD;
    rom[1258] = 8'h1D;
    rom[1259] = 8'hD1;
    rom[1260] = 8'hCC;
    rom[1261] = 8'h16;
    rom[1262] = 8'h40;
    rom[1263] = 8'hA6;
    rom[1264] = 8'h03;
    rom[1265] = 8'hB7;
    rom[1266] = 8'hB5;
    rom[1267] = 8'hCD;
    rom[1268] = 8'h14;
    rom[1269] = 8'hC8;
    rom[1270] = 8'h24;
    rom[1271] = 8'h35;
    rom[1272] = 8'hA6;
    rom[1273] = 8'h03;
    rom[1274] = 8'hCC;
    rom[1275] = 8'h12;
    rom[1276] = 8'hF5;
    rom[1277] = 8'hCD;
    rom[1278] = 8'h14;
    rom[1279] = 8'hC8;
    rom[1280] = 8'h24;
    rom[1281] = 8'h0B;
    rom[1282] = 8'hCD;
    rom[1283] = 8'h14;
    rom[1284] = 8'h8B;
    rom[1285] = 8'hB6;
    rom[1286] = 8'hBA;
    rom[1287] = 8'hCD;
    rom[1288] = 8'h1C;
    rom[1289] = 8'h71;
    rom[1290] = 8'hCC;
    rom[1291] = 8'h16;
    rom[1292] = 8'h40;
    rom[1293] = 8'hA6;
    rom[1294] = 8'h03;
    rom[1295] = 8'hB7;
    rom[1296] = 8'hB5;
    rom[1297] = 8'hCD;
    rom[1298] = 8'h14;
    rom[1299] = 8'hC8;
    rom[1300] = 8'h24;
    rom[1301] = 8'h17;
    rom[1302] = 8'hA6;
    rom[1303] = 8'h03;
    rom[1304] = 8'hCC;
    rom[1305] = 8'h12;
    rom[1306] = 8'hF5;
    rom[1307] = 8'hCD;
    rom[1308] = 8'h14;
    rom[1309] = 8'hC8;
    rom[1310] = 8'h24;
    rom[1311] = 8'h0D;
    rom[1312] = 8'hCD;
    rom[1313] = 8'h14;
    rom[1314] = 8'h8B;
    rom[1315] = 8'hB6;
    rom[1316] = 8'hBA;
    rom[1317] = 8'hCD;
    rom[1318] = 8'h1C;
    rom[1319] = 8'h71;
    rom[1320] = 8'hCD;
    rom[1321] = 8'h14;
    rom[1322] = 8'h3B;
    rom[1323] = 8'h98;
    rom[1324] = 8'h81;
    rom[1325] = 8'hA6;
    rom[1326] = 8'h03;
    rom[1327] = 8'hB7;
    rom[1328] = 8'hB5;
    rom[1329] = 8'hCD;
    rom[1330] = 8'h14;
    rom[1331] = 8'hC8;
    rom[1332] = 8'h24;
    rom[1333] = 8'hF7;
    rom[1334] = 8'hA6;
    rom[1335] = 8'h03;
    rom[1336] = 8'hCC;
    rom[1337] = 8'h12;
    rom[1338] = 8'hF5;
    rom[1339] = 8'h3F;
    rom[1340] = 8'hB9;
    rom[1341] = 8'hB6;
    rom[1342] = 8'h95;
    rom[1343] = 8'hB7;
    rom[1344] = 8'hBA;
    rom[1345] = 8'hB6;
    rom[1346] = 8'h98;
    rom[1347] = 8'hB7;
    rom[1348] = 8'hBB;
    rom[1349] = 8'hCD;
    rom[1350] = 8'h15;
    rom[1351] = 8'h49;
    rom[1352] = 8'hB6;
    rom[1353] = 8'hB9;
    rom[1354] = 8'hBE;
    rom[1355] = 8'hBA;
    rom[1356] = 8'hCD;
    rom[1357] = 8'h15;
    rom[1358] = 8'h86;
    rom[1359] = 8'h25;
    rom[1360] = 8'h38;
    rom[1361] = 8'hB6;
    rom[1362] = 8'h96;
    rom[1363] = 8'h26;
    rom[1364] = 8'h05;
    rom[1365] = 8'hB6;
    rom[1366] = 8'hBB;
    rom[1367] = 8'hCC;
    rom[1368] = 8'h16;
    rom[1369] = 8'h4D;
    rom[1370] = 8'hB6;
    rom[1371] = 8'hBB;
    rom[1372] = 8'hCD;
    rom[1373] = 8'h15;
    rom[1374] = 8'h93;
    rom[1375] = 8'h25;
    rom[1376] = 8'h28;
    rom[1377] = 8'h3F;
    rom[1378] = 8'h97;
    rom[1379] = 8'hB6;
    rom[1380] = 8'h96;
    rom[1381] = 8'h27;
    rom[1382] = 8'h22;
    rom[1383] = 8'hBE;
    rom[1384] = 8'h97;
    rom[1385] = 8'h9F;
    rom[1386] = 8'h4C;
    rom[1387] = 8'hB1;
    rom[1388] = 8'h96;
    rom[1389] = 8'h26;
    rom[1390] = 8'h02;
    rom[1391] = 8'h12;
    rom[1392] = 8'h01;
    rom[1393] = 8'hE6;
    rom[1394] = 8'h99;
    rom[1395] = 8'hCD;
    rom[1396] = 8'h15;
    rom[1397] = 8'h93;
    rom[1398] = 8'h25;
    rom[1399] = 8'h11;
    rom[1400] = 8'h3C;
    rom[1401] = 8'h97;
    rom[1402] = 8'hB6;
    rom[1403] = 8'h97;
    rom[1404] = 8'hB1;
    rom[1405] = 8'h96;
    rom[1406] = 8'h26;
    rom[1407] = 8'hE7;
    rom[1408] = 8'h01;
    rom[1409] = 8'h01;
    rom[1410] = 8'h51;
    rom[1411] = 8'hCD;
    rom[1412] = 8'h11;
    rom[1413] = 8'h49;
    rom[1414] = 8'h06;
    rom[1415] = 8'h01;
    rom[1416] = 8'hF7;
    rom[1417] = 8'h98;
    rom[1418] = 8'h81;
    rom[1419] = 8'hB6;
    rom[1420] = 8'hBA;
    rom[1421] = 8'hCD;
    rom[1422] = 8'h11;
    rom[1423] = 8'h21;
    rom[1424] = 8'h24;
    rom[1425] = 8'h0E;
    rom[1426] = 8'hB6;
    rom[1427] = 8'hBA;
    rom[1428] = 8'hA4;
    rom[1429] = 8'hF0;
    rom[1430] = 8'hB1;
    rom[1431] = 8'hB3;
    rom[1432] = 8'h27;
    rom[1433] = 8'h06;
    rom[1434] = 8'hBE;
    rom[1435] = 8'hB3;
    rom[1436] = 8'hB7;
    rom[1437] = 8'hB3;
    rom[1438] = 8'hBF;
    rom[1439] = 8'hB4;
    rom[1440] = 8'h81;
    rom[1441] = 8'hB7;
    rom[1442] = 8'hB6;
    rom[1443] = 8'h3F;
    rom[1444] = 8'hB5;
    rom[1445] = 8'hBE;
    rom[1446] = 8'hB5;
    rom[1447] = 8'hB3;
    rom[1448] = 8'hB6;
    rom[1449] = 8'h27;
    rom[1450] = 8'h0F;
    rom[1451] = 8'hCD;
    rom[1452] = 8'h14;
    rom[1453] = 8'hC8;
    rom[1454] = 8'h24;
    rom[1455] = 8'hF5;
    rom[1456] = 8'hBE;
    rom[1457] = 8'hB5;
    rom[1458] = 8'hB6;
    rom[1459] = 8'hB6;
    rom[1460] = 8'hB3;
    rom[1461] = 8'hB6;
    rom[1462] = 8'h27;
    rom[1463] = 8'h01;
    rom[1464] = 8'h99;
    rom[1465] = 8'h81;
    rom[1466] = 8'hCD;
    rom[1467] = 8'h14;
    rom[1468] = 8'hC8;
    rom[1469] = 8'h25;
    rom[1470] = 8'hF1;
    rom[1471] = 8'hCD;
    rom[1472] = 8'h14;
    rom[1473] = 8'hC8;
    rom[1474] = 8'h25;
    rom[1475] = 8'hEC;
    rom[1476] = 8'h3A;
    rom[1477] = 8'hB5;
    rom[1478] = 8'h20;
    rom[1479] = 8'hF7;
    rom[1480] = 8'hCD;
    rom[1481] = 8'h11;
    rom[1482] = 8'h49;
    rom[1483] = 8'h01;
    rom[1484] = 8'h01;
    rom[1485] = 8'h06;
    rom[1486] = 8'h04;
    rom[1487] = 8'h01;
    rom[1488] = 8'h05;
    rom[1489] = 8'h06;
    rom[1490] = 8'h01;
    rom[1491] = 8'hF4;
    rom[1492] = 8'h99;
    rom[1493] = 8'h81;
    rom[1494] = 8'hCD;
    rom[1495] = 8'h14;
    rom[1496] = 8'hEC;
    rom[1497] = 8'hBE;
    rom[1498] = 8'hB5;
    rom[1499] = 8'hE7;
    rom[1500] = 8'hB9;
    rom[1501] = 8'h3C;
    rom[1502] = 8'hB5;
    rom[1503] = 8'hCD;
    rom[1504] = 8'h11;
    rom[1505] = 8'h49;
    rom[1506] = 8'h01;
    rom[1507] = 8'h01;
    rom[1508] = 8'hEF;
    rom[1509] = 8'h05;
    rom[1510] = 8'h01;
    rom[1511] = 8'h03;
    rom[1512] = 8'h0A;
    rom[1513] = 8'hA3;
    rom[1514] = 8'hF4;
    rom[1515] = 8'h81;
    rom[1516] = 8'h9B;
    rom[1517] = 8'h1B;
    rom[1518] = 8'h05;
    rom[1519] = 8'h19;
    rom[1520] = 8'h01;
    rom[1521] = 8'h18;
    rom[1522] = 8'h01;
    rom[1523] = 8'h0A;
    rom[1524] = 8'h01;
    rom[1525] = 8'h00;
    rom[1526] = 8'h49;
    rom[1527] = 8'h19;
    rom[1528] = 8'h01;
    rom[1529] = 8'h18;
    rom[1530] = 8'h01;
    rom[1531] = 8'h0A;
    rom[1532] = 8'h01;
    rom[1533] = 8'h00;
    rom[1534] = 8'h49;
    rom[1535] = 8'h19;
    rom[1536] = 8'h01;
    rom[1537] = 8'h18;
    rom[1538] = 8'h01;
    rom[1539] = 8'h0A;
    rom[1540] = 8'h01;
    rom[1541] = 8'h00;
    rom[1542] = 8'h49;
    rom[1543] = 8'h19;
    rom[1544] = 8'h01;
    rom[1545] = 8'h18;
    rom[1546] = 8'h01;
    rom[1547] = 8'h0A;
    rom[1548] = 8'h01;
    rom[1549] = 8'h00;
    rom[1550] = 8'h49;
    rom[1551] = 8'h19;
    rom[1552] = 8'h01;
    rom[1553] = 8'h18;
    rom[1554] = 8'h01;
    rom[1555] = 8'h0A;
    rom[1556] = 8'h01;
    rom[1557] = 8'h00;
    rom[1558] = 8'h49;
    rom[1559] = 8'h19;
    rom[1560] = 8'h01;
    rom[1561] = 8'h18;
    rom[1562] = 8'h01;
    rom[1563] = 8'h0A;
    rom[1564] = 8'h01;
    rom[1565] = 8'h00;
    rom[1566] = 8'h49;
    rom[1567] = 8'h19;
    rom[1568] = 8'h01;
    rom[1569] = 8'h18;
    rom[1570] = 8'h01;
    rom[1571] = 8'h0A;
    rom[1572] = 8'h01;
    rom[1573] = 8'h00;
    rom[1574] = 8'h49;
    rom[1575] = 8'h19;
    rom[1576] = 8'h01;
    rom[1577] = 8'h18;
    rom[1578] = 8'h01;
    rom[1579] = 8'h0A;
    rom[1580] = 8'h01;
    rom[1581] = 8'h00;
    rom[1582] = 8'h49;
    rom[1583] = 8'h98;
    rom[1584] = 8'h19;
    rom[1585] = 8'hA3;
    rom[1586] = 8'h1A;
    rom[1587] = 8'hA3;
    rom[1588] = 8'h9A;
    rom[1589] = 8'h81;
    rom[1590] = 8'hB6;
    rom[1591] = 8'hB9;
    rom[1592] = 8'hB7;
    rom[1593] = 8'hB7;
    rom[1594] = 8'hB6;
    rom[1595] = 8'hBA;
    rom[1596] = 8'hB7;
    rom[1597] = 8'hB8;
    rom[1598] = 8'h3F;
    rom[1599] = 8'hB5;
    rom[1600] = 8'hCD;
    rom[1601] = 8'h14;
    rom[1602] = 8'hC8;
    rom[1603] = 8'h25;
    rom[1604] = 8'h03;
    rom[1605] = 8'hCD;
    rom[1606] = 8'h14;
    rom[1607] = 8'hC8;
    rom[1608] = 8'h81;
    rom[1609] = 8'h13;
    rom[1610] = 8'h01;
    rom[1611] = 8'hAE;
    rom[1612] = 8'h04;
    rom[1613] = 8'hCD;
    rom[1614] = 8'h1D;
    rom[1615] = 8'hD1;
    rom[1616] = 8'h19;
    rom[1617] = 8'h01;
    rom[1618] = 8'h18;
    rom[1619] = 8'h01;
    rom[1620] = 8'h19;
    rom[1621] = 8'h01;
    rom[1622] = 8'h18;
    rom[1623] = 8'h01;
    rom[1624] = 8'h19;
    rom[1625] = 8'h01;
    rom[1626] = 8'h18;
    rom[1627] = 8'h01;
    rom[1628] = 8'h19;
    rom[1629] = 8'h01;
    rom[1630] = 8'h18;
    rom[1631] = 8'h01;
    rom[1632] = 8'h19;
    rom[1633] = 8'h01;
    rom[1634] = 8'h18;
    rom[1635] = 8'h01;
    rom[1636] = 8'h19;
    rom[1637] = 8'h01;
    rom[1638] = 8'h18;
    rom[1639] = 8'h01;
    rom[1640] = 8'h19;
    rom[1641] = 8'h01;
    rom[1642] = 8'h18;
    rom[1643] = 8'h01;
    rom[1644] = 8'h19;
    rom[1645] = 8'h01;
    rom[1646] = 8'h18;
    rom[1647] = 8'h01;
    rom[1648] = 8'hCD;
    rom[1649] = 8'h11;
    rom[1650] = 8'h49;
    rom[1651] = 8'h01;
    rom[1652] = 8'h01;
    rom[1653] = 8'h0D;
    rom[1654] = 8'h06;
    rom[1655] = 8'h01;
    rom[1656] = 8'hF7;
    rom[1657] = 8'hCD;
    rom[1658] = 8'h11;
    rom[1659] = 8'h49;
    rom[1660] = 8'h01;
    rom[1661] = 8'h01;
    rom[1662] = 8'h04;
    rom[1663] = 8'h07;
    rom[1664] = 8'h01;
    rom[1665] = 8'hF7;
    rom[1666] = 8'h81;
    rom[1667] = 8'hCC;
    rom[1668] = 8'h0F;
    rom[1669] = 8'hAF;
    rom[1670] = 8'hBF;
    rom[1671] = 8'h94;
    rom[1672] = 8'hCD;
    rom[1673] = 8'h15;
    rom[1674] = 8'h93;
    rom[1675] = 8'h25;
    rom[1676] = 8'h05;
    rom[1677] = 8'hB6;
    rom[1678] = 8'h94;
    rom[1679] = 8'hCD;
    rom[1680] = 8'h15;
    rom[1681] = 8'h93;
    rom[1682] = 8'h81;
    rom[1683] = 8'hCD;
    rom[1684] = 8'h11;
    rom[1685] = 8'h49;
    rom[1686] = 8'h01;
    rom[1687] = 8'h01;
    rom[1688] = 8'h06;
    rom[1689] = 8'h05;
    rom[1690] = 8'h01;
    rom[1691] = 8'h07;
    rom[1692] = 8'h06;
    rom[1693] = 8'h01;
    rom[1694] = 8'hF4;
    rom[1695] = 8'h12;
    rom[1696] = 8'h01;
    rom[1697] = 8'h99;
    rom[1698] = 8'h81;
    rom[1699] = 8'hCD;
    rom[1700] = 8'h15;
    rom[1701] = 8'hC9;
    rom[1702] = 8'hCD;
    rom[1703] = 8'h11;
    rom[1704] = 8'h49;
    rom[1705] = 8'h01;
    rom[1706] = 8'h01;
    rom[1707] = 8'hF3;
    rom[1708] = 8'h04;
    rom[1709] = 8'h01;
    rom[1710] = 8'h0B;
    rom[1711] = 8'h08;
    rom[1712] = 8'hA3;
    rom[1713] = 8'h08;
    rom[1714] = 8'h04;
    rom[1715] = 8'h01;
    rom[1716] = 8'h05;
    rom[1717] = 8'h07;
    rom[1718] = 8'h01;
    rom[1719] = 8'hE7;
    rom[1720] = 8'h20;
    rom[1721] = 8'hEC;
    rom[1722] = 8'hCD;
    rom[1723] = 8'h11;
    rom[1724] = 8'h49;
    rom[1725] = 8'h01;
    rom[1726] = 8'h01;
    rom[1727] = 8'hDF;
    rom[1728] = 8'h05;
    rom[1729] = 8'h01;
    rom[1730] = 8'h05;
    rom[1731] = 8'h07;
    rom[1732] = 8'h01;
    rom[1733] = 8'hD9;
    rom[1734] = 8'h20;
    rom[1735] = 8'hF2;
    rom[1736] = 8'h81;
    rom[1737] = 8'h9B;
    rom[1738] = 8'h1A;
    rom[1739] = 8'h05;
    rom[1740] = 8'h19;
    rom[1741] = 8'h01;
    rom[1742] = 8'h49;
    rom[1743] = 8'h24;
    rom[1744] = 8'h04;
    rom[1745] = 8'h1A;
    rom[1746] = 8'h01;
    rom[1747] = 8'h20;
    rom[1748] = 8'h02;
    rom[1749] = 8'h1B;
    rom[1750] = 8'h01;
    rom[1751] = 8'h18;
    rom[1752] = 8'h01;
    rom[1753] = 8'h19;
    rom[1754] = 8'h01;
    rom[1755] = 8'h49;
    rom[1756] = 8'h24;
    rom[1757] = 8'h04;
    rom[1758] = 8'h1A;
    rom[1759] = 8'h01;
    rom[1760] = 8'h20;
    rom[1761] = 8'h02;
    rom[1762] = 8'h1B;
    rom[1763] = 8'h01;
    rom[1764] = 8'h18;
    rom[1765] = 8'h01;
    rom[1766] = 8'h19;
    rom[1767] = 8'h01;
    rom[1768] = 8'h49;
    rom[1769] = 8'h24;
    rom[1770] = 8'h04;
    rom[1771] = 8'h1A;
    rom[1772] = 8'h01;
    rom[1773] = 8'h20;
    rom[1774] = 8'h02;
    rom[1775] = 8'h1B;
    rom[1776] = 8'h01;
    rom[1777] = 8'h18;
    rom[1778] = 8'h01;
    rom[1779] = 8'h19;
    rom[1780] = 8'h01;
    rom[1781] = 8'h49;
    rom[1782] = 8'h24;
    rom[1783] = 8'h04;
    rom[1784] = 8'h1A;
    rom[1785] = 8'h01;
    rom[1786] = 8'h20;
    rom[1787] = 8'h02;
    rom[1788] = 8'h1B;
    rom[1789] = 8'h01;
    rom[1790] = 8'h18;
    rom[1791] = 8'h01;
    rom[1792] = 8'h19;
    rom[1793] = 8'h01;
    rom[1794] = 8'h49;
    rom[1795] = 8'h24;
    rom[1796] = 8'h04;
    rom[1797] = 8'h1A;
    rom[1798] = 8'h01;
    rom[1799] = 8'h20;
    rom[1800] = 8'h02;
    rom[1801] = 8'h1B;
    rom[1802] = 8'h01;
    rom[1803] = 8'h18;
    rom[1804] = 8'h01;
    rom[1805] = 8'h19;
    rom[1806] = 8'h01;
    rom[1807] = 8'h49;
    rom[1808] = 8'h24;
    rom[1809] = 8'h04;
    rom[1810] = 8'h1A;
    rom[1811] = 8'h01;
    rom[1812] = 8'h20;
    rom[1813] = 8'h02;
    rom[1814] = 8'h1B;
    rom[1815] = 8'h01;
    rom[1816] = 8'h18;
    rom[1817] = 8'h01;
    rom[1818] = 8'h19;
    rom[1819] = 8'h01;
    rom[1820] = 8'h49;
    rom[1821] = 8'h24;
    rom[1822] = 8'h04;
    rom[1823] = 8'h1A;
    rom[1824] = 8'h01;
    rom[1825] = 8'h20;
    rom[1826] = 8'h02;
    rom[1827] = 8'h1B;
    rom[1828] = 8'h01;
    rom[1829] = 8'h18;
    rom[1830] = 8'h01;
    rom[1831] = 8'h19;
    rom[1832] = 8'h01;
    rom[1833] = 8'h49;
    rom[1834] = 8'h24;
    rom[1835] = 8'h04;
    rom[1836] = 8'h1A;
    rom[1837] = 8'h01;
    rom[1838] = 8'h20;
    rom[1839] = 8'h02;
    rom[1840] = 8'h1B;
    rom[1841] = 8'h01;
    rom[1842] = 8'h18;
    rom[1843] = 8'h01;
    rom[1844] = 8'hCD;
    rom[1845] = 8'h1D;
    rom[1846] = 8'hF1;
    rom[1847] = 8'h1B;
    rom[1848] = 8'h05;
    rom[1849] = 8'h98;
    rom[1850] = 8'h19;
    rom[1851] = 8'hA3;
    rom[1852] = 8'h1A;
    rom[1853] = 8'hA3;
    rom[1854] = 8'h9A;
    rom[1855] = 8'h81;
    rom[1856] = 8'hCD;
    rom[1857] = 8'h15;
    rom[1858] = 8'h49;
    rom[1859] = 8'hB6;
    rom[1860] = 8'hB7;
    rom[1861] = 8'h5F;
    rom[1862] = 8'hCD;
    rom[1863] = 8'h15;
    rom[1864] = 8'h86;
    rom[1865] = 8'h25;
    rom[1866] = 8'h07;
    rom[1867] = 8'hB6;
    rom[1868] = 8'hB8;
    rom[1869] = 8'h12;
    rom[1870] = 8'h01;
    rom[1871] = 8'hCD;
    rom[1872] = 8'h15;
    rom[1873] = 8'h93;
    rom[1874] = 8'h12;
    rom[1875] = 8'h01;
    rom[1876] = 8'h01;
    rom[1877] = 8'h01;
    rom[1878] = 8'h0A;
    rom[1879] = 8'h06;
    rom[1880] = 8'h01;
    rom[1881] = 8'h02;
    rom[1882] = 8'h99;
    rom[1883] = 8'h81;
    rom[1884] = 8'hCD;
    rom[1885] = 8'h11;
    rom[1886] = 8'h49;
    rom[1887] = 8'h20;
    rom[1888] = 8'hF3;
    rom[1889] = 8'h99;
    rom[1890] = 8'h81;
    rom[1891] = 8'hCD;
    rom[1892] = 8'h15;
    rom[1893] = 8'h49;
    rom[1894] = 8'hB6;
    rom[1895] = 8'hB7;
    rom[1896] = 8'h5F;
    rom[1897] = 8'hCD;
    rom[1898] = 8'h15;
    rom[1899] = 8'h86;
    rom[1900] = 8'h25;
    rom[1901] = 8'h06;
    rom[1902] = 8'hB6;
    rom[1903] = 8'hB8;
    rom[1904] = 8'hCD;
    rom[1905] = 8'h15;
    rom[1906] = 8'h93;
    rom[1907] = 8'h81;
    rom[1908] = 8'h12;
    rom[1909] = 8'h01;
    rom[1910] = 8'h81;
    rom[1911] = 8'h4F;
    rom[1912] = 8'hCD;
    rom[1913] = 8'h14;
    rom[1914] = 8'hA1;
    rom[1915] = 8'h1F;
    rom[1916] = 8'hA2;
    rom[1917] = 8'h19;
    rom[1918] = 8'hA2;
    rom[1919] = 8'h3F;
    rom[1920] = 8'hCB;
    rom[1921] = 8'hCC;
    rom[1922] = 8'h16;
    rom[1923] = 8'h40;
    rom[1924] = 8'hA6;
    rom[1925] = 8'h01;
    rom[1926] = 8'hCD;
    rom[1927] = 8'h14;
    rom[1928] = 8'hA1;
    rom[1929] = 8'h24;
    rom[1930] = 8'h03;
    rom[1931] = 8'hCC;
    rom[1932] = 8'h12;
    rom[1933] = 8'hE6;
    rom[1934] = 8'h1F;
    rom[1935] = 8'hA2;
    rom[1936] = 8'hB6;
    rom[1937] = 8'hB9;
    rom[1938] = 8'h27;
    rom[1939] = 8'h0A;
    rom[1940] = 8'h1E;
    rom[1941] = 8'hA2;
    rom[1942] = 8'hA6;
    rom[1943] = 8'h30;
    rom[1944] = 8'hB7;
    rom[1945] = 8'hB3;
    rom[1946] = 8'hA6;
    rom[1947] = 8'h20;
    rom[1948] = 8'hB7;
    rom[1949] = 8'hB4;
    rom[1950] = 8'hCC;
    rom[1951] = 8'h16;
    rom[1952] = 8'h40;
    rom[1953] = 8'hA6;
    rom[1954] = 8'h02;
    rom[1955] = 8'hCD;
    rom[1956] = 8'h14;
    rom[1957] = 8'hA1;
    rom[1958] = 8'h24;
    rom[1959] = 8'h03;
    rom[1960] = 8'hCC;
    rom[1961] = 8'h12;
    rom[1962] = 8'hE6;
    rom[1963] = 8'hCD;
    rom[1964] = 8'h16;
    rom[1965] = 8'h63;
    rom[1966] = 8'h25;
    rom[1967] = 8'h1F;
    rom[1968] = 8'hA6;
    rom[1969] = 8'hC6;
    rom[1970] = 8'hB7;
    rom[1971] = 8'hA7;
    rom[1972] = 8'hB6;
    rom[1973] = 8'hB9;
    rom[1974] = 8'hB7;
    rom[1975] = 8'hA8;
    rom[1976] = 8'hB6;
    rom[1977] = 8'hBA;
    rom[1978] = 8'hB7;
    rom[1979] = 8'hA9;
    rom[1980] = 8'hA6;
    rom[1981] = 8'h81;
    rom[1982] = 8'hB7;
    rom[1983] = 8'hAA;
    rom[1984] = 8'hBD;
    rom[1985] = 8'hA7;
    rom[1986] = 8'hCD;
    rom[1987] = 8'h15;
    rom[1988] = 8'h93;
    rom[1989] = 8'h25;
    rom[1990] = 8'h08;
    rom[1991] = 8'h3C;
    rom[1992] = 8'hA9;
    rom[1993] = 8'h26;
    rom[1994] = 8'hF5;
    rom[1995] = 8'h3C;
    rom[1996] = 8'hA8;
    rom[1997] = 8'h20;
    rom[1998] = 8'hF1;
    rom[1999] = 8'hCC;
    rom[2000] = 8'h16;
    rom[2001] = 8'h52;
    rom[2002] = 8'h4F;
    rom[2003] = 8'hCD;
    rom[2004] = 8'h14;
    rom[2005] = 8'hA1;
    rom[2006] = 8'h24;
    rom[2007] = 8'h03;
    rom[2008] = 8'hCC;
    rom[2009] = 8'h12;
    rom[2010] = 8'hE6;
    rom[2011] = 8'hCD;
    rom[2012] = 8'h16;
    rom[2013] = 8'h63;
    rom[2014] = 8'h25;
    rom[2015] = 8'h27;
    rom[2016] = 8'hAE;
    rom[2017] = 8'h03;
    rom[2018] = 8'h9B;
    rom[2019] = 8'hE6;
    rom[2020] = 8'hAB;
    rom[2021] = 8'hE7;
    rom[2022] = 8'hB9;
    rom[2023] = 8'h5A;
    rom[2024] = 8'h2A;
    rom[2025] = 8'hF9;
    rom[2026] = 8'h11;
    rom[2027] = 8'hA2;
    rom[2028] = 8'h9A;
    rom[2029] = 8'hB6;
    rom[2030] = 8'hB9;
    rom[2031] = 8'hCD;
    rom[2032] = 8'h15;
    rom[2033] = 8'h93;
    rom[2034] = 8'h25;
    rom[2035] = 8'h13;
    rom[2036] = 8'hB6;
    rom[2037] = 8'hBA;
    rom[2038] = 8'hCD;
    rom[2039] = 8'h15;
    rom[2040] = 8'h93;
    rom[2041] = 8'h25;
    rom[2042] = 8'h0C;
    rom[2043] = 8'hB6;
    rom[2044] = 8'hBB;
    rom[2045] = 8'hCD;
    rom[2046] = 8'h15;
    rom[2047] = 8'h93;
    rom[2048] = 8'h25;
    rom[2049] = 8'h05;
    rom[2050] = 8'hB6;
    rom[2051] = 8'hBC;
    rom[2052] = 8'hCC;
    rom[2053] = 8'h16;
    rom[2054] = 8'h4D;
    rom[2055] = 8'hCC;
    rom[2056] = 8'h16;
    rom[2057] = 8'h52;
    rom[2058] = 8'h4F;
    rom[2059] = 8'hCD;
    rom[2060] = 8'h14;
    rom[2061] = 8'hA1;
    rom[2062] = 8'h24;
    rom[2063] = 8'h03;
    rom[2064] = 8'hCC;
    rom[2065] = 8'h12;
    rom[2066] = 8'hE6;
    rom[2067] = 8'hCD;
    rom[2068] = 8'h16;
    rom[2069] = 8'h63;
    rom[2070] = 8'h25;
    rom[2071] = 8'h0B;
    rom[2072] = 8'hA6;
    rom[2073] = 8'h10;
    rom[2074] = 8'hCD;
    rom[2075] = 8'h15;
    rom[2076] = 8'h93;
    rom[2077] = 8'h25;
    rom[2078] = 8'h04;
    rom[2079] = 8'h4F;
    rom[2080] = 8'hCC;
    rom[2081] = 8'h16;
    rom[2082] = 8'h4D;
    rom[2083] = 8'hCC;
    rom[2084] = 8'h16;
    rom[2085] = 8'h52;
    rom[2086] = 8'h4F;
    rom[2087] = 8'hCD;
    rom[2088] = 8'h14;
    rom[2089] = 8'hA1;
    rom[2090] = 8'h24;
    rom[2091] = 8'h03;
    rom[2092] = 8'hCC;
    rom[2093] = 8'h12;
    rom[2094] = 8'hE6;
    rom[2095] = 8'hCD;
    rom[2096] = 8'h16;
    rom[2097] = 8'h63;
    rom[2098] = 8'h25;
    rom[2099] = 8'h0C;
    rom[2100] = 8'hA6;
    rom[2101] = 8'h0F;
    rom[2102] = 8'hCD;
    rom[2103] = 8'h15;
    rom[2104] = 8'h93;
    rom[2105] = 8'h25;
    rom[2106] = 8'h05;
    rom[2107] = 8'hA6;
    rom[2108] = 8'h00;
    rom[2109] = 8'hCC;
    rom[2110] = 8'h16;
    rom[2111] = 8'h4D;
    rom[2112] = 8'hCC;
    rom[2113] = 8'h16;
    rom[2114] = 8'h52;
    rom[2115] = 8'h4F;
    rom[2116] = 8'hCD;
    rom[2117] = 8'h14;
    rom[2118] = 8'hA1;
    rom[2119] = 8'h24;
    rom[2120] = 8'h03;
    rom[2121] = 8'hCC;
    rom[2122] = 8'h12;
    rom[2123] = 8'hE6;
    rom[2124] = 8'hCD;
    rom[2125] = 8'h16;
    rom[2126] = 8'h63;
    rom[2127] = 8'h25;
    rom[2128] = 8'h0C;
    rom[2129] = 8'hA6;
    rom[2130] = 8'h0F;
    rom[2131] = 8'hCD;
    rom[2132] = 8'h15;
    rom[2133] = 8'h93;
    rom[2134] = 8'h25;
    rom[2135] = 8'h05;
    rom[2136] = 8'hA6;
    rom[2137] = 8'h5A;
    rom[2138] = 8'hCC;
    rom[2139] = 8'h16;
    rom[2140] = 8'h4D;
    rom[2141] = 8'hCC;
    rom[2142] = 8'h16;
    rom[2143] = 8'h52;
    rom[2144] = 8'hA6;
    rom[2145] = 8'h02;
    rom[2146] = 8'hCD;
    rom[2147] = 8'h14;
    rom[2148] = 8'hA1;
    rom[2149] = 8'h24;
    rom[2150] = 8'h03;
    rom[2151] = 8'hCC;
    rom[2152] = 8'h12;
    rom[2153] = 8'hE6;
    rom[2154] = 8'hB6;
    rom[2155] = 8'hB9;
    rom[2156] = 8'h27;
    rom[2157] = 8'h0A;
    rom[2158] = 8'hA1;
    rom[2159] = 8'h01;
    rom[2160] = 8'h26;
    rom[2161] = 8'h28;
    rom[2162] = 8'hB6;
    rom[2163] = 8'hBA;
    rom[2164] = 8'hA1;
    rom[2165] = 8'h08;
    rom[2166] = 8'h24;
    rom[2167] = 8'h22;
    rom[2168] = 8'hCD;
    rom[2169] = 8'h16;
    rom[2170] = 8'h63;
    rom[2171] = 8'h25;
    rom[2172] = 8'h1A;
    rom[2173] = 8'hA6;
    rom[2174] = 8'hC6;
    rom[2175] = 8'hB7;
    rom[2176] = 8'hA7;
    rom[2177] = 8'hA6;
    rom[2178] = 8'h81;
    rom[2179] = 8'hB7;
    rom[2180] = 8'hAA;
    rom[2181] = 8'hCD;
    rom[2182] = 8'h17;
    rom[2183] = 8'hA7;
    rom[2184] = 8'hBD;
    rom[2185] = 8'hA7;
    rom[2186] = 8'hCD;
    rom[2187] = 8'h15;
    rom[2188] = 8'h93;
    rom[2189] = 8'h25;
    rom[2190] = 8'h08;
    rom[2191] = 8'h3C;
    rom[2192] = 8'hBA;
    rom[2193] = 8'h26;
    rom[2194] = 8'hF2;
    rom[2195] = 8'h3C;
    rom[2196] = 8'hB9;
    rom[2197] = 8'h20;
    rom[2198] = 8'hEE;
    rom[2199] = 8'hCC;
    rom[2200] = 8'h16;
    rom[2201] = 8'h52;
    rom[2202] = 8'hB6;
    rom[2203] = 8'hB7;
    rom[2204] = 8'hB7;
    rom[2205] = 8'hB9;
    rom[2206] = 8'hB6;
    rom[2207] = 8'hB8;
    rom[2208] = 8'hB7;
    rom[2209] = 8'hBA;
    rom[2210] = 8'hA6;
    rom[2211] = 8'h04;
    rom[2212] = 8'hCC;
    rom[2213] = 8'h12;
    rom[2214] = 8'hF5;
    rom[2215] = 8'hB6;
    rom[2216] = 8'hB9;
    rom[2217] = 8'h27;
    rom[2218] = 8'h0E;
    rom[2219] = 8'hA1;
    rom[2220] = 8'h01;
    rom[2221] = 8'h26;
    rom[2222] = 8'h06;
    rom[2223] = 8'hB6;
    rom[2224] = 8'hBA;
    rom[2225] = 8'hA1;
    rom[2226] = 8'h08;
    rom[2227] = 8'h25;
    rom[2228] = 8'h04;
    rom[2229] = 8'h3F;
    rom[2230] = 8'hB9;
    rom[2231] = 8'h3F;
    rom[2232] = 8'hBA;
    rom[2233] = 8'hB6;
    rom[2234] = 8'hB9;
    rom[2235] = 8'hA4;
    rom[2236] = 8'h01;
    rom[2237] = 8'hA8;
    rom[2238] = 8'h01;
    rom[2239] = 8'hB7;
    rom[2240] = 8'hA8;
    rom[2241] = 8'h27;
    rom[2242] = 8'h05;
    rom[2243] = 8'hB6;
    rom[2244] = 8'hBA;
    rom[2245] = 8'hB7;
    rom[2246] = 8'hA9;
    rom[2247] = 8'h81;
    rom[2248] = 8'hB6;
    rom[2249] = 8'hBA;
    rom[2250] = 8'hAB;
    rom[2251] = 8'hD7;
    rom[2252] = 8'hB7;
    rom[2253] = 8'hA9;
    rom[2254] = 8'h81;
    rom[2255] = 8'hCD;
    rom[2256] = 8'h15;
    rom[2257] = 8'h36;
    rom[2258] = 8'h24;
    rom[2259] = 8'h03;
    rom[2260] = 8'hCC;
    rom[2261] = 8'h12;
    rom[2262] = 8'hE6;
    rom[2263] = 8'hA6;
    rom[2264] = 8'hC7;
    rom[2265] = 8'hB7;
    rom[2266] = 8'hA7;
    rom[2267] = 8'hB6;
    rom[2268] = 8'hB9;
    rom[2269] = 8'hB7;
    rom[2270] = 8'hA8;
    rom[2271] = 8'hB6;
    rom[2272] = 8'hBA;
    rom[2273] = 8'hB7;
    rom[2274] = 8'hA9;
    rom[2275] = 8'hA6;
    rom[2276] = 8'h81;
    rom[2277] = 8'hB7;
    rom[2278] = 8'hAA;
    rom[2279] = 8'h3F;
    rom[2280] = 8'hB5;
    rom[2281] = 8'hCD;
    rom[2282] = 8'h14;
    rom[2283] = 8'hC8;
    rom[2284] = 8'h25;
    rom[2285] = 8'h06;
    rom[2286] = 8'hBD;
    rom[2287] = 8'hA7;
    rom[2288] = 8'h3C;
    rom[2289] = 8'hA9;
    rom[2290] = 8'h20;
    rom[2291] = 8'hF3;
    rom[2292] = 8'hCC;
    rom[2293] = 8'h16;
    rom[2294] = 8'h40;
    rom[2295] = 8'hA6;
    rom[2296] = 8'h04;
    rom[2297] = 8'hCD;
    rom[2298] = 8'h14;
    rom[2299] = 8'hA1;
    rom[2300] = 8'h24;
    rom[2301] = 8'h03;
    rom[2302] = 8'hCC;
    rom[2303] = 8'h12;
    rom[2304] = 8'hE6;
    rom[2305] = 8'hAE;
    rom[2306] = 8'h03;
    rom[2307] = 8'h9B;
    rom[2308] = 8'hE6;
    rom[2309] = 8'hB9;
    rom[2310] = 8'hE7;
    rom[2311] = 8'hAB;
    rom[2312] = 8'h5A;
    rom[2313] = 8'h2A;
    rom[2314] = 8'hF9;
    rom[2315] = 8'h9A;
    rom[2316] = 8'hCC;
    rom[2317] = 8'h16;
    rom[2318] = 8'h40;
    rom[2319] = 8'h4F;
    rom[2320] = 8'hCD;
    rom[2321] = 8'h14;
    rom[2322] = 8'hA1;
    rom[2323] = 8'h24;
    rom[2324] = 8'h03;
    rom[2325] = 8'hCC;
    rom[2326] = 8'h12;
    rom[2327] = 8'hE6;
    rom[2328] = 8'h0B;
    rom[2329] = 8'h00;
    rom[2330] = 8'h0A;
    rom[2331] = 8'hCD;
    rom[2332] = 8'h11;
    rom[2333] = 8'hC1;
    rom[2334] = 8'h25;
    rom[2335] = 8'h05;
    rom[2336] = 8'h15;
    rom[2337] = 8'hA3;
    rom[2338] = 8'hCC;
    rom[2339] = 8'h0F;
    rom[2340] = 8'hAF;
    rom[2341] = 8'hCC;
    rom[2342] = 8'h16;
    rom[2343] = 8'h40;
    rom[2344] = 8'h98;
    rom[2345] = 8'h81;
    rom[2346] = 8'hA6;
    rom[2347] = 8'h04;
    rom[2348] = 8'hCD;
    rom[2349] = 8'h14;
    rom[2350] = 8'hA1;
    rom[2351] = 8'h24;
    rom[2352] = 8'h03;
    rom[2353] = 8'hCC;
    rom[2354] = 8'h12;
    rom[2355] = 8'hE6;
    rom[2356] = 8'h0A;
    rom[2357] = 8'h00;
    rom[2358] = 8'h05;
    rom[2359] = 8'hA6;
    rom[2360] = 8'h02;
    rom[2361] = 8'hCC;
    rom[2362] = 8'h12;
    rom[2363] = 8'hF5;
    rom[2364] = 8'hAE;
    rom[2365] = 8'h03;
    rom[2366] = 8'h9B;
    rom[2367] = 8'hE6;
    rom[2368] = 8'hB9;
    rom[2369] = 8'hE7;
    rom[2370] = 8'hAF;
    rom[2371] = 8'h5A;
    rom[2372] = 8'h2A;
    rom[2373] = 8'hF9;
    rom[2374] = 8'h9A;
    rom[2375] = 8'hCC;
    rom[2376] = 8'h16;
    rom[2377] = 8'h40;
    rom[2378] = 8'hCD;
    rom[2379] = 8'h15;
    rom[2380] = 8'h36;
    rom[2381] = 8'h24;
    rom[2382] = 8'h03;
    rom[2383] = 8'hCC;
    rom[2384] = 8'h12;
    rom[2385] = 8'hE6;
    rom[2386] = 8'hB6;
    rom[2387] = 8'hB9;
    rom[2388] = 8'h27;
    rom[2389] = 8'h0A;
    rom[2390] = 8'hA1;
    rom[2391] = 8'h01;
    rom[2392] = 8'h26;
    rom[2393] = 8'h27;
    rom[2394] = 8'hB6;
    rom[2395] = 8'hBA;
    rom[2396] = 8'hA1;
    rom[2397] = 8'h08;
    rom[2398] = 8'h24;
    rom[2399] = 8'h21;
    rom[2400] = 8'hA6;
    rom[2401] = 8'hC7;
    rom[2402] = 8'hB7;
    rom[2403] = 8'hA7;
    rom[2404] = 8'hA6;
    rom[2405] = 8'h81;
    rom[2406] = 8'hB7;
    rom[2407] = 8'hAA;
    rom[2408] = 8'hCD;
    rom[2409] = 8'h17;
    rom[2410] = 8'hA7;
    rom[2411] = 8'hA6;
    rom[2412] = 8'h02;
    rom[2413] = 8'hB7;
    rom[2414] = 8'hB5;
    rom[2415] = 8'hCD;
    rom[2416] = 8'h14;
    rom[2417] = 8'hC8;
    rom[2418] = 8'h25;
    rom[2419] = 8'h0A;
    rom[2420] = 8'hBD;
    rom[2421] = 8'hA7;
    rom[2422] = 8'h3C;
    rom[2423] = 8'hBA;
    rom[2424] = 8'h26;
    rom[2425] = 8'hEE;
    rom[2426] = 8'h3C;
    rom[2427] = 8'hB9;
    rom[2428] = 8'h20;
    rom[2429] = 8'hEA;
    rom[2430] = 8'hCC;
    rom[2431] = 8'h16;
    rom[2432] = 8'h40;
    rom[2433] = 8'hCD;
    rom[2434] = 8'h14;
    rom[2435] = 8'hC8;
    rom[2436] = 8'h24;
    rom[2437] = 8'hFB;
    rom[2438] = 8'hCC;
    rom[2439] = 8'h17;
    rom[2440] = 8'h9A;
    rom[2441] = 8'hA6;
    rom[2442] = 8'h01;
    rom[2443] = 8'hCD;
    rom[2444] = 8'h14;
    rom[2445] = 8'hA1;
    rom[2446] = 8'h24;
    rom[2447] = 8'h03;
    rom[2448] = 8'hCC;
    rom[2449] = 8'h12;
    rom[2450] = 8'hE6;
    rom[2451] = 8'h1D;
    rom[2452] = 8'hA2;
    rom[2453] = 8'hB6;
    rom[2454] = 8'hB9;
    rom[2455] = 8'h27;
    rom[2456] = 8'h02;
    rom[2457] = 8'h1C;
    rom[2458] = 8'hA2;
    rom[2459] = 8'hCC;
    rom[2460] = 8'h16;
    rom[2461] = 8'h40;
    rom[2462] = 8'hA6;
    rom[2463] = 8'h04;
    rom[2464] = 8'hCD;
    rom[2465] = 8'h14;
    rom[2466] = 8'hA1;
    rom[2467] = 8'hB6;
    rom[2468] = 8'hB5;
    rom[2469] = 8'h27;
    rom[2470] = 8'h04;
    rom[2471] = 8'hA1;
    rom[2472] = 8'h04;
    rom[2473] = 8'h23;
    rom[2474] = 8'h03;
    rom[2475] = 8'hCC;
    rom[2476] = 8'h12;
    rom[2477] = 8'hE6;
    rom[2478] = 8'h3F;
    rom[2479] = 8'hB6;
    rom[2480] = 8'h1C;
    rom[2481] = 8'h05;
    rom[2482] = 8'hBE;
    rom[2483] = 8'hB6;
    rom[2484] = 8'hE6;
    rom[2485] = 8'hB9;
    rom[2486] = 8'hAE;
    rom[2487] = 8'h08;
    rom[2488] = 8'h1F;
    rom[2489] = 8'h01;
    rom[2490] = 8'h49;
    rom[2491] = 8'h24;
    rom[2492] = 8'h04;
    rom[2493] = 8'h1C;
    rom[2494] = 8'h01;
    rom[2495] = 8'h20;
    rom[2496] = 8'h02;
    rom[2497] = 8'h1D;
    rom[2498] = 8'h01;
    rom[2499] = 8'h1E;
    rom[2500] = 8'h01;
    rom[2501] = 8'h5A;
    rom[2502] = 8'h26;
    rom[2503] = 8'hF0;
    rom[2504] = 8'h3C;
    rom[2505] = 8'hB6;
    rom[2506] = 8'hB6;
    rom[2507] = 8'hB6;
    rom[2508] = 8'hB1;
    rom[2509] = 8'hB5;
    rom[2510] = 8'h26;
    rom[2511] = 8'hE2;
    rom[2512] = 8'h18;
    rom[2513] = 8'h00;
    rom[2514] = 8'h19;
    rom[2515] = 8'h00;
    rom[2516] = 8'h1C;
    rom[2517] = 8'h01;
    rom[2518] = 8'h1D;
    rom[2519] = 8'h05;
    rom[2520] = 8'hCC;
    rom[2521] = 8'h16;
    rom[2522] = 8'h40;
    rom[2523] = 8'h4F;
    rom[2524] = 8'hCD;
    rom[2525] = 8'h14;
    rom[2526] = 8'hA1;
    rom[2527] = 8'h24;
    rom[2528] = 8'h03;
    rom[2529] = 8'hCC;
    rom[2530] = 8'h12;
    rom[2531] = 8'hE6;
    rom[2532] = 8'hAE;
    rom[2533] = 8'h3F;
    rom[2534] = 8'hE6;
    rom[2535] = 8'h90;
    rom[2536] = 8'hB7;
    rom[2537] = 8'hC0;
    rom[2538] = 8'hA6;
    rom[2539] = 8'h01;
    rom[2540] = 8'hE7;
    rom[2541] = 8'h90;
    rom[2542] = 8'hE1;
    rom[2543] = 8'h90;
    rom[2544] = 8'h26;
    rom[2545] = 8'h0C;
    rom[2546] = 8'h49;
    rom[2547] = 8'h24;
    rom[2548] = 8'hF7;
    rom[2549] = 8'hB6;
    rom[2550] = 8'hC0;
    rom[2551] = 8'hE7;
    rom[2552] = 8'h90;
    rom[2553] = 8'h5A;
    rom[2554] = 8'h2A;
    rom[2555] = 8'hEA;
    rom[2556] = 8'h20;
    rom[2557] = 8'h0E;
    rom[2558] = 8'hB7;
    rom[2559] = 8'hBB;
    rom[2560] = 8'hB6;
    rom[2561] = 8'hC0;
    rom[2562] = 8'hE7;
    rom[2563] = 8'h90;
    rom[2564] = 8'hA6;
    rom[2565] = 8'h02;
    rom[2566] = 8'hB7;
    rom[2567] = 8'hB9;
    rom[2568] = 8'hBF;
    rom[2569] = 8'hBA;
    rom[2570] = 8'h20;
    rom[2571] = 8'h79;
    rom[2572] = 8'h1E;
    rom[2573] = 8'h00;
    rom[2574] = 8'hCD;
    rom[2575] = 8'h1D;
    rom[2576] = 8'hF1;
    rom[2577] = 8'h0C;
    rom[2578] = 8'h00;
    rom[2579] = 8'h0A;
    rom[2580] = 8'h1F;
    rom[2581] = 8'h00;
    rom[2582] = 8'hCD;
    rom[2583] = 8'h1D;
    rom[2584] = 8'hF1;
    rom[2585] = 8'h0D;
    rom[2586] = 8'h00;
    rom[2587] = 8'h02;
    rom[2588] = 8'h20;
    rom[2589] = 8'h0C;
    rom[2590] = 8'h1F;
    rom[2591] = 8'h00;
    rom[2592] = 8'hA6;
    rom[2593] = 8'h03;
    rom[2594] = 8'hB7;
    rom[2595] = 8'hB9;
    rom[2596] = 8'h3F;
    rom[2597] = 8'hBA;
    rom[2598] = 8'h3F;
    rom[2599] = 8'hBB;
    rom[2600] = 8'h20;
    rom[2601] = 8'h5B;
    rom[2602] = 8'h3F;
    rom[2603] = 8'h93;
    rom[2604] = 8'hA6;
    rom[2605] = 8'hC6;
    rom[2606] = 8'hB7;
    rom[2607] = 8'hA7;
    rom[2608] = 8'hA6;
    rom[2609] = 8'h0F;
    rom[2610] = 8'hB7;
    rom[2611] = 8'hA8;
    rom[2612] = 8'h3F;
    rom[2613] = 8'hA9;
    rom[2614] = 8'hA6;
    rom[2615] = 8'h81;
    rom[2616] = 8'hB7;
    rom[2617] = 8'hAA;
    rom[2618] = 8'hBD;
    rom[2619] = 8'hA7;
    rom[2620] = 8'hB8;
    rom[2621] = 8'h93;
    rom[2622] = 8'hB8;
    rom[2623] = 8'hA9;
    rom[2624] = 8'hB8;
    rom[2625] = 8'hA8;
    rom[2626] = 8'hB7;
    rom[2627] = 8'h93;
    rom[2628] = 8'h3C;
    rom[2629] = 8'hA9;
    rom[2630] = 8'h26;
    rom[2631] = 8'h02;
    rom[2632] = 8'h3C;
    rom[2633] = 8'hA8;
    rom[2634] = 8'hB6;
    rom[2635] = 8'hA9;
    rom[2636] = 8'hA1;
    rom[2637] = 8'hB6;
    rom[2638] = 8'h26;
    rom[2639] = 8'hEA;
    rom[2640] = 8'hB6;
    rom[2641] = 8'hA8;
    rom[2642] = 8'hA1;
    rom[2643] = 8'h1E;
    rom[2644] = 8'h26;
    rom[2645] = 8'hE4;
    rom[2646] = 8'hA6;
    rom[2647] = 8'hF0;
    rom[2648] = 8'hAE;
    rom[2649] = 8'h1F;
    rom[2650] = 8'hBF;
    rom[2651] = 8'hA8;
    rom[2652] = 8'hB7;
    rom[2653] = 8'hA9;
    rom[2654] = 8'hBD;
    rom[2655] = 8'hA7;
    rom[2656] = 8'hB8;
    rom[2657] = 8'h93;
    rom[2658] = 8'hB8;
    rom[2659] = 8'hA9;
    rom[2660] = 8'hB8;
    rom[2661] = 8'hA8;
    rom[2662] = 8'hB7;
    rom[2663] = 8'h93;
    rom[2664] = 8'h3C;
    rom[2665] = 8'hA9;
    rom[2666] = 8'h26;
    rom[2667] = 8'hF2;
    rom[2668] = 8'hC6;
    rom[2669] = 8'h1E;
    rom[2670] = 8'hB6;
    rom[2671] = 8'hB1;
    rom[2672] = 8'h93;
    rom[2673] = 8'h27;
    rom[2674] = 8'h0C;
    rom[2675] = 8'hB7;
    rom[2676] = 8'hBA;
    rom[2677] = 8'hB6;
    rom[2678] = 8'h93;
    rom[2679] = 8'hB7;
    rom[2680] = 8'hBB;
    rom[2681] = 8'hA6;
    rom[2682] = 8'h01;
    rom[2683] = 8'hB7;
    rom[2684] = 8'hB9;
    rom[2685] = 8'h20;
    rom[2686] = 8'h06;
    rom[2687] = 8'h3F;
    rom[2688] = 8'hB9;
    rom[2689] = 8'h3F;
    rom[2690] = 8'hBA;
    rom[2691] = 8'h3F;
    rom[2692] = 8'hBB;
    rom[2693] = 8'hCD;
    rom[2694] = 8'h15;
    rom[2695] = 8'h49;
    rom[2696] = 8'hA6;
    rom[2697] = 8'h01;
    rom[2698] = 8'hBE;
    rom[2699] = 8'hB9;
    rom[2700] = 8'hCD;
    rom[2701] = 8'h15;
    rom[2702] = 8'h86;
    rom[2703] = 8'h25;
    rom[2704] = 8'h13;
    rom[2705] = 8'hA6;
    rom[2706] = 8'h0F;
    rom[2707] = 8'hCD;
    rom[2708] = 8'h15;
    rom[2709] = 8'h93;
    rom[2710] = 8'h25;
    rom[2711] = 8'h0C;
    rom[2712] = 8'hB6;
    rom[2713] = 8'hBA;
    rom[2714] = 8'hCD;
    rom[2715] = 8'h15;
    rom[2716] = 8'h93;
    rom[2717] = 8'h25;
    rom[2718] = 8'h05;
    rom[2719] = 8'hB6;
    rom[2720] = 8'hBB;
    rom[2721] = 8'hCC;
    rom[2722] = 8'h16;
    rom[2723] = 8'h4D;
    rom[2724] = 8'hCC;
    rom[2725] = 8'h16;
    rom[2726] = 8'h52;
    rom[2727] = 8'h4F;
    rom[2728] = 8'hCD;
    rom[2729] = 8'h14;
    rom[2730] = 8'hA1;
    rom[2731] = 8'h24;
    rom[2732] = 8'h03;
    rom[2733] = 8'hCC;
    rom[2734] = 8'h12;
    rom[2735] = 8'hE6;
    rom[2736] = 8'h0B;
    rom[2737] = 8'h00;
    rom[2738] = 8'h05;
    rom[2739] = 8'hA6;
    rom[2740] = 8'h02;
    rom[2741] = 8'hCC;
    rom[2742] = 8'h12;
    rom[2743] = 8'hF5;
    rom[2744] = 8'hCD;
    rom[2745] = 8'h16;
    rom[2746] = 8'h63;
    rom[2747] = 8'h25;
    rom[2748] = 8'h0A;
    rom[2749] = 8'h00;
    rom[2750] = 8'h00;
    rom[2751] = 8'h00;
    rom[2752] = 8'h4F;
    rom[2753] = 8'h24;
    rom[2754] = 8'h01;
    rom[2755] = 8'h4A;
    rom[2756] = 8'hCC;
    rom[2757] = 8'h16;
    rom[2758] = 8'h4D;
    rom[2759] = 8'hCC;
    rom[2760] = 8'h16;
    rom[2761] = 8'h52;
    rom[2762] = 8'h4F;
    rom[2763] = 8'hCD;
    rom[2764] = 8'h14;
    rom[2765] = 8'hA1;
    rom[2766] = 8'h24;
    rom[2767] = 8'h03;
    rom[2768] = 8'hCC;
    rom[2769] = 8'h12;
    rom[2770] = 8'hE6;
    rom[2771] = 8'h1F;
    rom[2772] = 8'hA1;
    rom[2773] = 8'hCC;
    rom[2774] = 8'h0F;
    rom[2775] = 8'hAF;
    rom[2776] = 8'hA6;
    rom[2777] = 8'h01;
    rom[2778] = 8'hCD;
    rom[2779] = 8'h14;
    rom[2780] = 8'hA1;
    rom[2781] = 8'h24;
    rom[2782] = 8'h03;
    rom[2783] = 8'hCC;
    rom[2784] = 8'h12;
    rom[2785] = 8'hE6;
    rom[2786] = 8'h0B;
    rom[2787] = 8'h00;
    rom[2788] = 8'h05;
    rom[2789] = 8'hA6;
    rom[2790] = 8'h02;
    rom[2791] = 8'hCC;
    rom[2792] = 8'h12;
    rom[2793] = 8'hF5;
    rom[2794] = 8'h13;
    rom[2795] = 8'h00;
    rom[2796] = 8'hB6;
    rom[2797] = 8'hB9;
    rom[2798] = 8'h27;
    rom[2799] = 8'h02;
    rom[2800] = 8'h12;
    rom[2801] = 8'h00;
    rom[2802] = 8'hCC;
    rom[2803] = 8'h16;
    rom[2804] = 8'h40;
    rom[2805] = 8'hA6;
    rom[2806] = 8'h01;
    rom[2807] = 8'hCD;
    rom[2808] = 8'h14;
    rom[2809] = 8'hA1;
    rom[2810] = 8'h24;
    rom[2811] = 8'h03;
    rom[2812] = 8'hCC;
    rom[2813] = 8'h12;
    rom[2814] = 8'hE6;
    rom[2815] = 8'h0A;
    rom[2816] = 8'h00;
    rom[2817] = 8'h05;
    rom[2818] = 8'hA6;
    rom[2819] = 8'h02;
    rom[2820] = 8'hCC;
    rom[2821] = 8'h12;
    rom[2822] = 8'hF5;
    rom[2823] = 8'h15;
    rom[2824] = 8'hA3;
    rom[2825] = 8'hB6;
    rom[2826] = 8'hB9;
    rom[2827] = 8'h27;
    rom[2828] = 8'h02;
    rom[2829] = 8'h14;
    rom[2830] = 8'hA3;
    rom[2831] = 8'hCC;
    rom[2832] = 8'h16;
    rom[2833] = 8'h40;
    rom[2834] = 8'hA6;
    rom[2835] = 8'h01;
    rom[2836] = 8'hCD;
    rom[2837] = 8'h14;
    rom[2838] = 8'hA1;
    rom[2839] = 8'h24;
    rom[2840] = 8'h03;
    rom[2841] = 8'hCC;
    rom[2842] = 8'h12;
    rom[2843] = 8'hE6;
    rom[2844] = 8'hB6;
    rom[2845] = 8'hB9;
    rom[2846] = 8'hB7;
    rom[2847] = 8'hA4;
    rom[2848] = 8'hCC;
    rom[2849] = 8'h16;
    rom[2850] = 8'h40;
    rom[2851] = 8'h4F;
    rom[2852] = 8'hCD;
    rom[2853] = 8'h14;
    rom[2854] = 8'hA1;
    rom[2855] = 8'h24;
    rom[2856] = 8'h03;
    rom[2857] = 8'hCC;
    rom[2858] = 8'h12;
    rom[2859] = 8'hE6;
    rom[2860] = 8'hCD;
    rom[2861] = 8'h16;
    rom[2862] = 8'h63;
    rom[2863] = 8'h25;
    rom[2864] = 8'h0C;
    rom[2865] = 8'hA6;
    rom[2866] = 8'h01;
    rom[2867] = 8'hCD;
    rom[2868] = 8'h15;
    rom[2869] = 8'h93;
    rom[2870] = 8'h25;
    rom[2871] = 8'h05;
    rom[2872] = 8'hA6;
    rom[2873] = 8'h08;
    rom[2874] = 8'hCC;
    rom[2875] = 8'h16;
    rom[2876] = 8'h4D;
    rom[2877] = 8'hCC;
    rom[2878] = 8'h16;
    rom[2879] = 8'h52;
    rom[2880] = 8'h4F;
    rom[2881] = 8'hCD;
    rom[2882] = 8'h14;
    rom[2883] = 8'hA1;
    rom[2884] = 8'h24;
    rom[2885] = 8'h03;
    rom[2886] = 8'hCC;
    rom[2887] = 8'h12;
    rom[2888] = 8'hE6;
    rom[2889] = 8'hCD;
    rom[2890] = 8'h16;
    rom[2891] = 8'h63;
    rom[2892] = 8'h25;
    rom[2893] = 8'h05;
    rom[2894] = 8'hB6;
    rom[2895] = 8'hA4;
    rom[2896] = 8'hCC;
    rom[2897] = 8'h16;
    rom[2898] = 8'h4D;
    rom[2899] = 8'hCC;
    rom[2900] = 8'h16;
    rom[2901] = 8'h52;
    rom[2902] = 8'hA6;
    rom[2903] = 8'h01;
    rom[2904] = 8'hCD;
    rom[2905] = 8'h14;
    rom[2906] = 8'hA1;
    rom[2907] = 8'h24;
    rom[2908] = 8'h03;
    rom[2909] = 8'hCC;
    rom[2910] = 8'h12;
    rom[2911] = 8'hE6;
    rom[2912] = 8'hB6;
    rom[2913] = 8'hB9;
    rom[2914] = 8'hB7;
    rom[2915] = 8'hA5;
    rom[2916] = 8'hCC;
    rom[2917] = 8'h16;
    rom[2918] = 8'h40;
    rom[2919] = 8'h4F;
    rom[2920] = 8'hCD;
    rom[2921] = 8'h14;
    rom[2922] = 8'hA1;
    rom[2923] = 8'h24;
    rom[2924] = 8'h03;
    rom[2925] = 8'hCC;
    rom[2926] = 8'h12;
    rom[2927] = 8'hE6;
    rom[2928] = 8'hCD;
    rom[2929] = 8'h16;
    rom[2930] = 8'h63;
    rom[2931] = 8'h25;
    rom[2932] = 8'h05;
    rom[2933] = 8'hB6;
    rom[2934] = 8'hA5;
    rom[2935] = 8'hCC;
    rom[2936] = 8'h16;
    rom[2937] = 8'h4D;
    rom[2938] = 8'hCC;
    rom[2939] = 8'h16;
    rom[2940] = 8'h52;
    rom[2941] = 8'hA6;
    rom[2942] = 8'h02;
    rom[2943] = 8'hCD;
    rom[2944] = 8'h14;
    rom[2945] = 8'hA1;
    rom[2946] = 8'h24;
    rom[2947] = 8'h03;
    rom[2948] = 8'hCC;
    rom[2949] = 8'h12;
    rom[2950] = 8'hE6;
    rom[2951] = 8'hB6;
    rom[2952] = 8'hB9;
    rom[2953] = 8'hB7;
    rom[2954] = 8'hC9;
    rom[2955] = 8'hB6;
    rom[2956] = 8'hBA;
    rom[2957] = 8'hB7;
    rom[2958] = 8'hCA;
    rom[2959] = 8'hCC;
    rom[2960] = 8'h16;
    rom[2961] = 8'h40;
    rom[2962] = 8'h4F;
    rom[2963] = 8'hCD;
    rom[2964] = 8'h14;
    rom[2965] = 8'hA1;
    rom[2966] = 8'h24;
    rom[2967] = 8'h03;
    rom[2968] = 8'hCC;
    rom[2969] = 8'h12;
    rom[2970] = 8'hE6;
    rom[2971] = 8'hCD;
    rom[2972] = 8'h16;
    rom[2973] = 8'h63;
    rom[2974] = 8'h25;
    rom[2975] = 8'h0C;
    rom[2976] = 8'hB6;
    rom[2977] = 8'hC9;
    rom[2978] = 8'hCD;
    rom[2979] = 8'h15;
    rom[2980] = 8'h93;
    rom[2981] = 8'h25;
    rom[2982] = 8'h05;
    rom[2983] = 8'hB6;
    rom[2984] = 8'hCA;
    rom[2985] = 8'hCC;
    rom[2986] = 8'h16;
    rom[2987] = 8'h4D;
    rom[2988] = 8'hCC;
    rom[2989] = 8'h16;
    rom[2990] = 8'h52;
    rom[2991] = 8'hA6;
    rom[2992] = 8'h01;
    rom[2993] = 8'hCD;
    rom[2994] = 8'h14;
    rom[2995] = 8'hA1;
    rom[2996] = 8'h24;
    rom[2997] = 8'h03;
    rom[2998] = 8'hCC;
    rom[2999] = 8'h12;
    rom[3000] = 8'hE6;
    rom[3001] = 8'h19;
    rom[3002] = 8'hA2;
    rom[3003] = 8'h3F;
    rom[3004] = 8'hCB;
    rom[3005] = 8'hB6;
    rom[3006] = 8'hB9;
    rom[3007] = 8'h27;
    rom[3008] = 8'h0A;
    rom[3009] = 8'hA1;
    rom[3010] = 8'h04;
    rom[3011] = 8'h24;
    rom[3012] = 8'h06;
    rom[3013] = 8'h18;
    rom[3014] = 8'hA2;
    rom[3015] = 8'hB7;
    rom[3016] = 8'hCB;
    rom[3017] = 8'h10;
    rom[3018] = 8'hA2;
    rom[3019] = 8'hCC;
    rom[3020] = 8'h16;
    rom[3021] = 8'h40;
    rom[3022] = 8'hCD;
    rom[3023] = 8'h1E;
    rom[3024] = 8'h4E;
    rom[3025] = 8'h09;
    rom[3026] = 8'hA2;
    rom[3027] = 8'h21;
    rom[3028] = 8'h07;
    rom[3029] = 8'hA2;
    rom[3030] = 8'h1E;
    rom[3031] = 8'hA6;
    rom[3032] = 8'h01;
    rom[3033] = 8'hB7;
    rom[3034] = 8'hB7;
    rom[3035] = 8'hA6;
    rom[3036] = 8'h03;
    rom[3037] = 8'hB7;
    rom[3038] = 8'hB8;
    rom[3039] = 8'h00;
    rom[3040] = 8'hA2;
    rom[3041] = 8'h0C;
    rom[3042] = 8'hB6;
    rom[3043] = 8'hCB;
    rom[3044] = 8'h4A;
    rom[3045] = 8'h27;
    rom[3046] = 8'h07;
    rom[3047] = 8'h4A;
    rom[3048] = 8'h27;
    rom[3049] = 8'h0D;
    rom[3050] = 8'h4A;
    rom[3051] = 8'h27;
    rom[3052] = 8'h0F;
    rom[3053] = 8'h81;
    rom[3054] = 8'hCD;
    rom[3055] = 8'h16;
    rom[3056] = 8'hDB;
    rom[3057] = 8'h17;
    rom[3058] = 8'hA2;
    rom[3059] = 8'h98;
    rom[3060] = 8'h81;
    rom[3061] = 8'h99;
    rom[3062] = 8'h81;
    rom[3063] = 8'hCD;
    rom[3064] = 8'h16;
    rom[3065] = 8'h40;
    rom[3066] = 8'h20;
    rom[3067] = 8'hF5;
    rom[3068] = 8'hCD;
    rom[3069] = 8'h15;
    rom[3070] = 8'h49;
    rom[3071] = 8'hA6;
    rom[3072] = 8'h03;
    rom[3073] = 8'h12;
    rom[3074] = 8'h01;
    rom[3075] = 8'hCD;
    rom[3076] = 8'h15;
    rom[3077] = 8'h93;
    rom[3078] = 8'hCD;
    rom[3079] = 8'h11;
    rom[3080] = 8'h49;
    rom[3081] = 8'h01;
    rom[3082] = 8'h01;
    rom[3083] = 8'hE9;
    rom[3084] = 8'h06;
    rom[3085] = 8'h01;
    rom[3086] = 8'hF7;
    rom[3087] = 8'h20;
    rom[3088] = 8'hE0;
    rom[3089] = 8'hA6;
    rom[3090] = 8'h01;
    rom[3091] = 8'hCD;
    rom[3092] = 8'h14;
    rom[3093] = 8'hA1;
    rom[3094] = 8'h24;
    rom[3095] = 8'h03;
    rom[3096] = 8'hCC;
    rom[3097] = 8'h12;
    rom[3098] = 8'hE6;
    rom[3099] = 8'h15;
    rom[3100] = 8'hA2;
    rom[3101] = 8'hB6;
    rom[3102] = 8'hB9;
    rom[3103] = 8'h27;
    rom[3104] = 8'h02;
    rom[3105] = 8'h14;
    rom[3106] = 8'hA2;
    rom[3107] = 8'hCC;
    rom[3108] = 8'h16;
    rom[3109] = 8'h40;
    rom[3110] = 8'hA6;
    rom[3111] = 8'h01;
    rom[3112] = 8'hCD;
    rom[3113] = 8'h14;
    rom[3114] = 8'hA1;
    rom[3115] = 8'h24;
    rom[3116] = 8'h03;
    rom[3117] = 8'hCC;
    rom[3118] = 8'h12;
    rom[3119] = 8'hE6;
    rom[3120] = 8'h1D;
    rom[3121] = 8'hA1;
    rom[3122] = 8'hB6;
    rom[3123] = 8'hB9;
    rom[3124] = 8'h27;
    rom[3125] = 8'h02;
    rom[3126] = 8'h1C;
    rom[3127] = 8'hA1;
    rom[3128] = 8'hCC;
    rom[3129] = 8'h16;
    rom[3130] = 8'h40;
    rom[3131] = 8'hA6;
    rom[3132] = 8'h01;
    rom[3133] = 8'hCD;
    rom[3134] = 8'h14;
    rom[3135] = 8'hA1;
    rom[3136] = 8'h24;
    rom[3137] = 8'h03;
    rom[3138] = 8'hCC;
    rom[3139] = 8'h12;
    rom[3140] = 8'hE6;
    rom[3141] = 8'hB6;
    rom[3142] = 8'hB9;
    rom[3143] = 8'h26;
    rom[3144] = 8'h01;
    rom[3145] = 8'h4A;
    rom[3146] = 8'hB7;
    rom[3147] = 8'hA6;
    rom[3148] = 8'hCC;
    rom[3149] = 8'h16;
    rom[3150] = 8'h40;
    rom[3151] = 8'h4F;
    rom[3152] = 8'hCD;
    rom[3153] = 8'h14;
    rom[3154] = 8'hA1;
    rom[3155] = 8'h24;
    rom[3156] = 8'h03;
    rom[3157] = 8'hCC;
    rom[3158] = 8'h12;
    rom[3159] = 8'hE6;
    rom[3160] = 8'hCD;
    rom[3161] = 8'h16;
    rom[3162] = 8'h63;
    rom[3163] = 8'h25;
    rom[3164] = 8'h05;
    rom[3165] = 8'hB6;
    rom[3166] = 8'hA6;
    rom[3167] = 8'hCC;
    rom[3168] = 8'h16;
    rom[3169] = 8'h4D;
    rom[3170] = 8'hCC;
    rom[3171] = 8'h16;
    rom[3172] = 8'h52;
    rom[3173] = 8'h36;
    rom[3174] = 8'h37;
    rom[3175] = 8'h7F;
    rom[3176] = 8'hB7;
    rom[3177] = 8'h93;
    rom[3178] = 8'hA4;
    rom[3179] = 8'h7F;
    rom[3180] = 8'hAE;
    rom[3181] = 8'h02;
    rom[3182] = 8'hD1;
    rom[3183] = 8'h1B;
    rom[3184] = 8'h65;
    rom[3185] = 8'h27;
    rom[3186] = 8'h0A;
    rom[3187] = 8'h5A;
    rom[3188] = 8'h2A;
    rom[3189] = 8'hF8;
    rom[3190] = 8'hB6;
    rom[3191] = 8'hA1;
    rom[3192] = 8'hA4;
    rom[3193] = 8'hC0;
    rom[3194] = 8'hB7;
    rom[3195] = 8'hA1;
    rom[3196] = 8'h81;
    rom[3197] = 8'h9F;
    rom[3198] = 8'h48;
    rom[3199] = 8'h97;
    rom[3200] = 8'hB6;
    rom[3201] = 8'h93;
    rom[3202] = 8'hDC;
    rom[3203] = 8'h1B;
    rom[3204] = 8'hB9;
    rom[3205] = 8'h2A;
    rom[3206] = 8'h04;
    rom[3207] = 8'h11;
    rom[3208] = 8'hA1;
    rom[3209] = 8'h20;
    rom[3210] = 8'h34;
    rom[3211] = 8'h10;
    rom[3212] = 8'hA1;
    rom[3213] = 8'h20;
    rom[3214] = 8'h30;
    rom[3215] = 8'h2A;
    rom[3216] = 8'h04;
    rom[3217] = 8'h15;
    rom[3218] = 8'hA1;
    rom[3219] = 8'h20;
    rom[3220] = 8'h2A;
    rom[3221] = 8'h14;
    rom[3222] = 8'hA1;
    rom[3223] = 8'h20;
    rom[3224] = 8'h26;
    rom[3225] = 8'hB7;
    rom[3226] = 8'h94;
    rom[3227] = 8'hB6;
    rom[3228] = 8'h99;
    rom[3229] = 8'hA4;
    rom[3230] = 8'h7F;
    rom[3231] = 8'hB7;
    rom[3232] = 8'h93;
    rom[3233] = 8'hB6;
    rom[3234] = 8'h9A;
    rom[3235] = 8'hA4;
    rom[3236] = 8'h7F;
    rom[3237] = 8'hA1;
    rom[3238] = 8'h7F;
    rom[3239] = 8'h26;
    rom[3240] = 8'h16;
    rom[3241] = 8'hB8;
    rom[3242] = 8'h93;
    rom[3243] = 8'h26;
    rom[3244] = 8'h12;
    rom[3245] = 8'hB6;
    rom[3246] = 8'h94;
    rom[3247] = 8'h2A;
    rom[3248] = 8'h04;
    rom[3249] = 8'h1B;
    rom[3250] = 8'hA1;
    rom[3251] = 8'h20;
    rom[3252] = 8'h0A;
    rom[3253] = 8'h1A;
    rom[3254] = 8'hA1;
    rom[3255] = 8'h20;
    rom[3256] = 8'h06;
    rom[3257] = 8'h20;
    rom[3258] = 8'hCA;
    rom[3259] = 8'h20;
    rom[3260] = 8'hD2;
    rom[3261] = 8'h20;
    rom[3262] = 8'hDA;
    rom[3263] = 8'hB6;
    rom[3264] = 8'hA1;
    rom[3265] = 8'hA4;
    rom[3266] = 8'h3F;
    rom[3267] = 8'h27;
    rom[3268] = 8'h08;
    rom[3269] = 8'hA1;
    rom[3270] = 8'h25;
    rom[3271] = 8'h27;
    rom[3272] = 8'h05;
    rom[3273] = 8'hA1;
    rom[3274] = 8'h24;
    rom[3275] = 8'h27;
    rom[3276] = 8'h18;
    rom[3277] = 8'h81;
    rom[3278] = 8'hB6;
    rom[3279] = 8'hA1;
    rom[3280] = 8'hA4;
    rom[3281] = 8'hC0;
    rom[3282] = 8'hB7;
    rom[3283] = 8'hA1;
    rom[3284] = 8'h0D;
    rom[3285] = 8'hA2;
    rom[3286] = 8'h03;
    rom[3287] = 8'h1D;
    rom[3288] = 8'hA2;
    rom[3289] = 8'h81;
    rom[3290] = 8'h17;
    rom[3291] = 8'h00;
    rom[3292] = 8'h17;
    rom[3293] = 8'h02;
    rom[3294] = 8'h16;
    rom[3295] = 8'h06;
    rom[3296] = 8'h1E;
    rom[3297] = 8'hA1;
    rom[3298] = 8'hCC;
    rom[3299] = 8'h0F;
    rom[3300] = 8'hAF;
    rom[3301] = 8'h05;
    rom[3302] = 8'hA2;
    rom[3303] = 8'h0D;
    rom[3304] = 8'hCD;
    rom[3305] = 8'h1E;
    rom[3306] = 8'h92;
    rom[3307] = 8'hB6;
    rom[3308] = 8'hA1;
    rom[3309] = 8'hA4;
    rom[3310] = 8'hC0;
    rom[3311] = 8'hB7;
    rom[3312] = 8'hA1;
    rom[3313] = 8'h12;
    rom[3314] = 8'h95;
    rom[3315] = 8'h3F;
    rom[3316] = 8'h96;
    rom[3317] = 8'h81;
    rom[3318] = 8'h1E;
    rom[3319] = 8'h00;
    rom[3320] = 8'hAE;
    rom[3321] = 8'h06;
    rom[3322] = 8'hBF;
    rom[3323] = 8'h92;
    rom[3324] = 8'h49;
    rom[3325] = 8'h25;
    rom[3326] = 8'h16;
    rom[3327] = 8'hAE;
    rom[3328] = 8'h09;
    rom[3329] = 8'hCD;
    rom[3330] = 8'h1D;
    rom[3331] = 8'hD1;
    rom[3332] = 8'hCD;
    rom[3333] = 8'h1D;
    rom[3334] = 8'hF1;
    rom[3335] = 8'h20;
    rom[3336] = 8'h00;
    rom[3337] = 8'h1F;
    rom[3338] = 8'h00;
    rom[3339] = 8'hAE;
    rom[3340] = 8'h04;
    rom[3341] = 8'hCD;
    rom[3342] = 8'h1D;
    rom[3343] = 8'hD1;
    rom[3344] = 8'hCD;
    rom[3345] = 8'h1D;
    rom[3346] = 8'hF1;
    rom[3347] = 8'h20;
    rom[3348] = 8'h12;
    rom[3349] = 8'hAE;
    rom[3350] = 8'h04;
    rom[3351] = 8'hCD;
    rom[3352] = 8'h1D;
    rom[3353] = 8'hD1;
    rom[3354] = 8'h20;
    rom[3355] = 8'h00;
    rom[3356] = 8'h1F;
    rom[3357] = 8'h00;
    rom[3358] = 8'hAE;
    rom[3359] = 8'h0B;
    rom[3360] = 8'hCD;
    rom[3361] = 8'h1D;
    rom[3362] = 8'hD1;
    rom[3363] = 8'h9D;
    rom[3364] = 8'h9D;
    rom[3365] = 8'h20;
    rom[3366] = 8'h00;
    rom[3367] = 8'h1E;
    rom[3368] = 8'h00;
    rom[3369] = 8'h49;
    rom[3370] = 8'h25;
    rom[3371] = 8'h11;
    rom[3372] = 8'hAE;
    rom[3373] = 8'h0B;
    rom[3374] = 8'hCD;
    rom[3375] = 8'h1D;
    rom[3376] = 8'hD1;
    rom[3377] = 8'h9D;
    rom[3378] = 8'h1F;
    rom[3379] = 8'h00;
    rom[3380] = 8'hAE;
    rom[3381] = 8'h04;
    rom[3382] = 8'hCD;
    rom[3383] = 8'h1D;
    rom[3384] = 8'hD1;
    rom[3385] = 8'h20;
    rom[3386] = 8'h00;
    rom[3387] = 8'h20;
    rom[3388] = 8'h11;
    rom[3389] = 8'hAE;
    rom[3390] = 8'h04;
    rom[3391] = 8'hCD;
    rom[3392] = 8'h1D;
    rom[3393] = 8'hD1;
    rom[3394] = 8'h20;
    rom[3395] = 8'h00;
    rom[3396] = 8'h20;
    rom[3397] = 8'h00;
    rom[3398] = 8'h9D;
    rom[3399] = 8'h1F;
    rom[3400] = 8'h00;
    rom[3401] = 8'hAE;
    rom[3402] = 8'h0B;
    rom[3403] = 8'hCD;
    rom[3404] = 8'h1D;
    rom[3405] = 8'hD1;
    rom[3406] = 8'h3A;
    rom[3407] = 8'h92;
    rom[3408] = 8'h26;
    rom[3409] = 8'hD5;
    rom[3410] = 8'h1E;
    rom[3411] = 8'h00;
    rom[3412] = 8'h49;
    rom[3413] = 8'h25;
    rom[3414] = 8'h08;
    rom[3415] = 8'hAE;
    rom[3416] = 8'h0B;
    rom[3417] = 8'hCD;
    rom[3418] = 8'h1D;
    rom[3419] = 8'hD1;
    rom[3420] = 8'h1F;
    rom[3421] = 8'h00;
    rom[3422] = 8'h81;
    rom[3423] = 8'hAE;
    rom[3424] = 8'h04;
    rom[3425] = 8'hCD;
    rom[3426] = 8'h1D;
    rom[3427] = 8'hD1;
    rom[3428] = 8'h9D;
    rom[3429] = 8'h20;
    rom[3430] = 8'h00;
    rom[3431] = 8'h20;
    rom[3432] = 8'h00;
    rom[3433] = 8'h1F;
    rom[3434] = 8'h00;
    rom[3435] = 8'hAE;
    rom[3436] = 8'h05;
    rom[3437] = 8'hCD;
    rom[3438] = 8'h1D;
    rom[3439] = 8'hD1;
    rom[3440] = 8'h81;
    rom[3441] = 8'h00;
    rom[3442] = 8'h01;
    rom[3443] = 8'h03;
    rom[3444] = 8'hCC;
    rom[3445] = 8'h0F;
    rom[3446] = 8'hAF;
    rom[3447] = 8'h9B;
    rom[3448] = 8'hB7;
    rom[3449] = 8'h98;
    rom[3450] = 8'hB6;
    rom[3451] = 8'h95;
    rom[3452] = 8'hA4;
    rom[3453] = 8'h40;
    rom[3454] = 8'hB7;
    rom[3455] = 8'h95;
    rom[3456] = 8'hA6;
    rom[3457] = 8'h01;
    rom[3458] = 8'hB7;
    rom[3459] = 8'h97;
    rom[3460] = 8'h11;
    rom[3461] = 8'h95;
    rom[3462] = 8'h1E;
    rom[3463] = 8'h00;
    rom[3464] = 8'hAE;
    rom[3465] = 8'hA5;
    rom[3466] = 8'hCD;
    rom[3467] = 8'h1D;
    rom[3468] = 8'hD1;
    rom[3469] = 8'h20;
    rom[3470] = 8'h00;
    rom[3471] = 8'h20;
    rom[3472] = 8'h00;
    rom[3473] = 8'h9D;
    rom[3474] = 8'h1F;
    rom[3475] = 8'h00;
    rom[3476] = 8'hAE;
    rom[3477] = 8'h0A;
    rom[3478] = 8'hCD;
    rom[3479] = 8'h1D;
    rom[3480] = 8'hD1;
    rom[3481] = 8'h20;
    rom[3482] = 8'h00;
    rom[3483] = 8'h9D;
    rom[3484] = 8'h9D;
    rom[3485] = 8'hB6;
    rom[3486] = 8'h98;
    rom[3487] = 8'hCD;
    rom[3488] = 8'h1B;
    rom[3489] = 8'hF6;
    rom[3490] = 8'hAE;
    rom[3491] = 8'h05;
    rom[3492] = 8'hCD;
    rom[3493] = 8'h1D;
    rom[3494] = 8'hD1;
    rom[3495] = 8'h1E;
    rom[3496] = 8'h00;
    rom[3497] = 8'hAE;
    rom[3498] = 8'h0C;
    rom[3499] = 8'hCD;
    rom[3500] = 8'h1D;
    rom[3501] = 8'hD1;
    rom[3502] = 8'h20;
    rom[3503] = 8'h00;
    rom[3504] = 8'h9D;
    rom[3505] = 8'h9D;
    rom[3506] = 8'h1F;
    rom[3507] = 8'h00;
    rom[3508] = 8'h20;
    rom[3509] = 8'h00;
    rom[3510] = 8'h20;
    rom[3511] = 8'h00;
    rom[3512] = 8'h0C;
    rom[3513] = 8'h00;
    rom[3514] = 8'h16;
    rom[3515] = 8'h10;
    rom[3516] = 8'h95;
    rom[3517] = 8'hAE;
    rom[3518] = 8'h4A;
    rom[3519] = 8'h0C;
    rom[3520] = 8'h00;
    rom[3521] = 8'h0B;
    rom[3522] = 8'h5A;
    rom[3523] = 8'h26;
    rom[3524] = 8'hFA;
    rom[3525] = 8'h14;
    rom[3526] = 8'h95;
    rom[3527] = 8'h11;
    rom[3528] = 8'h95;
    rom[3529] = 8'h12;
    rom[3530] = 8'h95;
    rom[3531] = 8'h9A;
    rom[3532] = 8'h81;
    rom[3533] = 8'h20;
    rom[3534] = 8'h00;
    rom[3535] = 8'h20;
    rom[3536] = 8'h00;
    rom[3537] = 8'hB6;
    rom[3538] = 8'h98;
    rom[3539] = 8'hA4;
    rom[3540] = 8'h0C;
    rom[3541] = 8'hA1;
    rom[3542] = 8'h08;
    rom[3543] = 8'h26;
    rom[3544] = 8'h03;
    rom[3545] = 8'hCC;
    rom[3546] = 8'h1D;
    rom[3547] = 8'h92;
    rom[3548] = 8'hAE;
    rom[3549] = 8'h2F;
    rom[3550] = 8'h3F;
    rom[3551] = 8'h96;
    rom[3552] = 8'hA6;
    rom[3553] = 8'hF8;
    rom[3554] = 8'hB7;
    rom[3555] = 8'h97;
    rom[3556] = 8'h0D;
    rom[3557] = 8'h00;
    rom[3558] = 8'h08;
    rom[3559] = 8'h5A;
    rom[3560] = 8'h26;
    rom[3561] = 8'hFA;
    rom[3562] = 8'h12;
    rom[3563] = 8'h95;
    rom[3564] = 8'hCC;
    rom[3565] = 8'h1D;
    rom[3566] = 8'h90;
    rom[3567] = 8'hAE;
    rom[3568] = 8'h0B;
    rom[3569] = 8'h0C;
    rom[3570] = 8'h00;
    rom[3571] = 8'h08;
    rom[3572] = 8'h5A;
    rom[3573] = 8'h26;
    rom[3574] = 8'hFA;
    rom[3575] = 8'h12;
    rom[3576] = 8'h95;
    rom[3577] = 8'hCC;
    rom[3578] = 8'h1D;
    rom[3579] = 8'h90;
    rom[3580] = 8'hAE;
    rom[3581] = 8'h0F;
    rom[3582] = 8'h0D;
    rom[3583] = 8'h00;
    rom[3584] = 8'h06;
    rom[3585] = 8'h5A;
    rom[3586] = 8'h26;
    rom[3587] = 8'hFA;
    rom[3588] = 8'hCC;
    rom[3589] = 8'h1D;
    rom[3590] = 8'h4A;
    rom[3591] = 8'hA6;
    rom[3592] = 8'h01;
    rom[3593] = 8'h5F;
    rom[3594] = 8'h0C;
    rom[3595] = 8'h00;
    rom[3596] = 8'h05;
    rom[3597] = 8'h5C;
    rom[3598] = 8'h26;
    rom[3599] = 8'hFA;
    rom[3600] = 8'h20;
    rom[3601] = 8'h40;
    rom[3602] = 8'h9D;
    rom[3603] = 8'h20;
    rom[3604] = 8'h00;
    rom[3605] = 8'h0D;
    rom[3606] = 8'h00;
    rom[3607] = 8'h13;
    rom[3608] = 8'h5A;
    rom[3609] = 8'h2A;
    rom[3610] = 8'hFA;
    rom[3611] = 8'h0D;
    rom[3612] = 8'h00;
    rom[3613] = 8'h0D;
    rom[3614] = 8'hAE;
    rom[3615] = 8'h8F;
    rom[3616] = 8'h9D;
    rom[3617] = 8'h9D;
    rom[3618] = 8'h0D;
    rom[3619] = 8'h00;
    rom[3620] = 8'h06;
    rom[3621] = 8'h5A;
    rom[3622] = 8'h2B;
    rom[3623] = 8'hFA;
    rom[3624] = 8'h98;
    rom[3625] = 8'h20;
    rom[3626] = 8'h1F;
    rom[3627] = 8'h59;
    rom[3628] = 8'h49;
    rom[3629] = 8'h25;
    rom[3630] = 8'h0B;
    rom[3631] = 8'h0D;
    rom[3632] = 8'h00;
    rom[3633] = 8'h00;
    rom[3634] = 8'h0D;
    rom[3635] = 8'h00;
    rom[3636] = 8'h00;
    rom[3637] = 8'h9D;
    rom[3638] = 8'h9D;
    rom[3639] = 8'h9D;
    rom[3640] = 8'h20;
    rom[3641] = 8'h0B;
    rom[3642] = 8'hBE;
    rom[3643] = 8'h97;
    rom[3644] = 8'hD7;
    rom[3645] = 8'hFF;
    rom[3646] = 8'hA1;
    rom[3647] = 8'h3C;
    rom[3648] = 8'h97;
    rom[3649] = 8'h27;
    rom[3650] = 8'h07;
    rom[3651] = 8'hA6;
    rom[3652] = 8'h01;
    rom[3653] = 8'hAE;
    rom[3654] = 8'h02;
    rom[3655] = 8'h0D;
    rom[3656] = 8'h00;
    rom[3657] = 8'hC4;
    rom[3658] = 8'hA6;
    rom[3659] = 8'h08;
    rom[3660] = 8'hBB;
    rom[3661] = 8'h97;
    rom[3662] = 8'hB7;
    rom[3663] = 8'h96;
    rom[3664] = 8'h20;
    rom[3665] = 8'h04;
    rom[3666] = 8'h16;
    rom[3667] = 8'h95;
    rom[3668] = 8'h20;
    rom[3669] = 8'h3A;
    rom[3670] = 8'hB6;
    rom[3671] = 8'h98;
    rom[3672] = 8'hA4;
    rom[3673] = 8'h0F;
    rom[3674] = 8'hA1;
    rom[3675] = 8'h0F;
    rom[3676] = 8'h26;
    rom[3677] = 8'h02;
    rom[3678] = 8'h19;
    rom[3679] = 8'h99;
    rom[3680] = 8'hB6;
    rom[3681] = 8'h98;
    rom[3682] = 8'hA1;
    rom[3683] = 8'h2C;
    rom[3684] = 8'h26;
    rom[3685] = 8'h0D;
    rom[3686] = 8'h9A;
    rom[3687] = 8'hB6;
    rom[3688] = 8'h99;
    rom[3689] = 8'hCD;
    rom[3690] = 8'h1B;
    rom[3691] = 8'h68;
    rom[3692] = 8'hB6;
    rom[3693] = 8'h9A;
    rom[3694] = 8'hCD;
    rom[3695] = 8'h1B;
    rom[3696] = 8'h68;
    rom[3697] = 8'h20;
    rom[3698] = 8'h1D;
    rom[3699] = 8'h0D;
    rom[3700] = 8'hA1;
    rom[3701] = 8'h02;
    rom[3702] = 8'h20;
    rom[3703] = 8'h18;
    rom[3704] = 8'hA1;
    rom[3705] = 8'h2E;
    rom[3706] = 8'h26;
    rom[3707] = 8'h14;
    rom[3708] = 8'hB6;
    rom[3709] = 8'h99;
    rom[3710] = 8'hA4;
    rom[3711] = 8'h1F;
    rom[3712] = 8'hA1;
    rom[3713] = 8'h06;
    rom[3714] = 8'h26;
    rom[3715] = 8'h05;
    rom[3716] = 8'hCD;
    rom[3717] = 8'h1B;
    rom[3718] = 8'hD4;
    rom[3719] = 8'h20;
    rom[3720] = 8'h07;
    rom[3721] = 8'hA1;
    rom[3722] = 8'h0E;
    rom[3723] = 8'h26;
    rom[3724] = 8'h03;
    rom[3725] = 8'hCD;
    rom[3726] = 8'h1B;
    rom[3727] = 8'hE5;
    rom[3728] = 8'h9A;
    rom[3729] = 8'h81;
    rom[3730] = 8'hAE;
    rom[3731] = 8'h26;
    rom[3732] = 8'hCD;
    rom[3733] = 8'h1D;
    rom[3734] = 8'hD1;
    rom[3735] = 8'h1E;
    rom[3736] = 8'h00;
    rom[3737] = 8'hAE;
    rom[3738] = 8'h05;
    rom[3739] = 8'hCD;
    rom[3740] = 8'h1D;
    rom[3741] = 8'hD1;
    rom[3742] = 8'h9D;
    rom[3743] = 8'h9D;
    rom[3744] = 8'h1F;
    rom[3745] = 8'h00;
    rom[3746] = 8'hAE;
    rom[3747] = 8'h0A;
    rom[3748] = 8'hCD;
    rom[3749] = 8'h1D;
    rom[3750] = 8'hD1;
    rom[3751] = 8'h20;
    rom[3752] = 8'h00;
    rom[3753] = 8'hBE;
    rom[3754] = 8'h97;
    rom[3755] = 8'hE6;
    rom[3756] = 8'h98;
    rom[3757] = 8'hCD;
    rom[3758] = 8'h1B;
    rom[3759] = 8'hF6;
    rom[3760] = 8'hAE;
    rom[3761] = 8'h02;
    rom[3762] = 8'hCD;
    rom[3763] = 8'h1D;
    rom[3764] = 8'hD1;
    rom[3765] = 8'h3C;
    rom[3766] = 8'h97;
    rom[3767] = 8'hB6;
    rom[3768] = 8'h97;
    rom[3769] = 8'hB1;
    rom[3770] = 8'h96;
    rom[3771] = 8'h26;
    rom[3772] = 8'hEC;
    rom[3773] = 8'hCD;
    rom[3774] = 8'h1D;
    rom[3775] = 8'hF1;
    rom[3776] = 8'h9D;
    rom[3777] = 8'h1E;
    rom[3778] = 8'h00;
    rom[3779] = 8'hAE;
    rom[3780] = 8'h0C;
    rom[3781] = 8'hCD;
    rom[3782] = 8'h1D;
    rom[3783] = 8'hD1;
    rom[3784] = 8'h20;
    rom[3785] = 8'h00;
    rom[3786] = 8'h20;
    rom[3787] = 8'h00;
    rom[3788] = 8'h9D;
    rom[3789] = 8'h1F;
    rom[3790] = 8'h00;
    rom[3791] = 8'h9A;
    rom[3792] = 8'h81;
    rom[3793] = 8'h9D;
    rom[3794] = 8'h9D;
    rom[3795] = 8'h5A;
    rom[3796] = 8'h26;
    rom[3797] = 8'hFB;
    rom[3798] = 8'h81;
    rom[3799] = 8'hB6;
    rom[3800] = 8'hA5;
    rom[3801] = 8'hAE;
    rom[3802] = 8'h13;
    rom[3803] = 8'hCD;
    rom[3804] = 8'h1D;
    rom[3805] = 8'hD1;
    rom[3806] = 8'h4A;
    rom[3807] = 8'h26;
    rom[3808] = 8'hF8;
    rom[3809] = 8'h81;
    rom[3810] = 8'hA3;
    rom[3811] = 8'h00;
    rom[3812] = 8'h27;
    rom[3813] = 8'h0B;
    rom[3814] = 8'hA6;
    rom[3815] = 8'hAE;
    rom[3816] = 8'h9D;
    rom[3817] = 8'h9D;
    rom[3818] = 8'h9D;
    rom[3819] = 8'h4A;
    rom[3820] = 8'h26;
    rom[3821] = 8'hFA;
    rom[3822] = 8'h5A;
    rom[3823] = 8'h26;
    rom[3824] = 8'hF5;
    rom[3825] = 8'h81;
    rom[3826] = 8'h0C;
    rom[3827] = 8'h07;
    rom[3828] = 8'h06;
    rom[3829] = 8'hAE;
    rom[3830] = 8'h0F;
    rom[3831] = 8'hCD;
    rom[3832] = 8'h1D;
    rom[3833] = 8'hD1;
    rom[3834] = 8'h81;
    rom[3835] = 8'hAE;
    rom[3836] = 8'h0A;
    rom[3837] = 8'hCD;
    rom[3838] = 8'h1D;
    rom[3839] = 8'hE2;
    rom[3840] = 8'h81;
    rom[3841] = 8'h0C;
    rom[3842] = 8'h07;
    rom[3843] = 8'h06;
    rom[3844] = 8'hAE;
    rom[3845] = 8'hA1;
    rom[3846] = 8'hCD;
    rom[3847] = 8'h1D;
    rom[3848] = 8'hD1;
    rom[3849] = 8'h81;
    rom[3850] = 8'hAE;
    rom[3851] = 8'h64;
    rom[3852] = 8'hCD;
    rom[3853] = 8'h1D;
    rom[3854] = 8'hE2;
    rom[3855] = 8'h81;
    rom[3856] = 8'h04;
    rom[3857] = 8'h01;
    rom[3858] = 8'h21;
    rom[3859] = 8'h1B;
    rom[3860] = 8'hA3;
    rom[3861] = 8'h3C;
    rom[3862] = 8'hCC;
    rom[3863] = 8'h2A;
    rom[3864] = 8'h05;
    rom[3865] = 8'h10;
    rom[3866] = 8'hA3;
    rom[3867] = 8'hCD;
    rom[3868] = 8'h1E;
    rom[3869] = 8'h4E;
    rom[3870] = 8'h07;
    rom[3871] = 8'hA2;
    rom[3872] = 8'h02;
    rom[3873] = 8'h10;
    rom[3874] = 8'hA2;
    rom[3875] = 8'h04;
    rom[3876] = 8'h01;
    rom[3877] = 8'h07;
    rom[3878] = 8'h1B;
    rom[3879] = 8'hA3;
    rom[3880] = 8'h16;
    rom[3881] = 8'hA2;
    rom[3882] = 8'h1D;
    rom[3883] = 8'h12;
    rom[3884] = 8'h80;
    rom[3885] = 8'h18;
    rom[3886] = 8'hA3;
    rom[3887] = 8'h16;
    rom[3888] = 8'hA2;
    rom[3889] = 8'h1D;
    rom[3890] = 8'h12;
    rom[3891] = 8'h80;
    rom[3892] = 8'h18;
    rom[3893] = 8'hA3;
    rom[3894] = 8'h3C;
    rom[3895] = 8'hCC;
    rom[3896] = 8'h2A;
    rom[3897] = 8'h05;
    rom[3898] = 8'h10;
    rom[3899] = 8'hA3;
    rom[3900] = 8'hCD;
    rom[3901] = 8'h1E;
    rom[3902] = 8'h4E;
    rom[3903] = 8'h07;
    rom[3904] = 8'hA2;
    rom[3905] = 8'h02;
    rom[3906] = 8'h10;
    rom[3907] = 8'hA2;
    rom[3908] = 8'h04;
    rom[3909] = 8'h01;
    rom[3910] = 8'hE6;
    rom[3911] = 8'h1B;
    rom[3912] = 8'hA3;
    rom[3913] = 8'h16;
    rom[3914] = 8'hA2;
    rom[3915] = 8'h1D;
    rom[3916] = 8'h12;
    rom[3917] = 8'h80;
    rom[3918] = 8'hB6;
    rom[3919] = 8'hCC;
    rom[3920] = 8'h27;
    rom[3921] = 8'h2C;
    rom[3922] = 8'h3C;
    rom[3923] = 8'hAE;
    rom[3924] = 8'h26;
    rom[3925] = 8'h0A;
    rom[3926] = 8'h3C;
    rom[3927] = 8'hAD;
    rom[3928] = 8'h26;
    rom[3929] = 8'h06;
    rom[3930] = 8'h3C;
    rom[3931] = 8'hAC;
    rom[3932] = 8'h26;
    rom[3933] = 8'h02;
    rom[3934] = 8'h3C;
    rom[3935] = 8'hAB;
    rom[3936] = 8'hB6;
    rom[3937] = 8'hB2;
    rom[3938] = 8'hB1;
    rom[3939] = 8'hAE;
    rom[3940] = 8'h26;
    rom[3941] = 8'h14;
    rom[3942] = 8'hB6;
    rom[3943] = 8'hB1;
    rom[3944] = 8'hB1;
    rom[3945] = 8'hAD;
    rom[3946] = 8'h26;
    rom[3947] = 8'h0E;
    rom[3948] = 8'hB6;
    rom[3949] = 8'hB0;
    rom[3950] = 8'hB1;
    rom[3951] = 8'hAC;
    rom[3952] = 8'h26;
    rom[3953] = 8'h08;
    rom[3954] = 8'hB6;
    rom[3955] = 8'hAF;
    rom[3956] = 8'hB1;
    rom[3957] = 8'hAB;
    rom[3958] = 8'h26;
    rom[3959] = 8'h02;
    rom[3960] = 8'h1C;
    rom[3961] = 8'hA3;
    rom[3962] = 8'h3A;
    rom[3963] = 8'hCC;
    rom[3964] = 8'h26;
    rom[3965] = 8'hD4;
    rom[3966] = 8'h81;
    rom[3967] = 8'h0F;
    rom[3968] = 8'h08;
    rom[3969] = 8'h04;
    rom[3970] = 8'h1B;
    rom[3971] = 8'h08;
    rom[3972] = 8'h1F;
    rom[3973] = 8'h08;
    rom[3974] = 8'h0C;
    rom[3975] = 8'h08;
    rom[3976] = 8'h04;
    rom[3977] = 8'h19;
    rom[3978] = 8'h08;
    rom[3979] = 8'h1D;
    rom[3980] = 8'h08;
    rom[3981] = 8'h80;
    rom[3982] = 8'hCD;
    rom[3983] = 8'h1E;
    rom[3984] = 8'h92;
    rom[3985] = 8'h80;
    rom[3986] = 8'h0A;
    rom[3987] = 8'h00;
    rom[3988] = 8'h15;
    rom[3989] = 8'hB6;
    rom[3990] = 8'h02;
    rom[3991] = 8'hA4;
    rom[3992] = 8'hF8;
    rom[3993] = 8'hB7;
    rom[3994] = 8'h02;
    rom[3995] = 8'hB6;
    rom[3996] = 8'h06;
    rom[3997] = 8'hAA;
    rom[3998] = 8'h07;
    rom[3999] = 8'hB7;
    rom[4000] = 8'h06;
    rom[4001] = 8'hCD;
    rom[4002] = 8'h1D;
    rom[4003] = 8'hF1;
    rom[4004] = 8'hA4;
    rom[4005] = 8'hF8;
    rom[4006] = 8'hB7;
    rom[4007] = 8'h06;
    rom[4008] = 8'h20;
    rom[4009] = 8'h0B;
    rom[4010] = 8'h15;
    rom[4011] = 8'h02;
    rom[4012] = 8'h14;
    rom[4013] = 8'h06;
    rom[4014] = 8'hCD;
    rom[4015] = 8'h1D;
    rom[4016] = 8'hF1;
    rom[4017] = 8'h15;
    rom[4018] = 8'h06;
    rom[4019] = 8'h14;
    rom[4020] = 8'h02;
    rom[4021] = 8'h81;
    rom[4022] = 8'h43;
    rom[4023] = 8'h00;
    rom[4024] = 8'h00;
    rom[4025] = 8'h00;
    rom[4026] = 8'h00;
    rom[4027] = 8'h00;
    rom[4028] = 8'h00;
    rom[4029] = 8'h00;
    rom[4030] = 8'h00;
    rom[4031] = 8'h00;
    rom[4032] = 8'h00;
    rom[4033] = 8'h00;
    rom[4034] = 8'h00;
    rom[4035] = 8'h00;
    rom[4036] = 8'h00;
    rom[4037] = 8'h00;
    rom[4038] = 8'h00;
    rom[4039] = 8'h00;
    rom[4040] = 8'h00;
    rom[4041] = 8'h00;
    rom[4042] = 8'h00;
    rom[4043] = 8'h00;
    rom[4044] = 8'h00;
    rom[4045] = 8'h00;
    rom[4046] = 8'h00;
    rom[4047] = 8'h00;
    rom[4048] = 8'h00;
    rom[4049] = 8'h00;
    rom[4050] = 8'h00;
    rom[4051] = 8'h00;
    rom[4052] = 8'h00;
    rom[4053] = 8'h00;
    rom[4054] = 8'h00;
    rom[4055] = 8'h00;
    rom[4056] = 8'h00;
    rom[4057] = 8'h00;
    rom[4058] = 8'h00;
    rom[4059] = 8'h00;
    rom[4060] = 8'h00;
    rom[4061] = 8'h00;
    rom[4062] = 8'h00;
    rom[4063] = 8'h00;
    rom[4064] = 8'h00;
    rom[4065] = 8'h00;
    rom[4066] = 8'h00;
    rom[4067] = 8'h00;
    rom[4068] = 8'h00;
    rom[4069] = 8'h00;
    rom[4070] = 8'h00;
    rom[4071] = 8'h00;
    rom[4072] = 8'h00;
    rom[4073] = 8'h00;
    rom[4074] = 8'h00;
    rom[4075] = 8'h00;
    rom[4076] = 8'h00;
    rom[4077] = 8'h00;
    rom[4078] = 8'h00;
    rom[4079] = 8'h00;
    rom[4080] = 8'h00;
    rom[4081] = 8'h00;
    rom[4082] = 8'h00;
    rom[4083] = 8'h00;
    rom[4084] = 8'h00;
    rom[4085] = 8'h00;
    rom[4086] = 8'h00;
    rom[4087] = 8'h00;
    rom[4088] = 8'h00;
    rom[4089] = 8'h00;
    rom[4090] = 8'h00;
    rom[4091] = 8'h00;
    rom[4092] = 8'h00;
    rom[4093] = 8'h00;
    rom[4094] = 8'h00;
    rom[4095] = 8'h00;
    rom[4096] = 8'hD8;
    rom[4097] = 8'h0F;
    rom[4098] = 8'h00;
    rom[4099] = 8'h81;
    rom[4100] = 8'hAE;
    rom[4101] = 8'h04;
    rom[4102] = 8'hD6;
    rom[4103] = 8'h1E;
    rom[4104] = 8'hFF;
    rom[4105] = 8'hE7;
    rom[4106] = 8'h8F;
    rom[4107] = 8'h5A;
    rom[4108] = 8'h26;
    rom[4109] = 8'hF8;
    rom[4110] = 8'h81;
    rom[4111] = 8'h5F;
    rom[4112] = 8'h10;
    rom[4113] = 8'h06;
    rom[4114] = 8'hAD;
    rom[4115] = 8'h47;
    rom[4116] = 8'h2F;
    rom[4117] = 8'hFC;
    rom[4118] = 8'h3F;
    rom[4119] = 8'h02;
    rom[4120] = 8'hB6;
    rom[4121] = 8'h00;
    rom[4122] = 8'hD7;
    rom[4123] = 8'h01;
    rom[4124] = 8'h00;
    rom[4125] = 8'h5C;
    rom[4126] = 8'hC6;
    rom[4127] = 8'h01;
    rom[4128] = 8'h00;
    rom[4129] = 8'h4A;
    rom[4130] = 8'h33;
    rom[4131] = 8'h02;
    rom[4132] = 8'hC7;
    rom[4133] = 8'h01;
    rom[4134] = 8'h00;
    rom[4135] = 8'h26;
    rom[4136] = 8'hE9;
    rom[4137] = 8'hCC;
    rom[4138] = 8'h01;
    rom[4139] = 8'h01;
    rom[4140] = 8'h04;
    rom[4141] = 8'h01;
    rom[4142] = 8'h05;
    rom[4143] = 8'h00;
    rom[4144] = 8'h01;
    rom[4145] = 8'hDD;
    rom[4146] = 8'hBC;
    rom[4147] = 8'h90;
    rom[4148] = 8'h07;
    rom[4149] = 8'h01;
    rom[4150] = 8'h34;
    rom[4151] = 8'h08;
    rom[4152] = 8'h07;
    rom[4153] = 8'hFD;
    rom[4154] = 8'h1C;
    rom[4155] = 8'h07;
    rom[4156] = 8'h20;
    rom[4157] = 8'h2D;
    rom[4158] = 8'h03;
    rom[4159] = 8'h96;
    rom[4160] = 8'h02;
    rom[4161] = 8'h44;
    rom[4162] = 8'h44;
    rom[4163] = 8'hE7;
    rom[4164] = 8'h04;
    rom[4165] = 8'hA6;
    rom[4166] = 8'h0A;
    rom[4167] = 8'h02;
    rom[4168] = 8'h96;
    rom[4169] = 8'h02;
    rom[4170] = 8'hA6;
    rom[4171] = 8'h66;
    rom[4172] = 8'hF7;
    rom[4173] = 8'hF1;
    rom[4174] = 8'h26;
    rom[4175] = 8'h0F;
    rom[4176] = 8'h43;
    rom[4177] = 8'h03;
    rom[4178] = 8'h96;
    rom[4179] = 8'h02;
    rom[4180] = 8'hA4;
    rom[4181] = 8'h0F;
    rom[4182] = 8'hF7;
    rom[4183] = 8'hF1;
    rom[4184] = 8'h26;
    rom[4185] = 8'h05;
    rom[4186] = 8'h81;
    rom[4187] = 8'hC7;
    rom[4188] = 8'h1F;
    rom[4189] = 8'hF0;
    rom[4190] = 8'h81;
    rom[4191] = 8'h4F;
    rom[4192] = 8'hAD;
    rom[4193] = 8'hF9;
    rom[4194] = 8'h20;
    rom[4195] = 8'hFB;
    rom[4196] = 8'h9A;
    rom[4197] = 8'h9B;
    rom[4198] = 8'h07;
    rom[4199] = 8'h94;
    rom[4200] = 8'hF6;
    rom[4201] = 8'h4F;
    rom[4202] = 8'h83;
    rom[4203] = 8'h3F;
    rom[4204] = 8'h00;
    rom[4205] = 8'hAE;
    rom[4206] = 8'h02;
    rom[4207] = 8'hBF;
    rom[4208] = 8'h96;
    rom[4209] = 8'hA6;
    rom[4210] = 8'hF0;
    rom[4211] = 8'hAD;
    rom[4212] = 8'hC9;
    rom[4213] = 8'hA6;
    rom[4214] = 8'h0F;
    rom[4215] = 8'hAD;
    rom[4216] = 8'hC5;
    rom[4217] = 8'h5A;
    rom[4218] = 8'hBF;
    rom[4219] = 8'h96;
    rom[4220] = 8'hA3;
    rom[4221] = 8'h00;
    rom[4222] = 8'h2A;
    rom[4223] = 8'hF1;
    rom[4224] = 8'h4F;
    rom[4225] = 8'h83;
    rom[4226] = 8'hAE;
    rom[4227] = 8'h90;
    rom[4228] = 8'hF7;
    rom[4229] = 8'hF1;
    rom[4230] = 8'h26;
    rom[4231] = 8'hD7;
    rom[4232] = 8'h4C;
    rom[4233] = 8'h7C;
    rom[4234] = 8'h26;
    rom[4235] = 8'hF9;
    rom[4236] = 8'hAD;
    rom[4237] = 8'hCD;
    rom[4238] = 8'h7C;
    rom[4239] = 8'h5C;
    rom[4240] = 8'h26;
    rom[4241] = 8'hF2;
    rom[4242] = 8'h83;
    rom[4243] = 8'hAD;
    rom[4244] = 8'h06;
    rom[4245] = 8'h83;
    rom[4246] = 8'hAD;
    rom[4247] = 8'h39;
    rom[4248] = 8'h83;
    rom[4249] = 8'h20;
    rom[4250] = 8'hC9;
    rom[4251] = 8'hAE;
    rom[4252] = 8'h13;
    rom[4253] = 8'hBF;
    rom[4254] = 8'h08;
    rom[4255] = 8'hBF;
    rom[4256] = 8'h12;
    rom[4257] = 8'hAD;
    rom[4258] = 8'hB8;
    rom[4259] = 8'h0F;
    rom[4260] = 8'h08;
    rom[4261] = 8'hFB;
    rom[4262] = 8'hB6;
    rom[4263] = 8'h09;
    rom[4264] = 8'hAB;
    rom[4265] = 8'h03;
    rom[4266] = 8'hB7;
    rom[4267] = 8'h95;
    rom[4268] = 8'h21;
    rom[4269] = 8'hFE;
    rom[4270] = 8'hB6;
    rom[4271] = 8'h09;
    rom[4272] = 8'hB1;
    rom[4273] = 8'h95;
    rom[4274] = 8'h26;
    rom[4275] = 8'hAB;
    rom[4276] = 8'h0F;
    rom[4277] = 8'h08;
    rom[4278] = 8'hEF;
    rom[4279] = 8'h4F;
    rom[4280] = 8'hAD;
    rom[4281] = 8'hA1;
    rom[4282] = 8'h0D;
    rom[4283] = 8'h08;
    rom[4284] = 8'hFA;
    rom[4285] = 8'h0D;
    rom[4286] = 8'h12;
    rom[4287] = 8'hF7;
    rom[4288] = 8'h81;
    rom[4289] = 8'h26;
    rom[4290] = 8'h9C;
    rom[4291] = 8'h3C;
    rom[4292] = 8'h01;
    rom[4293] = 8'hAD;
    rom[4294] = 8'h94;
    rom[4295] = 8'h80;
    rom[4296] = 8'h19;
    rom[4297] = 8'h12;
    rom[4298] = 8'h20;
    rom[4299] = 8'h02;
    rom[4300] = 8'h19;
    rom[4301] = 8'h08;
    rom[4302] = 8'h38;
    rom[4303] = 8'h94;
    rom[4304] = 8'h80;
    rom[4305] = 8'hCD;
    rom[4306] = 8'h1F;
    rom[4307] = 8'h04;
    rom[4308] = 8'h4F;
    rom[4309] = 8'h5F;
    rom[4310] = 8'hBD;
    rom[4311] = 8'h90;
    rom[4312] = 8'h5C;
    rom[4313] = 8'h26;
    rom[4314] = 8'hFB;
    rom[4315] = 8'h3C;
    rom[4316] = 8'h91;
    rom[4317] = 8'hAE;
    rom[4318] = 8'h20;
    rom[4319] = 8'hB3;
    rom[4320] = 8'h91;
    rom[4321] = 8'h26;
    rom[4322] = 8'hF2;
    rom[4323] = 8'h43;
    rom[4324] = 8'h81;
    rom[4325] = 8'h44;
    rom[4326] = 8'h1F;
    rom[4327] = 8'hC8;
    rom[4328] = 8'h1F;
    rom[4329] = 8'hCC;
    rom[4330] = 8'h1F;
    rom[4331] = 8'hCE;
    rom[4332] = 8'h1F;
    rom[4333] = 8'hC1;
    rom[4334] = 8'h1F;
    rom[4335] = 8'h2C;
    rom[4336] = 8'h0F;
    rom[4337] = 8'h71;
    rom[4338] = 8'h0F;
    rom[4339] = 8'h71;
    rom[4340] = 8'h0F;
    rom[4341] = 8'h71;
    rom[4342] = 8'h1E;
    rom[4343] = 8'h10;
    rom[4344] = 8'h1E;
    rom[4345] = 8'h7F;
    rom[4346] = 8'h1E;
    rom[4347] = 8'h8E;
    rom[4348] = 8'h0F;
    rom[4349] = 8'h71;
    rom[4350] = 8'h0F;
    rom[4351] = 8'h71;
    pram[0] = 8'h00;
    pram[1] = 8'h80;
    pram[2] = 8'h4F;
    pram[3] = 8'h48;
    pram[4] = 8'h00;
    pram[5] = 8'h00;
    pram[6] = 8'h00;
    pram[7] = 8'h00;
    pram[8] = 8'h13;
    pram[9] = 8'h88;
    pram[10] = 8'h00;
    pram[11] = 8'h4C;
    pram[12] = 8'h4E;
    pram[13] = 8'h75;
    pram[14] = 8'h4D;
    pram[15] = 8'h63;
    pram[16] = 8'hA8;
    pram[17] = 8'h00;
    pram[18] = 8'h00;
    pram[19] = 8'h00;
    pram[20] = 8'hCC;
    pram[21] = 8'h0A;
    pram[22] = 8'hCC;
    pram[23] = 8'h0A;
    pram[24] = 8'h00;
    pram[25] = 8'h00;
    pram[26] = 8'h00;
    pram[27] = 8'h00;
    pram[28] = 8'h00;
    pram[29] = 8'h02;
    pram[30] = 8'h63;
    pram[31] = 8'h00;
    pram[32] = 8'h00;
    pram[33] = 8'h00;
    pram[34] = 8'h00;
    pram[35] = 8'h00;
    pram[36] = 8'h00;
    pram[37] = 8'h00;
    pram[38] = 8'h00;
    pram[39] = 8'h00;
    pram[40] = 8'h00;
    pram[41] = 8'h00;
    pram[42] = 8'h00;
    pram[43] = 8'h00;
    pram[44] = 8'h00;
    pram[45] = 8'h00;
    pram[46] = 8'h00;
    pram[47] = 8'h00;
    pram[48] = 8'h00;
    pram[49] = 8'h00;
    pram[50] = 8'h00;
    pram[51] = 8'h00;
    pram[52] = 8'h00;
    pram[53] = 8'h00;
    pram[54] = 8'h00;
    pram[55] = 8'h00;
    pram[56] = 8'h00;
    pram[57] = 8'h00;
    pram[58] = 8'h00;
    pram[59] = 8'h00;
    pram[60] = 8'h00;
    pram[61] = 8'h00;
    pram[62] = 8'h00;
    pram[63] = 8'h00;
    pram[64] = 8'h00;
    pram[65] = 8'h00;
    pram[66] = 8'h00;
    pram[67] = 8'h00;
    pram[68] = 8'h00;
    pram[69] = 8'h00;
    pram[70] = 8'h00;
    pram[71] = 8'h00;
    pram[72] = 8'h00;
    pram[73] = 8'h00;
    pram[74] = 8'h00;
    pram[75] = 8'h00;
    pram[76] = 8'h00;
    pram[77] = 8'h00;
    pram[78] = 8'h00;
    pram[79] = 8'h00;
    pram[80] = 8'h00;
    pram[81] = 8'h00;
    pram[82] = 8'h00;
    pram[83] = 8'h00;
    pram[84] = 8'h00;
    pram[85] = 8'h00;
    pram[86] = 8'h00;
    pram[87] = 8'h29;
    pram[88] = 8'h82;
    pram[89] = 8'hA6;
    pram[90] = 8'h06;
    pram[91] = 8'h00;
    pram[92] = 8'h00;
    pram[93] = 8'h00;
    pram[94] = 8'h00;
    pram[95] = 8'h00;
    pram[96] = 8'h00;
    pram[97] = 8'h00;
    pram[98] = 8'h00;
    pram[99] = 8'h00;
    pram[100] = 8'h00;
    pram[101] = 8'h00;
    pram[102] = 8'h00;
    pram[103] = 8'h00;
    pram[104] = 8'h00;
    pram[105] = 8'h00;
    pram[106] = 8'h00;
    pram[107] = 8'h00;
    pram[108] = 8'h00;
    pram[109] = 8'h00;
    pram[110] = 8'h00;
    pram[111] = 8'h00;
    pram[112] = 8'h00;
    pram[113] = 8'h00;
    pram[114] = 8'h00;
    pram[115] = 8'h00;
    pram[116] = 8'h00;
    pram[117] = 8'h00;
    pram[118] = 8'h00;
    pram[119] = 8'h01;
    pram[120] = 8'hFF;
    pram[121] = 8'hFF;
    pram[122] = 8'hFF;
    pram[123] = 8'hDF;
    pram[124] = 8'h00;
    pram[125] = 8'h00;
    pram[126] = 8'h00;
    pram[127] = 8'h00;
    pram[128] = 8'h00;
    pram[129] = 8'h00;
    pram[130] = 8'h00;
    pram[131] = 8'h00;
    pram[132] = 8'h00;
    pram[133] = 8'h00;
    pram[134] = 8'h00;
    pram[135] = 8'h00;
    pram[136] = 8'h00;
    pram[137] = 8'h00;
    pram[138] = 8'h00;
    pram[139] = 8'h00;
    pram[140] = 8'h00;
    pram[141] = 8'h00;
    pram[142] = 8'h00;
    pram[143] = 8'h00;
    pram[144] = 8'h00;
    pram[145] = 8'h00;
    pram[146] = 8'h00;
    pram[147] = 8'h00;
    pram[148] = 8'h00;
    pram[149] = 8'h00;
    pram[150] = 8'h00;
    pram[151] = 8'h00;
    pram[152] = 8'h00;
    pram[153] = 8'h00;
    pram[154] = 8'h00;
    pram[155] = 8'h00;
    pram[156] = 8'h00;
    pram[157] = 8'h00;
    pram[158] = 8'h00;
    pram[159] = 8'h00;
    pram[160] = 8'h00;
    pram[161] = 8'h00;
    pram[162] = 8'h00;
    pram[163] = 8'h00;
    pram[164] = 8'h00;
    pram[165] = 8'h00;
    pram[166] = 8'h00;
    pram[167] = 8'h00;
    pram[168] = 8'h00;
    pram[169] = 8'h00;
    pram[170] = 8'h00;
    pram[171] = 8'h00;
    pram[172] = 8'h00;
    pram[173] = 8'h00;
    pram[174] = 8'h00;
    pram[175] = 8'h00;
    pram[176] = 8'h00;
    pram[177] = 8'h00;
    pram[178] = 8'h00;
    pram[179] = 8'h00;
    pram[180] = 8'h00;
    pram[181] = 8'h00;
    pram[182] = 8'h00;
    pram[183] = 8'h00;
    pram[184] = 8'h00;
    pram[185] = 8'h00;
    pram[186] = 8'h00;
    pram[187] = 8'h00;
    pram[188] = 8'h00;
    pram[189] = 8'h00;
    pram[190] = 8'h00;
    pram[191] = 8'h00;
    pram[192] = 8'h00;
    pram[193] = 8'h00;
    pram[194] = 8'h00;
    pram[195] = 8'h00;
    pram[196] = 8'h00;
    pram[197] = 8'h00;
    pram[198] = 8'h00;
    pram[199] = 8'h00;
    pram[200] = 8'h00;
    pram[201] = 8'h00;
    pram[202] = 8'h00;
    pram[203] = 8'h00;
    pram[204] = 8'h00;
    pram[205] = 8'h00;
    pram[206] = 8'h00;
    pram[207] = 8'h00;
    pram[208] = 8'h00;
    pram[209] = 8'h00;
    pram[210] = 8'h00;
    pram[211] = 8'h00;
    pram[212] = 8'h00;
    pram[213] = 8'h00;
    pram[214] = 8'h00;
    pram[215] = 8'h00;
    pram[216] = 8'h00;
    pram[217] = 8'h00;
    pram[218] = 8'h00;
    pram[219] = 8'h00;
    pram[220] = 8'h00;
    pram[221] = 8'h00;
    pram[222] = 8'h00;
    pram[223] = 8'h00;
    pram[224] = 8'h00;
    pram[225] = 8'h00;
    pram[226] = 8'h00;
    pram[227] = 8'h00;
    pram[228] = 8'h00;
    pram[229] = 8'h00;
    pram[230] = 8'h00;
    pram[231] = 8'h00;
    pram[232] = 8'h00;
    pram[233] = 8'h00;
    pram[234] = 8'h00;
    pram[235] = 8'h00;
    pram[236] = 8'h00;
    pram[237] = 8'h00;
    pram[238] = 8'h00;
    pram[239] = 8'h00;
    pram[240] = 8'h00;
    pram[241] = 8'h00;
    pram[242] = 8'h00;
    pram[243] = 8'h00;
    pram[244] = 8'h00;
    pram[245] = 8'h00;
    pram[246] = 8'h00;
    pram[247] = 8'h00;
    pram[248] = 8'h00;
    pram[249] = 8'h00;
    pram[250] = 8'h00;
    pram[251] = 8'h00;
    pram[252] = 8'h00;
    pram[253] = 8'h00;
    pram[254] = 8'h00;
    pram[255] = 8'h00;

    // Initialize RAM to zeros (critical for proper Egret firmware operation)
    // 368 bytes = 0x170 = RAM from 0x90-0x1FF
    for (init_i = 0; init_i < 368; init_i = init_i + 1) begin
        intram[init_i] = 8'h00;
    end

    pram_loaded = 1'b0;
    pc_bit3_prev = 1'b0;
end

// Address decoding - M68HC05E1 memory map from MAME
// The M68HC05E1 uses 13-bit addressing (0x0000-0x1FFF), higher bits wrap
// Ports: 0x00-0x02, DDRs: 0x04-0x06, PLL: 0x07, Timer: 0x08-0x09, OneSecond: 0x12
// RAM: 0x90-0x1FF
// ROM: 4352 bytes (0x1100) mapped at CPU 0x0F00-0x1FFF
// Simple linear mapping: ROM offset = CPU address - 0x0F00
wire [12:0] addr13 = cpu_addr[12:0];  // Mask to 13 bits for M68HC05E1
wire port_cs = (addr13 < 13'h0020);  // I/O registers at 0x00-0x1F (includes timer, onesec)
wire ram_cs  = (addr13 >= 13'h0090) && (addr13 < 13'h0200);  // RAM at 0x90-0x1FF
wire rom_cs  = (addr13 >= 13'h0F00);  // ROM covers 0x0F00-0x1FFF
// ROM address: simple offset from 0x0F00
// CPU 0x0F00 -> ROM[0x000], CPU 0x1FFF -> ROM[0x10FF]
wire [12:0] rom_addr = addr13 - 13'h0F00;
wire [8:0]  ram_addr = addr13[8:0] - 9'h90;  // RAM offset

// ============================================================================
// 68HC05E1 Timer (from MAME m68hc05e1.cpp)
// ============================================================================
// 0x07: PLL Control - sets timer rate (bits 0-1 = clock divider)
// 0x08: Timer Control Register
//       Bit 7: Timer flag (set on tick, cleared by writing 0)
//       Bit 6: Alternate timer flag
//       Bit 5: Timer interrupt enable
// 0x09: Timer Counter - 8-bit free-running, (total_cycles / 4) % 256
// 0x12: One-second timer (for RTC)

reg [7:0]  pll_ctrl;     // PLL control (0x07)
reg [7:0]  timer_ctrl;   // Timer control (0x08)
reg [7:0]  onesec_ctrl;  // One-second control (0x12)
reg [31:0] cycle_total;  // Total cycles for timer counter

// Timer prescaler based on PLL setting
reg [15:0] timer_prescale;
reg [15:0] timer_prescale_max;
reg [15:0] pll_lock_counter;

// Timer hardware logic is merged into the port/register write block below
// to avoid multiple drivers on pll_ctrl, timer_ctrl, onesec_ctrl

// Timer IRQ is level-sensitive: stays asserted while timer flag (bit 7) AND enable (bit 5) are set
// Firmware clears by writing 0 to bit 7 of timer_ctrl
wire timer_irq_n = ~(timer_ctrl[7] & timer_ctrl[5]);

// Timer counter reads as (total_cycles / 4) % 256
wire [7:0] timer_counter = cycle_total[9:2];  // Divide by 4, take lower 8 bits

// ============================================================================
// One-second timer (M68HC05E1-specific)
// ============================================================================
// The M68HC05E1 has a dedicated one-second timer that:
// 1. Generates an interrupt (vector $FFF6) -> ISR at $1E10 increments $CC
// 2. Sets Port C bit 1 as a hardware flag (for polling by firmware)
// Register $12 bit 4 enables the timer, bit 6 is cleared by ISR
//
// Real hardware: 32.768 kHz crystal / 32768 = 1 Hz
// Our timebase: cen = clk32/8 = 32.5 MHz/8 = 4.0625 MHz exactly (pll outclk_1).
// The counter spans 0..PERIOD inclusive, so a fire every PERIOD+1 cen ticks:
// 4,062,499 + 1 = 4,062,500 cen = 1.000 s. (The old 4,000,000 at an assumed
// 4 MHz ran the guest wall clock ~1.56% fast.)
`ifdef SIMULATION
localparam ONESEC_PERIOD = 22'd8192;    // ~2ms (fast for simulation)
`else
localparam ONESEC_PERIOD = 22'd4062499; // exactly 1 second at 4.0625 MHz
`endif

reg [21:0] onesec_counter;
reg        onesec_irq_flag;  // Sticky flag; held until the ISR's $12 bit-6 ack
wire       onesec_irq_n = ~onesec_irq_flag;

`ifdef SIMULATION
// One-second liveness witness: FIRE when the flag sets, ACK when the ISR's
// BCLR 6,$12 clears it. A FIRE with no following ACK = the ISR is dead (the
// pre-fix edge-drop deadlock signature). ~500 pairs/sim-second (2ms period).
reg onesec_flag_d;
always @(posedge clk) begin
    onesec_flag_d <= onesec_irq_flag;
    if (onesec_irq_flag && !onesec_flag_d)
        $display("EGRET_ONESEC_FIRE @%0t", $time);
    if (!onesec_irq_flag && onesec_flag_d)
        $display("EGRET_ONESEC_ACK @%0t", $time);
end
`endif

always @(posedge clk) begin
    if (reset) begin
        onesec_counter <= 22'd0;
        onesec_irq_flag <= 1'b0;
    end else if (cen) begin
        // Count when one-second timer is enabled (onesec_ctrl bit 4)
        if (onesec_ctrl[4]) begin
            if (onesec_counter >= ONESEC_PERIOD) begin
                onesec_counter <= 22'd0;
                onesec_irq_flag <= 1'b1;
                `ifdef VERBOSE_TRACE
                $display("EGRET_ONESEC[%0d]: Timer fired! Setting PC1 and IRQ", cycle_count);
                `endif
            end else begin
                onesec_counter <= onesec_counter + 22'd1;
            end
        end

        // Clear IRQ flag when firmware clears bit 6 of onesec_ctrl ($12)
        // The ISR does "BCLR 6,$12" as its last action before RTI
        if (port_cs && !cpu_wr && cpu_addr[4:0] == 5'h12) begin
            if (!(cpu_dout & 8'h40)) begin  // Writing 0 to bit 6
                onesec_irq_flag <= 1'b0;
            end
        end
    end
end

// ============================================================================
// Port A - ADB and system control
// ============================================================================
// Bit 7 (O): ADB data line out
// Bit 6 (I): ADB data line in
// Bit 5 (I): System type (1 = Egret controls power)
// Bit 4 (O): DFAC latch
// Bit 3 (O): 680x0 reset pulse
// Bit 2 (I): Keyboard power switch
// Bit 1-0: PSU control

// Port A input - bit 5 is system type (1 = Mac LC)
wire [7:0] pa_external = {
    1'b1,                   // Bit 7: tied high
    adb_data_in,            // Bit 6: ADB data in
    1'b1,                   // Bit 5: System type (1 = Mac LC)
    1'b1,                   // Bit 4: tied high
    1'b1,                   // Bit 3: tied high
    1'b1,                   // Bit 2: tied high
    1'b1,                   // Bit 1: tied high
    1'b1                    // Bit 0: tied high
};

wire [7:0] pa_in = port_test_done ?
    ((pa_latch & pa_ddr) | (pa_external & ~pa_ddr)) :
    pa_latch;

always @(*) begin
    adb_data_out = pa_out[7];
end

`ifdef SIMULATION
// Log ADB line drive edges with timestamps so we can derive the HC05's wire-level
// ADB cell timing (attention/sync/data cells) when designing the ADB device.
reg adb_out_prev;
always @(posedge clk) begin
    if (cen) begin
        if (adb_data_out !== adb_out_prev) begin
            $display("ADBLINE[%0d]: out=%b (line=%b) PA7=%b", cycle_count, adb_data_out, ~adb_data_out, pa_out[7]);
            adb_out_prev <= adb_data_out;
        end
    end
end
`endif

// ============================================================================
// Port B - VIA interface (this is the key interface)
// ============================================================================
// Bit 7 (O): DFAC clock (I2C SCL)
// Bit 6 (I/O): DFAC data (I2C SDA)
// Bit 5 (I/O): VIA shift register data = CB2
// Bit 4 (O): VIA clock = CB1
// Bit 3 (I): VIA SYS_SESSION = TIP from VIA
// Bit 2 (I): VIA_FULL (tied high for now)
// Bit 1 (O): VIA XCEIVER SESSION = TREQ to VIA
// Bit 0 (I): +5V sense

// Port B input handling:
// - During port test: use latch mode (so test passes)
// - After port test: use DDR-based mixing (for VIA communication)
//
// External signals (matching MAME's pb_r()):
// - bit 0: +5V sense (always 1)
// - bit 2: via_full/byteack from VIA
// - bit 3: sys_session (TIP from VIA PB5)
// - bit 5: via_data (CB2 from VIA)
// - bit 6: DFAC data (tied high)
// - bit 7: DFAC clock (output, external = 0)
// Clock domain crossing synchronizers for VIA signals
// CRITICAL: Sample on every system clock (32MHz), not just cen (4MHz)!
// The 68020 runs at 8MHz+ and can set TIP=0, then TIP=1 before Egret
// would see TIP=0 if we only sample at 4MHz. Sampling at 32MHz gives
// Egret 3 system cycles (~94ns) to see TIP changes.
reg [2:0] via_tip_sync;
reg [2:0] via_cb2_in_sync;
reg [2:0] via_byteack_in_sync;

always @(posedge clk) begin
    if (reset) begin
        via_tip_sync <= 3'b111;       // TIP idle high initially
        via_cb2_in_sync <= 3'b111;
        via_byteack_in_sync <= 3'b111;
    end else begin
        // Sample on EVERY clock, not just cen, to catch fast TIP changes
        via_tip_sync <= {via_tip_sync[1:0], via_tip};
        via_cb2_in_sync <= {via_cb2_in_sync[1:0], via_cb2_in};
        via_byteack_in_sync <= {via_byteack_in_sync[1:0], via_byteack_in};
    end
end

wire via_tip_stable = via_tip_sync[2];
wire via_cb2_in_stable = via_cb2_in_sync[2];
wire via_byteack_in_stable = via_byteack_in_sync[2];

// pb_external represents external signals read by Egret firmware
// BYTEACK (bit 2): LOW when VIA is ready for data, HIGH when VIA has data pending
// The firmware uses BYTEACK in two contexts:
// - At 0x12AF: brset 2, $01, $12A1 - needs BYTEACK=0 to proceed (start of communication)
// - At 0x14CE: brset 2, $01, $14D6 - needs BYTEACK=1 to call CB1 clocking (VIA has data)
wire [7:0] pb_external = {
    1'b0,                   // Bit 7: DFAC clock (external reads as 0)
    1'b1,                   // Bit 6: DFAC data (tied high)
    via_cb2_in_stable,      // Bit 5: CB2 data from VIA (synchronized)
    1'b0,                   // Bit 4: CB1 clock (external reads as 0)
    via_tip_effective,      // Bit 3: TIP from VIA (gated after reset release)
    via_byteack_in_stable,  // Bit 2: VIA_FULL/BYTEACK - from VIA PB4
    1'b0,                   // Bit 1: TREQ (external reads as 0)
    1'b1                    // Bit 0: +5V sense (always active)
};

// Port test mode: use latch reads for first N cycles, then switch to DDR mode
reg port_test_done;
reg [15:0] port_test_counter;

always @(posedge clk) begin
    if (reset) begin
        port_test_done <= 1'b0;
        port_test_counter <= 16'h0;
    end else if (cen && !port_test_done) begin
        port_test_counter <= port_test_counter + 1;
        if (port_test_counter >= 16'd500) begin  // ~500 cycles for port test
            port_test_done <= 1'b1;
        end
    end
end

// Use latch during port test, DDR-based mixing after
wire [7:0] pb_in = port_test_done ?
    ((pb_latch & pb_ddr) | (pb_external & ~pb_ddr)) :
    pb_latch;

// Handshake initialization state machine
// CRITICAL: XCVR_SESSION (TREQ) must be LOW before CB1 clocking starts (per MAME)
typedef enum logic [2:0] {
    INIT_WAIT,    // Wait for stable power
    INIT_ASSERT,  // Assert TREQ (XCVR_SESSION)
    INIT_DELAY,   // Hold TREQ before allowing clocking
    RUNNING       // Normal operation
} init_state_t;

init_state_t init_state;
reg [15:0] handshake_timer;
reg handshake_done;
reg force_treq;

// TIP gate: hold TIP at idle level (0 = no session) from Egret's perspective
// until the Egret firmware has entered its idle/communication loop. The firmware's main init loop at
// $1034-$1066 probes for ADB devices (dozens of iterations with timeouts),
// which takes far longer than the 68020's boot-to-first-Egret-command time.
// We detect when the Egret PC reaches $12AC (the TIP polling instruction in
// the idle loop) and only then release the gate, letting Egret see TIP.
reg        tip_gate_holding;
reg        egret_idle_reached;  // Set once Egret PC hits $12AC

always @(posedge clk) begin
    if (reset) begin
        tip_gate_holding <= 1'b0;
        egret_idle_reached <= 1'b0;
    end else if (cen) begin
        // Start holding TIP when 68020 is released from reset
        if (!reset_680x0_latched && !tip_gate_holding && !egret_idle_reached) begin
            tip_gate_holding <= 1'b1;
`ifdef SIMULATION
            $display("EGRET[%0d]: TIP gate started (waiting for idle loop at $12AC)", cycle_count);
`endif
        end

        // Detect Egret firmware reaching idle loop TIP poll at $12AC
        // addr13 is the 13-bit ROM address; $12AC in CPU space = $12AC
        if (!egret_idle_reached && rom_cs && cpu_wr && addr13 == 13'h12AC) begin
            egret_idle_reached <= 1'b1;
            tip_gate_holding <= 1'b0;
`ifdef SIMULATION
            $display("EGRET[%0d]: Idle loop reached! TIP gate released (real TIP=%b)", cycle_count, via_tip_stable);
`endif
        end
    end
end

// Gate value 0 = idle (PB5=0 on host = no session). Matches MAME: sys_session=0 means idle.
wire via_tip_effective = tip_gate_holding ? 1'b0 : via_tip_stable;

always @(posedge clk) begin
    if (reset) begin
        handshake_timer <= 0;
        handshake_done <= 0;
        force_treq <= 0;
        init_state <= INIT_WAIT;
    end else if (cen) begin
        // Removed force_treq state machine - let firmware control TREQ from start
        // The firmware initializes Port B with 0x92 (TREQ inactive), then asserts
        // TREQ via bclr 1, $01 at address 0x1549 when ready to transfer data.
        // The old state machine was forcing TREQ active from cycle 8192-14336,
        // which conflicted with firmware's 0x92 write at cycle ~9520.
        case (init_state)
            INIT_WAIT: begin
                // Skip straight to RUNNING - firmware controls TREQ
                if (handshake_timer == 16'h2000) begin
                    handshake_done <= 1'b1;
                    init_state <= RUNNING;
                    `ifdef SIMULATION
                    $display("EGRET_INIT[%0d]: Entering RUNNING state (firmware controls TREQ)", handshake_timer);
                    `endif
                end else begin
                    handshake_timer <= handshake_timer + 1;
                end
            end

            INIT_ASSERT: begin
                // Not used anymore
                init_state <= RUNNING;
            end

            INIT_DELAY: begin
                // Not used anymore
                init_state <= RUNNING;
            end

            RUNNING: begin
                // Normal operation - Egret controls TREQ via pb_out[1]
            end
        endcase
    end
end

// Output assignments
// CB1: Pass directly from Egret firmware. The V8 protocol uses TIP pulses as part
// of the handshake, so we should NOT gate CB1 based on TIP. The firmware controls
// CB1 timing explicitly for shift register clocking.
assign cuda_cb1    = pb_out[4];
assign cuda_cb2    = pb_out[5];
assign cuda_cb2_oe = pb_ddr[5];
// TREQ signal polarity with DDR gating and port test guard:
// - pb_out[1]=0 AND pb_ddr[1]=1 means Egret asserts TREQ (drives pin LOW = has data)
// - pb_out[1]=1 or pb_ddr[1]=0 means TREQ released (pin floats HIGH = idle)
// CRITICAL: Must check DDR to prevent spurious TREQ assertion during early boot
// when firmware clears port latches before setting DDR (pb_out=0x00, pb_ddr=0x00)
// ALSO: Don't assert TREQ during port test phase (firmware init writes 0x00 to Port B)
// dataController expects cuda_treq=1 when TREQ is asserted
assign cuda_treq = port_test_done & pb_ddr[1] & ~pb_out[1];

`ifdef VERBOSE_TRACE
// Debug cuda_treq formula - trace each component
reg cuda_treq_prev;
always @(posedge clk) begin
    if (reset) begin
        cuda_treq_prev <= 0;
    end else if (cen) begin
        if (cuda_treq != cuda_treq_prev) begin
            $display("EGRET_TREQ[%0d]: cuda_treq=%b->%b (port_test_done=%b, pb_ddr[1]=%b, pb_out[1]=%b, pb_latch[1]=%b, pb_ddr=%02x, pb_out=%02x)",
                     cycle_count, cuda_treq_prev, cuda_treq,
                     port_test_done, pb_ddr[1], pb_out[1], pb_latch[1], pb_ddr, pb_out);
        end
        cuda_treq_prev <= cuda_treq;
    end
end
`endif
assign cuda_byteack = 1'b0;       // Not used in Egret

assign cuda_portb    = pb_out;
assign cuda_portb_oe = pb_ddr;

// Debug outputs
assign dbg_cen            = cen;
assign dbg_port_test_done = port_test_done;
assign dbg_handshake_done = handshake_done;
assign dbg_treq           = cuda_treq;
assign dbg_tip_in         = via_tip_stable;
assign dbg_byteack_in     = via_byteack_in_stable;
assign dbg_pb_out         = pb_out;
assign dbg_pc_out         = pc_out;
assign dbg_cpu_running    = ~reset;

// ============================================================================
// Port C - 68000 control
// ============================================================================
// Bit 3 (O): 680x0 reset
// Bit 2: IPL2
// Bit 1-0: IPL1-0

// Port C input - use latch values for port test, but handle bit 3 specially.
// Port C is mostly outputs (reset, IPL) so we don't need external reads.
// The port test writes a value and expects to read it back.
//
// CRITICAL: When bit 3 (reset) is configured as INPUT (DDR[3]=0), the firmware
// expects it to read as 0 (reset released). This happens at 0x1291 after the
// firmware re-asserts reset at 0x128F. If we return the latch value (1), the
// 68020 stays in reset forever.
wire [7:0] pc_in = {pc_latch[7:4], (pc_ddr[3] ? pc_latch[3] : 1'b0), pc_latch[2:0]};

// 68020 reset control - match MAME behavior exactly
// Per MAME egret.cpp and egret.sv: pc_out[3]=1 means RELEASE, pc_out[3]=0 means HOLD
// (egret.sv uses: reset_680x0 = ~pc_out[3], so pc_out[3]=1 → reset_680x0=0 → release)
//
// We latch the reset state so that when the firmware switches pc_ddr[3] back to input
// (at $1291: BCLR3 $06), the 68020 stays in its last commanded state (released).
// Hold in reset until port_test_done AND firmware has configured PC bit 3 as output.
reg reset_680x0_latched;

always @(posedge clk) begin
    if (reset) begin
        reset_680x0_latched <= 1'b1;  // Hold 68020 in reset during Egret reset
    end else if (cen && port_test_done && pc_ddr[3]) begin
        // Only update when firmware is actively driving PC bit 3 as output
        // Invert: pc_out[3]=1 means release (reset_680x0=0), pc_out[3]=0 means hold (reset_680x0=1)
        reset_680x0_latched <= ~pc_out[3];
    end
end

always @(*) begin
    // Also hold the 68020 in reset until the saved PRAM has been copied into the
    // Egret's working RAM (pram_loaded) — so the CPU never starts executing on
    // pre-load PRAM, no matter how late the SD load of pram[] completes.
    reset_680x0 = reset_680x0_latched | ~pram_loaded;
    nmi_680x0 = 1'b0;
end

// ============================================================================
// Port output logic (68HC05 style: out = (latch & ddr) | (in & ~ddr))
// ============================================================================
always @(posedge clk) begin
    if (reset) begin
        pa_out <= 8'h00;
        pb_out <= 8'h00;
        pc_out <= 8'h08;  // Bit 3 = 1: hold 68020 in reset initially (MAME behavior)
        pc_bit3_prev <= 1'b1;  // Match initial pc_out[3]
    end else if (cen) begin
        pa_out <= (pa_latch & pa_ddr) | (pa_in & ~pa_ddr);
        pb_out <= (pb_latch & pb_ddr) | (pb_in & ~pb_ddr);
        pc_out <= (pc_latch & pc_ddr) | (pc_in & ~pc_ddr);

        // Track Port C bit 3 (the PRAM boot-copy block below latches the 1->0
        // edge and waits for pram_ready before copying / setting pram_loaded)
        pc_bit3_prev <= pc_out[3];
    end
end

// PRAM loading flag - actual copy is done in the intram write block below
// to avoid multiple drivers on intram

// ============================================================================
// Port and DDR register writes
// ============================================================================
always @(posedge clk) begin
    if (reset) begin
        pa_latch <= 8'h00;
        pb_latch <= 8'h02;  // Bit 1 = 1 means TREQ inactive on startup (firmware writes 0x92 later)
        pc_latch <= 8'h00;
        pa_ddr   <= 8'h00;
        pb_ddr   <= 8'h00;  // All inputs on reset (firmware sets 0x92 = bits 7,4,1 outputs)
        pc_ddr   <= 8'h00;
        pll_ctrl <= 8'h00;
        timer_ctrl <= 8'h00;
        onesec_ctrl <= 8'h00;
        cycle_total <= 32'h0;
        timer_prescale <= 16'h0;
        timer_prescale_max <= 16'd1024;
        pll_lock_counter <= 16'h0;
    end else if (cen) begin
        // --- Timer hardware (runs every cen tick) ---
        // PLL lock after 500 cycles
        if (pll_lock_counter < 16'd500) begin
            pll_lock_counter <= pll_lock_counter + 1;
        end else begin
            pll_ctrl[6] <= 1'b1;  // Set LOCK bit
        end
        // Total cycle counter for timer
        cycle_total <= cycle_total + 1;
        // Prescaled timer tick
        timer_prescale <= timer_prescale + 1;
        if (timer_prescale >= timer_prescale_max) begin
            timer_prescale <= 0;
            timer_ctrl[7] <= 1'b1;
            `ifdef VERBOSE_TRACE
            if (timer_ctrl[5] && !timer_ctrl[7])
                $display("TIMER[%0d]: Tick, flag set (timer_ctrl=%02x)", cycle_count, timer_ctrl);
            `endif
        end

        // --- Port/register writes from CPU ---
        if (port_cs && !cpu_wr) begin  // !cpu_wr means write
        case (cpu_addr[4:0])  // 5 bits for 0x00-0x1F
            5'h00: pa_latch <= cpu_dout;
            5'h01: begin
                pb_latch <= cpu_dout;
`ifdef SIMULATION
                if (cpu_dout[4] != pb_latch[4])  // CB1 changed
                    $display("EGRET[%0d]: PB_W 0x%02x CB1=%b PC=%04x", cycle_count, cpu_dout, cpu_dout[4], last_pc);
`endif
            end
            5'h02: pc_latch <= cpu_dout;
            5'h04: pa_ddr   <= cpu_dout;
            5'h05: pb_ddr <= cpu_dout;  // Allow DDR writes - cuda_treq DDR gating prevents early TREQ
            5'h06: pc_ddr   <= cpu_dout;
            // M68HC05E1 registers
            5'h07: begin  // PLL control - sets timer rate
                pll_ctrl <= cpu_dout;
                // Set timer prescaler based on PLL clock bits
                case (cpu_dout[1:0])
                    2'b00: timer_prescale_max <= 16'd2048;   // 512 kHz / 1024 = ~500 Hz
                    2'b01: timer_prescale_max <= 16'd1024;   // 1 MHz / 1024 = ~1 kHz
                    2'b10: timer_prescale_max <= 16'd512;    // 2 MHz / 1024 = ~2 kHz
                    2'b11: timer_prescale_max <= 16'd256;    // 4 MHz / 1024 = ~4 kHz
                endcase
                `ifdef VERBOSE_TRACE
                $display("EGRET[%0d]: PLL write = 0x%02x (clock rate %0d)", cycle_count, cpu_dout, cpu_dout[1:0]);
                `endif
            end
            5'h08: begin  // Timer control
                // Clear flags by writing 0 to bits 7 or 6
                if (!(cpu_dout & 8'h80)) timer_ctrl[7] <= 1'b0;
                if (!(cpu_dout & 8'h40)) timer_ctrl[6] <= 1'b0;
                timer_ctrl[5:0] <= cpu_dout[5:0];
                `ifdef VERBOSE_TRACE
                $display("EGRET[%0d]: Timer ctrl write = 0x%02x", cycle_count, cpu_dout);
                `endif
            end
            5'h12: begin  // One-second timer
                onesec_ctrl <= cpu_dout;
            end
        endcase
        end // port_cs write

        // One-second timer hardware: set Port C bit 1 when timer fires
        // Per MAME m68hc05e1: m_portc_data |= 0x02 on one-second tick
        // This flag persists until firmware clears it (via Port C write)
        if (onesec_irq_flag && !pc_latch[1]) begin
            pc_latch[1] <= 1'b1;
            `ifdef VERBOSE_TRACE
            $display("EGRET_ONESEC[%0d]: Setting PC1 flag (Port C bit 1)", cycle_count);
            `endif
        end
    end // cen
end

// ============================================================================
// RAM (368 bytes at 0x90-0x1FF for M68HC05E1)
// ============================================================================
// RAM read is combinational, write is synchronous
always @(*) begin
    if (ram_cs) begin
        ram_dout = intram[ram_addr];
    end else begin
        ram_dout = 8'h00;
    end
end

`ifdef VERBOSE_TRACE
// Debug RAM access around stack area - only log first 100 and critical writes
reg [31:0] stack_write_count;
always @(posedge clk) begin
    if (reset) begin
        stack_write_count <= 0;
    end else if (cen && ram_cs && (cpu_addr >= 16'h00F0) && (cpu_addr <= 16'h00FF)) begin
        if (!cpu_wr) begin  // Write
            if (stack_write_count < 100 || cpu_dout == 8'hFF)
                $display("HC05 RAM[%0d]: WRITE stack 0x%04x = 0x%02x (ram_addr=%d)", cycle_count, cpu_addr, cpu_dout, ram_addr);
            stack_write_count <= stack_write_count + 1;
        end
    end
end
`endif

// PRAM loading and normal RAM writes - single always block to avoid multiple drivers
// PRAM loading: copy to internal RAM when 680x0 reset asserts (PC bit 3: 1->0)
// Per MAME egret.cpp: write_internal_ram(0x70 + byte, data)
// intram[x] corresponds to CPU address 0x90 + x (RAM mapped at 0x90-0x1FF)
// So PRAM goes to intram[0x70-0x16F] = CPU addresses 0x100-0x1FF
//
// ★ SEQUENTIAL boot-copy (ported 2026-08-08 from MacIIvi 9f5d0d3, MLAB
// conversion): the old for-loop wrote all 256 bytes in ONE clk — 256 parallel
// write ports, which forced intram[] (and pram[], read 256-wide) to
// synthesize as registers + giant muxes (~1.5-2k ALMs). Now one byte per clk
// over 260 cycles (256 PRAM + 4 RTC-seed) ≈ 8 µs at clk32. Atomicity vs the
// HC05 is preserved by construction: pram_copy_busy gates the HC05 cen (see
// u_cpu instantiation), so the firmware observes the copy as a single instant
// exactly like the old one-cycle burst — it cannot read a half-copied PRAM
// region or land a store mid-copy. The 68020 is separately held in reset
// until pram_loaded (unchanged).
reg        pram_copy_busy = 1'b0;
reg [8:0]  pram_copy_idx;             // 0-255 = PRAM bytes, 256-259 = RTC seed
wire [7:0] pram_copy_rdata = pram[pram_copy_idx[7:0]];
// Unix epoch (1970) -> Mac epoch (1904): see the seed comment below.
wire [31:0] mac_seconds = timestamp[31:0] + 32'd2082844800;
always @(posedge clk) begin
    if (reset) begin
        pram_loaded    <= 1'b0;
        pram_copy_busy <= 1'b0;
    end else begin
        // Boot-copy the saved PRAM into the Egret's working RAM as the LAST write
        // before the 68k runs. Gate it on the HC05 firmware having RELEASED the 68k
        // (reset_680x0_latched==0) — which is AFTER the firmware's own startup
        // PRAM-clear. Our previous trigger fired on the reset-ASSERT edge (PC3 1->0,
        // pre-clear), so on a fast SD load the firmware's clear then wiped the loaded
        // image and the ROM ran InitUtil every boot (MAME ground truth: it injects
        // PRAM at the post-clear reset-release; see docs/mame_pram_findings.md). Also
        // require pram_ready (the SD image is in pram[], or no image/timeout). The
        // 68020 is held in reset until pram_loaded (see reset_680x0 above), so it
        // never reads pre-copy PRAM.
        if (!pram_loaded && !pram_copy_busy && !reset_680x0_latched && pram_ready) begin
            pram_copy_busy <= 1'b1;
            pram_copy_idx  <= 9'd0;
            `ifdef SIMULATION
            $display("EGRET_PRAM: Loading PRAM and RTC time (post-clear release, sequential)");
            `endif
        end else if (pram_copy_busy) begin
            // Copy PRAM to internal RAM: PRAM[0-255] -> CPU 0x100-0x1FF
            // (offset 0x70 = 0x100 - 0x90), then seed RTC seconds
            // (CPU 0xAB-0xAE -> intram[0x1B-0x1E]) from the host timestamp.
            // ★ mac_seconds: the host gives a UNIX epoch (1970); the Mac RTC
            // counts seconds since 1904-01-01. Without the 2,082,844,800 s
            // offset the guest ran with correct wall time but year 1960 (the
            // 66-year offset is exactly 24,107 days, and 1904->1970 and
            // 1960->2026 contain the same number of leap days, so the error
            // hid in the menu-bar clock and only showed in dates). Wraps in
            // 2040, when the 32-bit Mac epoch ends anyway.
            case (pram_copy_idx)
                9'd256:  intram[16'hAB - 16'h90] <= mac_seconds[31:24];
                9'd257:  intram[16'hAC - 16'h90] <= mac_seconds[23:16];
                9'd258:  intram[16'hAD - 16'h90] <= mac_seconds[15:8];
                9'd259:  intram[16'hAE - 16'h90] <= mac_seconds[7:0];
                default: intram[pram_copy_idx + 9'h070] <= pram_copy_rdata;
            endcase
            if (pram_copy_idx == 9'd259) begin
                pram_copy_busy <= 1'b0;
                pram_loaded    <= 1'b1;
            end else
                pram_copy_idx <= pram_copy_idx + 9'd1;
        end else if (ram_cs && !cpu_wr && cen) begin  // !cpu_wr means write
            intram[ram_addr] <= cpu_dout;
        `ifdef VERBOSE_TRACE
        if (ram_addr == 9'h04) begin
            $display("EGRET_RAM_WRITE[%0d]: PC=%04x addr=$94 data=%02x",
                     cycle_count, last_pc, cpu_dout);
        end
        if (ram_addr == 9'h3C) begin
            $display("EGRET_RAM_WRITE[%0d]: PC=%04x addr=$CC data=%02x",
                     cycle_count, last_pc, cpu_dout);
        end
        if (ram_addr == 9'h13) begin
            $display("EGRET_A3_WRITE[%0d]: PC=%04x addr=$A3 data=%02x (bit7=%b)",
                     cycle_count, last_pc, cpu_dout, cpu_dout[7]);
        end
        `endif
        end   // close: else if (ram_cs ... write)
    end       // close: else (not in reset)
end

// ============================================================================
// PRAM persistence: keep pram[] (the canonical NVRAM image) in sync
// ============================================================================
// pram[] is seeded from egret.pram ($readmemh) and copied into intram by the
// boot-copy block above. AFTER that copy (pram_loaded), we mirror every firmware
// write to the PRAM region (intram 0x70..0x16F = CPU 0x100..0x1FF) into pram[],
// so the top level can snapshot it; the top level can also overwrite pram[] from
// a save file via pram_load_*. ONE always block (load wins over mirror) keeps
// Quartus to a single driver on pram[]. Gating the mirror on pram_loaded means
// the boot-copy still sees exactly the loaded image (boot behavior unchanged).
wire        pram_region_wr = ram_cs && !cpu_wr && cen && pram_loaded
                             && (ram_addr >= 9'h070) && (ram_addr < 9'h170);
wire  [7:0] pram_wr_idx    = ram_addr - 9'h070;   // 0..255 within the region
always @(posedge clk) begin
    if (pram_load_wr)
        pram[pram_load_addr] <= pram_load_data;
    else if (pram_region_wr)
        pram[pram_wr_idx]    <= cpu_dout;
end
assign pram_save_data = pram[pram_save_addr];
assign pram_wr_stb    = pram_region_wr;

`ifdef SIMULATION
// PRAM write-path witness (2026-08-19, the "colors reset every boot" report):
// counts firmware writes that reach the canonical pram[] and checksums the
// array, so a sim boot can prove whether guest/ROM PRAM writes actually land.
integer pw_cnt = 0;
integer pw_i;
reg [15:0] pw_sum;
always @(posedge clk) begin
    if (pram_region_wr) begin
        pw_cnt = pw_cnt + 1;
        if (pw_cnt <= 8 || pw_cnt % 64 == 0) begin
            pw_sum = 0;
            for (pw_i = 0; pw_i < 256; pw_i = pw_i + 1) pw_sum = pw_sum + pram[pw_i];
            $display("PRAM_WITNESS: write #%0d addr=%02x data=%02x pram_sum=%04x @%0t",
                     pw_cnt, pram_wr_idx, cpu_dout, pw_sum, $time);
        end
    end
end
`endif

`ifdef VERBOSE_TRACE
// Debug stack reads around RTS execution
always @(posedge clk) begin
    if (cen && ram_cs && cpu_wr) begin  // cpu_wr=1 means read
        // Log stack reads (addresses 0xF0-0xFF) during the RTS time window
        if (cpu_addr >= 16'h00F0 && cpu_addr <= 16'h00FF) begin
            if (cycle_count >= 279460 && cycle_count <= 279500) begin
                $display("EGRET_STACK_READ[%0d]: addr=0x%04x ram_addr=%d ram_dout=0x%02x intram=0x%02x cpu_din=0x%02x",
                         cycle_count, cpu_addr, ram_addr, ram_dout, intram[ram_addr], cpu_din);
            end
        end
    end
end
`endif

// ============================================================================
// ROM (4KB at 0x0F00-0x1FFF for M68HC05E1)
// ============================================================================
// CRITICAL: Make ROM read combinational (not registered) so data is available same cycle
always @(*) begin
    if (rom_cs) begin
        rom_dout = rom[rom_addr];
    end else begin
        rom_dout = 8'hFF;
    end
end

`ifdef SIMULATION
// Debug ROM reads (commented out - enable if needed for debugging)
/*
always @(posedge clk) begin
    if (rom_cs && cycle_count < 20) begin
        $display("EGRET_ROM_READ[%0d]: addr=%04x rom_addr=%03x data=%02x rom_cs=%b", 
                 cycle_count, cpu_addr, rom_addr, rom[rom_addr], rom_cs);
    end
end
*/
`endif

// ============================================================================
// CPU data input mux
// ============================================================================
reg [7:0] cpu_din_r;
always @(*) begin
    if (port_cs) begin
        case (cpu_addr[4:0])  // 5 bits for 0x00-0x1F
            5'h00: begin
                cpu_din_r = pa_in;
                `ifdef VERBOSE_TRACE
                if (cycle_count < 10000)
                    $display("EGRET[%0d]: PORT A READ = 0x%02x (pa_out=%02x pa_in=%02x pa_ddr=%02x)",
                             cycle_count, cpu_din_r, pa_out, pa_in, pa_ddr);
                `endif
            end
            5'h01: cpu_din_r = pb_in;
            5'h02: cpu_din_r = pc_out;  // Full 8-bit read for port test
            5'h04: cpu_din_r = pa_ddr;
            5'h05: cpu_din_r = pb_ddr;
            5'h06: cpu_din_r = pc_ddr;
            5'h07: begin
                cpu_din_r = pll_ctrl;       // PLL control
                `ifdef VERBOSE_TRACE
                // Log early PLL reads (during init) and later reads (during handshake)
                if (cycle_count <= 10000 || (cycle_count >= 276000 && cycle_count <= 280000))
                    $display("EGRET_PLL_READ[%0d]: PC~%04x pll_ctrl=0x%02x bit6=%b",
                             cycle_count, last_pc, cpu_din_r, cpu_din_r[6]);
                `endif
            end
            5'h08: cpu_din_r = timer_ctrl;     // Timer control
            5'h09: cpu_din_r = timer_counter;  // Timer counter (8-bit, free-running)
            // One-second timer control. Reads must include the FIRED flag in
            // bit 6 (MAME m68hc05e1: seconds_tick does m_onesec |= 0x40, visible
            // on read until the firmware writes bit6=0). Without it, any RMW bit
            // op on $12 (e.g. BSET 5,$12 in the set-time path) reads bit6=0 and
            // writes bit6=0 back — spuriously acking a pending second. The ISR's
            // BCLR 6,$12 still clears the flag through the same write path.
            5'h12: cpu_din_r = onesec_ctrl | (onesec_irq_flag ? 8'h40 : 8'h00);
            default: cpu_din_r = 8'h00;  // Unmapped ports return 0 (makes bit tests fail safely)
        endcase
    end else if (ram_cs) begin
        cpu_din_r = ram_dout;
    end else if (rom_cs) begin
        cpu_din_r = rom_dout;
    end else begin
        // Unmapped space returns NOP (0x9D) instead of 0xFF to prevent runaway execution
        cpu_din_r = 8'h9D;
    end
end

assign cpu_din = cpu_din_r;

// ============================================================================
// IRQ generation - M68HC05E1 has three interrupt sources
// ============================================================================
// Each source has its own vector in the CPU core:
// - onesec_irq_n -> $FFF6 (one-second timer ISR)
// - timer_irq_n  -> $FFF8 (timer/counter ISR)
// - combined_irq_n -> $FFFA (external IRQ ISR)
wire combined_irq_n = 1'b1;  // No external IRQ source currently

// Track TIP for edge detection in debug/display only
reg via_tip_prev;
reg via_byteack_prev;

always @(posedge clk) begin
    if (reset) begin
        via_tip_prev <= 1'b1;  // TIP is idle high
        via_byteack_prev <= 1'b0;
    end else if (cen) begin
        via_tip_prev <= via_tip_stable;
        via_byteack_prev <= via_byteack_in_stable;
    end
end

// ============================================================================
// CPU instantiation - m68hc05_core
// ============================================================================
m68hc05_core u_cpu (
    .clk(clk),
    // cen gated by pram_copy_busy: the HC05 is frozen for the ~8 µs sequential
    // PRAM boot-copy so the copy stays atomic from firmware's point of view —
    // identical observable behavior to the old single-cycle burst copy. The
    // one-second timer keeps counting (8 µs once per boot is noise).
    .cen(cen && !pram_copy_busy),  // 4 MHz clock enable (32 MHz / 8)
    .rst(~reset),      // m68hc05_core uses active-low reset
    .irq(combined_irq_n),     // External IRQ (active-low) -> $FFFA
    .timer_irq(timer_irq_n),  // Timer interrupt (active-low) -> $FFF8
    .onesec_irq(onesec_irq_n),// One-second timer (active-low) -> $FFF6
    .addr(cpu_addr),
    .wr(cpu_wr),
    .datain(cpu_din),
    .state(cpu_state),
    .dataout(cpu_dout)
);

// ============================================================================
// Stub for cuda_sr_irq (not implemented yet)
// ============================================================================
always @(posedge clk) begin
    if (reset)
        cuda_sr_irq <= 1'b0;
    // TODO: Implement SR interrupt logic if needed
end

// ============================================================================
// Debug (simulation only)
// ============================================================================
`ifdef SIMULATION
reg [7:0] pb_out_prev, pb_latch_prev, pb_ddr_prev;
reg [7:0] pa_out_prev;
// via_tip_prev is declared and managed earlier
reg [31:0] cycle_count;
reg [15:0] last_pc;
reg       treq_prev;
reg       reset_680x0_prev;

always @(posedge clk) begin
    if (reset) begin
        cycle_count <= 0;
        pb_out_prev <= 8'hFF;
        pb_latch_prev <= 0;
        pb_ddr_prev <= 0;
        pa_out_prev <= 0;
        last_pc <= 0;
        treq_prev <= 1;  // pb_out[1]=1 means TREQ deasserted initially
        reset_680x0_prev <= 1;
    end else if (cen) begin
        cycle_count <= cycle_count + 1;
        pb_out_prev <= pb_out;
        pa_out_prev <= pa_out;
        treq_prev <= pb_out[1];  // Track pb_out[1] directly
        reset_680x0_prev <= reset_680x0;

        // Log 68020 reset release
        if (reset_680x0 != reset_680x0_prev) begin
            if (reset_680x0)
                $display("EGRET[%0d]: *** 68020 RESET ASSERTED ***", cycle_count);
            else
                $display("EGRET[%0d]: *** 68020 RESET RELEASED (pc_out[3]=%b, pc_ddr[3]=%b) ***",
                         cycle_count, pc_out[3], pc_ddr[3]);
        end

        `ifdef VERBOSE_TRACE
        // Log Port B and C latch/DDR writes
        if (port_cs && !cpu_wr) begin
            case (cpu_addr[4:0])
                5'h00: $display("EGRET[%0d] PC=%04x: Port A LATCH write = 0x%02x (was 0x%02x)",
                              cycle_count, cpu_addr, cpu_dout, pa_latch);
                5'h01: $display("EGRET[%0d] PC=%04x: Port B LATCH write = 0x%02x (was 0x%02x)",
                              cycle_count, cpu_addr, cpu_dout, pb_latch);
                5'h02: $display("EGRET[%0d] PC=%04x: Port C LATCH write = 0x%02x (bit3=%b -> reset_680x0 will be %b)",
                              cycle_count, cpu_addr, cpu_dout, cpu_dout[3], cpu_dout[3]);
                5'h04: $display("EGRET[%0d] PC=%04x: Port A DDR write = 0x%02x",
                              cycle_count, cpu_addr, cpu_dout);
                5'h05: $display("EGRET[%0d] PC=%04x: Port B DDR write = 0x%02x",
                              cycle_count, cpu_addr, cpu_dout);
                5'h06: $display("EGRET[%0d] PC=%04x: Port C DDR write = 0x%02x",
                              cycle_count, cpu_addr, cpu_dout);
                5'h12: $display("EGRET[%0d] PC=%04x: One-second timer write = 0x%02x",
                              cycle_count, cpu_addr, cpu_dout);
                default: $display("EGRET[%0d] PC=%04x: Port write addr=%02x data=%02x",
                              cycle_count, cpu_addr, cpu_addr[4:0], cpu_dout);
            endcase
        end

        // Log Port B accesses (for communication tracking)
        if (port_cs && cpu_addr[4:0] == 5'h01) begin
            $display("EGRET[%0d]: PB %s data=0x%02x (PC=0x%04x) TIP=%b BYTEACK=%b",
                     cycle_count, cpu_wr ? "READ" : "WRITE", cpu_wr ? cpu_din : cpu_dout,
                     last_pc, ~via_tip_stable, ~via_byteack_in_stable);
        end

        // Log Port B output changes
        if (pb_out != pb_out_prev) begin
            $display("EGRET[%0d]: PB OUT 0x%02x->0x%02x (CB1=%b CB2=%b TREQ=%b) TIP_in=%b",
                     cycle_count, pb_out_prev, pb_out,
                     pb_out[4], pb_out[5], pb_out[1], via_tip_stable);
        end
        `endif

        // Log TREQ transitions (pb_out[1]=0 means TREQ active)
`ifdef SIMULATION
        if (pb_out[1] != treq_prev) begin
            if (~pb_out[1])
                $display("EGRET[%0d]: TREQ ACTIVE", cycle_count);
            else
                $display("EGRET[%0d]: TREQ INACTIVE", cycle_count);
        end
`endif

        // Log TIP input changes from VIA
`ifdef SIMULATION
        if (via_tip_stable != via_tip_prev) begin
            $display("EGRET[%0d]: TIP %b->%b (effective=%b)", cycle_count, via_tip_prev, via_tip_stable, via_tip_effective);
        end
`endif

`ifdef SIMULATION
        // Log CB1 clock edges
        if (pb_out[4] != pb_out_prev[4]) begin
            $display("EGRET[%0d]: CB1 %s", cycle_count, pb_out[4] ? "RISE" : "FALL");
        end
`endif

`ifdef SIMULATION
        // Log BYTEACK input changes
        if (via_byteack_in_stable != via_byteack_prev) begin
            $display("EGRET[%0d]: BYTEACK %b->%b", cycle_count, via_byteack_prev, via_byteack_in_stable);
        end
        // Periodic PC dump when TIP is active (every 256 cycles)
        if (!via_tip_effective && cycle_count[7:0] == 8'h00) begin
            $display("EGRET[%0d]: PC=%04x TIP_active PB_in=%02x PB_out=%02x PB_ddr=%02x",
                     cycle_count, last_pc, pb_in, pb_out, pb_ddr);
        end
`endif

        // Track program counter
        if (rom_cs && cpu_wr) begin  // cpu_wr=1 means read
            last_pc <= cpu_addr;
`ifdef EGRET_SRDEBUG
            // Focused stuck-loop finder for the byte-4 BYTEACK/TREQ turnaround.
            // Samples the HC05 PC + handshake inputs every ~32k fetches, so a hung
            // loop prints its PC repeatedly (a few dozen lines/run, no flood).
            // Enable with +define+EGRET_SRDEBUG; grep "EGRET_SRDBG" in the output.
            if (cycle_count[14:0] == 15'd0)
                $display("EGRET_SRDBG[%0d]: PC=0x%04x BYTEACK=%b TIP=%b TREQ_out=%b CB1out=%b",
                         cycle_count, addr13, via_byteack_in_stable, via_tip_stable, pb_out[1], pb_out[4]);
`endif
`ifdef SIMULATION
            // Track key init milestones
            if (addr13 == 13'h0FAF)
                $display("EGRET[%0d]: >>> INIT RESTART ($0FAF)", cycle_count);
            if (addr13 == 13'h12C2)
                $display("EGRET[%0d]: >>> COMM HANDLER ($12C2) TIP=%b BYTEACK=%b", cycle_count, via_tip_effective, via_byteack_in_stable);
            if (addr13 == 13'h132B)
                $display("EGRET[%0d]: >>> SESSION HANDLER ($132B) TIP=%b", cycle_count, via_tip_effective);
            if (addr13 == 13'h14C8)
                $display("EGRET[%0d]: >>> CB1 HANDLER ($14C8) TIP=%b BYTEACK=%b", cycle_count, via_tip_effective, via_byteack_in_stable);
            if (addr13 == 13'h14EC)
                $display("EGRET[%0d]: >>> CB1 STROBING ($14EC)", cycle_count);
            if (addr13 == 13'h1034)
                $display("EGRET[%0d]: >>> PROBE LOOP ($1034)", cycle_count);
            if (addr13 == 13'h1068)
                $display("EGRET[%0d]: >>> PROBE DONE ($1068) — post-probe init", cycle_count);
            if (addr13 == 13'h1445)
                $display("EGRET[%0d]: >>> SESSION XFER ($1445) TIP=%b", cycle_count, via_tip_effective);
            if (addr13 == 13'h1549)
                $display("EGRET[%0d]: >>> WAIT_BYTEACK ($1549) TIP=%b", cycle_count, via_tip_effective);
`endif
            `ifdef VERBOSE_TRACE
            // Key firmware addresses (match MAME trace points)
            if (addr13 == 13'h120A ||  // Init check loop entry
                addr13 == 13'h1210 ||  // BCLR 6, $A3
                addr13 == 13'h1212 ||  // BRSET 6, $07 (PLL check)
                addr13 == 13'h1219 ||  // JSR $1E01
                addr13 == 13'h121C ||  // BCLR 4, $07
                addr13 == 13'h121E ||  // JSR $1E01 (2nd)
                addr13 == 13'h1221 ||  // BRCLR 0, $01
                addr13 == 13'h1224 ||  // BSET 6, $07 (set PLL bit)
                addr13 == 13'h1226 ||  // After PLL check branch
                addr13 == 13'h1228 ||  // "Ready" branch target
                addr13 == 13'h1236 ||  // BSET 7, $A3 (set init flag)
                addr13 == 13'h123B ||  // Error exit
                addr13 == 13'h1246 ||  // Continue init
                addr13 == 13'h1251 ||  // Main loop JSR $120A
                addr13 == 13'h1E01 ||  // PLL wait subroutine
                addr13 == 13'h12A3 ||  // CLI (enable interrupts)
                addr13 == 13'h12AD ||  // TIP polling loop (BRSET 3, $01)
                addr13 == 13'h14C8 ||  // Main message handler (JSR $1149)
                addr13 == 13'h14CD ||  // Main loop TIP check
                addr13 == 13'h1549 ||  // TREQ assertion (BCLR 1, $01)
                addr13 == 13'h1640 ||  // TREQ setup
                (addr13 >= 13'h14EF && addr13 <= 13'h152B))  // CB1 clocking
                $display("EGRET[%0d]: KEY PC=0x%04x TIP=%b TREQ=%b",
                         cycle_count, addr13, ~via_tip_stable, ~pb_out[1]);
            `endif
        end

        `ifdef VERBOSE_TRACE
        // MAME-format Port B read logging
        if (port_cs && cpu_wr && cpu_addr[4:0] == 5'h01) begin
            if (!via_tip_stable || (cycle_count[7:0] == 8'h00 && last_pc >= 16'h12A0 && last_pc <= 16'h12B5)) begin
                $display("EGRET pb_r: %02x TIP=%b BYTEACK=%b (PC=%04x) [ext=%02x]",
                         pb_in, ~via_tip_stable, ~via_byteack_in_stable, last_pc, pb_external);
            end
        end

        // MAME-format Port B write logging
        if (port_cs && !cpu_wr && cpu_addr[4:0] == 5'h01) begin
            $display("EGRET pb_w: %02x CB1=%b CB2=%b TREQ=%b (PC=%04x)",
                     cpu_dout, cpu_dout[4], cpu_dout[5], ~cpu_dout[1], last_pc);
        end

        // Log first 100 CPU cycles
        if (cycle_count < 100) begin
            $display("EGRET_CPU[%0d]: pc=%04x din=%02x dout=%02x",
                     cycle_count, cpu_addr, cpu_din, cpu_dout);
        end
        `endif
    end
end

// Periodic status - only log every ~1M cycles to reduce output
reg [19:0] status_timer;
always @(posedge clk) begin
    if (reset) begin
        status_timer <= 0;
    end else if (cen) begin
        status_timer <= status_timer + 1;
        `ifdef VERBOSE_TRACE
        if (status_timer == 0) begin
            // MAME-style status: show key signals
            $display("EGRET[%0d] STATUS: PC=%04x TIP=%b TREQ=%b CB1=%b",
                     cycle_count, last_pc, ~via_tip_stable, ~pb_out[1], pb_out[4]);
        end
        `endif
    end
end
`endif

endmodule

`default_nettype wire