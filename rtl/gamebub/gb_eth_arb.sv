// ============================================================================
// gb_eth_arb.sv — two-client arbiter for the SDRAM controller's eth port,
// a drop-in for eth_port_arb (rtl/maclc/floppy_sd.sv) without its latency.
//
// Same protocol on both sides: LEVEL request with address/data held by the
// requester until its ack, LEVEL ack that falls after the request drops.
// eth_port_arb registers the grant, then the request (2 clk to pass a
// request on, 1 to release, and an idle cycle between owners). Three of
// them in series between gb_host and the controller cost ~15 clk per 16-bit
// access, too slow for the MCU's fixed-rate file streaming (gb_host.sv). This
// one passes the owner's request through combinationally and holds the
// ownership only while that request or the port's ack is up; client A wins
// when both ask in the same cycle.
// ============================================================================
`default_nettype none

module gb_eth_arb (
	input  wire        clk,
	input  wire        reset,

	// client A (priority)
	input  wire        a_req,
	input  wire        a_we,
	input  wire [23:0] a_addr,
	input  wire [15:0] a_din,
	output wire        a_ack,
	output wire [15:0] a_dout,

	// client B
	input  wire        b_req,
	input  wire        b_we,
	input  wire [23:0] b_addr,
	input  wire [15:0] b_din,
	output wire        b_ack,
	output wire [15:0] b_dout,

	// the port
	output wire        m_req,
	output wire        m_we,
	output wire [23:0] m_addr,
	output wire [15:0] m_din,
	input  wire        m_ack,
	input  wire [15:0] m_dout
);

	reg lock_a = 1'b0, lock_b = 1'b0;   // an access is in progress for A / B

	wire sel_a = lock_a || (!lock_b && a_req);
	wire sel_b = !sel_a && (lock_b || b_req);

	assign m_req  = sel_a ? a_req  : sel_b ? b_req : 1'b0;
	assign m_we   = sel_a ? a_we   : b_we;
	assign m_addr = sel_a ? a_addr : b_addr;
	assign m_din  = sel_a ? a_din  : b_din;
	assign a_ack  = sel_a && m_ack;
	assign b_ack  = sel_b && m_ack;
	assign a_dout = m_dout;
	assign b_dout = m_dout;

	always @(posedge clk) begin
		if (reset) begin
			lock_a <= 1'b0;
			lock_b <= 1'b0;
		end else begin
			lock_a <= sel_a && (a_req || m_ack);
			lock_b <= sel_b && (b_req || m_ack);
		end
	end

endmodule

`default_nettype wire
