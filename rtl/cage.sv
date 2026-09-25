// CAGE sound board: C31 core + memory subsystem + peripherals + host
// interface wired together. Everything here
// runs on clk_sys (85.9 MHz); the host port arrives from cpu_bus on
// clk_cpu (42.955 MHz) and is crossed with toggle synchronizers.
module cage
(
	input         clk_sys,
	input         reset,        // clk_sys-domain system reset
	input         rom_download, // hold CAGE in reset during the ROM load
	input         prescan_done, // unused: kept for port compatibility

	// host port, clk_cpu domain, from cpu_bus
	input         clk_cpu,
	input         cage_data_wr,
	input         cage_ctrl_wr,
	input         cage_data_rd,
	input  [15:0] cage_wdata,
	output [15:0] cage_data_rdata,
	output [15:0] cage_ctrl_rdata,
	output        cage_data_rd_ack,
	output [15:0] dsp_we_count,
	output        irq3,

	// DDR3 master, arbiter's cage port
	output [28:0] ddr_addr,
	output        ddr_rd,
	output        ddr_we,
	output  [7:0] ddr_burstcnt,
	output [63:0] ddr_din,
	output  [7:0] ddr_be,
	input  [63:0] ddr_dout,
	input         ddr_dout_ready,
	input         ddr_busy,

	output signed [15:0] audio_l,
	output signed [15:0] audio_r
);

// held while the DSP must stay in reset: host control bits both zero (via
// cage_host's own dsp_hold_reset) or during the ROM download. CAGE is an
// independent sound board and does not depend on the video object prescan;
// gating on prescan_done left it stuck in reset forever whenever the video
// prescan is skipped (sim's --no-prescan host workaround never pulses it).
wire dsp_hold_reset;
wire cage_reset = reset | rom_download | dsp_hold_reset;

// ---------------------------------------------------------------------
// boot / run sequencing: on release of cage_reset, run the boot walk,
// then release the C31 core with pc preloaded from entry_pc
// ---------------------------------------------------------------------
localparam CG_HELD = 2'd0, CG_BOOT = 2'd1, CG_RUN = 2'd2;
reg [1:0] cg_state;
reg       c31_reset;
reg       boot_start;

wire       boot_done;
wire [23:0] entry_pc;

always @(posedge clk_sys) begin
	if (cage_reset) begin
		cg_state   <= CG_HELD;
		c31_reset  <= 1'b1;
		boot_start <= 1'b0;
	end else begin
		boot_start <= 1'b0;
		case (cg_state)
			CG_HELD: begin
				boot_start <= 1'b1;
				cg_state   <= CG_BOOT;
			end
			CG_BOOT: begin
				if (boot_done) begin
					c31_reset <= 1'b0;
					cg_state  <= CG_RUN;
				end
			end
			CG_RUN: ; // stays here until cage_reset
			default: cg_state <= CG_HELD;
		endcase
	end
end

wire c31_run = (cg_state == CG_RUN);

// ---------------------------------------------------------------------
// ack and rdata are forwarded straight from cm_ack/cm_rdata, no register:
// the requester captures data into its own register on the ack cycle,
// and ar_state falls back to AR_IDLE on that same clock edge.
// ---------------------------------------------------------------------
wire        c31_mem_req, c31_mem_we;
wire [23:0] c31_mem_addr;
wire [31:0] c31_mem_wdata;
wire [31:0] c31_mem_rdata;
wire        c31_mem_ack;

localparam AR_IDLE = 1'd0, AR_BUSY = 1'd1;
reg ar_state;
reg ar_is_cpu; // which requester the in-flight grant belongs to
reg held_id;   // this grant's req_id, latched at AR_IDLE->AR_BUSY

wire        cm_req_i, cm_we_i, cm_ack, cm_ready;
wire        cm_req_id_i;
wire [23:0] cm_addr_i;
wire [31:0] cm_wdata_i, cm_rdata;

wire        dma_req;
wire [23:0] dma_addr;
wire [31:0] dma_rdata;
wire        dma_ack;

// safety net for the rare collision (arbiter mid-DMA-grant the exact
// cycle mem_req pulses): remembers only "a grant is owed", since
// mem_addr/we/wdata are already stable registers on the core (held
// unchanged until it sees mem_ack) and need no latching of their own.
// The common case (arbiter idle) never sets this and is granted the
// same cycle via the raw c31_mem_req term below.
reg c31_pending;
wire ar_idle_ready  = (ar_state == AR_IDLE) && cm_ready;
wire c31_req_live   = c31_mem_req || c31_pending;
// DMA now wins the AR_IDLE tie (see ar_grant_cpu below), so the core's
// request is only actually granted this cycle when DMA isn't also asking
wire c31_granted_now = ar_idle_ready && !dma_req;

always @(posedge clk_sys) begin
	if (c31_reset) begin
		c31_pending <= 1'b0;
	end else begin
		if (c31_mem_req && !c31_granted_now) c31_pending <= 1'b1;
		else if (c31_pending && c31_granted_now) c31_pending <= 1'b0;
	end
end

// AR_IDLE: a pending requester is forwarded into cage_mem combinationally,
// the same cycle it arrives; DMA now has priority (a late DMA word is
// audible, a one-access stall of the core is harmless -- the core just
// re-requests via c31_pending above). AR_BUSY: hold the grant until cm_ack.
wire ar_grant_cpu = (ar_state == AR_IDLE) ? (c31_req_live && !dma_req) : ar_is_cpu;

assign cm_req_i    = (ar_state == AR_IDLE) ? (c31_req_live || dma_req) : 1'b1;
assign cm_req_id_i = (ar_state == AR_IDLE) ? ~held_id : held_id;
assign cm_we_i     = ar_grant_cpu ? c31_mem_we : 1'b0;
assign cm_addr_i   = ar_grant_cpu ? c31_mem_addr : dma_addr;
assign cm_wdata_i  = c31_mem_wdata;

assign c31_mem_ack   = (ar_state == AR_BUSY) && ar_is_cpu && cm_ack;
assign c31_mem_rdata = cm_rdata;
assign dma_ack       = (ar_state == AR_BUSY) && !ar_is_cpu && cm_ack;
assign dma_rdata     = cm_rdata;

always @(posedge clk_sys) begin
	// cage_reset, not reset: a DSP reset mid-access would leave a stale grant that
	// cage_mem re-serves after the boot walk, shifting every later ack by one
	if (cage_reset) begin
		ar_state <= AR_IDLE;
		held_id  <= 1'b0;
	end else begin
		case (ar_state)
			AR_IDLE: if (cm_req_i && cm_ready) begin
				ar_is_cpu <= ar_grant_cpu;
				held_id   <= ~held_id;
				ar_state  <= AR_BUSY;
			end
			AR_BUSY: if (cm_ack) ar_state <= AR_IDLE;
			default: ar_state <= AR_IDLE;
		endcase
	end
end

// ---------------------------------------------------------------------
// peripheral register bus (cage_mem -> cage_periph)
// ---------------------------------------------------------------------
wire        periph_we;
wire [7:0]  periph_addr;
wire [31:0] periph_wdata;
wire [31:0] periph_rdata;

wire tint0, tint1, dint, xint, rint;
wire int0_n, int1_n;
wire [7:0] iof_bits;

wire        dsp_re, dsp_we;
wire [15:0] dsp_wdata;
wire [15:0] dsp_rdata;

// counts host-latch writes for the board probe; resets like cage_host
// itself (not cage_reset), so a game write of control=0 does not clear it
reg [15:0] dsp_we_cnt;
always @(posedge clk_sys) if (reset | rom_download) dsp_we_cnt <= 16'd0; else if (dsp_we) dsp_we_cnt <= dsp_we_cnt + 16'd1;
assign dsp_we_count = dsp_we_cnt;

cage_mem cage_mem
(
	.clk(clk_sys),
	// cage_mem runs the boot table walk itself during CG_BOOT, while
	// c31_reset is still held on the C31 core; resetting cage_mem on
	// c31_reset too would hold its own boot walker in reset the whole
	// time, so boot_done could never fire. Use cage_reset (clears at the
	// start of CG_BOOT) instead.
	.reset(cage_reset),

	.req(cm_req_i),
	.req_id(cm_req_id_i),
	.we(cm_we_i),
	.addr(cm_addr_i),
	.wdata(cm_wdata_i),
	.rdata(cm_rdata),
	.ack(cm_ack),
	.ready(cm_ready),

	.periph_we(periph_we),
	.periph_addr(periph_addr),
	.periph_wdata(periph_wdata),
	.periph_rdata(periph_rdata),

	.host_re(dsp_re),
	.host_we(dsp_we),
	.host_wdata(dsp_wdata),
	.host_rdata(dsp_rdata),

	.boot_start(boot_start),
	.boot_done(boot_done),
	.entry_pc(entry_pc),

	.ddr_addr(ddr_addr),
	.ddr_rd(ddr_rd),
	.ddr_we(ddr_we),
	.ddr_burstcnt(ddr_burstcnt),
	.ddr_din(ddr_din),
	.ddr_be(ddr_be),
	.ddr_dout(ddr_dout),
	.ddr_dout_ready(ddr_dout_ready),
	.ddr_busy(ddr_busy)
);

cage_periph cage_periph
(
	.clk(clk_sys),
	.reset(c31_reset),
	.host_dsp_reset(dsp_hold_reset),

	.periph_we(periph_we),
	.periph_addr(periph_addr),
	.periph_wdata(periph_wdata),
	.periph_rdata(periph_rdata),

	.tint0(tint0),
	.tint1(tint1),
	.dint(dint),
	.xint(xint),
	.rint(rint),

	.dma_req(dma_req),
	.dma_addr(dma_addr),
	.dma_rdata(dma_rdata),
	.dma_ack(dma_ack),

	.audio_l(audio_l),
	.audio_r(audio_r),
	.audio_valid()
);

c31_core c31_core
(
	.clk(clk_sys),
	.reset(c31_reset),
	.run(c31_run),
	.boot_pc(entry_pc), // the core loads pc under reset, so the registered copy arrived one clock late

	.mem_req(c31_mem_req),
	.mem_we(c31_mem_we),
	.mem_addr(c31_mem_addr),
	.mem_wdata(c31_mem_wdata),
	.mem_rdata(c31_mem_rdata),
	.mem_ack(c31_mem_ack),

	.int_n({2'b11, int1_n, int0_n}),
	.xint(xint),
	.rint(rint),
	.tint0(tint0),
	.tint1(tint1),
	.dint(dint),

	// bits 3/7 (cpu_to_cage_ready / cage_to_cpu_ready) read back from
	// cage_host's own latches instead of the core's IOF register bits
	.iof_in(iof_bits),
	.iof_in_mask(8'h88),

	.insn_done(),
	.pc(),

	.r0(), .r1(), .r2(), .r3(), .r4(), .r5(), .r6(), .r7(),
	.ar0(), .ar1(), .ar2(), .ar3(), .ar4(), .ar5(), .ar6(), .ar7(),
	.ir0(), .ir1(), .bk(), .sp(), .st(), .ie(), .if_(), .iof(),
	.rs(), .re(), .rc(), .dp()
);

// ---------------------------------------------------------------------
// host port: 3 request pulses cross clk_cpu -> clk_sys via toggle
// synchronizers; the two read-data buses are plain clk_sys registers
// read directly from clk_cpu (static once settled, per the coordinator).
// cage_data_rdata is valid once cage_data_rd_ack toggles.
// ---------------------------------------------------------------------
wire data_wr_p, ctrl_wr_p, data_rd_p;

cdc_pulse cdc_data_wr (.src_clk(clk_cpu), .src_pulse(cage_data_wr), .dst_clk(clk_sys), .dst_pulse(data_wr_p));
cdc_pulse cdc_ctrl_wr (.src_clk(clk_cpu), .src_pulse(cage_ctrl_wr), .dst_clk(clk_sys), .dst_pulse(ctrl_wr_p));
cdc_pulse cdc_data_rd (.src_clk(clk_cpu), .src_pulse(cage_data_rd), .dst_clk(clk_sys), .dst_pulse(data_rd_p));

// cage_wdata is a clk_cpu-domain bus; it is stable for the whole pulse
// (held by cpu_bus across its own req/ack cycle) so no extra sync needed
reg  [15:0] host_wdata_r;
always @(posedge clk_sys) host_wdata_r <= cage_wdata;

wire        host_we       = data_wr_p | ctrl_wr_p;
wire        host_re       = data_rd_p;
wire        host_data_sel = data_wr_p | data_rd_p; // 0 (control) otherwise
wire [15:0] host_rdata;

reg [15:0] ctrl_rdata_r = 16'h0000, data_rdata_r = 16'h0000;
always @(posedge clk_sys) begin
	if (!host_data_sel) ctrl_rdata_r <= host_rdata;
	if (data_rd_p)       data_rdata_r <= host_rdata;
end

reg rd_ack_t;
always @(posedge clk_sys) if (reset) rd_ack_t <= 1'b0; else if (data_rd_p) rd_ack_t <= ~rd_ack_t;
assign cage_data_rd_ack = rd_ack_t;

assign cage_ctrl_rdata = ctrl_rdata_r;
assign cage_data_rdata = data_rdata_r;

cage_host cage_host
(
	.clk(clk_sys),
	// cage_host drives dsp_hold_reset, the very thing that feeds c31_reset:
	// resetting it on c31_reset would wipe the host's control_reg write
	// (the one meant to release the DSP) every cycle c31_reset is held,
	// deadlocking CAGE forever. It only needs to clear on the real reset.
	.reset(reset | rom_download), // rom_download also carries the OSD DSP-off bit: keep the host latch quiet like build 25

	.host_we(host_we),
	.host_re(host_re),
	.host_data_sel(host_data_sel),
	.host_wdata(host_wdata_r),
	.host_rdata(host_rdata),

	.irq3(irq3),

	// dsp_re/dsp_we come from the C31's own 0xA00000 window, decoded in
	// cage_mem (MAME cage.cpp 0xA00000 rw)
	.dsp_re(dsp_re),
	.dsp_we(dsp_we),
	.dsp_wdata(dsp_wdata),
	.dsp_rdata(dsp_rdata),

	.int0_n(int0_n),
	.int1_n(int1_n),
	.iof_bits(iof_bits),

	.dsp_hold_reset(dsp_hold_reset),
	// latch bit21 (MAME reset_w) stays unwired: Primal Rage drops it only
	// once at boot, while control_reg still holds the DSP in reset
	.ext_reset(1'b0)
);

endmodule


// ---------------------------------------------------------------------
// cdc_pulse: toggle-based single-pulse clock domain crossing. src_pulse
// need only be one src_clk cycle wide; dst_pulse comes out one dst_clk
// cycle wide, some cycles later.
// ---------------------------------------------------------------------
module cdc_pulse
(
	input  src_clk,
	input  src_pulse,
	input  dst_clk,
	output dst_pulse
);

reg src_toggle = 1'b0;
always @(posedge src_clk) if (src_pulse) src_toggle <= ~src_toggle;

reg [2:0] dst_sync = 3'b000;
always @(posedge dst_clk) dst_sync <= {dst_sync[1:0], src_toggle};

assign dst_pulse = dst_sync[2] ^ dst_sync[1];

endmodule
