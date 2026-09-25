module cpu_bus_decode (
	input  logic        clk_cpu,
	input  logic        reset,

	input  logic [31:0] cpu_addr,
	input  logic [15:0] cpu_dout,
	input  logic        cpu_rd,
	input  logic        cpu_wr,
	input  logic        cpu_uds,
	input  logic        cpu_lds,
	output logic [15:0] cpu_din,
	output logic         cpu_ready,

	output logic [24:1] sdram_addr,
	output logic [15:0] sdram_din,
	output logic         sdram_wrl,
	output logic         sdram_wrh,
	output logic         sdram_req,
	input  logic         sdram_ack,
	input  logic [15:0] sdram_dout,

	output logic [15:1] vram_addr,
	output logic [15:0] vram_din,
	output logic [1:0]  vram_we,
	input  logic [15:0] vram_dout,

	output logic [11:1] oram_addr,
	output logic [15:0] oram_din,
	output logic [1:0]  oram_we,
	input  logic [15:0] oram_dout,

	output logic [18:1] cram_addr,
	output logic [15:0] cram_din,
	output logic [1:0]  cram_we,
	output logic         cram_rd,
	input  logic [15:0] cram_dout,

	output logic [18:1] xga_addr,
	output logic [15:0] xga_din,
	output logic         xga_we,
	output logic         xga_rd,
	input  logic         xga_override,
	input  logic [15:0] xga_dout,

	output logic         cage_data_wr,
	output logic         cage_ctrl_wr,
	output logic [15:0] cage_wdata,
	output logic         cage_data_rd,
	input  logic [15:0] cage_data_rdata,
	input  logic [15:0] cage_ctrl_rdata,
	input  logic         cage_data_rd_ack,
	input  logic         irq3,

	output logic [15:0] latch,
	output logic [2:0]  mo_control,
	output logic [15:0] mo_command,

	input  logic         scan_tick,
	output logic         irq_scanline,
	output logic         irq_vblank,
	output logic [2:0]  ipl,

	input  logic [31:0] p1p2,
	input  logic [15:0] service,
	input  logic [15:0] coin,
	input  logic         vblank,

	input  logic         xga_busy,

	input  logic         clk_nv,
	input  logic [10:0] nv_addr,
	input  logic [7:0]  nv_din,
	input  logic         nv_we,
	output logic [7:0]  nv_q,
	output logic         nv_wr_toggle = 1'b0
);

	localparam [1:0] ST_IDLE   = 2'd0;
	localparam [1:0] ST_SDRAM  = 2'd1;
	localparam [1:0] ST_DELAY  = 2'd2;
	localparam [1:0] ST_DELAY2 = 2'd3;

	logic [1:0] state;
	logic       cage_rd_expect;
	logic       req_toggle_r;
	logic [24:1] sdram_addr_r;
	logic [15:0] sdram_din_r;
	logic        sdram_wrl_r;
	logic        sdram_wrh_r;
	logic        xga_override_r;
	logic [15:0] xga_dout_r;
	logic        vblank_prev;

	// address decode, byte addresses, upper bits assumed zero
	wire [23:0] a = cpu_addr[23:0];
	wire is_rom     = (a < 24'h200000);
	wire is_workram = (a >= 24'hF80000);
	wire is_cage    = (a >= 24'hC00000) && (a < 24'hC00004);
	wire is_analog  = (a >= 24'hD00010) && (a < 24'hD00020);
	wire is_eeprom  = (a >= 24'hD20000) && (a < 24'hD21000);
	wire is_vidram  = (a >= 24'hD70000) && (a < 24'hD80000);
	wire is_oram    = (a >= 24'hD78000) && (a < 24'hD79000);
	wire is_cram    = (a >= 24'hD80000) && (a < 24'hE00000);
	wire is_latch   = (a >= 24'hE08000) && (a < 24'hE08004);
	wire is_scanack = (a >= 24'hE0A000) && (a < 24'hE0A004);
	wire is_vblnack = (a >= 24'hE0C000) && (a < 24'hE0C004);
	wire is_p1p2    = (a >= 24'hE80000) && (a < 24'hE80004);
	wire is_service = (a >= 24'hE82000) && (a < 24'hE82004);
	wire is_coin    = (a >= 24'hE82004) && (a < 24'hE82008);

	wire need_sdram = (is_rom && cpu_rd) || (is_workram && (cpu_rd || cpu_wr));
	wire need_wait  = cpu_rd && (is_cram || is_vidram || is_eeprom || is_cage);
	// 136094-0004A drives the D80000-DFFFFF window itself while busy: every
	// access there, read or write, has to wait for it, not just reads
	wire cram_ok    = !(is_cram && xga_busy);

	wire [24:1] sdram_addr_comb = is_rom ? {4'b0, cpu_addr[20:1]} : (24'h100000 + {6'b0, cpu_addr[18:1]});

	// iverilog cannot index an array with a part-select directly inside
	// an always block, so the index is a plain wire instead
	wire [10:0] eeprom_idx = cpu_addr[11:1];
	wire        addr_lo    = cpu_addr[1];
	wire        cage_rd_wait = is_cage && !addr_lo && cpu_rd;
	wire [15:0] p1p2_hi    = p1p2[31:16];
	wire [15:0] p1p2_lo    = p1p2[15:0];
	// bit 7 is the live VBLANK signal (atarigt.cpp SERVICE port), not a switch
	wire [15:0] service_eff = {service[15:8], vblank, service[6:0]};

	logic [7:0] eeprom_q;

	wire we_eeprom = !reset && state == ST_IDLE && cpu_wr && cram_ok && !need_sdram
						   && !need_wait && is_eeprom && cpu_uds;

	tdp_ram #(.AW(11), .DW(8)) u_eeprom (
		.clk_a(clk_cpu), .addr_a(eeprom_idx), .din_a(cpu_dout[15:8]), .we_a(we_eeprom), .q_a(eeprom_q),
		.clk_b(clk_nv), .addr_b(nv_addr), .din_b(nv_din), .we_b(nv_we), .q_b(nv_q)
	);

`ifdef SIMULATION
	// real EEPROM contents are meaningless here, just avoid X reads of
	// never-written bytes; smaller than a valid-bit shadow array
	initial begin
		for (int i = 0; i < 2048; i++) u_eeprom.mem[i] = 8'h00;
	end
`endif

	// toggles once per accepted EEPROM write, hps_io side samples it to
	// detect a new save is needed; must survive reset or a reset flip fakes one
	always_ff @(posedge clk_cpu)
		if (we_eeprom) nv_wr_toggle <= ~nv_wr_toggle;

	always_comb begin
		case (state)
			ST_IDLE:   cpu_ready = (cpu_rd || cpu_wr) ? (cram_ok && !(need_sdram || need_wait)) : 1'b1;
			ST_SDRAM:  cpu_ready = (sdram_ack == req_toggle_r);
			// cram is byte-split BRAM plus a registered select, valid 2
			// clocks after the address; vram/oram/eeprom are 1 clock;
			// cage reads wait for cage_data_rd_ack to toggle (rtl/cage.sv)
			ST_DELAY:  cpu_ready = is_cram ? 1'b0 : cage_rd_wait ? (cage_data_rd_ack == cage_rd_expect) : cram_ok;
			ST_DELAY2: cpu_ready = cram_ok;
			default:   cpu_ready = 1'b1;
		endcase
	end

	always_comb begin
		cpu_din = 16'h0000;
		if (state == ST_SDRAM) begin
			cpu_din = sdram_dout;
		end else if (state == ST_DELAY2) begin
			// cram only; xga_prot answered a clock ago (right after xga_rd),
			// so its result was latched into xga_override_r/xga_dout_r
			// already, not sampled live here
			cpu_din = xga_override_r ? xga_dout_r : cram_dout;
		end else if (state == ST_DELAY) begin
			if (is_oram)
				cpu_din = oram_dout;
			else if (is_eeprom)
				cpu_din = {eeprom_q, 8'h00};
			else if (is_cage)
				cpu_din = addr_lo ? cage_ctrl_rdata : cage_data_rdata;
			else if (is_vidram)
				cpu_din = vram_dout;
		end else begin
			if (is_analog)
				cpu_din = 16'hFF00;
			else if (is_p1p2)
				cpu_din = addr_lo ? p1p2_lo : p1p2_hi;
			else if (is_service)
				cpu_din = service_eff ^ 16'h0001;
			else if (is_coin)
				cpu_din = coin ^ {14'b0, irq_scanline, irq_vblank};
		end
	end

	assign vram_addr = cpu_addr[15:1];
	assign vram_din  = cpu_dout;
	assign vram_we   = (state == ST_IDLE && cpu_wr && is_vidram && !is_oram) ? {cpu_uds, cpu_lds} : 2'b00;

	assign oram_addr = cpu_addr[11:1];
	assign oram_din  = cpu_dout;
	assign oram_we   = (state == ST_IDLE && cpu_wr && is_oram) ? {cpu_uds, cpu_lds} : 2'b00;

	assign cram_addr = cpu_addr[18:1];
	assign cram_din  = cpu_dout;
	assign cram_we   = (state == ST_IDLE && cpu_wr && is_cram && cram_ok) ? {cpu_uds, cpu_lds} : 2'b00;
	assign cram_rd   = (state == ST_IDLE && cpu_rd && is_cram && cram_ok);

	assign xga_addr = cpu_addr[18:1];
	assign xga_din  = cpu_dout;
	assign xga_we   = (state == ST_IDLE && cpu_wr && is_cram && cram_ok);
	assign xga_rd   = cram_rd;

	assign cage_wdata   = cpu_dout;
	assign cage_data_wr = (state == ST_IDLE && cpu_wr && is_cage && !addr_lo);
	assign cage_ctrl_wr = (state == ST_IDLE && cpu_wr && is_cage && addr_lo);
	// control_r has no side effect in MAME cage.cpp, only main_r does
	assign cage_data_rd = (state == ST_IDLE && cpu_rd && is_cage && !addr_lo);

	assign sdram_addr = sdram_addr_r;
	assign sdram_din  = sdram_din_r;
	assign sdram_wrl  = sdram_wrl_r;
	assign sdram_wrh  = sdram_wrh_r;
	assign sdram_req  = req_toggle_r;

	assign mo_control = {latch[13], latch[12], latch[11]};

	always_comb begin
		if (irq_scanline)      ipl = 3'b001; // ~6
		else if (irq_vblank)   ipl = 3'b011; // ~4
		else if (irq3)         ipl = 3'b100; // ~3
		else                   ipl = 3'b111;
	end

	always_ff @(posedge clk_cpu) begin
		if (reset) begin
			state        <= ST_IDLE;
			cage_rd_expect <= 1'b0;
			// sdram.sv's ack0 has no reset of its own: start the toggle
			// matched to whatever it happens to hold, not a fixed 0
			req_toggle_r <= sdram_ack;
			sdram_addr_r <= 24'h0;
			sdram_din_r  <= 16'h0000;
			sdram_wrl_r  <= 1'b0;
			sdram_wrh_r  <= 1'b0;
			xga_override_r <= 1'b0;
			xga_dout_r      <= 16'h0000;
			vblank_prev  <= 1'b0;
			latch        <= 16'h0000;
			mo_command   <= 16'h0000;
			irq_scanline <= 1'b0;
			irq_vblank   <= 1'b0;
		end else begin
			case (state)
				ST_IDLE: begin
					if ((cpu_rd || cpu_wr) && cram_ok) begin
						if (need_sdram) begin
							state        <= ST_SDRAM;
							req_toggle_r <= ~req_toggle_r;
							sdram_addr_r <= sdram_addr_comb;
							sdram_din_r  <= cpu_dout;
							sdram_wrl_r  <= cpu_wr && cpu_lds && !is_rom;
							sdram_wrh_r  <= cpu_wr && cpu_uds && !is_rom;
						end else if (need_wait) begin
							state <= ST_DELAY;
							if (is_cage && !addr_lo) cage_rd_expect <= ~cage_data_rd_ack;
						end
					end
				end
				ST_SDRAM: if (cpu_ready) state <= ST_IDLE;
				ST_DELAY: begin
					if (is_cram) begin
						if (cram_ok) begin
							// xga_prot's answer is valid this cycle (one
							// clock after xga_rd); latch it for ST_DELAY2,
							// where cram_dout has only just become valid
							xga_override_r <= xga_override;
							xga_dout_r     <= xga_dout;
							state          <= ST_DELAY2;
						end
					end else if (cpu_ready) begin
						state <= ST_IDLE;
					end
				end
				ST_DELAY2: if (cram_ok) state <= ST_IDLE;
				default:  state <= ST_IDLE;
			endcase

			if (state == ST_IDLE && cpu_wr && cram_ok && !need_sdram && !need_wait) begin
				if (is_latch && !addr_lo) begin
					// MAME latch_w: ACCESSING_BITS_24_31 (uds, MO control)
					// and ACCESSING_BITS_16_23 (lds, cage/coin) are masked
					// separately, so an lds-only write must not touch the
					// MO control bits that live in the uds byte
					if (cpu_uds) latch[15:8] <= cpu_dout[15:8];
					if (cpu_lds) latch[7:0]  <= cpu_dout[7:0];
				end
				if (is_scanack)
					irq_scanline <= 1'b0;
				if (is_vblnack)
					irq_vblank <= 1'b0;
				if (is_vidram && !is_oram && (a == 24'hD7A202)) begin
					// mo_command_w: COMBINE_DATA, byte lanes independent
					if (cpu_uds) mo_command[15:8] <= cpu_dout[15:8];
					if (cpu_lds) mo_command[7:0]  <= cpu_dout[7:0];
				end
			end

			if (scan_tick)
				irq_scanline <= 1'b1;
			// clk_sys vblank_rise pulses are never fed across to clk_cpu:
			// edge-detect the level here instead
			vblank_prev <= vblank;
			if (vblank && !vblank_prev)
				irq_vblank <= 1'b1;
		end
	end

endmodule

module cpu_bus (
	input  logic clk_cpu,
	input  logic reset,
	input  logic cpu_reset,

	output logic [24:1] sdram_addr,
	output logic [15:0] sdram_din,
	output logic         sdram_wrl,
	output logic         sdram_wrh,
	output logic         sdram_req,
	input  logic         sdram_ack,
	input  logic [15:0] sdram_dout,

	output logic [15:1] vram_addr,
	output logic [15:0] vram_din,
	output logic [1:0]  vram_we,
	input  logic [15:0] vram_dout,

	output logic [11:1] oram_addr,
	output logic [15:0] oram_din,
	output logic [1:0]  oram_we,
	input  logic [15:0] oram_dout,

	output logic [18:1] cram_addr,
	output logic [15:0] cram_din,
	output logic [1:0]  cram_we,
	output logic         cram_rd,
	input  logic [15:0] cram_dout,

	output logic [18:1] xga_addr,
	output logic [15:0] xga_din,
	output logic         xga_we,
	output logic         xga_rd,
	input  logic         xga_override,
	input  logic [15:0] xga_dout,

	output logic         cage_data_wr,
	output logic         cage_ctrl_wr,
	output logic [15:0] cage_wdata,
	output logic         cage_data_rd,
	input  logic [15:0] cage_data_rdata,
	input  logic [15:0] cage_ctrl_rdata,
	input  logic         cage_data_rd_ack,
	input  logic         irq3,

	output logic [15:0] latch,
	output logic [2:0]  mo_control,
	output logic [15:0] mo_command,

	input  logic         scan_tick,
	output logic         irq_scanline,
	output logic         irq_vblank,

	input  logic [31:0] p1p2,
	input  logic [15:0] service,
	input  logic [15:0] coin,
	input  logic         vblank,

	input  logic         xga_busy,

	input  logic         clk_nv,
	input  logic [10:0] nv_addr,
	input  logic [7:0]  nv_din,
	input  logic         nv_we,
	output logic [7:0]  nv_q,
	output logic         nv_wr_toggle
);

	wire [31:0] cpu_addr;
	wire [15:0] cpu_din;
	wire [15:0] cpu_dout;
	wire        nwr, nuds, nlds;
	wire [1:0]  busstate;
	wire        cpu_ready;
	wire [2:0]  ipl;

	wire cpu_rd = (busstate == 2'b00) || (busstate == 2'b10);
	wire cpu_wr = (busstate == 2'b11);

	// clkena must stay low for the whole access, including its first
	// clock, or the kernel samples data_in before cpu_ready ever went high
	wire clkena = cpu_ready;

	cpu_bus_decode decode (
		.clk_cpu(clk_cpu), .reset(reset),
		.cpu_addr(cpu_addr), .cpu_dout(cpu_dout),
		.cpu_rd(cpu_rd), .cpu_wr(cpu_wr),
		.cpu_uds(~nuds), .cpu_lds(~nlds),
		.cpu_din(cpu_din), .cpu_ready(cpu_ready),
		.sdram_addr(sdram_addr), .sdram_din(sdram_din),
		.sdram_wrl(sdram_wrl), .sdram_wrh(sdram_wrh),
		.sdram_req(sdram_req), .sdram_ack(sdram_ack), .sdram_dout(sdram_dout),
		.vram_addr(vram_addr), .vram_din(vram_din), .vram_we(vram_we), .vram_dout(vram_dout),
		.oram_addr(oram_addr), .oram_din(oram_din), .oram_we(oram_we), .oram_dout(oram_dout),
		.cram_addr(cram_addr), .cram_din(cram_din), .cram_we(cram_we), .cram_rd(cram_rd), .cram_dout(cram_dout),
		.xga_addr(xga_addr), .xga_din(xga_din), .xga_we(xga_we), .xga_rd(xga_rd),
		.xga_override(xga_override), .xga_dout(xga_dout),
		.cage_data_wr(cage_data_wr), .cage_ctrl_wr(cage_ctrl_wr), .cage_wdata(cage_wdata),
		.cage_data_rd(cage_data_rd),
		.cage_data_rdata(cage_data_rdata), .cage_ctrl_rdata(cage_ctrl_rdata),
		.cage_data_rd_ack(cage_data_rd_ack), .irq3(irq3),
		.latch(latch), .mo_control(mo_control), .mo_command(mo_command),
		.scan_tick(scan_tick),
		.irq_scanline(irq_scanline), .irq_vblank(irq_vblank), .ipl(ipl),
		.p1p2(p1p2), .service(service), .coin(coin),
		.vblank(vblank), .xga_busy(xga_busy),
		.clk_nv(clk_nv), .nv_addr(nv_addr), .nv_din(nv_din), .nv_we(nv_we),
		.nv_q(nv_q), .nv_wr_toggle(nv_wr_toggle)
	);

	TG68KdotC_Kernel #(.sr_read(2), .vbr_stackframe(2), .extaddr_mode(2), .mul_mode(2), .div_mode(2), .bitfield(2))
	cpu_inst (
		.clk(clk_cpu),
		.nreset(~(reset || cpu_reset)),
		.clkena_in(clkena),
		.data_in(cpu_din),
		.ipl(ipl),
		.ipl_autovector(1'b1),
		.berr(1'b0),
		.cpu(2'b11),
		.addr_out(cpu_addr),
		.data_write(cpu_dout),
		.nwr(nwr),
		.nuds(nuds),
		.nlds(nlds),
		.busstate(busstate),
		.longword(),
		.nresetout(),
		.regin_out(),
		.cacr_out(),
		.d_cache_out(),
		.vbr_out()
	);

endmodule
