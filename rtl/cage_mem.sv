// C31 (CAGE DSP) memory subsystem: address decode, unified DDR3 cache and
// the boot table walker, ahead of the CPU.
// Address map: MAME cage.cpp.
// Follow-up (not here): a second CPU-side word port for the parallel
// instruction forms; one port is enough until the C31 core exists.

module cage_mem (
	input               clk,
	input               reset,

	// CPU-side word port, 24-bit word address, one req/ack per access.
	// req/we/addr/wdata may be held for as many cycles as the caller needs
	// (a fresh request is not required to appear as a pulse), but req_id
	// must be flipped to a value different from the one used for the
	// previous access before/while presenting a new access -- including a
	// new access to the same address. A dispatch is recognised the cycle
	// req is asserted with an req_id that differs from the last one this
	// module actually serviced (tracked internally), which is exactly once
	// per access regardless of how long the caller continues to hold req
	// afterward: no dead cycle is required between one access's ack and
	// the next access's dispatch, even back to back.
	input               req,
	input               req_id,
	input               we,
	input        [23:0] addr,
	input        [31:0] wdata,
	output logic [31:0] rdata,
	output logic        ack,
	// low during the post-reset cache clear and during the boot walk;
	// req is not accepted (and must be held) while this is low
	output logic        ready,

	// peripheral register bus, 0x808000-0x8080FF: cage_mem is the bus
	// front end, an external block (not written yet) models timers/DMA
	// by driving periph_rdata for the address on periph_addr
	output logic        periph_we,
	output logic [7:0]  periph_addr,
	output logic [31:0] periph_wdata,
	input        [31:0] periph_rdata,

	// CAGE host latch, word address 0xA00000 (MAME cage.cpp cage_from_main_r/
	// cage_to_main_w): re/we are one-clock pulses,
	// exactly one per access even if req is held
	output logic        host_re,
	output logic        host_we,
	output logic [15:0] host_wdata,
	input        [15:0] host_rdata,

	// boot table walker
	input               boot_start,
	output logic        boot_done,
	output logic [23:0] entry_pc,

	// DDR3 master, held-request semantics as rtl/rle_objects.sv: address/
	// control/data held until a cycle where ddr_busy is low, then accepted;
	// never issue a new request while one is outstanding
	output logic [28:0] ddr_addr,
	output logic        ddr_rd,
	output logic        ddr_we,
	output logic [7:0]  ddr_burstcnt,
	output logic [63:0] ddr_din,
	output logic [7:0]  ddr_be,
	input        [63:0] ddr_dout,
	input               ddr_dout_ready,
	input               ddr_busy
);

	// ------------------------------------------------------------------
	// address map
	// ------------------------------------------------------------------
	localparam [23:0] INTRAM_LO  = 24'h809800, INTRAM_HI  = 24'h809FFF;
	localparam [23:0] PERIPH_LO  = 24'h808000, PERIPH_HI  = 24'h8080FF;
	localparam [23:0] CAGERAM_LO = 24'h000000, CAGERAM_HI = 24'h00FFFF;
	localparam [23:0] BOOTROM_LO = 24'h400000, BOOTROM_HI = 24'h47FFFF;
	localparam [23:0] HOSTLATCH_ADDR = 24'hA00000;
	// Only the populated part of the 0xC00000-0xFFFFFF sound ROM bank
	// (MAME cage.cpp: "the DSP sees the pair at 0xD00000-0xDFFFFF ... the
	// rest of that window mirrors the empty part of the region"); the rest
	// falls through to the unmapped default of 0.
	localparam [23:0] SOUNDROM_LO = 24'hD00000, SOUNDROM_HI = 24'hDFFFFF;

	// DDR3 byte bases as qword (8 byte)
	// addresses for the ddr_addr bus, matching rle_objects.sv's convention
	localparam [28:0] BOOTROM_QBASE = 29'h06A20000; // 0x35100000 >> 3

	// same bases as 32-bit word addresses for the cache engine
	localparam [27:0] CAGERAM_WBASE = 28'hD400000; // 0x35000000 >> 2
	localparam [27:0] SOUNDROM_WBASE = 28'hD800000; // 0x36000000 >> 2

	// ------------------------------------------------------------------
	// internal port: the boot walker drives it exclusively while active,
	// the external CPU port otherwise (the real C31 stays in reset during
	// boot, so there is never contention in practice)
	// ------------------------------------------------------------------
	logic        boot_active;
	logic        bw_req, bw_we, bw_ack;
	logic [23:0] bw_addr;
	logic [31:0] bw_wdata, bw_rdata;
	// the boot walker's own req_id: toggles once per completed access, same
	// rule as the external port (see bw_ack toggling always_ff below)
	logic        bw_id;

	wire        p_req    = boot_active ? bw_req   : req;
	wire        p_req_id = boot_active ? bw_id    : req_id;
	wire        p_we     = boot_active ? bw_we    : we;
	wire [23:0] p_addr   = boot_active ? bw_addr  : addr;
	wire [31:0] p_wdata  = boot_active ? bw_wdata : wdata;
	logic       p_ack;
	logic [31:0] p_rdata;
	logic hit_now, p_ack_c;
	logic [31:0] p_rdata_c;

	assign bw_ack = boot_active ? p_ack_c : 1'b0;
	assign bw_rdata = p_rdata_c;
	assign ack   = boot_active ? 1'b0 : p_ack_c;
	assign rdata = p_rdata_c;

	always_ff @(posedge clk) begin
		if (reset) boot_active <= 1'b0;
		else if (boot_start) boot_active <= 1'b1;
		else if (boot_done) boot_active <= 1'b0;
	end

	always_ff @(posedge clk) begin
		if (reset) bw_id <= 1'b0;
		else if (bw_ack) bw_id <= ~bw_id;
	end

	wire ip_is_intram   = p_addr >= INTRAM_LO   && p_addr <= INTRAM_HI;
	wire ip_is_periph   = p_addr >= PERIPH_LO   && p_addr <= PERIPH_HI;
	wire ip_is_host     = p_addr == HOSTLATCH_ADDR;
	wire ip_is_cageram  = p_addr >= CAGERAM_LO  && p_addr <= CAGERAM_HI;
	wire ip_is_bootrom  = p_addr >= BOOTROM_LO  && p_addr <= BOOTROM_HI;
	wire ip_is_soundrom = p_addr >= SOUNDROM_LO && p_addr <= SOUNDROM_HI;
	wire ip_is_cached   = ip_is_cageram || ip_is_soundrom;
	// MAME cage set_mcbl_mode: the C31 boot ROM overlays 0x000000-0x000FFF and
	// trap n loads its vector from word n there, which holds 0x809FC0+n
	wire ip_is_vec      = !p_we && p_addr[23:6] == 18'h0 && p_addr[5:0] != 6'h0;
	// sound ROM and boot ROM are read-only (MAME cage.cpp 3.1): a write to
	// either is simply dropped rather than allocating/dirtying a line
	wire ip_read_only_write = p_we && (ip_is_soundrom || ip_is_bootrom);

	// ------------------------------------------------------------------
	// internal RAM, 2K x 32, mandatory BRAM
	// ------------------------------------------------------------------
	(* ramstyle = "M10K" *) logic [31:0] intram [0:2047];
	initial for (int i = 0; i < 2048; i++) intram[i] = 32'h0;
	logic [31:0] intram_q;
	wire  [10:0] intram_idx = p_addr[10:0];
	always_ff @(posedge clk) intram_q <= intram[intram_idx];

	// ------------------------------------------------------------------
	// top dispatch FSM: accepts one request at a time, exactly once. Since
	// req may be held for many cycles (or even stay high back to back into
	// a genuinely new access), "exactly once" is enforced by req_id: a
	// dispatch is only recognised when req_id differs from served_id, the
	// id of the access this module last actually serviced (served_valid
	// guards the very first access after reset, when there is no "last"
	// id to compare against yet). This replaces the older req-must-drop-
	// before-the-next-dispatch rule, so a new access can be dispatched the
	// very cycle after the previous one's ack, no dead cycle required.
	// ------------------------------------------------------------------
	typedef enum logic [2:0] {
		T_IDLE, T_INTRAM, T_PERIPH, T_HOST, T_CACHE, T_BOOTROM, T_OTHER, T_VEC
	} top_state_t;
	top_state_t tstate;

	logic cache_ack;
	logic [31:0] cache_rdata;

	logic served_valid, served_id;

	logic br_req, br_ack;
	logic [23:0] br_addr;
	logic [31:0] br_rdata;

	logic clearing;
	assign ready = !clearing && !boot_active;

	typedef enum logic [3:0] {
		CC_CLEAR, CC_IDLE, CC_LOOKUP, CC_HIT,
		CC_WB, CC_FILL_REQ, CC_FILL_WAIT, CC_FILL_DONE
	} cache_state_t;
	cache_state_t cstate;

	// combinational: a cached request is being newly dispatched this very
	// cycle (T_IDLE), so the cache's own CC_IDLE branch can react the same
	// cycle instead of waiting a registered cache_req pulse a cycle later
	wire cache_dispatch = (tstate == T_IDLE) && p_req && !clearing && ip_is_cached && !ip_read_only_write && !ip_is_vec &&
						  (!served_valid || p_req_id != served_id);


	// ------------------------------------------------------------------
	// unified cache: 64K words, direct mapped, 8-word (32 byte) lines,
	// write back with a dirty bit. Tags in BRAM: 8192 lines x (12 tag +
	// valid + dirty). A hit acks combinationally one clock after the request
	// (array read address applied in the dispatch cycle, compare and ack in
	// CC_LOOKUP). A miss writes back a dirty victim as 4 single-qword
	// writes, then fills as one burst-of-4 read, matching the DDR3 access
	// patterns already used by rle_objects.sv.
	// ------------------------------------------------------------------
	localparam int TAG_W = 12;
	// one M10K array per word-offset-within-line (as rtl/rle_objects.sv
	// does for its qword lanes): a fill beat lands two words from one
	// qword in the same cycle, which needs two different arrays since
	// each array here has only one write port
	(* ramstyle = "M10K" *) logic [31:0] cdata0 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata1 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata2 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata3 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata4 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata5 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata6 [0:8191];
	(* ramstyle = "M10K" *) logic [31:0] cdata7 [0:8191];
	(* ramstyle = "M10K" *) logic [TAG_W+1:0] ctag [0:8191]; // {dirty,valid,tag}

	logic [31:0] cdata_q [0:7];
	logic [TAG_W+1:0] ctag_q;

	function automatic [27:0] to_cword(input [23:0] a, input is_sound);
		if (is_sound) to_cword = SOUNDROM_WBASE + {4'h0, a - SOUNDROM_LO};
		else          to_cword = CAGERAM_WBASE  + {4'h0, a};
	endfunction

	wire [27:0] c_cword = to_cword(p_addr, p_addr >= SOUNDROM_LO);
	wire [TAG_W-1:0] c_tag   = c_cword[27:16];
	wire [12:0]      c_index = c_cword[15:3];
	wire [2:0]       c_off   = c_cword[2:0];

	logic [12:0] clear_idx;

	logic        r_we;
	logic [31:0] r_wdata;
	logic [TAG_W-1:0] r_tag;
	logic [12:0]      r_index;
	logic [2:0]       r_off;
	logic [2:0]       wb_beat, fill_beat;
	logic [31:0]      fill_buf [0:7]; // shadow of the line being filled,
									   // read back same cycle it lands so
									   // a read miss need not wait a BRAM
									   // read-after-write latency

	// array read address: c_index (comb, from the incoming request) while
	// still in CC_IDLE, so the array's own one-cycle registered read lands
	// exactly on the CC_IDLE -> CC_LOOKUP transition (the 1-clock hit
	// latency); r_index otherwise, so cdata_q keeps returning this line's
	// words while CC_WB/CC_FILL run (used for the writeback beats)
	wire [12:0] look_addr = (cstate == CC_IDLE && cache_dispatch) ? c_index : r_index;
	always_ff @(posedge clk) cdata_q[0] <= cdata0[look_addr];
	always_ff @(posedge clk) cdata_q[1] <= cdata1[look_addr];
	always_ff @(posedge clk) cdata_q[2] <= cdata2[look_addr];
	always_ff @(posedge clk) cdata_q[3] <= cdata3[look_addr];
	always_ff @(posedge clk) cdata_q[4] <= cdata4[look_addr];
	always_ff @(posedge clk) cdata_q[5] <= cdata5[look_addr];
	always_ff @(posedge clk) cdata_q[6] <= cdata6[look_addr];
	always_ff @(posedge clk) cdata_q[7] <= cdata7[look_addr];
	always_ff @(posedge clk) ctag_q     <= ctag[look_addr];

	wire hit = ctag_q[TAG_W] && (ctag_q[TAG_W-1:0] == r_tag);
	assign hit_now = (tstate == T_CACHE) && (cstate == CC_LOOKUP) && hit;
	assign p_ack_c = p_ack || (tstate == T_INTRAM) || (tstate == T_VEC) || hit_now;
	// rdata is only sampled with ack, so the data select skips the tag compare
	assign p_rdata_c = (tstate == T_INTRAM) ? intram_q :
					   (tstate == T_CACHE && cstate == CC_LOOKUP) ? (r_we ? r_wdata : cdata_q[r_off]) : p_rdata;
	wire [28:0] line_ddr_addr = {2'b00, ctag_q[TAG_W-1:0], r_index, 2'b00};
	wire [28:0] new_line_ddr_addr = {2'b00, r_tag, r_index, 2'b00};

	// per-lane write, one process per array (one write port each): a hit
	// write touches one lane (CC_LOOKUP), a fill beat touches the two
	// lanes of that qword (CC_FILL_WAIT), a write-miss applies its word
	// once the new line has landed (CC_FILL_DONE); exactly one of the
	// three can be true for a given lane on a given cycle, since they are
	// different cstate values
	wire hitwr  = (cstate == CC_LOOKUP)    && hit && r_we;
	wire fillw  = (cstate == CC_FILL_WAIT) && ddr_dout_ready;
	wire donewr = (cstate == CC_FILL_DONE) && r_we;
	always_ff @(posedge clk) begin
		if ((hitwr && r_off == 3'd0) || (fillw && fill_beat == 3'd0) || (donewr && r_off == 3'd0))
			cdata0[r_index] <= fillw ? ddr_dout[31:0] : r_wdata;
		if ((hitwr && r_off == 3'd1) || (fillw && fill_beat == 3'd0) || (donewr && r_off == 3'd1))
			cdata1[r_index] <= fillw ? ddr_dout[63:32] : r_wdata;
		if ((hitwr && r_off == 3'd2) || (fillw && fill_beat == 3'd1) || (donewr && r_off == 3'd2))
			cdata2[r_index] <= fillw ? ddr_dout[31:0] : r_wdata;
		if ((hitwr && r_off == 3'd3) || (fillw && fill_beat == 3'd1) || (donewr && r_off == 3'd3))
			cdata3[r_index] <= fillw ? ddr_dout[63:32] : r_wdata;
		if ((hitwr && r_off == 3'd4) || (fillw && fill_beat == 3'd2) || (donewr && r_off == 3'd4))
			cdata4[r_index] <= fillw ? ddr_dout[31:0] : r_wdata;
		if ((hitwr && r_off == 3'd5) || (fillw && fill_beat == 3'd2) || (donewr && r_off == 3'd5))
			cdata5[r_index] <= fillw ? ddr_dout[63:32] : r_wdata;
		if ((hitwr && r_off == 3'd6) || (fillw && fill_beat == 3'd3) || (donewr && r_off == 3'd6))
			cdata6[r_index] <= fillw ? ddr_dout[31:0] : r_wdata;
		if ((hitwr && r_off == 3'd7) || (fillw && fill_beat == 3'd3) || (donewr && r_off == 3'd7))
			cdata7[r_index] <= fillw ? ddr_dout[63:32] : r_wdata;
	end

	always_ff @(posedge clk) begin
		cache_ack <= 1'b0;
		case (cstate)
			// reset clears every valid bit before the cache accepts requests;
			// ready stays low for this whole stretch (top dispatch FSM)
			CC_CLEAR: begin
				ctag[clear_idx] <= '0;
				if (clear_idx == 13'd8191) begin
					cstate   <= CC_IDLE;
					clearing <= 1'b0;
				end else clear_idx <= clear_idx + 13'd1;
			end

			CC_IDLE: begin
				if (cache_dispatch) begin
					r_we    <= p_we;
					r_wdata <= p_wdata;
					r_tag   <= c_tag;
					r_index <= c_index;
					r_off   <= c_off;
					cstate  <= CC_LOOKUP;
				end
			end

			CC_LOOKUP: begin
				if (hit) begin
					if (r_we) begin
						// word write itself happens in the hitwr always_ff above
						if (!ctag_q[TAG_W+1]) ctag[r_index] <= {1'b1, ctag_q[TAG_W:0]};
						cache_rdata <= r_wdata;
					end else begin
						cache_rdata <= cdata_q[r_off];
					end
					cache_ack <= 1'b1;
					cstate <= CC_IDLE;
				end else if (ctag_q[TAG_W] && ctag_q[TAG_W+1]) begin
					wb_beat <= 3'd0;
					cstate  <= CC_WB;
				end else begin
					cstate <= CC_FILL_REQ;
				end
			end

			// write back the dirty victim line, one qword (two words) per beat
			CC_WB: begin
				if (!ddr_busy) begin
					if (wb_beat == 3'd3) cstate <= CC_FILL_REQ;
					else wb_beat <= wb_beat + 3'd1;
				end
			end

			CC_FILL_REQ: begin
				if (!ddr_busy) begin
					fill_beat <= 3'd0;
					cstate <= CC_FILL_WAIT;
				end
			end

			CC_FILL_WAIT: begin
				if (ddr_dout_ready) begin
					// per-lane array writes happen in the fillw always_ff above
					fill_buf[{fill_beat[1:0], 1'b0}] <= ddr_dout[31:0];
					fill_buf[{fill_beat[1:0], 1'b1}] <= ddr_dout[63:32];
					if (fill_beat == 3'd3) cstate <= CC_FILL_DONE;
					else fill_beat <= fill_beat + 3'd1;
				end
			end

			CC_FILL_DONE: begin
				ctag[r_index] <= {r_we, 1'b1, r_tag};
				if (r_we) begin
					// word write itself happens in the donewr always_ff above
					cache_rdata <= r_wdata;
				end else begin
					cache_rdata <= fill_buf[r_off];
				end
				cache_ack <= 1'b1;
				cstate <= CC_IDLE;
			end

			default: cstate <= CC_IDLE;
		endcase
		if (reset) begin
			cstate    <= CC_CLEAR;
			clear_idx <= 13'd0;
			clearing  <= 1'b1;
			cache_ack <= 1'b0;
		end
	end

	// ------------------------------------------------------------------
	always_ff @(posedge clk) begin
		br_req    <= 1'b0;
		p_ack     <= 1'b0;
		periph_we <= 1'b0;
		host_we   <= 1'b0;
		host_re   <= 1'b0;
		case (tstate)
			// gated on !clearing only (not the full `ready`, which also
			// reports boot_active): during boot, p_req is the boot
			// walker's own request and must still be served, while an
			// external req is already excluded from p_req by the mux above.
			// served_id gate: see served_valid/served_id declaration above.
			T_IDLE: if (p_req && !clearing && (!served_valid || p_req_id != served_id)) begin
				served_valid <= 1'b1;
				served_id    <= p_req_id;
				if (ip_is_intram) begin
					if (p_we) intram[intram_idx] <= p_wdata;
					tstate <= T_INTRAM;
				end else if (ip_is_periph) begin
					periph_addr <= p_addr[7:0];
					if (p_we) begin
						periph_we    <= 1'b1;
						periph_wdata <= p_wdata;
					end
					tstate <= T_PERIPH;
				end else if (ip_is_host) begin
					if (p_we) begin
						host_we    <= 1'b1;
						host_wdata <= p_wdata[15:0];
					end else begin
						host_re <= 1'b1;
					end
					tstate <= T_HOST;
				end else if (ip_read_only_write) begin
					tstate <= T_OTHER; // ack the write and drop it
				end else if (ip_is_vec) begin
					p_rdata <= {8'h0, 24'h809FC0 | {18'h0, p_addr[5:0]}};
					tstate  <= T_VEC;
				end else if (ip_is_cached) begin
					// cache_dispatch (comb, see below) kicks the cache's own
					// CC_IDLE->CC_LOOKUP transition this same cycle
					tstate <= T_CACHE;
				end else if (ip_is_bootrom) begin
					br_req  <= 1'b1;
					br_addr <= p_addr;
					tstate  <= T_BOOTROM;
				end else tstate <= T_OTHER;
			end

			T_INTRAM, T_VEC: begin
				tstate  <= T_IDLE;
			end

			T_PERIPH: begin
				p_rdata <= periph_rdata;
				p_ack   <= 1'b1;
				tstate  <= T_IDLE;
			end

			T_HOST: begin
				p_rdata <= {16'h0, host_rdata};
				p_ack   <= 1'b1;
				tstate  <= T_IDLE;
			end

			// a hit acks combinationally in the CC_LOOKUP cycle (see
			// hit_now/p_ack_c above); a miss falls through to cache_ack.
			T_CACHE: if (cstate == CC_LOOKUP) begin
				if (hit) tstate <= T_IDLE;
			end else if (cache_ack) begin
				p_rdata <= cache_rdata;
				p_ack   <= 1'b1;
				tstate  <= T_IDLE;
			end

			T_BOOTROM: if (br_ack) begin
				p_rdata <= br_rdata;
				p_ack   <= 1'b1;
				tstate  <= T_IDLE;
			end

			T_OTHER: begin
				p_rdata <= 32'h0;
				p_ack   <= 1'b1;
				tstate  <= T_IDLE;
			end

			default: tstate <= T_IDLE;
		endcase
		// boot_done: the boot walker and the external port share one
		// served_id namespace but never run at the same time -- without
		// this, the external side's first req_id can coincidentally equal
		// whatever id the boot walker last used and be silently ignored
		// forever. Treat the handoff as a fresh start, same as reset.
		if (boot_done) served_valid <= 1'b0;
		if (reset) begin
			tstate       <= T_IDLE;
			p_ack        <= 1'b0;
			served_valid <= 1'b0;
		end
	end

	// boot ROM direct path: uncached, byte-wide (MAME cage.cpp 3.1: one
	// file byte per DSP word, stored packed as 512KB in DDR3)
	// ------------------------------------------------------------------
	typedef enum logic [1:0] { BR_IDLE, BR_REQ, BR_WAIT } br_state_t;
	br_state_t br_state;
	logic [28:0] br_ddr_addr;
	logic [2:0]  br_lane;

	always_ff @(posedge clk) begin
		br_ack <= 1'b0;
		case (br_state)
			BR_IDLE: if (br_req) begin
				br_ddr_addr <= BOOTROM_QBASE + 29'((br_addr - BOOTROM_LO) >> 3);
				br_lane     <= br_addr[2:0];
				br_state    <= BR_REQ;
			end
			BR_REQ: if (!ddr_busy) br_state <= BR_WAIT;
			BR_WAIT: if (ddr_dout_ready) begin
				br_rdata <= {24'h0, ddr_dout[br_lane*8 +: 8]};
				br_ack   <= 1'b1;
				br_state <= BR_IDLE;
			end
			default: br_state <= BR_IDLE;
		endcase
		if (reset) begin
			br_state <= BR_IDLE;
			br_ack   <= 1'b0;
		end
	end

	// cache DDR3 request bus, driven from the states above
	always_comb begin
		ddr_addr     = 29'h0;
		ddr_rd       = 1'b0;
		ddr_we       = 1'b0;
		ddr_burstcnt = 8'h0;
		ddr_din      = 64'h0;
		ddr_be       = 8'h0;
		case (cstate)
			CC_WB: begin
				ddr_addr     = line_ddr_addr + {26'h0, wb_beat};
				ddr_we       = 1'b1;
				ddr_burstcnt = 8'd1;
				ddr_din      = {cdata_q[{2'(wb_beat), 1'b1}], cdata_q[{2'(wb_beat), 1'b0}]};
				ddr_be       = 8'hFF;
			end
			CC_FILL_REQ, CC_FILL_WAIT: begin
				ddr_addr     = new_line_ddr_addr;
				ddr_rd       = (cstate == CC_FILL_REQ);
				ddr_burstcnt = 8'd4;
			end
			default: if (br_state != BR_IDLE) begin
				ddr_addr     = br_ddr_addr;
				ddr_rd       = (br_state == BR_REQ);
				ddr_burstcnt = 8'd1;
			end
		endcase
	end

	// ------------------------------------------------------------------
	// boot table walker (TI c31boot format): width byte, bus control
	// word, then {count, destination, count words} blocks little-endian
	// assembled from the byte-wide ROM, terminated by count == 0
	// ------------------------------------------------------------------
	// BW_RD_BYTE and BW_WRITE hold bw_req asserted every cycle they are
	// active (not just a one-shot pulse) and only leave once bw_ack
	// arrives: the top dispatch FSM's ready gate can defer acceptance
	// (post-reset cache clear), and a one-shot pulse issued into a busy
	// dispatcher would be silently missed and hang forever
	typedef enum logic [3:0] {
		BW_IDLE, BW_SKIP_HDR, BW_RD_BYTE,
		BW_COUNT_DONE, BW_DEST_DONE, BW_DATA_DONE, BW_WRITE, BW_DONE
	} bw_state_t;
	bw_state_t bw_state;

	logic [23:0] rom_ptr;      // next boot ROM word address to read
	logic [2:0]  byte_cnt;     // bytes assembled so far into field_acc, 0..3
	logic [31:0] field_acc;
	logic [1:0]  field_kind;   // 0=skip(width/busctl), 1=count, 2=dest, 3=data
	logic [31:0] blk_count, blk_i;
	logic [23:0] blk_dest;
	logic [1:0]  hdr_left;     // 2 header fields to skip (width, busctl)
	logic        first_block;  // entry_pc is the first block's destination

	always_ff @(posedge clk) begin
		boot_done <= 1'b0;
		case (bw_state)
			BW_IDLE: if (boot_start) begin
				rom_ptr   <= BOOTROM_LO;
				hdr_left  <= 2'd2;
				byte_cnt  <= 3'd0;
				field_acc <= 32'h0;
				field_kind <= 2'd0;
				first_block <= 1'b1;
				bw_state  <= BW_RD_BYTE;
			end

			BW_RD_BYTE: begin
				bw_req  <= 1'b1;
				bw_we   <= 1'b0;
				bw_addr <= rom_ptr;
				if (bw_ack) begin
					bw_req    <= 1'b0;
					field_acc <= {bw_rdata[7:0], field_acc[31:8]};
					rom_ptr   <= rom_ptr + 24'd1;
					if (byte_cnt == 3'd3) begin
						byte_cnt <= 3'd0;
						case (field_kind)
							2'd0: bw_state <= BW_SKIP_HDR;
							2'd1: bw_state <= BW_COUNT_DONE;
							2'd2: bw_state <= BW_DEST_DONE;
							default: bw_state <= BW_DATA_DONE;
						endcase
					end else begin
						byte_cnt <= byte_cnt + 3'd1;
					end
				end
			end

			BW_SKIP_HDR: begin
				if (hdr_left == 2'd1) begin
					hdr_left <= 2'd0;
					field_kind <= 2'd1; // next field: block count
				end else begin
					hdr_left <= hdr_left - 2'd1;
				end
				bw_state <= BW_RD_BYTE;
			end

			// field_acc: successive bytes shift in at the top and the
			// earlier ones ride down, so after 4 bytes the first byte read
			// (the file's low-order byte) ends up in bits [7:0] -- already
			// the correct little-endian 32-bit assembly, no reversal needed.
			// A zero count is the terminator (TI boot loader,
			// c31boot.asm): the walk stops right here, with no
			// destination field following it and the terminator itself
			// not counted as a block.
			BW_COUNT_DONE: begin
				blk_count <= field_acc;
				if (field_acc == 32'h0) bw_state <= BW_DONE;
				else begin
					field_kind <= 2'd2;
					bw_state <= BW_RD_BYTE;
				end
			end

			// entry_pc is the first block's destination address itself
			// (the first block sets the entry point), not
			// whatever data word later gets written there
			BW_DEST_DONE: begin
				blk_dest <= field_acc;
				if (first_block) begin
					entry_pc    <= field_acc[23:0];
					first_block <= 1'b0;
				end
				blk_i <= 32'h0;
				field_kind <= 2'd3;
				bw_state <= BW_RD_BYTE;
			end

			BW_DATA_DONE: begin
				bw_state <= BW_WRITE;
			end

			BW_WRITE: begin
				bw_req   <= 1'b1;
				bw_we    <= 1'b1;
				bw_addr  <= blk_dest[23:0] + blk_i[23:0];
				bw_wdata <= field_acc;
				if (bw_ack) begin
					bw_req <= 1'b0;
					if (blk_i == blk_count - 32'd1) begin
						field_kind <= 2'd1; // next block header, count field
					end else begin
						blk_i <= blk_i + 32'd1;
					end
					bw_state <= BW_RD_BYTE;
				end
			end

			BW_DONE: begin
				boot_done <= 1'b1;
				bw_state  <= BW_IDLE;
			end

			default: bw_state <= BW_IDLE;
		endcase
		if (reset) begin
			bw_state  <= BW_IDLE;
			bw_req    <= 1'b0;
			boot_done <= 1'b0;
		end
	end

endmodule
