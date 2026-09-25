// Reference: MAME atarirle.cpp, modesc masks in MAME atarigt.cpp.
// ROM_WORDS lets a testbench shrink the object ROM for fast simulation.

module rle_objects #(
	parameter integer ROM_WORDS = 32'd16777216
) (
	input               clk,
	input               clk_cpu, // CPU-domain clock for the oram tdp_ram CPU port (clk_sys/2, phase aligned)
	input               reset,

	input        [2:0]  control,
	input        [15:0] mo_command,
	input               vblank_rise,

	// CPU port to object RAM (D78000, 2K x 16)
	input        [11:1] oram_addr,
	input        [15:0] oram_din,
	input        [1:0]  oram_we,
	output       [15:0] oram_dout,

	// frame buffer read port to the mixer
	input        [8:0]  fb_rd_x,
	input        [7:0]  fb_rd_y,
	input               fb_rd_frame,
	output       [15:0] fb_rd_data,
	input               hblank,
	input               vblank,
	input        [7:0]  next_line,

	// DDR3 read port: object ROM and width/height table readback
	output logic [28:0] ddr_addr,
	output logic        ddr_rd,
	output logic [7:0]  ddr_burstcnt,
	input        [63:0] ddr_dout,
	input               ddr_dout_ready,
	input               ddr_busy,

	// DDR3 write port: width/height table only
	output logic        ddr_we,
	output logic [63:0] ddr_din,
	output logic [7:0]  ddr_be,

	input               prescan_start,
	output              prescan_done
);

	localparam [28:0] ROM_WORD_BASE   = 29'h0600_0000; // byte 0x30000000 >> 3
	localparam [28:0] TABLE_WORD_BASE = 29'h0640_0000; // byte 0x32000000 >> 3
	localparam integer FB_W = 336;
	localparam integer FB_H = 240;

	// ------------------------------------------------------------------
	// build_rle_tables as pure functions of the byte (no ROM storage)
	// ------------------------------------------------------------------
	function automatic [15:0] rle_lookup(input [7:0] b, input [2:0] which);
		logic [15:0] v4, v5, v6;
		v4 = ((({8'h0, b} & 16'h00f0) + 16'h0010) << 4) | {12'h0, b[3:0]};
		v5 = ((({8'h0, b} & 16'h00e0) + 16'h0020) << 3) | {11'h0, b[4:0]};
		v6 = ((({8'h0, b} & 16'h00c0) + 16'h0040) << 2) | {10'h0, b[5:0]};
		case (which)
			3'd0:       rle_lookup = v4;
			3'd1:       rle_lookup = (b[3:0] == 4'h0) ? v4 : v5;
			3'd2, 3'd3: rle_lookup = v5;
			3'd4, 3'd6: rle_lookup = (b[3:0] == 4'h0) ? v4 : v6;
			default:    rle_lookup = v6; // which 5, 7
		endcase
	endfunction

	// ------------------------------------------------------------------
	// object RAM: 2K x 16, CPU port + internal port
	// ------------------------------------------------------------------
	// two byte-wide true dual-port M10Ks (rtl/tdp_ram.sv): an inferred
	// array with two read ports duplicates under Quartus 17, so the
	// altsyncram BIDIR_DUAL_PORT primitive is instantiated directly
	logic [10:0] oram_i_addr;
	logic        oram_i_we;
	logic [15:0] oram_i_din;
	logic [15:0] oram_dout_r;
	logic [15:0] oram_i_dout_r;

	tdp_ram #(.AW(11), .DW(8)) oram_lo (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(oram_addr),   .din_a(oram_din[7:0]),  .we_a(oram_we[0]), .q_a(oram_dout_r[7:0]),
		.addr_b(oram_i_addr), .din_b(oram_i_din[7:0]), .we_b(oram_i_we), .q_b(oram_i_dout_r[7:0])
	);
	tdp_ram #(.AW(11), .DW(8)) oram_hi (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(oram_addr),   .din_a(oram_din[15:8]),  .we_a(oram_we[1]), .q_a(oram_dout_r[15:8]),
		.addr_b(oram_i_addr), .din_b(oram_i_din[15:8]), .we_b(oram_i_we), .q_b(oram_i_dout_r[15:8])
	);
	assign oram_dout = oram_dout_r;

	// ------------------------------------------------------------------
	// single DDR3 owner: whichever issuer's request is actually granted
	// and accepted keeps exclusive use of the port (address, and for a
	// read, ddr_dout/ddr_dout_ready) until that issuer's data has fully
	// arrived; a write releases it again the instant its request is
	// accepted, since it has no data phase.
	//
	// The request itself is registered and held the MiSTer/Avalon way: on
	// a grant, ddr_addr/rd/we/burstcnt/din/be are latched into req_* and
	// asserted; they stay asserted, unchanged, until a cycle where
	// ddr_busy is observed low while the request is still asserted (that
	// cycle is the acceptance, not the grant) and only then is ddr_owner
	// set (for reads) and the request dropped. A grant is never given
	// while a request is still pending, so at most one request is ever
	// latched at a time, and the grant itself never depends on ddr_busy
	// (busy may be high for reasons unrelated to this module). Every
	// issuer below waits for req_accepted with its own req_owner before
	// advancing, not merely for the grant, so nobody assumes a request
	// went out before the port actually took it.
	// ------------------------------------------------------------------
	typedef enum logic [2:0] {OWN_NONE, OWN_PREFETCH, OWN_ROM, OWN_WHTAB, OWN_TABWR, OWN_FLUSH} ddr_owner_t;
	ddr_owner_t ddr_owner;
	ddr_owner_t ddr_grant;

	logic        req_pending;
	ddr_owner_t  req_owner;
	wire         req_accepted = req_pending && !ddr_busy;

	// ------------------------------------------------------------------
	// main state machine (declared early: the frame buffer and prefetch
	// logic above reference state/S_FB_FLUSH_RD)
	// ------------------------------------------------------------------
	typedef enum logic [5:0] {
		S_IDLE,

		S_ROM_FETCH, S_ROM_FETCH_WAIT, S_ROM_FETCH_SEL, S_ROM_FETCH_DONE,

		S_PS0_CLEAR,
		S_PS1_GOT,

		S_PS2_NEXT_OBJ, S_PS2_HDR2, S_PS2_HDR3,
		S_PS2_ROWCNT, S_PS2_ROWCNT2, S_PS2_ENTRY, S_PS2_ROW_ADV, S_PS2_TABWR,

		S_CKSUM_READ0, S_CKSUM_GOT0, S_CKSUM_LOOP, S_CKSUM_WR,

		S_DRAW_ERASE,
		S_DRAW_CACHE_ORDER, S_DRAW_CACHE_WAIT, S_DRAW_CACHE_GOT, S_DRAW_SORT_RD,
		S_DRAW_SORT,
		S_DRAW_FIELD, S_DRAW_FIELD_GOT,
		S_DRAW_ROMHDR_GOT,
		S_DRAW_WHTAB_REQ, S_DRAW_WHTAB_WAIT,
		S_DRAW_COMPUTE1, S_DRAW_COMPUTE1B, S_DRAW_COMPUTE2, S_DRAW_COMPUTE2B,
		S_DRAW_DIVSETUP, S_DRAW_DIV_DX_RUN, S_DRAW_DIV_DX_DONE,
		S_DRAW_DIV_DY_RUN, S_DRAW_DIV_DY_DONE,
		S_DRAW_CLIP, S_DRAW_CLIP2,
		S_FB_ROW_OPEN, S_FB_ROW_OPEN2,
		S_FB_FLUSH_SCAN, S_FB_FLUSH_RD, S_FB_FLUSH_QW,
		S_DRAW_ROW_ADV_CHECK, S_DRAW_ROW_ADV_GOT,
		S_DRAW_ROW_HDR_GOT,
		S_DRAW_ROW_ENTRY_GOT,
		S_DRAW_PIXEL_LO, S_DRAW_PIXEL_HI,
		S_DRAW_ROW_NEXT,
		S_DRAW_OBJ_NEXT
	} state_t;

	state_t state, return_state;

	// fb flush qword cursor and lane read registers: declared early, used
	// by both the frame buffer section below and the draw FSM's flush states
	logic [6:0]  flush_qw_idx;
	logic [15:0] flush_p0_r, flush_p1_r, flush_p2_r, flush_p3_r;

	// ------------------------------------------------------------------
	// motion object frame buffers now live in DDR3 (byte base 0x34000000
	// buffer 0, 0x34100000 buffer 1; row stride 512 px = 1024 bytes so the
	// row base is a shift, not the y*336 multiply the BRAM version had).
	// Only the 336 visible pixels (84 qwords) of each 128-qword row are
	// ever touched; qwords 84..127 are allocated by the stride but never
	// written or read (smaller than the design note's literal
	// 512-wide buffer, same DDR layout, less BRAM).
	//
	// Write side: one row is assembled in BRAM at a time (writes inside an
	// object row are monotonic in x, S_DRAW_PIXEL_LO/HI), with a per-pixel
	// dirty vector. A row-open (new object, or S_DRAW_ROW_NEXT's y change)
	// flushes the previous row if dirty, then opens the new one. There is
	// no zero-fill step: the flush itself sends 0 for any lane that is not
	// dirty (background), and real drawn data otherwise, so the assembly
	// array never needs clearing and is only ever written from the pixel
	// write process. On a not-yet-valid row every qword flushes (be=0xff,
	// no read-modify-write, stale DDR3 content never shows); on an
	// already-valid row only dirty qwords flush, byte-enabled.
	//
	// Erase: a 240-bit row_valid vector per buffer, cleared entirely at
	// MOGO instead of writing 80640 zeros; an unfetched/invalid row reads
	// as zero on the read side without touching DDR3.
	//
	// Both the write-side assembly buffer and the two read-side ping-pong
	// line buffers are split into 4 lane arrays of 84 x 16 (lane = x[1:0],
	// address = x[8:2]): a plain single write port, single read port array
	// each, instead of one wide array needing 4 simultaneous addresses.
	// ------------------------------------------------------------------
	localparam integer FB_ROW_QW   = 84;          // 336 px / 4 px per qword
	localparam [28:0]  FB0_QW_BASE = 29'h0680_0000; // byte 0x34000000 >> 3
	localparam [28:0]  FB1_QW_BASE = 29'h0682_0000; // byte 0x34100000 >> 3

	(* ramstyle = "M10K" *) logic [15:0] asm_lane0 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] asm_lane1 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] asm_lane2 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] asm_lane3 [0:FB_ROW_QW-1];
	logic [FB_W-1:0] px_dirty; // per pixel: a partially written qword must
								// only flush the lanes actually drawn
	logic        asm_open;
	logic        asm_buf;
	logic [7:0]  asm_row;
	logic        asm_first_touch; // row was not row_valid when opened

	logic [FB_H-1:0] row_valid0, row_valid1;

	// pixel write request from the draw pipeline
	logic        fb_wr_en;
	logic [8:0]  fb_wr_x;
	logic [15:0] fb_wr_data;
	logic        fb_wr_buf;
	wire  [6:0]  fb_wr_qw   = fb_wr_x[8:2];
	wire  [1:0]  fb_wr_lane = fb_wr_x[1:0];

	always_ff @(posedge clk) if (fb_wr_en && fb_wr_lane == 2'd0) asm_lane0[fb_wr_qw] <= fb_wr_data;
	always_ff @(posedge clk) if (fb_wr_en && fb_wr_lane == 2'd1) asm_lane1[fb_wr_qw] <= fb_wr_data;
	always_ff @(posedge clk) if (fb_wr_en && fb_wr_lane == 2'd2) asm_lane2[fb_wr_qw] <= fb_wr_data;
	always_ff @(posedge clk) if (fb_wr_en && fb_wr_lane == 2'd3) asm_lane3[fb_wr_qw] <= fb_wr_data;

	// plain registered reads for the flush (S_FB_FLUSH_RD), one array each,
	// each the whole body of its own always block so Quartus infers M10K
	always_ff @(posedge clk) if (state == S_FB_FLUSH_RD) flush_p0_r <= asm_lane0[flush_qw_idx];
	always_ff @(posedge clk) if (state == S_FB_FLUSH_RD) flush_p1_r <= asm_lane1[flush_qw_idx];
	always_ff @(posedge clk) if (state == S_FB_FLUSH_RD) flush_p2_r <= asm_lane2[flush_qw_idx];
	always_ff @(posedge clk) if (state == S_FB_FLUSH_RD) flush_p3_r <= asm_lane3[flush_qw_idx];

	// px_dirty is set by a pixel write and cleared per flushed qword by
	// the flush process, both in one process (never two array-writing
	// processes racing on the same bits)
	always_ff @(posedge clk) begin
		if (reset) begin
			px_dirty <= {FB_W{1'b0}};
		end else if (fb_wr_en) begin
			px_dirty[fb_wr_x] <= 1'b1;
		end else if (state == S_FB_FLUSH_RD) begin
			px_dirty[{flush_qw_idx, 2'b00}]     <= 1'b0;
			px_dirty[{flush_qw_idx, 2'b00} + 1] <= 1'b0;
			px_dirty[{flush_qw_idx, 2'b00} + 2] <= 1'b0;
			px_dirty[{flush_qw_idx, 2'b00} + 3] <= 1'b0;
		end
	end

	// read side: ping-pong 336x16 (4 x 84x16 lanes) line buffers, one
	// prefetched (back) while the other is displayed (front)
	(* ramstyle = "M10K" *) logic [15:0] rd0_lane0 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd0_lane1 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd0_lane2 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd0_lane3 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd1_lane0 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd1_lane1 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd1_lane2 [0:FB_ROW_QW-1];
	(* ramstyle = "M10K" *) logic [15:0] rd1_lane3 [0:FB_ROW_QW-1];
	logic        rd_front_sel;   // 0: rd0 is front, 1: rd1 is front
	logic [7:0]  rd_front_row;

	wire [6:0] fb_rd_qw = (fb_rd_x < 9'd336) ? fb_rd_x[8:2] : 7'd0; // clamp: index never overruns the 84-entry lane arrays
	logic [1:0] fb_rd_lane_r;
	always_ff @(posedge clk) fb_rd_lane_r <= fb_rd_x[1:0];

	logic [15:0] rd0_q0_r, rd0_q1_r, rd0_q2_r, rd0_q3_r;
	logic [15:0] rd1_q0_r, rd1_q1_r, rd1_q2_r, rd1_q3_r;
	always_ff @(posedge clk) rd0_q0_r <= rd0_lane0[fb_rd_qw];
	always_ff @(posedge clk) rd0_q1_r <= rd0_lane1[fb_rd_qw];
	always_ff @(posedge clk) rd0_q2_r <= rd0_lane2[fb_rd_qw];
	always_ff @(posedge clk) rd0_q3_r <= rd0_lane3[fb_rd_qw];
	always_ff @(posedge clk) rd1_q0_r <= rd1_lane0[fb_rd_qw];
	always_ff @(posedge clk) rd1_q1_r <= rd1_lane1[fb_rd_qw];
	always_ff @(posedge clk) rd1_q2_r <= rd1_lane2[fb_rd_qw];
	always_ff @(posedge clk) rd1_q3_r <= rd1_lane3[fb_rd_qw];

	wire [15:0] rd0_sel = fb_rd_lane_r == 2'd0 ? rd0_q0_r : fb_rd_lane_r == 2'd1 ? rd0_q1_r :
						   fb_rd_lane_r == 2'd2 ? rd0_q2_r : rd0_q3_r;
	wire [15:0] rd1_sel = fb_rd_lane_r == 2'd0 ? rd1_q0_r : fb_rd_lane_r == 2'd1 ? rd1_q1_r :
						   fb_rd_lane_r == 2'd2 ? rd1_q2_r : rd1_q3_r;

	// fb_rd_y must match the row actually prefetched into the front
	// buffer, and fb_rd_x must be in range; anything else reads zero
	// rather than a stale, wrong-row or out-of-bounds pixel
	assign fb_rd_data = (fb_rd_x >= 9'd336 || fb_rd_y != rd_front_row) ? 16'h0 :
						 (rd_front_sel ? rd1_sel : rd0_sel);

	// prefetch FSM: on hblank, fetch next_line of fb_rd_frame's buffer
	// into the back line buffer, then swap front/back. Independent of the
	// main draw/prescan FSM below; it only asks for the DDR3 port through
	// ddr_owner/ddr_grant like every other issuer (see the bottom of the
	// module), but gets first priority for the next grant.
	logic       hblank_r;
	logic       pf_active;
	logic [6:0] pf_qw_idx;       // next qword to fill, 0..83, then done
	logic [3:0] pf_burst_qw;     // qwords left to receive in this burst, 0 = issue a new one
	logic       pf_row_ok;       // fetched row is valid in DDR (else fill 0)
	logic       pf_back_sel;     // which rd_lineN the prefetch is filling
	logic       pf_buf_sel;      // which DDR mo buffer (fb_rd_frame at prefetch start)
	logic [7:0] pf_row;

	// the mo line prefetch wants the bus exactly when it is mid-row, has a
	// real (row-valid) burst to fetch, and is between bursts; whether it
	// actually gets it this cycle is decided once, in ddr_grant below
	wire pf_want_bus = pf_active && pf_row_ok && pf_burst_qw == 4'd0 &&
						pf_qw_idx < FB_ROW_QW[6:0];
	wire [6:0] pf_burst_left = FB_ROW_QW[6:0] - pf_qw_idx;
	wire [28:0] pf_ddr_addr = (pf_buf_sel ? FB1_QW_BASE : FB0_QW_BASE) +
							  {14'h0, pf_row, 7'b0} + {22'h0, pf_qw_idx};

	// next_line is 8 bits (0..255) but row_valid is only 240 deep: clamp
	// the index so it never reads out of range, and a row >= 240 is simply
	// never valid
	wire [7:0] pf_next_line_idx = (next_line < 8'd240) ? next_line : 8'd0;
	wire pf_next_line_ok = (next_line < 8'd240) &&
							(fb_rd_frame ? row_valid1[pf_next_line_idx] : row_valid0[pf_next_line_idx]);

	// a qword is "accepted" this cycle when the row is invalid (immediate
	// zero-fill, no DDR access at all) or the DDR3 port granted this
	// module's prefetch ownership and delivered a data beat
	wire pf_word_ok = !pf_row_ok || (ddr_owner == OWN_PREFETCH && ddr_dout_ready);
	wire [15:0] pf_p0 = pf_row_ok ? ddr_dout[15:0]  : 16'h0;
	wire [15:0] pf_p1 = pf_row_ok ? ddr_dout[31:16] : 16'h0;
	wire [15:0] pf_p2 = pf_row_ok ? ddr_dout[47:32] : 16'h0;
	wire [15:0] pf_p3 = pf_row_ok ? ddr_dout[63:48] : 16'h0;
	wire pf_write_now = pf_active && pf_burst_qw > 4'd0 && pf_word_ok;

	always_ff @(posedge clk) if (pf_write_now && !pf_back_sel) rd0_lane0[pf_qw_idx] <= pf_p0;
	always_ff @(posedge clk) if (pf_write_now && !pf_back_sel) rd0_lane1[pf_qw_idx] <= pf_p1;
	always_ff @(posedge clk) if (pf_write_now && !pf_back_sel) rd0_lane2[pf_qw_idx] <= pf_p2;
	always_ff @(posedge clk) if (pf_write_now && !pf_back_sel) rd0_lane3[pf_qw_idx] <= pf_p3;
	always_ff @(posedge clk) if (pf_write_now &&  pf_back_sel) rd1_lane0[pf_qw_idx] <= pf_p0;
	always_ff @(posedge clk) if (pf_write_now &&  pf_back_sel) rd1_lane1[pf_qw_idx] <= pf_p1;
	always_ff @(posedge clk) if (pf_write_now &&  pf_back_sel) rd1_lane2[pf_qw_idx] <= pf_p2;
	always_ff @(posedge clk) if (pf_write_now &&  pf_back_sel) rd1_lane3[pf_qw_idx] <= pf_p3;

	always_ff @(posedge clk) begin
		if (reset) begin
			hblank_r     <= 1'b0;
			pf_active    <= 1'b0;
			pf_row_ok    <= 1'b0;
			rd_front_sel <= 1'b0;
			rd_front_row <= 8'hff; // never matches a real row at reset
		end else begin
			hblank_r <= hblank;
			if (!pf_active) begin
				if (hblank && !hblank_r) begin
					pf_active   <= 1'b1;
					pf_qw_idx   <= 7'd0;
					pf_burst_qw <= 4'd0;
					pf_back_sel <= ~rd_front_sel;
					pf_buf_sel  <= fb_rd_frame;
					pf_row      <= next_line;
					pf_row_ok   <= pf_next_line_ok;
				end
			end else if (pf_qw_idx >= FB_ROW_QW[6:0]) begin
				pf_active    <= 1'b0;
				rd_front_sel <= pf_back_sel;
				rd_front_row <= pf_row;
			end else if (pf_burst_qw > 0) begin
				if (pf_word_ok) begin
					pf_qw_idx   <= pf_qw_idx + 7'd1;
					pf_burst_qw <= pf_burst_qw - 4'd1;
				end
			end else if (!pf_row_ok || (req_accepted && req_owner == OWN_PREFETCH)) begin
				// issue the next burst (or, for an invalid row, just size
				// the zero-fill loop the same way): the request itself was
				// already latched and held at grant time (see req_pending
				// below), this only starts counting once it was accepted
				logic [6:0] left;
				left        = FB_ROW_QW[6:0] - pf_qw_idx;
				pf_burst_qw <= (left >= 8) ? 4'd8 : {1'b0, left[3:0]};
			end
		end
	end

	// ------------------------------------------------------------------
	// checksum accumulator BRAM: 256 x 16
	// ------------------------------------------------------------------
	(* ramstyle = "M10K" *) logic [15:0] chksum_mem [0:255];

	// ddr_addr/ddr_rd/ddr_burstcnt/ddr_we/ddr_din/ddr_be are driven
	// combinationally near the bottom of the module (see the always_comb
	// below main_fsm): a registered pulse would sample !ddr_busy one
	// cycle before the request is actually seen, and it can arrive
	// while busy has since gone high.

	// streaming ROM cache: one 64-byte line (8 ddr words = 32 ROM words).
	// Only 8 entries, so it is flops either way; the point of rom_sel_r is
	// that the array read itself is a plain always_ff q<=mem[addr] (the
	// ddr-word lane select), with the byte-swap extraction a clock later
	// from that register, not combinational off rom_line directly.
	logic [63:0] rom_line [0:7];
	logic        rom_line_valid;
	logic [24:0] rom_line_tag; // rom word index >> 5
	logic [63:0] rom_sel_r;

	logic [24:0] rom_req_word;
	logic [15:0] rom_resp_data;
	logic [2:0]  rom_burst_cnt;

	logic [63:0] whtab_data;

	// ------------------------------------------------------------------
	// control edge detect / erase-seen tracking
	// ------------------------------------------------------------------
	logic [2:0] control_r;
	wire        mogo_rise = control[0] & ~control_r[0];
	logic       erase_seen;

	// a MOGO edge while the engine is busy must not be lost: latch it (and
	// the command/frame bit at that edge) for S_IDLE to act on once free
	logic       mogo_pending;
	logic [15:0] mogo_pending_cmd;
	logic       mogo_pending_frame;

	always_ff @(posedge clk) begin
		if (reset) begin
			control_r    <= 3'b0;
			erase_seen   <= 1'b0;
			mogo_pending <= 1'b0;
		end else begin
			control_r <= control;
			// vblank frame-aligns the accumulator (mirrors MAME's
			// m_partial_scanline reset at vblank); mogo consumes it.
			if (vblank_rise)    erase_seen <= control[1];
			else if (mogo_rise) erase_seen <= control[1];
			else                erase_seen <= erase_seen | control[1];

			if (mogo_rise && state != S_IDLE) begin
				mogo_pending       <= 1'b1;
				mogo_pending_cmd   <= mo_command;
				mogo_pending_frame <= control[2];
			end else if (state == S_IDLE) begin
				mogo_pending <= 1'b0; // consumed this cycle, or nothing pending
			end
		end
	end

	// prescan phase 1
	logic [7:0]  clear_idx;
	logic [24:0] word_idx;
	logic [24:0] lowest_address_r;
	logic [15:0] grp_w2;
	logic [23:0] objectcount_r;
	logic [15:0] chunk_acc; // running sum for the current 0x10000-word chunk

	// prescan phase 2
	logic [24:0] obj_idx;
	logic [15:0] flags_r;
	logic [2:0]  which_r;
	logic [24:0] data_offset_r;
	logic [24:0] row_ptr_r;
	logic [10:0] height_cnt;
	logic [23:0] width_max, tempwidth;
	logic [15:0] entry_count_r;
	logic [24:0] entry_idx_r;

	// checksum command
	logic [8:0] reqsums_r;
	logic [15:0] chksum_rd_r; // registered read of chksum_mem for S_CKSUM_LOOP
	logic [8:0] cksum_idx;

	// sort/draw: order_cache holds all 256 order bytes read once per draw
	// command so the order x objnum scan is a local compare, not 65K+
	// object-RAM round trips
	(* ramstyle = "M10K" *) logic [7:0]  order_cache [0:255];
	logic [7:0]  order_cache_rd_r; // registered read of order_cache for S_DRAW_SORT
	logic [7:0]  cache_idx;
	logic [7:0]  order_cnt, objnum_cnt;
	logic [14:0] code_r;
	logic        hflip_r, vram_r;
	logic [7:0]  colour_r;
	logic [2:0]  priority_r;
	logic signed [17:0] xpos_r, ypos_r;
	logic [15:0] scale_r;
	logic [2:0]  field_idx;
	logic [1:0]  hdr_idx;
	logic signed [17:0] xoffs_r, yoffs_r;
	logic [15:0] width_r, height_r;

	logic signed [39:0] scaled_xoffs_r, scaled_yoffs_r;
	// one-cycle pipeline regs so a multiply never feeds an add/compare in
	// the same clock (Quartus timing): raw products registered here,
	// shift/add/subtract/clamp done from these on the following cycle
	logic signed [39:0] sxo_r, syo_r;
	logic        [39:0] sw_full_r;
	logic        [39:0] sw_prod_r, sh_prod_r;
	logic        [63:0] neg_sy_prod_r;
	logic [15:0] palettebase_r;
	logic [19:0] scalex_r, scaley_r;
	logic [31:0] scaled_width_r, scaled_height_r;
	logic [31:0] dx_r, dy_r;
	logic signed [31:0] sx_r, sy_r, ex_r, ey_r;
	logic signed [31:0] y_hi;
	logic [31:0] sourcey_r;
	logic [8:0]  cur_y;
	// fb flush/row-open (no more row_base_addr: cur_y is the row directly,
	// the y*336 multiply is gone along with the BRAM frame buffer);
	// flush_qw_idx and flush_p0_r..flush_p3_r are declared earlier (used by
	// the frame buffer section above)
	logic [3:0]  flush_dirty_r;
	logic [7:0]  flush_be_r;
	logic [24:0] current_row_r;
	logic signed [17:0] x_cursor;
	logic signed [2:0]  x_dir;
	logic [31:0] sourcex_r, rle_end_r;
	logic [15:0] lut_lo, lut_hi;

	// sequential 32-bit unsigned restoring divider, shared by dx and dy
	logic [31:0] div_rem, div_quot, div_den;
	logic [5:0]  div_cnt;

	logic prescan_done_r;
	assign prescan_done = prescan_done_r;

	always_ff @(posedge clk) begin
		if (reset) begin
			state          <= S_IDLE;
			rom_line_valid <= 1'b0;
			prescan_done_r <= 1'b0;
			fb_wr_en       <= 1'b0;
			oram_i_we      <= 1'b0;
			objectcount_r  <= 24'd0;
			row_valid0     <= {FB_H{1'b0}};
			row_valid1     <= {FB_H{1'b0}};
		end else begin
			// default one-cycle pulses
			fb_wr_en  <= 1'b0;
			oram_i_we <= 1'b0;

			case (state)
			// ----------------------------------------------------------------
			S_IDLE: begin
				if (prescan_start && !prescan_done_r) begin
					// clear the checksum BRAM first (MAME memsets m_checksums
					// before accumulating; real BRAM power-on content is
					// likewise undefined)
					clear_idx <= 8'd0;
					state     <= S_PS0_CLEAR;
				end else if (mogo_rise || mogo_pending) begin
					logic [15:0] eff_cmd;
					logic        eff_frame;
					eff_cmd   = mogo_rise ? mo_command : mogo_pending_cmd;
					eff_frame = mogo_rise ? control[2] : mogo_pending_frame;
					if (eff_cmd == 16'd2) begin
						oram_i_addr <= 11'd0;
						state       <= S_CKSUM_READ0;
					end else begin
						fb_wr_buf <= ~eff_frame;
						asm_open  <= 1'b0; // no row carried over from a previous draw
						if (erase_seen | control[1]) begin
							state <= S_DRAW_ERASE;
						end else begin
							cache_idx <= 8'd0;
							state     <= S_DRAW_CACHE_ORDER;
						end
					end
				end
			end

			// ----------------------------------------------------------------
			// shared ROM word fetch: rom_req_word -> rom_resp_data, then return_state
			// ----------------------------------------------------------------
			S_ROM_FETCH: begin
				if (rom_line_valid && rom_line_tag == rom_req_word[24:5]) begin
					state <= S_ROM_FETCH_SEL;
				end else if (req_accepted && req_owner == OWN_ROM) begin
					// the request was latched at grant time (see
					// req_pending below) and is only now actually accepted
					// by the port
					rom_burst_cnt <= 3'd0;
					rom_line_tag  <= rom_req_word[24:5];
					state         <= S_ROM_FETCH_WAIT;
				end
			end
			S_ROM_FETCH_WAIT: begin
				// only this owner's data-ready pulses count: with a single
				// shared bus, ddr_dout_ready is meaningless to anyone else
				// while ddr_owner != OWN_ROM
				if (ddr_owner == OWN_ROM && ddr_dout_ready) begin
					rom_line[rom_burst_cnt] <= ddr_dout;
					if (rom_burst_cnt == 3'd7) begin
						rom_line_valid <= 1'b1;
						state          <= S_ROM_FETCH_SEL;
					end
					rom_burst_cnt <= rom_burst_cnt + 3'd1;
				end
			end
			// plain registered array read (Quartus M10K/LUTRAM friendly):
			// select the ddr-word lane first, extract the byte-swapped
			// 16-bit word from that register a clock later (rom_req_word
			// does not change across the two states)
			S_ROM_FETCH_SEL: begin
				rom_sel_r <= rom_line[rom_req_word[4:2]];
				state     <= S_ROM_FETCH_DONE;
			end
			S_ROM_FETCH_DONE: begin
				// loader packs bytes address-order into little-endian 64-bit
				// words; a big-endian ROM word is the byte-swap of the lane.
				rom_resp_data <= {rom_sel_r[16*rom_req_word[1:0] +: 8],
								   rom_sel_r[16*rom_req_word[1:0]+8 +: 8]};
				state <= return_state;
			end

			S_PS0_CLEAR: begin
				chksum_mem[clear_idx] <= 16'h0;
				if (clear_idx == 8'd255) begin
					word_idx         <= 25'd0;
					lowest_address_r <= ROM_WORDS[24:0];
					chunk_acc        <= 16'h0;
					rom_line_valid   <= 1'b0;
					// S_PS1_GOT assumes rom_sel_r is already primed for
					// word_idx: prime it for word 0 here via the shared
					// fetch machinery (always a miss, cache is empty)
					rom_req_word     <= 25'd0;
					return_state     <= S_PS1_GOT;
					state            <= S_ROM_FETCH;
				end else begin
					clear_idx <= clear_idx + 8'd1;
				end
			end

			// ----------------------------------------------------------------
			// prescan phase 1: count_objects + checksum, single linear pass.
			// One word per clock on a cache hit (the common case, 31/32
			// lines): the word is pulled straight out of the line buffer
			// and folded into a running per-chunk sum, so there is no
			// object-RAM-style read-modify-write and no extra state hop
			// through S_ROM_FETCH_DONE. A miss (crossing into a new 32-word
			// line) defers to the shared fetch machinery and comes back
			// here once the line is filled.
			// ----------------------------------------------------------------
			// rom_sel_r is always pre-loaded with word_idx's 64-bit ddr-word
			// group by the time this state runs (S_PS0_CLEAR primes it for
			// word 0; every later iteration prefetches one word ahead
			// below). rom_line[] holds 4 words per 64-bit entry, so 3 out
			// of every 4 words need no new array read at all: only when
			// the NEXT word starts a new 4-word group does this state
			// issue a fresh (plain, registered) select for it, staying at
			// one clock per word except at that boundary, and only paying
			// for an actual DDR burst on a real cache-line (32-word) miss.
			S_PS1_GOT: begin
				logic [15:0] w;
				logic [24:0] next_idx;
				w = {rom_sel_r[16*word_idx[1:0] +: 8],
					 rom_sel_r[16*word_idx[1:0]+8 +: 8]};

				// flush the chunk sum at each 0x10000-word boundary (and
				// at the very last word, for a ROM smaller than a chunk)
				if (word_idx[15:0] == 16'hffff ||
					{1'b0, word_idx} == ROM_WORDS[24:0] - 25'd1) begin
					chksum_mem[word_idx[23:16]] <= chunk_acc + w;
					chunk_acc <= 16'h0;
				end else begin
					chunk_acc <= chunk_acc + w;
				end

				if (word_idx[1:0] == 2'd2) grp_w2 <= w;
				if (word_idx[1:0] == 2'd3 && (word_idx - 24'd3) < lowest_address_r &&
					{grp_w2[7:0], w} > (word_idx - 24'd3) &&
					{grp_w2[7:0], w} < lowest_address_r)
					lowest_address_r <= {grp_w2[7:0], w};

				if ({1'b0, word_idx} == ROM_WORDS[24:0] - 25'd1) begin
					objectcount_r <= lowest_address_r >> 2;
					obj_idx       <= 24'd0;
					state         <= S_PS2_NEXT_OBJ;
				end else begin
					next_idx = word_idx + 25'd1;
					word_idx <= next_idx;
					if (word_idx[1:0] == 2'b11) begin
						if (rom_line_valid && rom_line_tag == next_idx[24:5]) begin
							rom_sel_r <= rom_line[next_idx[4:2]];
							state     <= S_PS1_GOT;
						end else begin
							rom_req_word <= next_idx;
							return_state <= S_PS1_GOT;
							state        <= S_ROM_FETCH;
						end
					end else begin
						state <= S_PS1_GOT; // rom_sel_r already covers the next word
					end
				end
			end

			// ----------------------------------------------------------------
			// prescan phase 2: per-object width/height row walk
			// ----------------------------------------------------------------
			S_PS2_NEXT_OBJ: begin
				if (obj_idx == objectcount_r) begin
					prescan_done_r <= 1'b1;
					state          <= S_IDLE;
				end else begin
					rom_req_word <= obj_idx * 24'd4 + 24'd2;
					return_state <= S_PS2_HDR2;
					state        <= S_ROM_FETCH;
				end
			end
			S_PS2_HDR2: begin
				flags_r      <= rom_resp_data;
				which_r      <= rom_resp_data[10:8];
				rom_req_word <= obj_idx * 24'd4 + 24'd3;
				return_state <= S_PS2_HDR3;
				state        <= S_ROM_FETCH;
			end
			S_PS2_HDR3: begin
				if ({flags_r[7:0], rom_resp_data} < (obj_idx * 24'd4) ||
					{1'b0, flags_r[7:0], rom_resp_data} >= ROM_WORDS[24:0]) begin
					width_max  <= 24'd0;
					height_cnt <= 11'd0;
					state      <= S_PS2_TABWR;
				end else begin
					data_offset_r <= {flags_r[7:0], rom_resp_data};
					row_ptr_r     <= {flags_r[7:0], rom_resp_data};
					width_max     <= 24'd0;
					height_cnt    <= 11'd0;
					state         <= S_PS2_ROWCNT; // checks its own cache hit/miss
				end
			end
			// ROWCNT and ENTRY read straight out of the line buffer on a
			// cache hit (the common case: row data is read sequentially),
			// deferring to the shared fetch machinery only on a miss, same
			// as S_PS1_GOT: one clock per word instead of the generic
			// ROM_FETCH/WAIT/DONE round trip on every single word.
			S_PS2_ROWCNT: begin
				if (rom_line_valid && rom_line_tag == row_ptr_r[24:5]) begin
					// plain registered read; row_ptr_r is untouched until
					// S_PS2_ROWCNT2 (or S_PS2_ROW_ADV), so its low bits
					// stay valid for the byte-swap extraction there
					rom_sel_r <= rom_line[row_ptr_r[4:2]];
					state     <= S_PS2_ROWCNT2;
				end else begin
					rom_req_word <= row_ptr_r;
					return_state <= S_PS2_ROWCNT;
					state        <= S_ROM_FETCH;
				end
			end
			S_PS2_ROWCNT2: begin
				logic [15:0] w;
				logic [24:0] first_target;
				w = {rom_sel_r[16*row_ptr_r[1:0] +: 8],
					 rom_sel_r[16*row_ptr_r[1:0]+8 +: 8]};
				if ((w[15] ? ~w : w) == 16'd0 ||
					height_cnt == 11'd1024 || {1'b0, row_ptr_r} >= ROM_WORDS[24:0]) begin
					state <= S_PS2_TABWR;
				end else begin
					entry_count_r <= w[15] ? ~w : w;
					entry_idx_r   <= 24'd0;
					tempwidth     <= 24'd0;
					// prime rom_sel_r for S_PS2_ENTRY's first word (which
					// assumes it is already loaded, same as S_PS1_GOT)
					first_target = row_ptr_r + 25'd1;
					if (rom_line_valid && rom_line_tag == first_target[24:5]) begin
						rom_sel_r <= rom_line[first_target[4:2]];
						state     <= S_PS2_ENTRY;
					end else begin
						rom_req_word <= first_target;
						return_state <= S_PS2_ENTRY;
						state        <= S_ROM_FETCH;
					end
				end
			end
			// rom_sel_r is always pre-loaded for row_ptr_r+1+entry_idx_r by
			// the time this state runs (S_PS2_ROWCNT2 primes the first
			// entry; every later iteration prefetches one ahead below),
			// same one-clock-per-word reasoning as S_PS1_GOT.
			S_PS2_ENTRY: begin
				logic [24:0] target, next_target;
				logic [15:0] w, lo, hi;
				target = row_ptr_r + 25'd1 + {1'b0, entry_idx_r};
				w = {rom_sel_r[16*target[1:0] +: 8],
					 rom_sel_r[16*target[1:0]+8 +: 8]};
				lo = rle_lookup(w[7:0], which_r);
				hi = rle_lookup(w[15:8], which_r);
				tempwidth <= tempwidth + {16'h0, lo[15:8]} + {16'h0, hi[15:8]};
				if (entry_idx_r + 24'd1 == {8'h0, entry_count_r}) begin
					state <= S_PS2_ROW_ADV;
				end else begin
					entry_idx_r <= entry_idx_r + 24'd1;
					next_target  = target + 25'd1;
					if (target[1:0] == 2'b11) begin
						if (rom_line_valid && rom_line_tag == next_target[24:5]) begin
							rom_sel_r <= rom_line[next_target[4:2]];
							state     <= S_PS2_ENTRY;
						end else begin
							rom_req_word <= next_target;
							return_state <= S_PS2_ENTRY;
							state        <= S_ROM_FETCH;
						end
					end else begin
						state <= S_PS2_ENTRY; // rom_sel_r already covers the next entry word
					end
				end
			end
			S_PS2_ROW_ADV: begin
				// clamp like height's natural 1024-row cap, in case a
				// corrupt row claims an absurd run length
				if (tempwidth > width_max)
					width_max <= (tempwidth > 24'd1024) ? 24'd1024 : tempwidth;
				row_ptr_r  <= row_ptr_r + 24'd1 + {8'h0, entry_count_r};
				height_cnt <= height_cnt + 11'd1;
				state      <= S_PS2_ROWCNT;
			end
			S_PS2_TABWR: begin
				// one 32-bit entry per object (width low, height high),
				// two objects per 64-bit DDR word, since draw_rle needs
				// both and height is not known before the row walk
				if (req_accepted && req_owner == OWN_TABWR) begin
					obj_idx <= obj_idx + 24'd1;
					state   <= S_PS2_NEXT_OBJ;
				end
			end

			// ----------------------------------------------------------------
			// checksum command: reqsums = oram[0]+1 capped at 256
			// ----------------------------------------------------------------
			S_CKSUM_READ0: state <= S_CKSUM_GOT0;
			S_CKSUM_GOT0: begin
				reqsums_r <= (({1'b0, oram_i_dout_r} + 17'd1) > 17'd256) ?
							 9'd256 : ({1'b0, oram_i_dout_r} + 17'd1);
				cksum_idx <= 9'd0;
				state     <= S_CKSUM_LOOP;
			end
			S_CKSUM_LOOP: begin
				if (cksum_idx == reqsums_r) begin
					state <= S_IDLE;
				end else begin
					// the only read of chksum_mem: a plain registered read,
					// consumed a cycle later in S_CKSUM_WR
					chksum_rd_r <= chksum_mem[cksum_idx[7:0]];
					state       <= S_CKSUM_WR;
				end
			end
			S_CKSUM_WR: begin
				oram_i_addr <= {2'b00, cksum_idx};
				oram_i_din  <= chksum_rd_r;
				oram_i_we   <= 1'b1;
				cksum_idx   <= cksum_idx + 9'd1;
				state       <= S_CKSUM_LOOP;
			end

			// ----------------------------------------------------------------
			// draw command: whole-buffer erase (simplification: MAME does a partial
			// erase from the current scanline; a full clear is simpler and
			// cheap enough here), then sort_and_render
			// ----------------------------------------------------------------
			// erase is now a bit-vector clear, not 80640 writes: DDR3
			// content is untouched, a row reads zero until its first flush
			S_DRAW_ERASE: begin
				if (fb_wr_buf) row_valid1 <= {FB_H{1'b0}};
				else            row_valid0 <= {FB_H{1'b0}};
				cache_idx <= 8'd0;
				state     <= S_DRAW_CACHE_ORDER;
			end

			// once per draw: read all 256 order bytes into order_cache so
			// the order x objnum scan below is a local compare instead of
			// 65K+ round trips through the object-RAM read port
			S_DRAW_CACHE_ORDER: begin
				oram_i_addr <= {cache_idx, 3'd6};
				state       <= S_DRAW_CACHE_WAIT;
			end
			S_DRAW_CACHE_WAIT: state <= S_DRAW_CACHE_GOT;
			S_DRAW_CACHE_GOT: begin
				order_cache[cache_idx] <= oram_i_dout_r[7:0];
				if (cache_idx == 8'd255) begin
					order_cnt  <= 8'd1;
					objnum_cnt <= 8'd255;
					state      <= S_DRAW_SORT_RD;
				end else begin
					cache_idx <= cache_idx + 8'd1;
					state     <= S_DRAW_CACHE_ORDER;
				end
			end

			// the only read of order_cache: a plain registered read for the
			// current objnum_cnt, one clock ahead of S_DRAW_SORT's compare
			S_DRAW_SORT_RD: begin
				order_cache_rd_r <= order_cache[objnum_cnt];
				state            <= S_DRAW_SORT;
			end
			// order 1..255 outer, objnum 255..0 inner (LIFO per order);
			// advancing objnum/order is S_DRAW_OBJ_NEXT's job (also used
			// after a full object draw), so it is decided in one place
			S_DRAW_SORT: begin
				if (order_cache_rd_r == order_cnt) begin
					field_idx   <= 3'd0;
					oram_i_addr <= {objnum_cnt, 3'd0};
					state       <= S_DRAW_FIELD;
				end else begin
					state <= S_DRAW_OBJ_NEXT;
				end
			end

			S_DRAW_FIELD: state <= S_DRAW_FIELD_GOT;
			S_DRAW_FIELD_GOT: begin
				case (field_idx)
					3'd0: begin code_r <= oram_i_dout_r[14:0]; hflip_r <= oram_i_dout_r[15]; end
					3'd1: begin
						colour_r   <= oram_i_dout_r[11:4];
						priority_r <= oram_i_dout_r[11:9];
						vram_r     <= oram_i_dout_r[15];
					end
					3'd2: xpos_r <= oram_i_dout_r[15] ? {8'hff, oram_i_dout_r[15:6]} : {8'h00, oram_i_dout_r[15:6]};
					3'd3: ypos_r <= oram_i_dout_r[15] ? {8'hff, oram_i_dout_r[15:6]} : {8'h00, oram_i_dout_r[15:6]};
					default: ; // field 4 (scale) handled below
				endcase

				if (field_idx == 3'd4) begin
					scale_r <= oram_i_dout_r;
					if (oram_i_dout_r == 16'h0 || ({9'h0, code_r} >= objectcount_r) || vram_r) begin
						state <= S_DRAW_OBJ_NEXT;
					end else begin
						hdr_idx      <= 2'd0;
						rom_req_word <= {7'b0, code_r, 2'b00};
						return_state <= S_DRAW_ROMHDR_GOT;
						state        <= S_ROM_FETCH;
					end
				end else begin
					field_idx   <= field_idx + 3'd1;
					oram_i_addr <= {objnum_cnt, field_idx + 3'd1};
					state       <= S_DRAW_FIELD;
				end
			end

			S_DRAW_ROMHDR_GOT: begin
				case (hdr_idx)
					2'd0: xoffs_r <= rom_resp_data[15] ? {2'b11, rom_resp_data} : {2'b00, rom_resp_data};
					2'd1: yoffs_r <= rom_resp_data[15] ? {2'b11, rom_resp_data} : {2'b00, rom_resp_data};
					2'd2: begin flags_r <= rom_resp_data; which_r <= rom_resp_data[10:8]; end
					2'd3: data_offset_r <= {flags_r[7:0], rom_resp_data};
				endcase
				if (hdr_idx == 2'd3) begin
					state <= S_DRAW_WHTAB_REQ;
				end else begin
					hdr_idx      <= hdr_idx + 2'd1;
					rom_req_word <= {7'b0, code_r, 2'b00} + {21'h0, hdr_idx + 2'd1};
					return_state <= S_DRAW_ROMHDR_GOT;
					state        <= S_ROM_FETCH;
				end
			end

			S_DRAW_WHTAB_REQ: begin
				// the request was latched at grant time; wait for the
				// port to actually accept it before waiting for data
				if (req_accepted && req_owner == OWN_WHTAB) state <= S_DRAW_WHTAB_WAIT;
			end
			S_DRAW_WHTAB_WAIT: begin
				if (ddr_owner == OWN_WHTAB && ddr_dout_ready) begin
					whtab_data <= ddr_dout;
					state      <= S_DRAW_COMPUTE1;
				end
			end

			// registers only the three raw products (scale*xoffs,
			// scale*yoffs, scale*width); the >>>12 shifts, the hflip
			// subtract and the palette mask are done from those registers
			// in S_DRAW_COMPUTE1B, so no multiplier output ever feeds an
			// adder/comparator in the same clock (Quartus timing)
			S_DRAW_COMPUTE1: begin
				logic [15:0] cw, ch;

				cw = code_r[0] ? whtab_data[47:32] : whtab_data[15:0];
				ch = code_r[0] ? whtab_data[63:48] : whtab_data[31:16];

				// width==0 or height==0 means prescan found an invalid data
				// offset for this object (matches MAME bailing on
				// info.data == nullptr in draw_rle, before it ever divides
				// by scaled_width/height)
				if (cw == 16'h0 || ch == 16'h0) begin
					state <= S_DRAW_OBJ_NEXT;
				end else begin
					width_r   <= cw;
					height_r  <= ch;
					sxo_r     <= $signed({1'b0, scale_r}) * $signed({{22{xoffs_r[17]}}, xoffs_r});
					syo_r     <= $signed({1'b0, scale_r}) * $signed({{22{yoffs_r[17]}}, yoffs_r});
					sw_full_r <= {1'b0, scale_r} * {24'h0, cw};
					scalex_r  <= {scale_r, 4'b0};
					scaley_r  <= {scale_r, 4'b0};
					state     <= S_DRAW_COMPUTE1B;
				end
			end
			S_DRAW_COMPUTE1B: begin
				logic [15:0] color_raw, bppmask;
				color_raw = {1'b0, priority_r, colour_r, 4'b0};
				bppmask   = (which_r == 3'd0) ? 16'hfff0 : (which_r <= 3'd3) ? 16'hffe0 : 16'hffc0;

				scaled_xoffs_r <= hflip_r ? (($signed({1'b0, sw_full_r}) >>> 12) - (sxo_r >>> 12)) : (sxo_r >>> 12);
				scaled_yoffs_r <= syo_r >>> 12;
				palettebase_r  <= color_raw & bppmask;
				state          <= S_DRAW_COMPUTE2;
			end

			// same one-clock-multiply-only rule: register scalex*width and
			// scaley*height here, do the +0x7fff/>>16/clamp in COMPUTE2B
			S_DRAW_COMPUTE2: begin
				sw_prod_r <= {20'h0, scalex_r} * {24'h0, width_r};
				sh_prod_r <= {20'h0, scaley_r} * {24'h0, height_r};
				state     <= S_DRAW_COMPUTE2B;
			end
			S_DRAW_COMPUTE2B: begin
				logic [39:0] sw, sh;
				logic [31:0] scw, sch;
				sw = sw_prod_r + 40'h7fff;
				sh = sh_prod_r + 40'h7fff;
				scw = sw[39:16];
				sch = sh[39:16];
				scaled_width_r  <= (scw == 32'd0) ? 32'd1 : scw;
				scaled_height_r <= (sch == 32'd0) ? 32'd1 : sch;
				sx_r <= $signed({{14{xpos_r[17]}}, xpos_r}) - $signed(scaled_xoffs_r[31:0]);
				sy_r <= $signed({{14{ypos_r[17]}}, ypos_r}) - $signed(scaled_yoffs_r[31:0]);
				state <= S_DRAW_DIVSETUP;
			end

			S_DRAW_DIVSETUP: begin
				div_rem  <= 32'd0;
				div_quot <= {width_r, 16'h0};
				div_den  <= scaled_width_r;
				div_cnt  <= 6'd32;
				state    <= S_DRAW_DIV_DX_RUN;
			end
			S_DRAW_DIV_DX_RUN: begin
				logic [32:0] sub;
				sub = {div_rem[30:0], div_quot[31]} - {1'b0, div_den};
				if (sub[32]) begin
					div_rem  <= {div_rem[30:0], div_quot[31]};
					div_quot <= {div_quot[30:0], 1'b0};
				end else begin
					div_rem  <= sub[31:0];
					div_quot <= {div_quot[30:0], 1'b1};
				end
				if (div_cnt == 6'd1) state <= S_DRAW_DIV_DX_DONE;
				div_cnt <= div_cnt - 6'd1;
			end
			S_DRAW_DIV_DX_DONE: begin
				dx_r     <= div_quot;
				div_rem  <= 32'd0;
				div_quot <= {height_r, 16'h0};
				div_den  <= scaled_height_r;
				div_cnt  <= 6'd32;
				state    <= S_DRAW_DIV_DY_RUN;
			end
			S_DRAW_DIV_DY_RUN: begin
				logic [32:0] sub;
				sub = {div_rem[30:0], div_quot[31]} - {1'b0, div_den};
				if (sub[32]) begin
					div_rem  <= {div_rem[30:0], div_quot[31]};
					div_quot <= {div_quot[30:0], 1'b0};
				end else begin
					div_rem  <= sub[31:0];
					div_quot <= {div_quot[30:0], 1'b1};
				end
				if (div_cnt == 6'd1) state <= S_DRAW_DIV_DY_DONE;
				div_cnt <= div_cnt - 6'd1;
			end
			S_DRAW_DIV_DY_DONE: begin
				dy_r  <= div_quot;
				ex_r  <= sx_r + $signed(scaled_width_r) - 32'sd1;
				ey_r  <= sy_r + $signed(scaled_height_r) - 32'sd1;
				state <= S_DRAW_CLIP;
			end

			// the top-clip sourcey term ((-sy)*dy) is a multiply; register
			// it alone here and add dy_r>>1 to it in S_DRAW_CLIP2 next
			// cycle, instead of chaining multiply+add in one clock
			S_DRAW_CLIP: begin
				if (sx_r > 32'sd335 || ex_r < 32'sd0) begin
					state <= S_DRAW_OBJ_NEXT;
				end else begin
					logic signed [31:0] lo_val, hi_val;
					lo_val = (sy_r < 32'sd0) ? 32'sd0 : sy_r;
					hi_val = (ey_r > 32'sd239) ? 32'sd239 : ey_r;
					if (lo_val > hi_val) begin
						state <= S_DRAW_OBJ_NEXT;
					end else begin
						cur_y         <= lo_val[8:0];
						y_hi          <= hi_val;
						neg_sy_prod_r <= $unsigned(-sy_r) * dy_r;
						current_row_r <= 25'd0;
						row_ptr_r     <= data_offset_r;
						x_dir         <= hflip_r ? -3'sd1 : 3'sd1;
						x_cursor      <= hflip_r ? ex_r[17:0] : sx_r[17:0];
						state         <= S_DRAW_CLIP2;
					end
				end
			end
			S_DRAW_CLIP2: begin
				sourcey_r <= (sy_r < 32'sd0) ? ((dy_r >> 1) + neg_sy_prod_r[31:0]) : (dy_r >> 1);
				state     <= S_FB_ROW_OPEN;
			end

			// make sure the line assembly buffer represents (fb_wr_buf,
			// cur_y) before any pixel write lands; flush a different row
			// first if one is open, then open the target row. There is no
			// zero-fill step: the assembly arrays are never cleared, the
			// flush itself sends 0 for any lane that is not dirty.
			S_FB_ROW_OPEN: begin
				if (asm_open && asm_buf == fb_wr_buf && asm_row == cur_y[7:0]) begin
					state <= S_DRAW_ROW_ADV_CHECK; // already the right row
				end else if (asm_open) begin
					return_state <= S_FB_ROW_OPEN2;
					flush_qw_idx <= 7'd0;
					state        <= S_FB_FLUSH_SCAN;
				end else begin
					state <= S_FB_ROW_OPEN2;
				end
			end
			S_FB_ROW_OPEN2: begin
				asm_buf  <= fb_wr_buf;
				asm_row  <= cur_y[7:0];
				asm_open <= 1'b1;
				asm_first_touch <= fb_wr_buf ? !row_valid1[cur_y] : !row_valid0[cur_y];
				state <= S_DRAW_ROW_ADV_CHECK;
			end

			// scan the row's qwords, flushing any that need it (all of
			// them on a not-yet-valid row, only the dirty ones otherwise);
			// no read-modify-write, byte enables leave untouched DDR3
			// lanes alone
			S_FB_FLUSH_SCAN: begin
				if (flush_qw_idx >= FB_ROW_QW[6:0]) begin
					if (asm_buf) row_valid1[asm_row] <= 1'b1;
					else          row_valid0[asm_row] <= 1'b1;
					asm_open <= 1'b0;
					state    <= return_state;
				end else begin
					logic any_dirty;
					any_dirty = px_dirty[{flush_qw_idx, 2'b00}]     |
								px_dirty[{flush_qw_idx, 2'b00} + 1] |
								px_dirty[{flush_qw_idx, 2'b00} + 2] |
								px_dirty[{flush_qw_idx, 2'b00} + 3];
					if (asm_first_touch || any_dirty) begin
						state <= S_FB_FLUSH_RD;
					end else begin
						flush_qw_idx <= flush_qw_idx + 7'd1;
					end
				end
			end
			// plain registered reads of the qword's 4 lanes (one array
			// each, one read port each); the dirty snapshot is taken here
			// too, before the px_dirty process (above) clears it this same
			// cycle, so S_FB_FLUSH_QW knows which lanes actually have real
			// data next cycle regardless of first-touch/be
			S_FB_FLUSH_RD: begin
				// asm_lane0..3 reads are hoisted into their own always_ff
				// blocks below (Quartus only infers M10K when q<=mem[addr]
				// is the whole body of its own always block); flush_qw_idx
				// is stable throughout this state, so those reads land in
				// flush_p0_r..flush_p3_r by the time S_FB_FLUSH_QW needs them
				flush_dirty_r <= {px_dirty[{flush_qw_idx, 2'b00} + 3], px_dirty[{flush_qw_idx, 2'b00} + 2],
								   px_dirty[{flush_qw_idx, 2'b00} + 1], px_dirty[{flush_qw_idx, 2'b00}]};
				flush_be_r <= asm_first_touch ? 8'hff :
							  {{2{px_dirty[{flush_qw_idx, 2'b00} + 3]}},
							   {2{px_dirty[{flush_qw_idx, 2'b00} + 2]}},
							   {2{px_dirty[{flush_qw_idx, 2'b00} + 1]}},
							   {2{px_dirty[{flush_qw_idx, 2'b00}]}}};
				state <= S_FB_FLUSH_QW;
			end
			S_FB_FLUSH_QW: begin
				// the request was latched at grant time; advance only
				// once the port actually accepted it
				if (req_accepted && req_owner == OWN_FLUSH) begin
					flush_qw_idx <= flush_qw_idx + 7'd1;
					state        <= S_FB_FLUSH_SCAN;
				end
			end

			S_DRAW_ROW_ADV_CHECK: begin
				rom_req_word <= row_ptr_r;
				if (current_row_r == (sourcey_r >> 16)) return_state <= S_DRAW_ROW_HDR_GOT;
				else                                    return_state <= S_DRAW_ROW_ADV_GOT;
				state <= S_ROM_FETCH;
			end
			S_DRAW_ROW_ADV_GOT: begin
				row_ptr_r     <= row_ptr_r + 24'd1 +
								 {8'h0, (rom_resp_data[15] ? ~rom_resp_data : rom_resp_data)};
				current_row_r <= current_row_r + 24'd1;
				state         <= S_DRAW_ROW_ADV_CHECK;
			end
			S_DRAW_ROW_HDR_GOT: begin
				entry_count_r <= rom_resp_data[15] ? ~rom_resp_data : rom_resp_data;
				entry_idx_r   <= 24'd0;
				sourcex_r     <= dx_r >> 1;
				rle_end_r     <= 32'd0;
				if ((rom_resp_data[15] ? ~rom_resp_data : rom_resp_data) == 16'd0) begin
					state <= S_DRAW_ROW_NEXT;
				end else begin
					rom_req_word <= row_ptr_r + 24'd1;
					return_state <= S_DRAW_ROW_ENTRY_GOT;
					state        <= S_ROM_FETCH;
				end
			end

			S_DRAW_ROW_ENTRY_GOT: begin
				logic [15:0] lo;
				lo = rle_lookup(rom_resp_data[7:0], which_r);
				lut_lo    <= lo;
				lut_hi    <= rle_lookup(rom_resp_data[15:8], which_r);
				rle_end_r <= rle_end_r + ({16'h0, lo[15:8]} << 16);
				state     <= S_DRAW_PIXEL_LO;
			end
			S_DRAW_PIXEL_LO: begin
				// dx_r == 0 would spin forever (sourcex_r never advances);
				// that should not happen once width/height are validated,
				// but treat it as an empty run instead of hanging
				if (sourcex_r < rle_end_r && dx_r != 32'h0) begin
					if (x_cursor >= 18'sd0 && x_cursor < 18'sd336 && lut_lo[7:0] != 8'h0) begin
						fb_wr_en   <= 1'b1;
						fb_wr_data <= {8'h0, lut_lo[7:0]} + palettebase_r;
						fb_wr_x    <= x_cursor[8:0];
					end
					x_cursor  <= x_cursor + {{15{x_dir[2]}}, x_dir};
					sourcex_r <= sourcex_r + dx_r;
				end else begin
					rle_end_r <= rle_end_r + ({16'h0, lut_hi[15:8]} << 16);
					state     <= S_DRAW_PIXEL_HI;
				end
			end
			S_DRAW_PIXEL_HI: begin
				if (sourcex_r < rle_end_r && dx_r != 32'h0) begin
					if (x_cursor >= 18'sd0 && x_cursor < 18'sd336 && lut_hi[7:0] != 8'h0) begin
						fb_wr_en   <= 1'b1;
						fb_wr_data <= {8'h0, lut_hi[7:0]} + palettebase_r;
						fb_wr_x    <= x_cursor[8:0];
					end
					x_cursor  <= x_cursor + {{15{x_dir[2]}}, x_dir};
					sourcex_r <= sourcex_r + dx_r;
				end else if (entry_idx_r + 24'd1 == {8'h0, entry_count_r}) begin
					state <= S_DRAW_ROW_NEXT;
				end else begin
					entry_idx_r  <= entry_idx_r + 24'd1;
					rom_req_word <= row_ptr_r + 24'd2 + entry_idx_r;
					return_state <= S_DRAW_ROW_ENTRY_GOT;
					state        <= S_ROM_FETCH;
				end
			end

			S_DRAW_ROW_NEXT: begin
				if (cur_y == y_hi[8:0]) begin
					state <= S_DRAW_OBJ_NEXT;
				end else begin
					cur_y         <= cur_y + 9'd1;
					sourcey_r     <= sourcey_r + dy_r;
					x_cursor      <= hflip_r ? ex_r[17:0] : sx_r[17:0];
					state         <= S_FB_ROW_OPEN;
				end
			end

			S_DRAW_OBJ_NEXT: begin
				if (objnum_cnt == 8'd0) begin
					if (order_cnt == 8'd255) begin
						// last object of the frame: flush any still-open
						// assembly row before going idle
						if (asm_open) begin
							return_state <= S_IDLE;
							flush_qw_idx <= 7'd0;
							state        <= S_FB_FLUSH_SCAN;
						end else begin
							state <= S_IDLE;
						end
					end else begin
						order_cnt  <= order_cnt + 8'd1;
						objnum_cnt <= 8'd255;
						state      <= S_DRAW_SORT_RD;
					end
				end else begin
					objnum_cnt <= objnum_cnt - 8'd1;
					state      <= S_DRAW_SORT_RD;
				end
			end

			default: state <= S_IDLE;
			endcase
		end
	end

	// ------------------------------------------------------------------
	// single DDR3 owner / grant
	// ------------------------------------------------------------------
	// what each non-prefetch issuer wants this cycle, independent of who
	// currently owns the bus (the draw/prescan FSM is single-threaded, so
	// at most one of these is ever true at a time)
	wire rom_want   = (state == S_ROM_FETCH) && !(rom_line_valid && rom_line_tag == rom_req_word[24:5]);
	wire tabwr_want = (state == S_PS2_TABWR);
	wire whtab_want = (state == S_DRAW_WHTAB_REQ);
	wire flush_want = (state == S_FB_FLUSH_QW);

	// grant priority: prefetch first (hard hblank deadline). A grant is
	// given only when nothing is latched/pending and no owner holds the
	// bus; it does NOT depend on ddr_busy (busy may be high for reasons
	// unrelated to this module - the request register below just waits
	// it out once latched, the Avalon way)
	always_comb begin
		ddr_grant = OWN_NONE;
		if (!req_pending && ddr_owner == OWN_NONE) begin
			if (pf_want_bus)        ddr_grant = OWN_PREFETCH;
			else if (rom_want)      ddr_grant = OWN_ROM;
			else if (tabwr_want)    ddr_grant = OWN_TABWR;
			else if (whtab_want)    ddr_grant = OWN_WHTAB;
			else if (flush_want)    ddr_grant = OWN_FLUSH;
		end
	end

	// a granted read keeps ownership until this state machine's own
	// data-arrived logic clears it (S_ROM_FETCH_WAIT/S_DRAW_WHTAB_WAIT/the
	// prefetch process, all gated on ddr_owner==OWN_SELF); a granted write
	// has no data phase and never becomes the persisted owner at all
	wire pf_owner_done = pf_active && pf_row_ok && ddr_owner == OWN_PREFETCH &&
						  ddr_dout_ready && pf_burst_qw == 4'd1;
	wire rom_owner_done = (state == S_ROM_FETCH_WAIT) && ddr_owner == OWN_ROM &&
						   ddr_dout_ready && rom_burst_cnt == 3'd7;
	wire whtab_owner_done = (state == S_DRAW_WHTAB_WAIT) && ddr_owner == OWN_WHTAB && ddr_dout_ready;

	always_ff @(posedge clk) begin
		if (reset) begin
			ddr_owner <= OWN_NONE;
		end else begin
			case (ddr_owner)
				OWN_NONE: begin
					// set only at acceptance, not at grant: a write is
					// done the instant it is accepted (no data phase), a
					// read becomes the owner from here until its data
					// arrives
					if (req_accepted && (req_owner == OWN_ROM || req_owner == OWN_WHTAB || req_owner == OWN_PREFETCH))
						ddr_owner <= req_owner;
				end
				OWN_ROM:      if (rom_owner_done)   ddr_owner <= OWN_NONE;
				OWN_WHTAB:    if (whtab_owner_done) ddr_owner <= OWN_NONE;
				OWN_PREFETCH: if (pf_owner_done)    ddr_owner <= OWN_NONE;
				default: ddr_owner <= OWN_NONE;
			endcase
		end
	end

	// request register: latched on grant, held stable (address/burstcnt/
	// we/rd/din/be unchanged) until a cycle where ddr_busy is observed
	// low while the request is still asserted - that cycle is the
	// acceptance (req_accepted above), not the grant. Only one request is
	// ever latched at a time (the grant above never fires while
	// req_pending), so this single register set covers every issuer.
	always_ff @(posedge clk) begin
		if (reset) begin
			req_pending <= 1'b0;
			ddr_rd      <= 1'b0;
			ddr_we      <= 1'b0;
		end else if (req_pending) begin
			if (req_accepted) begin
				req_pending <= 1'b0;
				ddr_rd      <= 1'b0;
				ddr_we      <= 1'b0;
			end
			// else: hold ddr_addr/rd/we/burstcnt/din/be exactly as latched
		end else if (ddr_grant != OWN_NONE) begin
			req_pending <= 1'b1;
			req_owner   <= ddr_grant;
			case (ddr_grant)
				OWN_PREFETCH: begin
					ddr_addr     <= pf_ddr_addr;
					ddr_rd       <= 1'b1;
					ddr_burstcnt <= (pf_burst_left >= 7'd8) ? 8'd8 : {1'b0, pf_burst_left};
					ddr_be       <= 8'hff;
				end
				OWN_ROM: begin
					ddr_addr     <= ROM_WORD_BASE + {rom_req_word[24:5], 3'b0};
					ddr_rd       <= 1'b1;
					ddr_burstcnt <= 8'd8;
					ddr_be       <= 8'hff;
				end
				OWN_TABWR: begin
					ddr_addr     <= TABLE_WORD_BASE + (obj_idx >> 1);
					ddr_we       <= 1'b1;
					ddr_burstcnt <= 8'd1; // Avalon wants 1, not 0, for a single-word write
					if (obj_idx[0]) begin
						ddr_din <= {{5'h0, height_cnt}, width_max[15:0], 32'h0};
						ddr_be  <= 8'hf0;
					end else begin
						ddr_din <= {32'h0, {5'h0, height_cnt}, width_max[15:0]};
						ddr_be  <= 8'h0f;
					end
				end
				OWN_WHTAB: begin
					ddr_addr     <= TABLE_WORD_BASE + ({9'h0, code_r} >> 1);
					ddr_rd       <= 1'b1;
					ddr_burstcnt <= 8'd1;
					ddr_be       <= 8'hff;
				end
				OWN_FLUSH: begin
					ddr_addr     <= (asm_buf ? FB1_QW_BASE : FB0_QW_BASE) +
									{14'h0, asm_row, 7'b0} + {22'h0, flush_qw_idx};
					ddr_we       <= 1'b1;
					ddr_burstcnt <= 8'd1;
					// dirty lanes get their real (just-flushed) value,
					// clean lanes 0: on a first-touch row be=0xff writes
					// every lane, so the clean ones must be background,
					// not stale data; on a partial flush be already masks
					// the clean lanes off
					ddr_din <= {flush_dirty_r[3] ? flush_p3_r : 16'h0,
								flush_dirty_r[2] ? flush_p2_r : 16'h0,
								flush_dirty_r[1] ? flush_p1_r : 16'h0,
								flush_dirty_r[0] ? flush_p0_r : 16'h0};
					ddr_be <= flush_be_r;
				end
				default: ;
			endcase
		end
	end

endmodule
