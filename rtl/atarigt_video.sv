// Atari GT playfield/alpha tilemap renderer, colour RAM and mixer.
// Reference: MAME atarigt_v.cpp (get_playfield_tile_info, get_alpha_tile_info,
// playfield_scan, scanline_update, screen_update Primal Rage branch) and
// MAME atarigt.cpp (pflayout, pftoplayout).
//
// Two related clocks: clk = clk_sys (85.909 MHz)
// runs timing, the renderer, the mixer, the line buffers and cram port B
// (tdp_ram's clk_b); clk_cpu = clk_sys/2, phase aligned, runs everything the
// CPU alone touches: vram/cram tdp_ram port A, tram/the MRAM shadows (plain
// single-clock arrays, nobody else reads them), the registered hit-flag +
// cram_dout select stage, vram_dout, and the colour_latch/mram_rg/mram_b
// write side (their read side is combinational in the clk-domain mixer, so
// reading a clk_cpu-written static register from clk needs no synchronizer,
// same as the other control-word crossings).
//
// cram_addr CPU read latency: 2 clk_cpu clocks (array read valid 1 clock
// after the address, then the registered-hit-flag select of cram/tram/
// latch/mrg/mb valid 1 clock after that) -- matches cpu_bus's existing
// ST_DELAY budget for a cram read exactly, so no change needed there.
//
// Pixel pipeline latency: 4 ce_pix cycles from address issue (hcnt_raw) to
// the registered r/g/b output (line-buffer read, mux the two buffers' reads,
// cram lookup, mram lookup + white override -> one ce_pix cycle each). The
// line-buffer read and the buf0/buf1 select are two separate stages so each
// array read stays a plain `q <= mem[addr]` with nothing else on the RHS
// (Quartus needs that shape to infer M10K; a muxed read like
// `sel ? buf1[i] : buf0[i]` synthesizes as flops instead). hblank/vblank/
// hsync/vsync outputs are delayed the same 4 ce_pix so they land with r/g/b
// (pixel 0 coincides with hblank falling). hcnt, vcnt and vblank_rise are
// the raw, undelayed counters (used for IRQ/scanline
// timing, not pixel-matched).

module atarigt_video (
	input               clk,     // clk_sys, 85.909 MHz: timing, renderer, mixer, line buffers
	input               clk_cpu, // clk_sys/2, phase aligned: CPU-only memories and their pipeline
	input               reset,

	output              ce_pix,
	output              hblank,
	output              vblank,
	output              hsync,
	output              vsync,
	output       [8:0]  hcnt,
	output       [8:0]  vcnt,
	output              vblank_rise,

	// CPU port to VRAM (32K x 16, byte lanes)
	input        [15:1] vram_addr,
	input        [15:0] vram_din,
	input        [1:0]  vram_we,
	output       [15:0] vram_dout,

	// CPU port to colour RAM (CRAM/TRAM/MRAM/latch, 512KB word space)
	input        [18:1] cram_addr,
	input        [15:0] cram_din,
	input        [1:0]  cram_we,
	output       [15:0] cram_dout,

	// Tile / alpha char ROM port (sdram.sv toggle protocol)
	output       [24:1] rom_addr,
	output              rom_req,
	input               rom_ack,
	input        [15:0] rom_dout,

	// MO frame buffer read (buffer lives in rle_objects, 1 cycle read latency)
	output       [8:0]  fb_rd_x,
	output       [7:0]  fb_rd_y,
	input        [15:0] fb_rd_data,
	input               frame_sel,

	input        [15:0] latch,

	output       [7:0]  r,
	output       [7:0]  g,
	output       [7:0]  b
);

	// buffer selection lives in rle_objects; kept for interface parity only
	wire unused_ok = &{1'b0, frame_sel, latch};

	// ------------------------------------------------------------------
	// Video timing: 456 x 262, visible 336 x 240
	// ------------------------------------------------------------------
	localparam H_VIS  = 336;
	localparam H_FP   = 8;
	localparam H_SYNC = 40;
	localparam H_TOT  = 456;

	localparam V_VIS  = 240;
	localparam V_FP   = 1;
	localparam V_SYNC = 3;
	localparam V_TOT  = 262;

	reg [3:0] pix_div;
	assign ce_pix = (pix_div == 4'd11);

	always @(posedge clk) begin
		if (reset) pix_div <= 4'd0;
		else       pix_div <= (pix_div == 4'd11) ? 4'd0 : pix_div + 4'd1;
	end

	reg [8:0] hcnt_raw, vcnt_raw;
	wire line_end = (hcnt_raw == H_TOT-1);

	always @(posedge clk) begin
		if (reset) begin
			hcnt_raw <= 9'd0;
			vcnt_raw <= 9'd0;
		end else if (ce_pix) begin
			if (line_end) begin
				hcnt_raw <= 9'd0;
				vcnt_raw <= (vcnt_raw == V_TOT-1) ? 9'd0 : vcnt_raw + 9'd1;
			end else begin
				hcnt_raw <= hcnt_raw + 9'd1;
			end
		end
	end

	wire hblank_raw = (hcnt_raw >= H_VIS);
	wire vblank_raw = (vcnt_raw >= V_VIS);
	wire hsync_raw  = ~((hcnt_raw >= H_VIS+H_FP) && (hcnt_raw < H_VIS+H_FP+H_SYNC));
	wire vsync_raw  = ~((vcnt_raw >= V_VIS+V_FP) && (vcnt_raw < V_VIS+V_FP+V_SYNC));

	reg vblank_d;
	always @(posedge clk) begin
		if (reset) vblank_d <= 1'b0;
		else if (ce_pix) vblank_d <= vblank_raw;
	end
	assign vblank_rise = ce_pix & vblank_raw & ~vblank_d;

	assign hcnt = hcnt_raw;
	assign vcnt = vcnt_raw;

	// hblank/vblank/hsync/vsync delayed 4 ce_pix to land with r/g/b (see
	// the mixer pipeline below); hcnt/vcnt above stay raw on purpose.
	reg hblank_s1, hblank_s2, hblank_s3, hblank_s4;
	reg vblank_s1, vblank_s2, vblank_s3, vblank_s4;
	reg hsync_s1, hsync_s2, hsync_s3, hsync_s4;
	reg vsync_s1, vsync_s2, vsync_s3, vsync_s4;

	always @(posedge clk) begin
		if (ce_pix) begin
			hblank_s1 <= hblank_raw; hblank_s2 <= hblank_s1; hblank_s3 <= hblank_s2; hblank_s4 <= hblank_s3;
			vblank_s1 <= vblank_raw; vblank_s2 <= vblank_s1; vblank_s3 <= vblank_s2; vblank_s4 <= vblank_s3;
			hsync_s1  <= hsync_raw;  hsync_s2  <= hsync_s1;  hsync_s3  <= hsync_s2;  hsync_s4  <= hsync_s3;
			vsync_s1  <= vsync_raw;  vsync_s2  <= vsync_s1;  vsync_s3  <= vsync_s2;  vsync_s4  <= vsync_s3;
		end
	end

	assign hblank = hblank_s4;
	assign vblank = vblank_s4;
	assign hsync  = hsync_s4;
	assign vsync  = vsync_s4;

	// ------------------------------------------------------------------
	// VRAM: 32K x 16, port A (CPU, byte enables), port B (renderer, read
	// only). Quartus 17 refuses to recognize an inferred array's second
	// (read-only) access as a true dual-port partner and instead duplicates
	// the whole array once per reader, so vram_lo/vram_hi are each a real
	// tdp_ram (rtl/tdp_ram.sv) instance: a behavioural model for simulation,
	// an explicit altsyncram BIDIR_DUAL_PORT M10K for synthesis. Both ports
	// register their read, 1 clock, same as the rest of this file.
	// ------------------------------------------------------------------
	// renderer read port address: driven combinationally from the FSM state
	// below, so the port B read becomes valid exactly one clock after the
	// state that drives the address (read it in the following state).
	reg [15:1] vram2_addr_c;

	wire [7:0] vram_lo_qa, vram_hi_qa; // port A (CPU) read
	wire [7:0] vram_lo_qb, vram_hi_qb; // port B (renderer) read

	tdp_ram #(.AW(15), .DW(8)) u_vram_lo (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(vram_addr), .din_a(vram_din[7:0]), .we_a(vram_we[0]), .q_a(vram_lo_qa),
		.addr_b(vram2_addr_c), .din_b(8'h00), .we_b(1'b0), .q_b(vram_lo_qb)
	);
	tdp_ram #(.AW(15), .DW(8)) u_vram_hi (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(vram_addr), .din_a(vram_din[15:8]), .we_a(vram_we[1]), .q_a(vram_hi_qa),
		.addr_b(vram2_addr_c), .din_b(8'h00), .we_b(1'b0), .q_b(vram_hi_qb)
	);

	assign vram_dout = {vram_hi_qa, vram_lo_qa};
	wire [15:0] vram2_dout = {vram_hi_qb, vram_lo_qb};

	// ------------------------------------------------------------------
	// Colour RAM: CRAM 16K, TRAM 16K, colour latch, MRAM 4x32 R/G and B.
	// CRAM/TRAM/the two MRAM shadows are each split into byte-wide lo/hi
	// arrays with one plain `if (we) mem[idx] <= byte;` write per array and
	// one plain `q <= mem[idx]` read per array (own always blocks, nothing
	// else on the read RHS) so Quartus infers M10K for each; a byte-sliced
	// write into a 16-bit array, or a read `if/else` chain selecting among
	// several arrays into one register, both defeat BRAM inference.
	// mram_rg/mram_b (128 entries, mixer-only) are small enough to stay as
	// plain registered arrays with a combinational read.
	// ------------------------------------------------------------------
	(* ramstyle = "M10K" *) reg [7:0] tram_lo [0:16383], tram_hi [0:16383];
	reg [15:0] mram_rg  [0:127];   // {R[7:0],G[7:0]} per {bank[1:0],value[4:0]}, mixer-only
	reg [15:0] mram_b   [0:127];   // blue word, low byte is the pen level, mixer-only
	reg [15:0] color_latch;

	// colorram_r in MAME returns whatever was last written at ANY word in
	// 0x20000-0x27fff/0x30000-0x37fff, not just the 128 entries the mixer
	// cares about. A full 32K x 16 x 2 shadow is too much BRAM (and 8K words
	// per channel was still over the M10K block budget), so alias on a
	// 10-bit index only (1K words each, ~0.03 Mbit total) for CPU readback;
	// {bank, a[7:0]} keeps the 128 real (qualifying) entries distinct since
	// their a[12:5] is always 0, it just aliases the rest.
	(* ramstyle = "M10K" *) reg [7:0] mrg_shadow_lo [0:1023], mrg_shadow_hi [0:1023];
	(* ramstyle = "M10K" *) reg [7:0] mb_shadow_lo  [0:1023], mb_shadow_hi  [0:1023];

	wire cram_hit  = (cram_addr < 18'h10000);
	wire tram_hit  = (cram_addr >= 18'h10000) && (cram_addr < 18'h14000);
	wire latch_hit = (cram_addr == 18'h18000);
	wire mrg_hit   = (cram_addr >= 18'h20000) && (cram_addr < 18'h28000);
	wire mb_hit    = (cram_addr >= 18'h30000) && (cram_addr < 18'h38000);

	// cram_addr is numbered [18:1] (word address taken straight from CPU byte
	// address bits, bit 0 does not exist), so every slice below is +1 vs. a
	// plain 0-based word index.
	wire [13:0] cram_idx = cram_addr[14:1];
	wire [13:0] tram_idx = cram_addr[14:1];

	// only entries where (a & 0x1fe0) == 0 matter: bits 12:5 of the (0-based) address
	wire mrg_qual = mrg_hit && (cram_addr[13:6] == 8'h00);
	wire mb_qual  = mb_hit  && (cram_addr[13:6] == 8'h00);
	wire [6:0]  mram_index  = {cram_addr[15:14], cram_addr[5:1]};
	wire [9:0] mram_shadow_idx = {cram_addr[15:14], cram_addr[8:1]}; // bank bits + a[7:0], rest aliased

	// cram_lo/cram_hi have two readers (the CPU port above and the mixer's
	// cram2 port below); same Quartus duplication problem as VRAM, same
	// fix: a real tdp_ram (rtl/tdp_ram.sv) instance per byte array.
	reg  [13:0] cram2_addr_c;
	wire [7:0] cram_lo_q, cram_hi_q;         // port A (CPU)
	wire [7:0] cram2_lo_q, cram2_hi_q;       // port B (mixer)

	tdp_ram #(.AW(14), .DW(8)) u_cram_lo (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(cram_idx), .din_a(cram_din[7:0]), .we_a(cram_hit && cram_we[0]), .q_a(cram_lo_q),
		.addr_b(cram2_addr_c), .din_b(8'h00), .we_b(1'b0), .q_b(cram2_lo_q)
	);
	tdp_ram #(.AW(14), .DW(8)) u_cram_hi (
		.clk_a(clk_cpu), .clk_b(clk),
		.addr_a(cram_idx), .din_a(cram_din[15:8]), .we_a(cram_hit && cram_we[1]), .q_a(cram_hi_q),
		.addr_b(cram2_addr_c), .din_b(8'h00), .we_b(1'b0), .q_b(cram2_hi_q)
	);
	wire [15:0] cram2_dout = {cram2_hi_q, cram2_lo_q};

	// tram/mrg_shadow/mb_shadow have only the one (CPU) reader, so a plain
	// write-array/read-array pair of always blocks per array is already a
	// single-port BRAM, no duplication risk. Nobody outside the CPU domain
	// touches them, so they (and their read registers) run on clk_cpu.
	reg [7:0] tram_lo_q, tram_hi_q;
	always @(posedge clk_cpu) if (tram_hit && cram_we[0]) tram_lo[tram_idx] <= cram_din[7:0];
	always @(posedge clk_cpu) if (tram_hit && cram_we[1]) tram_hi[tram_idx] <= cram_din[15:8];
	always @(posedge clk_cpu) tram_lo_q <= tram_lo[tram_idx];
	always @(posedge clk_cpu) tram_hi_q <= tram_hi[tram_idx];

	reg [7:0] mrg_lo_q, mrg_hi_q, mb_lo_q, mb_hi_q;
	always @(posedge clk_cpu) if (mrg_hit && cram_we[0]) mrg_shadow_lo[mram_shadow_idx] <= cram_din[7:0];
	always @(posedge clk_cpu) if (mrg_hit && cram_we[1]) mrg_shadow_hi[mram_shadow_idx] <= cram_din[15:8];
	always @(posedge clk_cpu) if (mb_hit && cram_we[0]) mb_shadow_lo[mram_shadow_idx] <= cram_din[7:0];
	always @(posedge clk_cpu) if (mb_hit && cram_we[1]) mb_shadow_hi[mram_shadow_idx] <= cram_din[15:8];
	always @(posedge clk_cpu) mrg_lo_q <= mrg_shadow_lo[mram_shadow_idx];
	always @(posedge clk_cpu) mrg_hi_q <= mrg_shadow_hi[mram_shadow_idx];
	always @(posedge clk_cpu) mb_lo_q  <= mb_shadow_lo[mram_shadow_idx];
	always @(posedge clk_cpu) mb_hi_q  <= mb_shadow_hi[mram_shadow_idx];

	// color_latch/mram_rg/mram_b are written here on clk_cpu; the mixer
	// reads mram_rg/mram_b combinationally and color_latch directly (both
	// in the clk domain) -- reading a clk_cpu-written static register from
	// clk needs no synchronizer (related clocks, control words).
	reg [15:0] latch_q;
	always @(posedge clk_cpu) begin
		if (latch_hit && cram_we[0]) color_latch[7:0]  <= cram_din[7:0];
		if (latch_hit && cram_we[1]) color_latch[15:8] <= cram_din[15:8];
		if (mrg_qual && cram_we[0]) mram_rg[mram_index][7:0]  <= cram_din[7:0];
		if (mrg_qual && cram_we[1]) mram_rg[mram_index][15:8] <= cram_din[15:8];
		if (mb_qual  && cram_we[0]) mram_b[mram_index][7:0]  <= cram_din[7:0];
		if (mb_qual  && cram_we[1]) mram_b[mram_index][15:8] <= cram_din[15:8];
		latch_q <= color_latch;
	end

	// cpu_bus already waits 2 clk_cpu clocks (ST_DELAY) for a cram read, and
	// this takes exactly that: array read valid 1 clock after the address,
	// the registered-hit-flag select valid 1 clock after that (2 total).
	reg cram_hit_d1, tram_hit_d1, latch_hit_d1, mrg_hit_d1, mb_hit_d1;
	always @(posedge clk_cpu) begin
		cram_hit_d1  <= cram_hit;
		tram_hit_d1  <= tram_hit;
		latch_hit_d1 <= latch_hit;
		mrg_hit_d1   <= mrg_hit;
		mb_hit_d1    <= mb_hit;
	end

	reg [15:0] cram_dout_r;
	assign cram_dout = cram_dout_r;

	always @(posedge clk_cpu) begin
		if (cram_hit_d1)       cram_dout_r <= {cram_hi_q, cram_lo_q};
		else if (tram_hit_d1)  cram_dout_r <= {tram_hi_q, tram_lo_q};
		else if (latch_hit_d1) cram_dout_r <= latch_q;
		else if (mrg_hit_d1)   cram_dout_r <= {mrg_hi_q, mrg_lo_q};
		else if (mb_hit_d1)    cram_dout_r <= {mb_hi_q, mb_lo_q};
		else                   cram_dout_r <= 16'h0000;
	end

	// ------------------------------------------------------------------
	// Line buffers: PF (14 bit, keeps the arithmetic-overflow bit used by
	// the white override) and AN (8 bit), two buffers ping-ponged per line.
	// ------------------------------------------------------------------
	(* ramstyle = "M10K" *) reg [13:0] pf_buf0 [0:335];
	(* ramstyle = "M10K" *) reg [13:0] pf_buf1 [0:335];
	(* ramstyle = "M10K" *) reg [7:0]  an_buf0 [0:335];
	(* ramstyle = "M10K" *) reg [7:0]  an_buf1 [0:335];

	reg disp_sel;   // which buffer the mixer currently displays
	reg fsm_wr_sel; // which buffer the renderer currently fills

	// ------------------------------------------------------------------
	// Playfield / alpha scroll state (persists across lines until updated)
	// ------------------------------------------------------------------
	reg [9:0] xscroll;
	reg [8:0] yscroll;
	reg [3:0] tile_bank;
	reg [4:0] color_bank;

	// ------------------------------------------------------------------
	// Renderer FSM: fills the line buffer for target_line one line ahead
	// ------------------------------------------------------------------
	localparam S_IDLE          = 5'd0;
	localparam S_SCROLL_XADDR  = 5'd1;
	localparam S_SCROLL_XWAIT  = 5'd2;
	localparam S_SCROLL_YADDR  = 5'd3;
	localparam S_SCROLL_YWAIT  = 5'd4;
	localparam S_PF_MAPADDR    = 5'd5;
	localparam S_PF_MAPWAIT    = 5'd6;
	localparam S_PF_ROM0_WAIT  = 5'd7;
	localparam S_PF_ROM1_WAIT  = 5'd8;
	localparam S_PF_ROM2_WAIT  = 5'd9;
	localparam S_PF_WRITE      = 5'd10;
	localparam S_PF_NEXT       = 5'd11;
	localparam S_AN_MAPADDR    = 5'd12;
	localparam S_AN_MAPWAIT    = 5'd13;
	localparam S_AN_ROM0_WAIT  = 5'd14;
	localparam S_AN_ROM1_WAIT  = 5'd15;
	localparam S_AN_WRITE      = 5'd16;
	localparam S_AN_NEXT       = 5'd17;
	localparam S_DONE          = 5'd18;
	localparam S_PF_ROM0_SET   = 5'd19;
	localparam S_AN_ROM0_SET   = 5'd20;

	reg [4:0] state;
	reg [8:0] target_line;
	reg [5:0] pf_col;   // 0..42
	reg [5:0] an_col;   // 0..41

	localparam PF_MAP_WBASE   = 15'h1000;
	localparam ALPHA_WBASE    = 15'h3000;
	localparam PF_R0_WBASE    = 24'h200000;
	localparam PF_R1_WBASE    = 24'h280000;
	localparam PF_R2_WBASE    = 24'h300000;
	localparam AN_ROM_WBASE   = 24'h380000;

	// scroll table addresses for target_line: offset = ((y & ~7)<<3)+48+2*(y&7)
	wire [8:0]  grp           = {target_line[8:3], 3'b000};
	wire [12:0] scroll_x_off  = {grp, 3'b000} + 13'd48 + {8'b0, target_line[2:0], 1'b0};
	wire [12:0] scroll_y_off  = scroll_x_off + 13'd1;
	wire        scroll_in_rng = (scroll_x_off < 13'h800);

	reg [15:0] scroll_x_word;

	// row/col for the whole line, constant once xscroll/yscroll are settled
	wire [8:0] pf_ysum = target_line + yscroll; // 9-bit wraparound add (mod 512)
	wire [5:0] pf_row  = pf_ysum[8:3];
	wire [2:0] pf_r    = pf_ysum[2:0];

	wire [7:0] pf_colsum  = {1'b0, xscroll[9:3]} + {2'b0, pf_col};
	wire [6:0] pf_col_eff = pf_colsum[6:0];
	wire [12:0] pf_map_idx = {~pf_col_eff[6], pf_row[5:0], pf_col_eff[5:0]};

	// target_line can reach 261+2=263 (one/two lines past the last visible
	// line); clamp so the alpha row never runs past the 32-row alpha map.
	wire [5:0] an_row_raw = target_line[8:3];
	wire [5:0] an_row     = (an_row_raw > 6'd31) ? 6'd31 : an_row_raw;
	wire [11:0] an_map_idx = {an_row[5:0], an_col[5:0]};

	reg [15:0] pf_tile_word, an_tile_word;
	wire [11:0] pf_code   = pf_tile_word[11:0];
	wire [2:0]  pf_colour = pf_tile_word[14:12];
	wire        pf_flip   = pf_tile_word[15];
	wire [15:0] pf_T      = {tile_bank, pf_code};

	wire [11:0] an_code   = an_tile_word[11:0];
	wire [3:0]  an_colour = an_tile_word[15:12];

	wire [23:0] pf_rom_off   = {pf_T, 3'b000} + {21'b0, pf_r};
	wire [23:0] pf_rom0_addr = PF_R0_WBASE + pf_rom_off;
	wire [23:0] pf_rom1_addr = PF_R1_WBASE + pf_rom_off;
	wire [23:0] pf_rom2_addr = PF_R2_WBASE + pf_rom_off;

	wire [23:0] an_rom0_addr = AN_ROM_WBASE + {an_code, 4'b0000} + {19'b0, target_line[2:0], 1'b0};
	wire [23:0] an_rom1_addr = an_rom0_addr + 24'd1;

	reg [7:0] pf_r0h, pf_r0l, pf_r1h, pf_r1l, pf_r2h, pf_r2l;
	reg [7:0] an_b0, an_b1, an_b2, an_b3;

	reg [24:1] rom_addr_r;
	reg        rom_req_r;
	assign rom_addr = rom_addr_r;
	assign rom_req  = rom_req_r;
	wire       rom_done = (rom_ack == rom_req_r);

	// renderer VRAM read address: combinational mux on state
	always @(*) begin
		case (state)
			S_SCROLL_XADDR: vram2_addr_c = ALPHA_WBASE + scroll_x_off[12:0];
			S_SCROLL_YADDR: vram2_addr_c = ALPHA_WBASE + scroll_y_off[12:0];
			S_PF_MAPADDR:   vram2_addr_c = PF_MAP_WBASE + pf_map_idx;
			S_AN_MAPADDR:   vram2_addr_c = ALPHA_WBASE + {3'b0, an_map_idx};
			default:        vram2_addr_c = 15'h0000;
		endcase
	end

	function automatic [5:0] pf_pixel_bits;
		input [7:0] r0h, r0l, r1h, r1l, r2h, r2l;
		input [2:0] idx;
		reg   half;
		reg [2:0] j;
		reg [7:0] nib_byte;
		reg [7:0] bit_byte;
		begin
			half = idx[2];
			j    = idx[1:0];
			nib_byte = (j[1] == 1'b0) ? (half ? r1l : r1h) : (half ? r0l : r0h);
			bit_byte = half ? r2l : r2h;
			pf_pixel_bits = { bit_byte[4+j], bit_byte[j],
							   (j[0] ? nib_byte[3:0] : nib_byte[7:4]) };
		end
	endfunction

	function automatic [3:0] an_pixel_bits;
		input [7:0] b0, b1, b2, b3;
		input [2:0] idx;
		begin
			case (idx)
				3'd0: an_pixel_bits = b0[7:4];
				3'd1: an_pixel_bits = b0[3:0];
				3'd2: an_pixel_bits = b1[7:4];
				3'd3: an_pixel_bits = b1[3:0];
				3'd4: an_pixel_bits = b2[7:4];
				3'd5: an_pixel_bits = b2[3:0];
				3'd6: an_pixel_bits = b3[7:4];
				default: an_pixel_bits = b3[3:0];
			endcase
		end
	endfunction

	reg [2:0]  wk; // sub-pixel counter for S_PF_WRITE/S_AN_WRITE, 0..7
	reg [2:0]  src_idx;
	reg [5:0]  pix6;
	reg [3:0]  pix4;
	reg [13:0] pf_val;
	reg signed [10:0] wx;

	always @(posedge clk) begin
		if (reset) begin
			state       <= S_IDLE;
			rom_req_r   <= 1'b0;
			disp_sel    <= 1'b0;
			fsm_wr_sel  <= 1'b1;
			xscroll     <= 10'd0;
			yscroll     <= 9'd0;
			tile_bank   <= 4'd0;
			color_bank  <= 5'd0;
			target_line <= 9'd0;
			pf_col      <= 6'd0;
			an_col      <= 6'd0;
			wk          <= 3'd0;
		end else if (ce_pix && line_end && (state == S_DONE || rom_done)) begin
			// kick off a fresh render pass at every line boundary, but never
			// abandon an outstanding ROM request (toggle without a matching
			// ack would desync the sdram.sv handshake); if a request is still
			// in flight, skip this boundary and catch the next one instead
			disp_sel    <= ~disp_sel;
			fsm_wr_sel  <= disp_sel;
			// wrap at 262 lines: lines 0 and 1 are rendered during vblank lines 260 and 261,
			// and their scroll entries (line 0 carries the frame values) must be read
			target_line <= (vcnt_raw >= 9'd260) ? vcnt_raw - 9'd260 : vcnt_raw + 9'd2;
			pf_col      <= 6'd0;
			state       <= S_SCROLL_XADDR;
		end else begin
			case (state)
				S_IDLE: ; // wait for next line boundary

				S_SCROLL_XADDR: state <= scroll_in_rng ? S_SCROLL_XWAIT : S_PF_MAPADDR;
				S_SCROLL_XWAIT: begin
					scroll_x_word <= vram2_dout;
					state <= S_SCROLL_YADDR;
				end
				S_SCROLL_YADDR: state <= S_SCROLL_YWAIT;
				S_SCROLL_YWAIT: begin
					if (scroll_x_word[15]) begin
						xscroll    <= scroll_x_word[14:5];
						color_bank <= scroll_x_word[4:0];
					end
					if (vram2_dout[15]) begin
						yscroll   <= (vram2_dout[14:6] - target_line[8:0]) & 9'h1ff;
						tile_bank <= vram2_dout[3:0];
					end
					state <= S_PF_MAPADDR;
				end

				S_PF_MAPADDR: state <= S_PF_MAPWAIT;
				S_PF_MAPWAIT: begin
					pf_tile_word <= vram2_dout;
					state <= S_PF_ROM0_SET;
				end
				// one cycle for pf_tile_word (and pf_T/pf_r derived from it) to settle
				S_PF_ROM0_SET: begin
					rom_addr_r <= pf_rom0_addr;
					rom_req_r  <= ~rom_req_r;
					state <= S_PF_ROM0_WAIT;
				end
				S_PF_ROM0_WAIT: if (rom_done) begin
					pf_r0h <= rom_dout[15:8];
					pf_r0l <= rom_dout[7:0];
					rom_addr_r <= pf_rom1_addr;
					rom_req_r  <= ~rom_req_r;
					state <= S_PF_ROM1_WAIT;
				end
				S_PF_ROM1_WAIT: if (rom_done) begin
					pf_r1h <= rom_dout[15:8];
					pf_r1l <= rom_dout[7:0];
					rom_addr_r <= pf_rom2_addr;
					rom_req_r  <= ~rom_req_r;
					state <= S_PF_ROM2_WAIT;
				end
				S_PF_ROM2_WAIT: if (rom_done) begin
					pf_r2h <= rom_dout[15:8];
					pf_r2l <= rom_dout[7:0];
					state <= S_PF_WRITE;
				end
				// one pixel per clock (8 clocks/tile) so pf_buf infers as a
				// plain single-write-port BRAM instead of 8 unrolled writes
				// verilator lint_off BLKSEQ
				S_PF_WRITE: begin
					src_idx = pf_flip ? (3'd7 - wk) : wk;
					pix6 = pf_pixel_bits(pf_r0h, pf_r0l, pf_r1h, pf_r1l, pf_r2h, pf_r2l, src_idx);
					// addition, not OR: colour_bank<<8 + colour<<5 + pixel (can carry into bit13)
					pf_val = {1'b0, color_bank, 8'b0} + {6'b0, pf_colour, 5'b0} + {8'b0, pix6};
					wx = $signed({2'b0, pf_col, 3'b000}) + $signed({8'b0, wk}) - $signed({8'b0, xscroll[2:0]});
					if (wx >= 0 && wx < 336) begin
						if (fsm_wr_sel) pf_buf1[wx[8:0]] <= pf_val;
						else            pf_buf0[wx[8:0]] <= pf_val;
					end
					if (wk == 3'd7) begin
						wk    <= 3'd0;
						state <= S_PF_NEXT;
					end else wk <= wk + 3'd1;
				end
				// verilator lint_on BLKSEQ
				S_PF_NEXT: begin
					if (pf_col == 6'd42) begin
						an_col <= 6'd0;
						state  <= S_AN_MAPADDR;
					end else begin
						pf_col <= pf_col + 6'd1;
						state  <= S_PF_MAPADDR;
					end
				end

				S_AN_MAPADDR: state <= S_AN_MAPWAIT;
				S_AN_MAPWAIT: begin
					an_tile_word <= vram2_dout;
					state <= S_AN_ROM0_SET;
				end
				// one cycle for an_tile_word (and an_code derived from it) to settle
				S_AN_ROM0_SET: begin
					rom_addr_r <= an_rom0_addr;
					rom_req_r  <= ~rom_req_r;
					state <= S_AN_ROM0_WAIT;
				end
				S_AN_ROM0_WAIT: if (rom_done) begin
					an_b0 <= rom_dout[15:8];
					an_b1 <= rom_dout[7:0];
					rom_addr_r <= an_rom1_addr;
					rom_req_r  <= ~rom_req_r;
					state <= S_AN_ROM1_WAIT;
				end
				S_AN_ROM1_WAIT: if (rom_done) begin
					an_b2 <= rom_dout[15:8];
					an_b3 <= rom_dout[7:0];
					state <= S_AN_WRITE;
				end
				// verilator lint_off BLKSEQ
				S_AN_WRITE: begin
					pix4 = an_pixel_bits(an_b0, an_b1, an_b2, an_b3, wk);
					if (fsm_wr_sel) an_buf1[{an_col, 3'b000} + {6'b0, wk}] <= {an_colour, pix4};
					else            an_buf0[{an_col, 3'b000} + {6'b0, wk}] <= {an_colour, pix4};
					if (wk == 3'd7) begin
						wk    <= 3'd0;
						state <= S_AN_NEXT;
					end else wk <= wk + 3'd1;
				end
				// verilator lint_on BLKSEQ
				S_AN_NEXT: begin
					if (an_col == 6'd41) state <= S_DONE;
					else begin
						an_col <= an_col + 6'd1;
						state  <= S_AN_MAPADDR;
					end
				end

				S_DONE: ; // wait for next line boundary

				default: state <= S_IDLE;
			endcase
		end
	end

	// ------------------------------------------------------------------
	// Display / mixer pipeline (4 ce_pix cycles: line-buffer read, buf0/buf1
	// select, cram lookup, mram lookup + white override -> registered r/g/b)
	// ------------------------------------------------------------------
	// fb_rd_x/y must be registered (gated by ce_pix, using the same pre-edge
	// hcnt_raw/vcnt_raw as the buffer reads below) rather than a
	// combinational passthrough of hcnt_raw: with a combinational fb_rd_x, a
	// 1-clock frame buffer read latency settles almost immediately (well
	// inside the 12-clk ce_pix period), so fb_rd_data would track the
	// CURRENT pixel while the playfield/alpha data lags it, landing the MO
	// word to the left of what it is mixed with.
	reg [8:0] fb_rd_x_r;
	reg [7:0] fb_rd_y_r;
	assign fb_rd_x = fb_rd_x_r;
	assign fb_rd_y = fb_rd_y_r;

	// reading past the end of the line during hblank would walk pf_buf/an_buf
	// (sized 0:335) with an index up to 455; clamp so no X leaks into cra_idx.
	// Registered one ce_pix cycle ahead of the reads below so each read stays
	// a plain `q <= mem[idx]` with idx a plain signal, no ternary/clamp in
	// the index expression itself (Quartus needs that exact shape for M10K).
	reg [8:0] disp_rd_idx_r;

	// stage 1: one plain, unconditional (posedge clk only, no other reads or
	// enables in the block) read per buffer. Quartus only infers M10K when
	// the read is the entire RHS of `q <= mem[addr]` in its own always block;
	// muxing buf0[i]/buf1[i] on one RHS, or mixing other assignments into the
	// same block, both synthesize as flops instead.
	reg [13:0] pf_buf0_q, pf_buf1_q;
	reg [7:0]  an_buf0_q, an_buf1_q;
	reg        disp_sel_d1;
	reg        de_d1;

	always @(posedge clk) pf_buf0_q <= pf_buf0[disp_rd_idx_r];
	always @(posedge clk) pf_buf1_q <= pf_buf1[disp_rd_idx_r];
	always @(posedge clk) an_buf0_q <= an_buf0[disp_rd_idx_r];
	always @(posedge clk) an_buf1_q <= an_buf1[disp_rd_idx_r];

	always @(posedge clk) begin
		if (ce_pix) begin
			fb_rd_x_r     <= hcnt_raw;
			fb_rd_y_r     <= vcnt_raw[7:0];
			disp_rd_idx_r <= hblank_raw ? 9'd0 : hcnt_raw;
			disp_sel_d1   <= disp_sel;
			de_d1         <= ~hblank_raw & ~vblank_raw;
		end
	end

	// stage 2: select between the two (already registered) buffer reads
	// with the (already registered) select, and delay fb_rd_data by the one
	// extra ce_pix cycle stage 1 added so it still lines up with pf_s1/an_s1
	reg [13:0] pf_s1;
	reg [7:0]  an_s1;
	reg [15:0] mo_d1;
	reg        de_s1;

	always @(posedge clk) begin
		if (ce_pix) begin
			pf_s1 <= disp_sel_d1 ? pf_buf1_q : pf_buf0_q;
			an_s1 <= disp_sel_d1 ? an_buf1_q : an_buf0_q;
			mo_d1 <= fb_rd_data;
			de_s1 <= de_d1;
		end
	end

	// cra selection (Primal Rage branch of screen_update)
	wire [2:0] pfpri = pf_s1[12:10];
	wire [2:0] mopri = mo_d1[14:12];
	wire       mgep  = (mopri >= pfpri) && !pfpri[2];

	wire an_opaque  = |(an_s1 & 8'h8f);
	wire mo_nonzero = |mo_d1[5:0];
	wire mo_wins    = mo_nonzero && (mo_d1[11] || mgep || (pf_s1[5:0] == 6'b0));

	wire [12:0] cra_idx = an_opaque ? {5'b0, an_s1} :
						   mo_wins  ? {2'b10, mo_d1[10:0]} :
									  {1'b0, pf_s1[11:0]};

	// registered (not combinational) so it only changes once per ce_pix: the
	// cram2 array read below is now unconditional/fast, so if the address
	// itself weren't frozen between ce_pix edges the read would keep
	// chasing pf_s1's latest value and reach cram2_dout a full stage ahead
	// of pf_s2 (which is ce_pix-gated), instead of holding still until
	// pf_s2 catches up to the same snapshot.
	always @(posedge clk) if (ce_pix) cram2_addr_c <= {color_latch[3], cra_idx};

	// stage 3: cram2_dout (declared with the colour RAM above) captures
	// cram2_addr_c here; carry pf_s1/de_s1 forward alongside it
	reg [13:0] pf_s2;
	reg        de_s2;
	always @(posedge clk) begin
		if (ce_pix) begin
			pf_s2 <= pf_s1;
			de_s2 <= de_s1;
		end
	end

	wire [4:0] cr5 = cram2_dout[14:10];
	wire [4:0] cg5 = cram2_dout[9:5];
	wire [4:0] cb5 = cram2_dout[4:0];
	wire [1:0] mbank = color_latch[7:6];

	wire [7:0] mix_r = mram_rg[{mbank, cr5}][15:8];
	wire [7:0] mix_g = mram_rg[{mbank, cg5}][7:0];
	wire [7:0] mix_b = mram_b [{mbank, cb5}][7:0];

	wire white_ov = (color_latch[2:0] != 3'b0) &&
					((pf_s2[5:0] == 6'b0) || (pf_s2[13] == 1'b0));

	// stage 4: final registered output
	reg [7:0] r_r, g_r, b_r;
	always @(posedge clk) begin
		if (ce_pix) begin
			if (!de_s2) begin
				r_r <= 8'h00; g_r <= 8'h00; b_r <= 8'h00;
			end else if (white_ov) begin
				r_r <= 8'hff; g_r <= 8'hff; b_r <= 8'hff;
			end else begin
				r_r <= mix_r; g_r <= mix_g; b_r <= mix_b;
			end
		end
	end

	assign r = r_r;
	assign g = g_r;
	assign b = b_r;

endmodule
