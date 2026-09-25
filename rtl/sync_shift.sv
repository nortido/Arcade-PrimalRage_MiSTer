// Shifts hsync/vsync pulses by hoff pixels / voff lines relative to
// hblank/vblank, measuring the incoming pulse position itself (no hard-coded
// timing constants) so it tracks whatever video_timing the upstream module uses.

module sync_shift
(
	input               clk,
	input               ce_pix,
	input               hblank,
	input               vblank,
	input               hsync,
	input               vsync,
	input  signed [5:0] hoff,
	input  signed [4:0] voff,
	output              hsync_o,
	output              vsync_o,
	output              vblank_o
);

	reg hblank_prev = 1'b0, vblank_prev = 1'b0, hsync_prev = 1'b1, vsync_prev = 1'b1;
	reg [8:0] hc_reg = 9'd0, vc_reg = 9'd0;

	// hc/vc are the position "as of this cycle": combinational restart on the
	// edge seen this cycle, so there is no extra latency before the window
	// compare below can react to it.
	wire hc_restart = hblank_prev & ~hblank;
	wire [8:0] hc = hc_restart ? 9'd0 : (hc_reg + 9'd1);

	wire vblank_restart = vblank_prev & ~vblank;
	wire [8:0] vc = !hc_restart ? vc_reg : (vblank_restart ? 9'd0 : (vc_reg + 9'd1));

	wire hsync_fall = hsync_prev & ~hsync;
	wire hsync_rise = ~hsync_prev & hsync;
	wire vsync_fall = vsync_prev & ~vsync;
	wire vsync_rise = ~vsync_prev & vsync;

	reg [8:0] hs_on = 9'd0, hs_off = 9'd0;
	reg [8:0] vs_on = 9'd0, vs_off = 9'd0;
	reg signed [9:0] hs_start = 10'sd0, hs_stop = 10'sd0;
	reg signed [9:0] vs_start = 10'sd0, vs_stop = 10'sd0;
	reg hseen = 1'b0, vseen = 1'b0, hlocked = 1'b0, vlocked = 1'b0;

	// active line count, measured (not hard-coded) at the line where vblank
	// rises after being low: needed to keep the last voff-1 active lines
	// blanked for voff>=2 so a shifted vsync always lands inside vblank_o.
	// Only trust it once a full active span was seen (vblank_restart before
	// this rise), so starting mid-frame does not latch a truncated count.
	reg [8:0] v_active = 9'd0;
	reg v_active_seen = 1'b0, v_active_span_ok = 1'b0;
	reg signed [4:0] voff_latched = 5'sd0;
	wire vblank_rise_now = hc_restart & vblank & ~vblank_prev;

	always @(posedge clk) begin
		if (ce_pix) begin
			hblank_prev <= hblank;
			vblank_prev <= vblank;
			hsync_prev  <= hsync;
			vsync_prev  <= vsync;
			hc_reg <= hc;
			vc_reg <= vc;

			if (hsync_fall) hs_on  <= hc;
			if (hsync_rise) begin hs_off <= hc; hseen <= 1'b1; end

			if (vsync_fall) vs_on  <= vc;
			if (vsync_rise) begin vs_off <= vc; vseen <= 1'b1; end

			if (vblank_rise_now) begin
				if (v_active_span_ok) begin v_active <= vc; v_active_seen <= 1'b1; end
			end

			// bounds are re-latched once per line/frame, from the previous
			// line's/frame's measurement plus the offset sampled right here:
			// a change to hoff/voff at any point takes effect from the next
			// full line/frame, never mid-pulse
			if (hc_restart) begin
				hs_start <= $signed({1'b0, hs_on})  - hoff;
				hs_stop  <= $signed({1'b0, hs_off}) - hoff;
				if (hseen) hlocked <= 1'b1;
			end
			if (vblank_restart) begin
				// until v_active is valid, force the effective offset to 0 for
				// both the vsync window and extra_vblank: a real voff here
				// could shift vsync into active lines before extra_vblank is
				// able to cover it, closing that window instead of leaving it
				vs_start <= $signed({1'b0, vs_on})  - (v_active_seen ? voff : 5'sd0);
				vs_stop  <= $signed({1'b0, vs_off}) - (v_active_seen ? voff : 5'sd0);
				voff_latched <= v_active_seen ? voff : 5'sd0;
				v_active_span_ok <= 1'b1;
				if (vseen) vlocked <= 1'b1;
			end
		end
	end

	wire locked = hlocked & vlocked;

	wire hsync_shifted = ~(($signed({1'b0, hc}) >= hs_start) && ($signed({1'b0, hc}) < hs_stop));
	wire vsync_shifted = ~(($signed({1'b0, vc}) >= vs_start) && ($signed({1'b0, vc}) < vs_stop));

	assign hsync_o = locked ? hsync_shifted : hsync;
	assign vsync_o = locked ? vsync_shifted : vsync;

	// extra blanking on the trailing (voff-1) active lines so a vsync pushed
	// earlier by voff>=2 lines still falls inside vblank_o, not active video
	wire signed [9:0] vc_s = $signed({1'b0, vc});
	wire signed [9:0] v_active_s = $signed({1'b0, v_active});
	wire signed [9:0] voff_ext = {{5{voff_latched[4]}}, voff_latched};
	wire extra_vblank = v_active_seen & (voff_latched >= 5'sd2) &
						 (vc_s >= (v_active_s - voff_ext + 10'sd1)) & (vc_s < v_active_s);

	assign vblank_o = vblank | extra_vblank;

endmodule
