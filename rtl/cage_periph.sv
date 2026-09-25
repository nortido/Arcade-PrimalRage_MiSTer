// CAGE board peripherals on the TMS320C31 internal bus, 0x808000-0x8080FF.
// Register map, timer/serial/DMA timing: MAME cage.cpp tms320c31_io_r/w,
// update_timer, update_serial, update_dma_state. Register offsets below are
// the DSP's own word address within the block (periph_addr), identical to
// the offs_t values cage.cpp indexes m_tms320c31_io_regs with.
// The 0x808064 primary bus control mask (0x1FFE) and reset value (0x10F8)
// are not in cage.cpp (MAME never touches that register); they come from an
// observed hardware trace.
module cage_periph (
	input               clk,
	input               reset,
	// held while the DSP is in reset (control bits 0/1 both 0, cage_host's
	// dsp_hold_reset): mirrors MAME cage.cpp control_w, which zeroes offsets
	// 0x00-0x5F of the peripheral array on this condition. 0x64 (primary
	// bus control) is outside that range and is left alone, matching MAME.
	input               host_dsp_reset,

	// register bus from cage_mem, 0x808000-0x8080FF
	input               periph_we,
	input        [7:0]  periph_addr,
	input        [31:0] periph_wdata,
	output logic [31:0] periph_rdata,

	// interrupts to the (not yet built) C31 core: one clk_sys pulse per
	// event, matching MAME's set_input_line() with no explicit clear for
	// TINT0/TINT1/DINT0 -- the core's own IF register is a rising-edge
	// latch, so a level-with-ack scheme here would be redundant state.
	// xint/rint are not modeled, MAME cage.cpp never asserts them for CAGE.
	output logic        tint0,
	output logic        tint1,
	output logic        dint,
	output logic        xint,
	output logic        rint,

	// DMA master port: held-request/ack, same convention as cage_mem's DDR3
	// port. Single port shared with the CPU-side cage_mem port for now; the
	// top level must arbitrate them once both exist (T4/T5 follow-up).
	output logic        dma_req,
	output logic [23:0] dma_addr,
	input        [31:0] dma_rdata,
	input               dma_ack,

	// audio out, MAME cage.cpp 3.3 and atarigt.cpp atarigt_stereo routing
	// (channels 0/3 -> right, 1/2 -> left)
	output logic signed [15:0] audio_l,
	output logic signed [15:0] audio_r,
	output logic                audio_valid
);

	assign xint = 1'b0;
	assign rint = 1'b0;

	// ------------------------------------------------------------------
	// H1 tick: 16.9344 MHz (33.8688 MHz OSC / 2) derived from clk_sys by a
	// phase accumulator. 32'd846623952 = round(2^32 * 16934400 / 85909091),
	// computed offline (Quartus rejects real-valued elaboration, $rtoi
	// included); verified error -0.0005 ppm. Re-derive by hand if
	// CLK_HZ ever changes: inc = round(2^32 * H1_HZ / CLK_HZ).
	// ------------------------------------------------------------------
	localparam int ACC_W = 32;
	localparam logic [ACC_W-1:0] H1_INC = 32'd846623952;

	logic [ACC_W-1:0] h1_acc;
	logic             h1_tick;
	always_ff @(posedge clk) begin
		if (reset) begin
			h1_acc  <= '0;
			h1_tick <= 1'b0;
		end else begin
			{h1_tick, h1_acc} <= {1'b0, h1_acc} + {1'b0, H1_INC};
		end
	end

	// ------------------------------------------------------------------
	// register offsets, MAME cage.cpp macros
	// ------------------------------------------------------------------
	localparam [7:0] A_DMA_CTL = 8'h00, A_DMA_SRC = 8'h04, A_DMA_DST = 8'h06, A_DMA_CNT = 8'h08;
	localparam [7:0] A_T0_CTL = 8'h20, A_T0_CNT = 8'h24, A_T0_PER = 8'h28;
	localparam [7:0] A_T1_CTL = 8'h30, A_T1_CNT = 8'h34, A_T1_PER = 8'h38;
	localparam [7:0] A_SP_CTL = 8'h40, A_SP_TXCTL = 8'h42, A_SP_RXCTL = 8'h43,
					 A_SP_TCTL = 8'h44, A_SP_TCNT = 8'h45, A_SP_TPER = 8'h46,
					 A_SP_TXDATA = 8'h48, A_SP_RXDATA = 8'h4c;
	localparam [7:0] A_PRI_BUS = 8'h64;

	// ------------------------------------------------------------------
	// generic storage: every offset 0x00-0xFF is a plain read/write
	// register, MAME cage.cpp COMBINE_DATA into m_tms320c31_io_regs[0x100].
	// Async (combinational) read is required, not M10K: cage_mem's
	// T_PERIPH state captures periph_rdata the same cycle periph_addr
	// first becomes stable (rtl/cage_mem.sv), leaving no spare cycle for a
	// registered array read. 256 x 32 as flops is a deliberate exception
	// to the usual ramstyle M10K rule for that reason.
	// ------------------------------------------------------------------
	logic [31:0] regs [0:255];

	wire [31:0] dma_ctl_reg    = regs[A_DMA_CTL];
	wire [23:0] dma_source_cur = regs[A_DMA_SRC][23:0];
	wire [31:0] dma_count_cur  = regs[A_DMA_CNT];
	wire [31:0] t0_ctl_reg     = regs[A_T0_CTL];
	wire [31:0] t0_period_reg  = regs[A_T0_PER];
	wire [31:0] t1_ctl_reg     = regs[A_T1_CTL];
	wire [31:0] t1_period_reg  = regs[A_T1_PER];
	wire [31:0] sp_ctl_reg     = regs[A_SP_CTL];
	wire [15:0] sp_tper_cur    = regs[A_SP_TPER][15:0];
	wire [7:0]  sp_rxctl_byte  = regs[A_SP_RXCTL][7:0];

	// ------------------------------------------------------------------
	// timers: real down-counters clocked by h1_tick (MAME's own COUNTER
	// register, offset 0x24/0x34, is write-through storage only -- see the
	// generic regs[] array above; this is separate internal state driving
	// the interrupt cadence MAME models, 2*period H1 ticks, update_timer)
	// ------------------------------------------------------------------
	wire t0_enabled = (t0_ctl_reg[7:6] == 2'b11);
	wire t1_enabled = (t1_ctl_reg[7:6] == 2'b11);
	logic t0_enabled_prev, t1_enabled_prev;
	logic [31:0] t0_count, t1_count;
	logic tint0_fire, tint1_fire;

	always_ff @(posedge clk) begin
		tint0_fire <= 1'b0;
		if (reset || host_dsp_reset) begin
			t0_count <= '0;
			t0_enabled_prev <= 1'b0;
		end else begin
			t0_enabled_prev <= t0_enabled;
			if (t0_enabled && !t0_enabled_prev) begin
				t0_count <= {t0_period_reg[30:0], 1'b0}; // 2*period, armed
			end else if (t0_enabled && h1_tick) begin
				if (t0_count <= 32'd1) begin
					t0_count   <= {t0_period_reg[30:0], 1'b0};
					tint0_fire <= 1'b1;
				end else begin
					t0_count <= t0_count - 32'd1;
				end
			end
		end
	end

	always_ff @(posedge clk) begin
		tint1_fire <= 1'b0;
		if (reset || host_dsp_reset) begin
			t1_count <= '0;
			t1_enabled_prev <= 1'b0;
		end else begin
			t1_enabled_prev <= t1_enabled;
			if (t1_enabled && !t1_enabled_prev) begin
				t1_count <= {t1_period_reg[30:0], 1'b0};
			end else if (t1_enabled && h1_tick) begin
				if (t1_count <= 32'd1) begin
					t1_count   <= {t1_period_reg[30:0], 1'b0};
					tint1_fire <= 1'b1;
				end else begin
					t1_count <= t1_count - 32'd1;
				end
			end
		end
	end

	// ------------------------------------------------------------------
	// serial word clock chain, MAME cage.cpp update_serial: h1_tick -> /2 or
	// /4 serial clock -> /SPORT_TIMER_PERIOD bit clock -> /bits_per_word
	// word clock. The divider constants are latched from the register file
	// one clock after a write to SPORT_GLOBAL_CTL/SPORT_TIMER_PERIOD, and
	// only when the candidate rate is valid (TPER!=0, derived freq <100kHz)
	// -- MAME cage.cpp update_serial's own dmadac gate. Once latched
	// (rate_valid) the chain free-runs off that snapshot regardless of the
	// live TPER/enable bit, so a mid-burst TPER=0 re-init (the DSP firmware
	// does this every DMA burst) cannot stall or overrun the word clock.
	// A prior attempt latched off the SAME-cycle regs (racing the write)
	// and broke the DSP handshake; the 1-cycle delay here fixes that.
	// ------------------------------------------------------------------
	wire [3:0]  serial_div_h1 = sp_ctl_reg[2] ? 4'd4 : 4'd2;
	wire        sp_period_zero = (sp_tper_cur == 16'h0);
	wire [15:0] eff_tper = sp_period_zero ? (sp_ctl_reg[2] ? 16'd1 : 16'd0) : sp_tper_cur;
	// 8*(((ctl>>18)&3)+1): 8,16,24,32 for n=0..3
	wire [5:0] bits_per_word_c = {4'd0, sp_ctl_reg[19:18]} * 6'd8 + 6'd8;

	// candidate word period in H1 ticks; valid needs freq = H1_HZ/(4*period)
	// in (0,100000) Hz, i.e. period > 16934400/400000 -> period >= 43.
	//
	// NOTE: the DSP firmware writes SPORT_GLOBAL_CTL=0 as the first step of
	// every SPORT re-init burst (TPER is rewritten a few writes later,
	// while CTL briefly still reads 0), so a candidate can transiently
	// form from a genuinely-written TPER combined with that not-yet-
	// restored CTL, latching a real-but-brief 4x-too-fast rate for the
	// re-init window. Rejecting it needs sp_ctl_reg != 0 at candidate time
	// (below) -- an earlier attempt at this fixed the audio rate but broke
	// the host<->DSP handshake, traced to the DINT timer stop rule using
	// live register bits instead of MAME's internal dma_enabled flag (see
	// the DMA block); that is now fixed, so the gate is restored here.
	//
	// Quartus build 33 flagged regs[0x46] -> Mult0 -> Mult1 -> lat_div4 as
	// the worst path (-0.66ns at 85.9MHz) when the product fed the latch
	// combinationally. The write-triggered path has plenty of clocks to
	// spare, so it is a dedicated two-stage multiply pipeline (mirroring
	// h1_per_word_p1/r below) with its own factors carried alongside it;
	// the compare/latch reads only the stage-2 register, one clock later.
	logic        sp_ctl_or_tper_we_d;
	logic        cand_we_p1, cand_we_p2;
	logic [15:0] cand_tper_p1, cand_tper_p2;
	logic        cand_div4_p1, cand_div4_p2;
	logic [5:0]  cand_bpw_p1, cand_bpw_p2;
	logic [31:0] cand_mul1_p1, cand_mul2_p2;
	logic        cand_ctl_nz_p1, cand_ctl_nz_p2;

	always_ff @(posedge clk) begin
		if (reset) begin
			sp_ctl_or_tper_we_d <= 1'b0;
			cand_we_p1          <= 1'b0;
			cand_we_p2          <= 1'b0;
		end else begin
			sp_ctl_or_tper_we_d <= periph_we && !host_dsp_reset &&
									(periph_addr == A_SP_CTL || periph_addr == A_SP_TPER);

			// stage 1: register the factors and the first multiply
			cand_we_p1     <= sp_ctl_or_tper_we_d;
			cand_tper_p1   <= eff_tper;
			cand_div4_p1   <= sp_ctl_reg[2];
			cand_bpw_p1    <= bits_per_word_c;
			cand_mul1_p1   <= {16'h0, eff_tper} * {28'h0, serial_div_h1};
			cand_ctl_nz_p1 <= (sp_ctl_reg != 32'h0);

			// stage 2: second multiply, all factors carried forward
			cand_we_p2     <= cand_we_p1;
			cand_tper_p2   <= cand_tper_p1;
			cand_div4_p2   <= cand_div4_p1;
			cand_bpw_p2    <= cand_bpw_p1;
			cand_mul2_p2   <= cand_mul1_p1 * {26'h0, cand_bpw_p1};
			cand_ctl_nz_p2 <= cand_ctl_nz_p1;
		end
	end

	logic        rate_valid;
	logic        lat_div4;
	logic [15:0] lat_tper;
	logic [5:0]  lat_bpw;
	// pulses the cycle a new rate actually latches, so the divider phase
	// below can restart at 0 against the new lat_* targets instead of
	// free-running against a comparand that just moved out from under it
	// (an exact-match counter can otherwise miss its target for a full
	// wrap of the counter width if the target drops below where it is)
	logic        rate_update;

	// compare/latch reads only the stage-2 register (cand_mul2_p2 and its
	// carried factors), one clock after that register lands -- no live
	// combinational term reaches this always_ff
	always_ff @(posedge clk) begin
		rate_update <= 1'b0;
		if (reset) begin
			rate_valid <= 1'b0;
		end else if (cand_we_p2 && cand_ctl_nz_p2 && (cand_tper_p2 != 16'h0) && (cand_mul2_p2 >= 32'd43)) begin
			rate_valid  <= 1'b1;
			rate_update <= 1'b1;
			lat_div4    <= cand_div4_p2;
			lat_tper    <= cand_tper_p2;
			lat_bpw     <= cand_bpw_p2;
		end
	end

	wire [1:0] serial_div_target = lat_div4 ? 2'd3 : 2'd1; // /4 or /2 h1_ticks
	logic [1:0] serial_div_cnt;
	logic       serial_clk_tick;

	always_ff @(posedge clk) begin
		serial_clk_tick <= 1'b0;
		if (reset || host_dsp_reset || rate_update) begin
			serial_div_cnt <= '0;
		end else if (rate_valid && h1_tick) begin
			if (serial_div_cnt == serial_div_target) begin
				serial_div_cnt  <= '0;
				serial_clk_tick <= 1'b1;
			end else begin
				serial_div_cnt <= serial_div_cnt + 2'd1;
			end
		end
	end

	logic [15:0] bit_div_cnt;
	logic        bit_clk_tick;

	always_ff @(posedge clk) begin
		bit_clk_tick <= 1'b0;
		if (reset || host_dsp_reset || rate_update) begin
			bit_div_cnt <= '0;
		end else if (serial_clk_tick) begin
			if (bit_div_cnt == lat_tper - 16'd1) begin
				bit_div_cnt  <= '0;
				bit_clk_tick <= 1'b1;
			end else begin
				bit_div_cnt <= bit_div_cnt + 16'd1;
			end
		end
	end

	logic [4:0] word_bit_cnt;
	logic       word_tick;

	always_ff @(posedge clk) begin
		word_tick <= 1'b0;
		if (reset || host_dsp_reset || rate_update) begin
			word_bit_cnt <= '0;
		end else if (bit_clk_tick) begin
			if (word_bit_cnt == lat_bpw[4:0] - 5'd1) begin
				word_bit_cnt <= '0;
				word_tick    <= 1'b1;
			end else begin
				word_bit_cnt <= word_bit_cnt + 5'd1;
			end
		end
	end

	// matches MAME cage.cpp SPORT_DATA_TX special case: MAME compares the
	// *derived word rate* to 22050*4 Hz (int(m_serial_period_per_word.
	// as_hz()) == 88200), not any one fixed register pattern -- e.g. both
	// {bits=32, no clock doubling, tper=3} and {bits=16, clock doubling,
	// tper=3} (the latter from the observed SPORT_GLOBAL_CTL=0x04851046
	// trace) give 88200 Hz. H1_HZ/88200 = 192 exactly, and
	// h1_ticks_per_word = serial_div * tper * bits_per_word by
	// construction of the divider chain above, so comparing that count to
	// 192 reproduces MAME's frequency check without any real division.
	// two-stage pipeline: Quartus build 28 timing showed a multiplier
	// sitting between regs[] and regs[] in one cycle (regs[0x46] ->
	// h1_per_word -> regs[0x43]); break the product into two registered
	// stages instead of one combinational chain off the register file
	logic [31:0] h1_per_word_p1;
	logic [31:0] h1_per_word_r;
	always_ff @(posedge clk) begin
		h1_per_word_p1 <= {16'h0, eff_tper} * {28'h0, serial_div_h1};
		h1_per_word_r  <= h1_per_word_p1 * {26'h0, bits_per_word_c};
	end
	wire word_rate_88200 = (eff_tper != 16'h0) && (h1_per_word_r == 32'd192);

	// ------------------------------------------------------------------
	// DMA, MAME cage.cpp update_dma_state/dma_timer_callback (258-331). Two
	// independent pieces, matching MAME's split between the (instant) bulk
	// transfer and the deferred periodic timer:
	//
	// (1) DINT timer: a free-running word-tick countdown, period = the
	// transfer count latched at the FIRST arm after the timer was stopped
	// (MAME cage.cpp only calls timer.adjust() when !m_dma_timer_enabled;
	// a re-arm that lands while the timer is already running does not
	// reset period or final address, same as MAME). MAME cage.cpp tracks
	// enable with its OWN m_dma_enabled flag, separate from the register
	// bits: dma_enabled_reg mirrors that. update_dma_state only runs on a
	// write to one of the four DMA regs, and only THAT write's forwarded
	// value (dma_ctl_nx/dma_cnt_nx/dma_src_nx) decides enabled_new -- using
	// the live register bits instead would misread a firmware write that
	// transiently looks like "disable" (e.g. CTL=0 as a re-init step while
	// a stale nonzero CNT is still sitting in the register) as a genuine
	// stop, when m_dma_enabled was already 0 since the last DINT.
	//
	// (2) burst queue, depth 2: the stream currently being fetched
	// (stream_src/stream_remaining) plus one pending burst queued behind
	// it if a re-arm lands mid-stream. Words are pulled through the
	// one-word prefetch buffer at the serial word rate; with nothing
	// streaming, the last word (or silence) repeats.
	// ------------------------------------------------------------------
	wire is_dma_reg_we = periph_we && !host_dsp_reset &&
		(periph_addr == A_DMA_CTL || periph_addr == A_DMA_SRC ||
		 periph_addr == A_DMA_DST || periph_addr == A_DMA_CNT);
	// forward this cycle's write into the enable check, mirroring MAME's
	// update_dma_state which runs after COMBINE_DATA already applied it
	wire [31:0] dma_ctl_nx = (periph_we && periph_addr == A_DMA_CTL) ? periph_wdata : dma_ctl_reg;
	wire [31:0] dma_cnt_nx = (periph_we && periph_addr == A_DMA_CNT) ? periph_wdata : dma_count_cur;
	wire [23:0] dma_src_nx = (periph_we && periph_addr == A_DMA_SRC) ? periph_wdata[23:0] : dma_source_cur;
	wire        enabled_new = (dma_ctl_nx[1:0] == 2'b11) && (dma_cnt_nx != 32'h0);
	logic       dma_enabled_reg;

	typedef enum logic { PF_IDLE, PF_WAIT } pf_state_t;
	pf_state_t   pf_state;
	logic [15:0] pf_buf;
	logic        pf_valid;
	logic        pf_fetched;

	// burst queue
	logic        stream_active;
	logic [23:0] stream_src;
	logic [31:0] stream_remaining;
	logic        pend_valid;
	logic [23:0] pend_src;
	logic [31:0] pend_count;

	// DINT periodic timer
	logic        dint_timer_running;
	logic [31:0] dint_period;
	logic [31:0] dint_counter;
	logic [23:0] dint_final_addr;

	logic [1:0]         chan_idx;
	logic signed [15:0] ch_sample [0:2];
	logic [15:0]        holding_reg;
	logic                dint_fire;

	// clamp derived from MAME's mixer formula (atarigt.cpp atarigt_stereo:
	// add_route(0/3, "speaker", 1.0, right), add_route(1/2, "speaker",
	// 1.0, left) -- two unity-gain routes summed per output, saturated to
	// the 16-bit DAC range)
	function automatic signed [15:0] clamp16(input signed [16:0] v);
		if (v > 17'sd32767) clamp16 = 16'sd32767;
		else if (v < -17'sd32768) clamp16 = 16'sh8000;
		else clamp16 = v[15:0];
	endfunction

	always_ff @(posedge clk) begin
		pf_fetched  <= 1'b0;
		dint_fire   <= 1'b0;
		audio_valid <= 1'b0;
		dma_req     <= 1'b0;
		if (reset || host_dsp_reset) begin
			pf_state         <= PF_IDLE;
			pf_valid         <= 1'b0;
			stream_active    <= 1'b0;
			stream_remaining <= 32'h0;
			pend_valid       <= 1'b0;
			dint_timer_running <= 1'b0;
			dma_enabled_reg  <= 1'b0;
			chan_idx         <= 2'd0;
			holding_reg      <= 16'h0;
			dma_addr         <= 24'h0;
			audio_l          <= 16'sd0;
			audio_r          <= 16'sd0;
		end else begin
			// MAME cage.cpp update_dma_state: runs only on a write to one of
			// the four DMA regs, comparing enabled_new to the internal flag
			if (is_dma_reg_we) begin
				if (enabled_new && !dma_enabled_reg) begin
					// MAME cage.cpp update_dma_state starts every transfer
					// at channel 0
					chan_idx <= 2'd0;
					if (!dint_timer_running) begin
						dint_timer_running <= 1'b1;
						dint_period        <= dma_cnt_nx;
						dint_counter       <= dma_cnt_nx;
						dint_final_addr    <= dma_src_nx + (dma_ctl_nx[4] ? dma_cnt_nx[23:0] : 24'h0);
					end
					if (!stream_active) begin
						stream_active    <= 1'b1;
						stream_src       <= dma_src_nx;
						stream_remaining <= dma_cnt_nx;
					end else begin
						pend_valid <= 1'b1;
						pend_src   <= dma_src_nx;
						pend_count <= dma_cnt_nx;
					end
				end else if (!enabled_new && dma_enabled_reg) begin
					dint_timer_running <= 1'b0;
				end
				dma_enabled_reg <= enabled_new;
			end

			// DINT periodic timer: counts word ticks regardless of DMA
			// state (rate_valid paces it below, same tick as playback).
			// dma_timer_callback: fires only while the internal flag is
			// still set; a fire with it already clear just shuts the
			// timer off instead (the write path above missed the edge
			// because nothing re-armed between the previous DINT and now)
			if (dint_timer_running && rate_valid && word_tick) begin
				if (dint_counter <= 32'd1) begin
					if (dma_enabled_reg) begin
						dint_fire       <= 1'b1;
						dint_counter    <= dint_period;
						dma_enabled_reg <= 1'b0;
					end else begin
						dint_timer_running <= 1'b0;
					end
				end else begin
					dint_counter <= dint_counter - 32'd1;
				end
			end

			// prefetch: keeps the one-word buffer full from the active
			// stream, as fast as the shared bus arbiter allows
			if (!stream_active) begin
				pf_state <= PF_IDLE;
			end else case (pf_state)
				PF_IDLE: if (!pf_valid) begin
					dma_req  <= 1'b1;
					dma_addr <= stream_src;
					pf_state <= PF_WAIT;
				end
				PF_WAIT: begin
					dma_req <= 1'b1; // held request, cage_mem-style convention
					if (dma_ack) begin
						dma_req    <= 1'b0;
						pf_buf     <= dma_rdata[15:0];
						pf_valid   <= 1'b1;
						pf_fetched <= 1'b1;
						pf_state   <= PF_IDLE;
						if (stream_remaining <= 32'd1) begin
							if (pend_valid) begin
								stream_src       <= pend_src;
								stream_remaining <= pend_count;
								pend_valid       <= 1'b0;
							end else begin
								stream_active    <= 1'b0;
								stream_remaining <= 32'h0;
							end
						end else begin
							stream_src       <= stream_src + {23'h0, dma_ctl_reg[4]};
							stream_remaining <= stream_remaining - 32'd1;
						end
					end
				end
				default: pf_state <= PF_IDLE;
			endcase

			// playback: one word every word_tick, gated on the SPORT being
			// configured (rate_valid) only -- the stream runs whether or
			// not the DMA is on, repeating the stale/silent holding_reg
			// when nothing is queued or the prefetch hasn't refilled yet,
			// same as MAME cage.cpp's dmadac
			if (rate_valid && word_tick) begin
				if (pf_valid) begin holding_reg <= pf_buf; pf_valid <= 1'b0; end
				if (chan_idx == 2'd3) begin
					chan_idx    <= 2'd0;
					audio_r     <= clamp16($signed(ch_sample[0]) + $signed(pf_valid ? pf_buf : holding_reg));
					audio_l     <= clamp16($signed(ch_sample[1]) + $signed(ch_sample[2]));
					audio_valid <= 1'b1;
				end else begin
					ch_sample[chan_idx] <= pf_valid ? pf_buf : holding_reg;
					chan_idx <= chan_idx + 2'd1;
				end
			end
		end
	end

	// interrupt pulses straight to the core: one clk_sys cycle, no ack
	assign tint0 = tint0_fire;
	assign tint1 = tint1_fire;
	assign dint  = dint_fire;

	// ------------------------------------------------------------------
	// register writes: a CPU write to a given offset always wins over a
	// same-cycle DMA-driven update of DMA_SOURCE_ADDR/DMA_TRANSFER_COUNT
	// (the only two offsets with an internal writer besides the CPU)
	// ------------------------------------------------------------------
	integer ri;
	reg host_dsp_reset_q;
	always_ff @(posedge clk) host_dsp_reset_q <= host_dsp_reset;
	always_ff @(posedge clk) begin
		if (reset) begin
			for (ri = 0; ri < 256; ri = ri + 1) regs[ri] <= 32'h0;
			regs[A_PRI_BUS] <= 32'h10F8;
		end else begin
			if (host_dsp_reset) begin
				// MAME cage.cpp control_w: memset(m_tms320c31_io_regs, 0, 0x60*4);
				// 0x64 and above are outside that range and left alone
				// clear once on the edge: a held level is a 96-way loop every clock in sim
				if (!host_dsp_reset_q) for (ri = 0; ri < 8'h60; ri = ri + 1) regs[ri] <= 32'h0;
			end else begin
				if (periph_we) begin
					if (periph_addr == A_PRI_BUS)
						regs[A_PRI_BUS] <= (regs[A_PRI_BUS] & ~32'h1FFE) | (periph_wdata & 32'h1FFE);
					else
						regs[periph_addr] <= periph_wdata;

					// MAME cage.cpp tms320c31_io_w, SPORT_DATA_TX case
					if (periph_addr == A_SP_TXDATA && word_rate_88200 && sp_rxctl_byte == 8'h62)
						regs[A_SP_RXCTL][11] <= ~regs[A_SP_RXCTL][11];
				end

				// MAME cage.cpp dma_timer_callback: DMA_SOURCE_ADDR/
				// DMA_TRANSFER_COUNT only change at DINT, to the final
				// address / 0 -- never incrementally, matching what the
				// CPU can observe (the transfer already happened up front)
				if (!(periph_we && periph_addr == A_DMA_SRC) && dint_fire)
					regs[A_DMA_SRC][23:0] <= dint_final_addr;
				if (!(periph_we && periph_addr == A_DMA_CNT) && dint_fire)
					regs[A_DMA_CNT] <= 32'h0;
			end
		end
	end

	// ------------------------------------------------------------------
	// read mux: cage_mem captures periph_rdata one clock after periph_addr
	// is presented (rtl/cage_mem.sv T_PERIPH), so this must be
	// combinational. MAME cage.cpp tms320c31_io_r assigns the array's
	// uint32_t element into a uint16_t before returning it, truncating
	// every read to 16 bits (zero-extended back to 32); that MAME quirk
	// is reproduced here deliberately, on the read side only -- the
	// internal logic above always uses the untruncated regs[] value.
	// ------------------------------------------------------------------
	logic [31:0] rd_raw;
	always_comb begin
		case (periph_addr)
			A_DMA_CTL: rd_raw = (regs[A_DMA_CTL] & ~32'hC) | (dma_enabled_reg ? 32'hC : 32'h0);
			default:   rd_raw = regs[periph_addr];
		endcase
	end
	assign periph_rdata = {16'h0, rd_raw[15:0]};

endmodule
