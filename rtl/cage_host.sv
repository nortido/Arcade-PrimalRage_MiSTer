// CAGE host interface: the 68020 side registers at C00000, MAME atarigt.cpp
// sound_data_r/sound_data_w, and the DSP side of the handshake, MAME cage.cpp
// main_r/main_w/control_r/control_w/cage_from_main_r/cage_to_main_w.
module cage_host (
	input               clk,
	input               reset,

	// host (68020) side of the C00000 longword: sound_data_r/w splits it as
	// ACCESSING_BITS_16_31 = data (high word, C00000), ACCESSING_BITS_0_15 =
	// control (low word, C00002)
	input               host_we,
	input               host_re,
	input               host_data_sel, // 1 = data word, 0 = control word
	input        [15:0] host_wdata,
	output logic [15:0] host_rdata,

	// to the 68020: nonzero reason asserts M68K_IRQ_3 (cage_irq_callback)
	output logic        irq3,

	// DSP side, 0xA00000 in the C31's own map 
	input               dsp_re,   // cage_from_main_r
	input               dsp_we,   // cage_to_main_w
	input        [15:0] dsp_wdata,
	output logic [15:0] dsp_rdata,

	// C31 interrupt lines, active-low (int_n convention): int0_n is the
	// host command-ready IRQ0 (MAME cage.cpp cage_deferred_w/
	// cage_from_main_r), int1_n mirrors TMS320C3X_IRQ1, asserted whenever
	// (control & 3) == 0 (MAME cage.cpp control_w). Both are levels, unlike
	// cage_periph's TINT/DINT pulses, because MAME explicitly asserts and
	// clears these two rather than only ever asserting them.
	output logic        int0_n,
	output logic        int1_n,
	// IOF bits this device drives: bit3 = cpu_to_cage_ready, bit7 =
	// cage_to_cpu_ready (MAME cage.cpp update_control_lines: val &= ~0x88).
	// Caller ORs iof_bits into IOF after masking those two bits out.
	output logic [7:0]  iof_bits,

	// held while the DSP must stay in reset: control bits 0/1 both 0, or
	// ext_reset (the 0xE08000 latch bit21 path, MAME atarigt.cpp reset_w)
	output logic        dsp_hold_reset,
	input               ext_reset
);

	logic [15:0] control_reg;
	logic [15:0] from_main;
	logic [15:0] to_cpu_latch;
	logic        cpu_to_cage_ready;
	logic        cage_to_cpu_ready;
	logic        dsp_irq0_level;

	wire irq1_level = ext_reset || (control_reg[1:0] == 2'b00);

	assign dsp_hold_reset = irq1_level;
	assign int0_n = ~dsp_irq0_level;
	assign int1_n = ~irq1_level;

	assign irq3 = (((control_reg[1:0] == 2'b11) && !cpu_to_cage_ready) ||
				   (control_reg[1] && cage_to_cpu_ready));

	assign iof_bits = {cage_to_cpu_ready, 3'b0, cpu_to_cage_ready, 3'b0};

	assign dsp_rdata = from_main;

	always_comb begin
		if (host_data_sel) host_rdata = to_cpu_latch;
		else                host_rdata = {14'h0, cpu_to_cage_ready, cage_to_cpu_ready};
	end

	always_ff @(posedge clk) begin
		if (reset) begin
			control_reg        <= 16'h0;
			from_main           <= 16'h0;
			to_cpu_latch        <= 16'h0;
			cpu_to_cage_ready   <= 1'b0;
			cage_to_cpu_ready   <= 1'b0;
			dsp_irq0_level      <= 1'b0;
		end else begin
			// dsp_re/dsp_we/host_re apply first, using this cycle's
			// pre-update state; host_we (main_w) applies after, so a
			// same-cycle "DSP reads the old command" + "host posts a new
			// one" ends with the new command's ready flag set, matching
			// MAME's machine().scheduler().synchronize() deferring main_w
			// to run after whatever the DSP does in the same instant.
			// dsp_we now comes after host_re: the 68020 already grabbed the old value this cycle, so a same-cycle DSP write must win and leave ready set.
			if (dsp_re) begin
				cpu_to_cage_ready <= 1'b0;
				dsp_irq0_level    <= 1'b0;
			end
			if (host_re && host_data_sel) begin
				cage_to_cpu_ready <= 1'b0;
			end
			if (dsp_we) begin
				to_cpu_latch      <= dsp_wdata;
				cage_to_cpu_ready <= 1'b1;
			end

			if (host_we) begin
				if (host_data_sel) begin
					// main_w: post command, set ready, assert IRQ0
					from_main         <= host_wdata;
					cpu_to_cage_ready <= 1'b1;
					dsp_irq0_level    <= 1'b1;
				end else begin
					control_reg <= host_wdata;
					if (host_wdata[1:0] == 2'b00) begin
						cpu_to_cage_ready <= 1'b0;
						cage_to_cpu_ready <= 1'b0;
						dsp_irq0_level    <= 1'b0;
					end
				end
			end

			// ext_reset (reset_w -> control_w(0)) always wins
			if (ext_reset) begin
				control_reg       <= 16'h0;
				cpu_to_cage_ready <= 1'b0;
				cage_to_cpu_ready <= 1'b0;
				dsp_irq0_level    <= 1'b0;
			end
		end
	end

endmodule
