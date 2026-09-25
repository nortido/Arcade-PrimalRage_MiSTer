module xga_prot (
	input  logic        clk_cpu,
	input  logic        reset,
	input  logic [18:1] addr,
	input  logic [15:0] din,
	input  logic        we,
	input  logic        rd,
	output logic         override,
	output logic [15:0] dout,
	output logic        busy
);

	// word addresses = byte offset (atarixga.cpp PR_*) >> 1
	localparam [18:1] PR_SETKEY   = 18'h22008;
	localparam [18:1] PR_DECIPHER = 18'h22011;
	localparam [18:1] PR_STATUS   = 18'h22380;
	localparam [18:1] PR_DONE0    = 18'h263e0;
	localparam [18:1] PR_RESULT   = 18'h263e1;
	localparam [18:1] PR_DONE4    = 18'h263e2;
	localparam [18:1] PR_DATA     = 18'h23c00;
	localparam [18:1] PR_CHAR0    = 18'h24380;
	localparam [18:1] PR_CHAR1    = 18'h32000;
	localparam [18:1] PR_CHAR2    = 18'h36000;

	localparam RAM_WORDS = 2048;
	localparam [18:1] PR_DATA_END = PR_DATA + 18'd2048;

	localparam [1:0] MODE_IDLE     = 2'd0;
	localparam [1:0] MODE_SETKEY   = 2'd1;
	localparam [1:0] MODE_DECIPHER = 2'd2;

	(* ramstyle = "M10K" *) logic [7:0] ram [0:RAM_WORDS-1];
	logic [1:0] mode;
	logic [15:0] taps;
	logic [15:0] reply;

	logic in_data_range;
	logic [10:0] data_index;
	assign in_data_range = (addr >= PR_DATA) && (addr < PR_DATA_END);
	assign data_index = addr[11:1] - PR_DATA[11:1]; // low 11 bits of the word address: the RAM index

	// decipher engine: one LFSR clock per cycle, matches atari_136094_0004a_device::decipher
	localparam [2:0] ENG_IDLE  = 3'd0;
	localparam [2:0] ENG_FETCH = 3'd1;
	localparam [2:0] ENG_KMAP  = 3'd2;
	localparam [2:0] ENG_SETUP = 3'd3;
	localparam [2:0] ENG_LOOP1 = 3'd4;
	localparam [2:0] ENG_CHECK = 3'd5;
	localparam [2:0] ENG_LOOP2 = 3'd6;

	logic [2:0]  eng_state;
	logic [10:0] eng_koff;   // registered key RAM read address, for M10K inference
	logic [7:0]  eng_kbyte;  // registered key RAM read data
	logic [7:0]  eng_n;      // registered kmap ROM read data
	logic [15:0] eng_c;
	logic [15:0] eng_x;
	logic [7:0]  eng_clocks;
	logic [7:0]  eng_cnt;
	logic        eng_early;

	function automatic [10:0] key_offset(input [10:0] index);
		// bit permutation of the query index, transcribed from atarixga.cpp
		key_offset = { (index[10] ^ 1'b0),
					   (index[9] ^ 1'b0),
					   (index[8] ^ 1'b0),
					   (index[6] ^ 1'b1),
					   (index[7] ^ 1'b0),
					   (index[1] ^ 1'b0),
					   (index[0] ^ 1'b1),
					   (index[4] ^ 1'b0),
					   (index[2] ^ 1'b1),
					   (index[5] ^ 1'b1),
					   (index[3] ^ 1'b0) };
	endfunction

	// kmap[k]: number of LFSR clocks for key byte k, verbatim from atarixga.cpp
	function automatic [7:0] kmap(input [7:0] k);
		case (k)
			8'h00: kmap = 8'h00; 8'h01: kmap = 8'h00; 8'h02: kmap = 8'h00; 8'h03: kmap = 8'h00;
			8'h04: kmap = 8'h00; 8'h05: kmap = 8'h59; 8'h06: kmap = 8'h17; 8'h07: kmap = 8'h7b;
			8'h08: kmap = 8'h00; 8'h09: kmap = 8'h00; 8'h0a: kmap = 8'h4f; 8'h0b: kmap = 8'h27;
			8'h0c: kmap = 8'h00; 8'h0d: kmap = 8'h00; 8'h0e: kmap = 8'h3d; 8'h0f: kmap = 8'h00;
			8'h10: kmap = 8'h00; 8'h11: kmap = 8'h00; 8'h12: kmap = 8'h00; 8'h13: kmap = 8'h00;
			8'h14: kmap = 8'h00; 8'h15: kmap = 8'h67; 8'h16: kmap = 8'h00; 8'h17: kmap = 8'h6f;
			8'h18: kmap = 8'h00; 8'h19: kmap = 8'h65; 8'h1a: kmap = 8'h57; 8'h1b: kmap = 8'h00;
			8'h1c: kmap = 8'h45; 8'h1d: kmap = 8'h4b; 8'h1e: kmap = 8'h00; 8'h1f: kmap = 8'h00;
			8'h20: kmap = 8'h1b; 8'h21: kmap = 8'h1d; 8'h22: kmap = 8'h19; 8'h23: kmap = 8'h00;
			8'h24: kmap = 8'h00; 8'h25: kmap = 8'h61; 8'h26: kmap = 8'h3f; 8'h27: kmap = 8'h00;
			8'h28: kmap = 8'h5f; 8'h29: kmap = 8'h00; 8'h2a: kmap = 8'h00; 8'h2b: kmap = 8'h00;
			8'h2c: kmap = 8'h00; 8'h2d: kmap = 8'h2d; 8'h2e: kmap = 8'h00; 8'h2f: kmap = 8'h00;
			8'h30: kmap = 8'h1f; 8'h31: kmap = 8'h00; 8'h32: kmap = 8'h5b; 8'h33: kmap = 8'h7d;
			8'h34: kmap = 8'h00; 8'h35: kmap = 8'h00; 8'h36: kmap = 8'h00; 8'h37: kmap = 8'h00;
			8'h38: kmap = 8'h00; 8'h39: kmap = 8'h23; 8'h3a: kmap = 8'h00; 8'h3b: kmap = 8'h53;
			8'h3c: kmap = 8'h15; 8'h3d: kmap = 8'h6d; 8'h3e: kmap = 8'h79; 8'h3f: kmap = 8'h00;
			8'h40: kmap = 8'h5d; 8'h41: kmap = 8'h21; 8'h42: kmap = 8'h00; 8'h43: kmap = 8'h00;
			8'h44: kmap = 8'h00; 8'h45: kmap = 8'h00; 8'h46: kmap = 8'h00; 8'h47: kmap = 8'h47;
			8'h48: kmap = 8'h00; 8'h49: kmap = 8'h6b; 8'h4a: kmap = 8'h2b; 8'h4b: kmap = 8'h13;
			8'h4c: kmap = 8'h75; 8'h4d: kmap = 8'h33; 8'h4e: kmap = 8'h00; 8'h4f: kmap = 8'h37;
			8'h50: kmap = 8'h00; 8'h51: kmap = 8'h00; 8'h52: kmap = 8'h69; 8'h53: kmap = 8'h71;
			8'h54: kmap = 8'h25; 8'h55: kmap = 8'h55; 8'h56: kmap = 8'h4d; 8'h57: kmap = 8'h00;
			8'h58: kmap = 8'h73; 8'h59: kmap = 8'h00; 8'h5a: kmap = 8'h31; 8'h5b: kmap = 8'h00;
			8'h5c: kmap = 8'h00; 8'h5d: kmap = 8'h00; 8'h5e: kmap = 8'h3b; 8'h5f: kmap = 8'h00;
			8'h60: kmap = 8'h00; 8'h61: kmap = 8'h00; 8'h62: kmap = 8'h00; 8'h63: kmap = 8'h41;
			8'h64: kmap = 8'h00; 8'h65: kmap = 8'h51; 8'h66: kmap = 8'h00; 8'h67: kmap = 8'h00;
			8'h68: kmap = 8'h00; 8'h69: kmap = 8'h00; 8'h6a: kmap = 8'h00; 8'h6b: kmap = 8'h00;
			8'h6c: kmap = 8'h00; 8'h6d: kmap = 8'h00; 8'h6e: kmap = 8'h00; 8'h6f: kmap = 8'h77;
			8'h70: kmap = 8'h00; 8'h71: kmap = 8'h00; 8'h72: kmap = 8'h00; 8'h73: kmap = 8'h63;
			8'h74: kmap = 8'h29; 8'h75: kmap = 8'h00; 8'h76: kmap = 8'h11; 8'h77: kmap = 8'h2f;
			8'h78: kmap = 8'h00; 8'h79: kmap = 8'h43; 8'h7a: kmap = 8'h00; 8'h7b: kmap = 8'h49;
			8'h7c: kmap = 8'h00; 8'h7d: kmap = 8'h00; 8'h7e: kmap = 8'h35; 8'h7f: kmap = 8'h39;
			8'h80: kmap = 8'h00; 8'h81: kmap = 8'h00; 8'h82: kmap = 8'h00; 8'h83: kmap = 8'h64;
			8'h84: kmap = 8'h00; 8'h85: kmap = 8'h22; 8'h86: kmap = 8'h00; 8'h87: kmap = 8'h42;
			8'h88: kmap = 8'h00; 8'h89: kmap = 8'h6a; 8'h8a: kmap = 8'h20; 8'h8b: kmap = 8'h00;
			8'h8c: kmap = 8'h00; 8'h8d: kmap = 8'h00; 8'h8e: kmap = 8'h1c; 8'h8f: kmap = 8'h00;
			8'h90: kmap = 8'h66; 8'h91: kmap = 8'h00; 8'h92: kmap = 8'h54; 8'h93: kmap = 8'h4a;
			8'h94: kmap = 8'h00; 8'h95: kmap = 8'h6c; 8'h96: kmap = 8'h00; 8'h97: kmap = 8'h00;
			8'h98: kmap = 8'h58; 8'h99: kmap = 8'h32; 8'h9a: kmap = 8'h00; 8'h9b: kmap = 8'h00;
			8'h9c: kmap = 8'h50; 8'h9d: kmap = 8'h2c; 8'h9e: kmap = 8'h60; 8'h9f: kmap = 8'h00;
			8'ha0: kmap = 8'h70; 8'ha1: kmap = 8'h00; 8'ha2: kmap = 8'h00; 8'ha3: kmap = 8'h00;
			8'ha4: kmap = 8'h7c; 8'ha5: kmap = 8'h48; 8'ha6: kmap = 8'h62; 8'ha7: kmap = 8'h52;
			8'ha8: kmap = 8'h00; 8'ha9: kmap = 8'h26; 8'haa: kmap = 8'h00; 8'hab: kmap = 8'h12;
			8'hac: kmap = 8'h00; 8'had: kmap = 8'h00; 8'hae: kmap = 8'h40; 8'haf: kmap = 8'h00;
			8'hb0: kmap = 8'h00; 8'hb1: kmap = 8'h00; 8'hb2: kmap = 8'h6e; 8'hb3: kmap = 8'h00;
			8'hb4: kmap = 8'h00; 8'hb5: kmap = 8'h38; 8'hb6: kmap = 8'h2e; 8'hb7: kmap = 8'h00;
			8'hb8: kmap = 8'h46; 8'hb9: kmap = 8'h00; 8'hba: kmap = 8'h7a; 8'hbb: kmap = 8'h36;
			8'hbc: kmap = 8'h00; 8'hbd: kmap = 8'h76; 8'hbe: kmap = 8'h00; 8'hbf: kmap = 8'h00;
			8'hc0: kmap = 8'h00; 8'hc1: kmap = 8'h72; 8'hc2: kmap = 8'h00; 8'hc3: kmap = 8'h00;
			8'hc4: kmap = 8'h00; 8'hc5: kmap = 8'h00; 8'hc6: kmap = 8'h1e; 8'hc7: kmap = 8'h00;
			8'hc8: kmap = 8'h00; 8'hc9: kmap = 8'h00; 8'hca: kmap = 8'h5c; 8'hcb: kmap = 8'h00;
			8'hcc: kmap = 8'h00; 8'hcd: kmap = 8'h5e; 8'hce: kmap = 8'h1a; 8'hcf: kmap = 8'h00;
			8'hd0: kmap = 8'h00; 8'hd1: kmap = 8'h00; 8'hd2: kmap = 8'h24; 8'hd3: kmap = 8'h44;
			8'hd4: kmap = 8'h28; 8'hd5: kmap = 8'h14; 8'hd6: kmap = 8'h00; 8'hd7: kmap = 8'h00;
			8'hd8: kmap = 8'h00; 8'hd9: kmap = 8'h74; 8'hda: kmap = 8'h00; 8'hdb: kmap = 8'h00;
			8'hdc: kmap = 8'h00; 8'hdd: kmap = 8'h00; 8'hde: kmap = 8'h00; 8'hdf: kmap = 8'h00;
			8'he0: kmap = 8'h68; 8'he1: kmap = 8'h56; 8'he2: kmap = 8'h00; 8'he3: kmap = 8'h30;
			8'he4: kmap = 8'h5a; 8'he5: kmap = 8'h00; 8'he6: kmap = 8'h00; 8'he7: kmap = 8'h00;
			8'he8: kmap = 8'h00; 8'he9: kmap = 8'h4e; 8'hea: kmap = 8'h00; 8'heb: kmap = 8'h2a;
			8'hec: kmap = 8'h18; 8'hed: kmap = 8'h00; 8'hee: kmap = 8'h00; 8'hef: kmap = 8'h00;
			8'hf0: kmap = 8'h4c; 8'hf1: kmap = 8'h00; 8'hf2: kmap = 8'h00; 8'hf3: kmap = 8'h3a;
			8'hf4: kmap = 8'h00; 8'hf5: kmap = 8'h34; 8'hf6: kmap = 8'h10; 8'hf7: kmap = 8'h78;
			8'hf8: kmap = 8'h00; 8'hf9: kmap = 8'h3c; 8'hfa: kmap = 8'h16; 8'hfb: kmap = 8'h00;
			8'hfc: kmap = 8'h3e; 8'hfd: kmap = 8'h00; 8'hfe: kmap = 8'h00; 8'hff: kmap = 8'h00;
			default: kmap = 8'h00;
		endcase
	endfunction

	function automatic [15:0] lfsr_step(input [15:0] x, input [15:0] t);
		lfsr_step = { x[14:0], ^(x & t) };
	endfunction

	logic [15:0] eng_x_next;
	assign eng_x_next = lfsr_step(eng_x, taps);

	// a write in DECIPHER mode (re)arms the engine and aborts any run still in
	// progress (MAME: last write wins); busy stays high up to about 6 us at clk_cpu = 42.955 MHz
	logic restart;
	assign restart = we && in_data_range && (mode == MODE_DECIPHER);

	always_ff @(posedge clk_cpu) begin
		if (reset) begin
			mode      <= MODE_IDLE;
			taps      <= 16'h0000;
			reply     <= 16'h0000;
			override  <= 1'b0;
			dout      <= 16'h0000;
			eng_state <= ENG_IDLE;
			busy      <= 1'b0;
		end else begin
			override <= 1'b0;

			if (we) begin
				if (in_data_range && mode == MODE_SETKEY) begin
					ram[data_index] <= din[7:0];
				end else if (!restart) begin
					if (addr == PR_CHAR0 || addr == PR_CHAR1 || addr == PR_CHAR2) begin
						case (din)
							16'h2694: taps <= 16'hbcc8; // Sauron, Diablo
							16'h6ee0: taps <= 16'haed5; // Blizzard, Talon
							16'h34f7: taps <= 16'h9d79; // Chaos
							16'h32b9: taps <= 16'hfd10; // Vertigo
							16'h4d5a: taps <= 16'h82a3; // Armadon
							default: ; // unknown word: taps unchanged
						endcase
					end
				end
			end

			if (rd) begin
				if (addr == PR_SETKEY) begin
					mode <= MODE_SETKEY;
				end else if (addr == PR_DECIPHER) begin
					mode <= MODE_DECIPHER;
				end else if (addr == PR_DONE0 || addr == PR_DONE4) begin
					if (mode == MODE_SETKEY)
						mode <= MODE_IDLE;
				end else if (addr == PR_STATUS) begin
					override <= 1'b1;
					dout     <= 16'h8000;
				end else if (addr == PR_RESULT) begin
					if (mode == MODE_DECIPHER) begin
						override <= 1'b1;
						dout     <= reply; // cpu_bus stalls on busy, so this is always the fresh reply
						mode     <= MODE_IDLE;
					end
				end
			end

			// decipher engine, one LFSR clock per cycle; restart always wins over
			// whatever step the engine was mid-way through
			if (restart) begin
				eng_c     <= din;
				eng_koff  <= key_offset(data_index);
				eng_state <= ENG_FETCH;
				busy      <= 1'b1;
			end else begin
				case (eng_state)
					ENG_IDLE: ; // waits for a write in DECIPHER mode to arm it

					ENG_FETCH: begin
						eng_kbyte <= ram[eng_koff]; // synchronous read, registered address
						eng_state <= ENG_KMAP;
					end

					ENG_KMAP: begin
						eng_n     <= kmap(eng_kbyte); // synchronous ROM read, registered address
						eng_state <= ENG_SETUP;
					end

					ENG_SETUP: begin
						eng_x      <= (eng_c == 16'h0000) ? 16'h0001 : eng_c;
						eng_clocks <= (eng_c == 16'h0000)
										? (eng_n == 8'h00 ? 8'h00 : eng_n - 8'h01)
										: eng_n;
						eng_cnt    <= 8'h00;
						eng_early  <= 1'b0;
						eng_state  <= ENG_LOOP1;
					end

					ENG_LOOP1: begin
						if (eng_cnt == eng_clocks) begin
							eng_state <= ENG_CHECK;
						end else begin
							eng_x   <= eng_x_next;
							if (eng_x_next == 16'h0001)
								eng_early <= 1'b1;
							eng_cnt <= eng_cnt + 8'h01;
						end
					end

					ENG_CHECK: begin
						if (eng_early) begin
							if (eng_x == 16'h0001) begin
								reply     <= 16'h0000;
								eng_state <= ENG_IDLE;
								busy      <= 1'b0;
							end else begin
								eng_x     <= eng_c;
								eng_cnt   <= 8'h00;
								eng_state <= ENG_LOOP2;
							end
						end else begin
							reply     <= eng_x;
							eng_state <= ENG_IDLE;
							busy      <= 1'b0;
						end
					end

					ENG_LOOP2: begin
						if (eng_cnt == eng_clocks - 8'h01) begin
							reply     <= eng_x;
							eng_state <= ENG_IDLE;
							busy      <= 1'b0;
						end else begin
							eng_x   <= eng_x_next;
							eng_cnt <= eng_cnt + 8'h01;
						end
					end

					default: eng_state <= ENG_IDLE;
				endcase
			end
		end
	end

endmodule
