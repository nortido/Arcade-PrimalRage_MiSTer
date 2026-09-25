// TMS320C31 core for the AtariGT CAGE board.
// Semantics follow MAME's 320c3x_ops.ipp instruction by instruction,
// not real silicon. Multi-cycle, one memory port, non-pipelined.

module c31_core (
	input  logic        clk,
	input  logic        reset,
	input  logic        run,
	// sampled into pc on reset release; the boot ROM entry point, since
	// this core has no other way to start anywhere but 0
	input  logic [23:0] boot_pc,

	output logic        mem_req,
	output logic        mem_we,
	output logic [23:0] mem_addr,
	output logic [31:0] mem_wdata,
	input  logic [31:0] mem_rdata,
	input  logic        mem_ack,

	input  logic [3:0]  int_n,
	input  logic        xint,
	input  logic        rint,
	input  logic        tint0,
	input  logic        tint1,
	input  logic        dint,

	// for bits where iof_in_mask is set, the IOF register reads iof_in instead of
	// its stored bit (bits 3 and 7 are driven by the CAGE host interface, matching
	// MAME's set_state_int(TMS320C3X_IOF)); writes to IOF always go to storage,
	// masked bits are simply overridden again on the next read
	input  logic [7:0]  iof_in,
	input  logic [7:0]  iof_in_mask,

	output logic        insn_done,
	output logic [23:0] pc,

	// architectural state read-outs for verification and debug
	output logic [39:0] r0, r1, r2, r3, r4, r5, r6, r7,
	output logic [23:0] ar0, ar1, ar2, ar3, ar4, ar5, ar6, ar7,
	output logic [31:0] ir0, ir1, bk, sp, st, ie, if_, iof, rs, re, rc, dp
);

	// ---------------------------------------------------------------
	// register file: R0-R7 double as float mantissa/int, plus exponent
	// ---------------------------------------------------------------
	logic [31:0]        rmant [0:27] /* verilator public */;
	logic signed [7:0]  rexp  [0:7] /* verilator public */;

	localparam int R0=0, AR0=8, DP=16, IR0=17, IR1=18, BK=19, SP=20,
					ST=21, IE=22, IFR=23, IOF=24, RS=25, RE=26, RC=27;

	// architectural state read-outs for debug
	assign r0 = {rexp[0], rmant[0]}; assign r1 = {rexp[1], rmant[1]};
	assign r2 = {rexp[2], rmant[2]}; assign r3 = {rexp[3], rmant[3]};
	assign r4 = {rexp[4], rmant[4]}; assign r5 = {rexp[5], rmant[5]};
	assign r6 = {rexp[6], rmant[6]}; assign r7 = {rexp[7], rmant[7]};
	assign ar0=rmant[8][23:0]; assign ar1=rmant[9][23:0]; assign ar2=rmant[10][23:0]; assign ar3=rmant[11][23:0];
	assign ar4=rmant[12][23:0]; assign ar5=rmant[13][23:0]; assign ar6=rmant[14][23:0]; assign ar7=rmant[15][23:0];
	assign dp=rmant[DP]; assign ir0=rmant[IR0]; assign ir1=rmant[IR1]; assign bk=rmant[BK];
	assign sp=rmant[SP]; assign st=rmant[ST]; assign ie=rmant[IE]; assign if_=rmant[IFR];
	wire [31:0] iof_effective = {rmant[IOF][31:8], (rmant[IOF][7:0] & ~iof_in_mask) | (iof_in & iof_in_mask)};
	assign iof=iof_effective; assign rs=rmant[RS]; assign re=rmant[RE]; assign rc=rmant[RC];

	logic [31:0] bkmask;
	logic        delay_active;
	logic [1:0]  delay_count;
	logic [23:0] delay_target;
	logic        delay_has_target;
	logic        irq_pending_latch;

	// ---------------------------------------------------------------
	// indirect addressing modifier
	// ---------------------------------------------------------------
	// step (1/IR0/IR1/disp8) is resolved by the caller, one cycle earlier, off
	// opcode fields; this unit only adds/subtracts and does the circular compare
	// on registers, so it fits in S_ADDR's one-cycle budget on its own
	task automatic indaddr(
		input  logic [4:0]  modf,
		input  logic [31:0] ar_val,
		input  logic [31:0] step,
		input  logic [31:0] bkv,
		input  logic [31:0] bkmaskv,
		output logic [31:0] addr,
		output logic [31:0] new_ar,
		output logic        wb,
		output logic        illegal
	);
		logic [31:0] temp, temp_dec, temp_sub, temp_add, new_temp_inc, new_temp_dec;
		begin
			illegal = 1'b0;
			wb      = 1'b0;
			new_ar  = ar_val;
			addr    = ar_val;
			if (modf[4:3] == 2'b11) begin
				if (modf[2:0] == 3'b000) begin
					addr = ar_val; // *ARn, no modification
				end else begin
					// mod19 (bit-reversed) and 0x1A-0x1F: MAME's handlers just log and
					// return address 0 with no AR side effect, and the access still
					// happens, so match that rather than aborting the instruction
					addr = 32'h0;
				end
			end else begin
				case (modf[2:0])
					3'b000: addr = ar_val + step;
					3'b001: addr = ar_val - step;
					3'b010: begin new_ar = ar_val + step; addr = new_ar; wb = 1'b1; end
					3'b011: begin new_ar = ar_val - step; addr = new_ar; wb = 1'b1; end
					3'b100: begin addr = ar_val; new_ar = ar_val + step; wb = 1'b1; end
					3'b101: begin addr = ar_val; new_ar = ar_val - step; wb = 1'b1; end
					3'b110: begin
						// temp and temp-bk run in parallel; the compare only picks
						// which of the two already-computed sums to keep
						addr = ar_val;
						temp_add = (ar_val & bkmaskv) + step;
						temp_sub = temp_add - bkv;
						new_temp_inc = (temp_add >= bkv) ? temp_sub : temp_add;
						new_ar = (ar_val & ~bkmaskv) | (new_temp_inc & bkmaskv);
						wb = 1'b1;
					end
					3'b111: begin
						addr = ar_val;
						temp_dec = (ar_val & bkmaskv) - step;
						temp = temp_dec + bkv;
						new_temp_dec = ($signed(temp_dec) < 0) ? temp : temp_dec;
						new_ar = (ar_val & ~bkmaskv) | (new_temp_dec & bkmaskv);
						wb = 1'b1;
					end
					default: ;
				endcase
			end
		end
	endtask

	// ---------------------------------------------------------------
	// leading zero / one count, 32-bit
	// ---------------------------------------------------------------
	// an arithmetic shift written inline next to an unsigned operand turns logical
	// (context signedness), so every >>> on a 32-bit value goes through here
	function automatic logic [31:0] asr32(input logic [31:0] v, input int n);
		asr32 = $signed(v) >>> n;
	endfunction

	function automatic int clz32(input logic [31:0] v);
		int i;
		logic [3:0] nz;
		logic [3:0][2:0] c;
		begin
			// byte tree instead of a 32-step priority chain (timing)
			for (i = 0; i < 4; i = i + 1) begin
				nz[i] = |v[8*i +: 8];
				casez (v[8*i +: 8])
					8'b1???????: c[i] = 3'd0;
					8'b01??????: c[i] = 3'd1;
					8'b001?????: c[i] = 3'd2;
					8'b0001????: c[i] = 3'd3;
					8'b00001???: c[i] = 3'd4;
					8'b000001??: c[i] = 3'd5;
					8'b0000001?: c[i] = 3'd6;
					default:     c[i] = 3'd7;
				endcase
			end
			clz32 = nz[3] ? {2'd0, c[3]} : nz[2] ? {2'd1, c[2]} :
					nz[1] ? {2'd2, c[1]} : nz[0] ? {2'd3, c[0]} : 32;
		end
	endfunction

	function automatic int clo32(input logic [31:0] v);
		clo32 = clz32(~v);
	endfunction

	// ---------------------------------------------------------------
	// float helpers, matching 320c3x_ops.ipp bit for bit
	// ---------------------------------------------------------------
	task automatic fneg(
		input  logic signed [7:0]  se,
		input  logic [31:0]        sm,
		output logic signed [7:0]  de,
		output logic [31:0]        dm
	);
		begin
			if (se == 8'sh80) begin
				dm = 32'h0; de = 8'sh80;
			end else if ((sm & 32'h7fffffff) != 0) begin
				dm = -sm; de = se;
			end else if (sm == 32'h0) begin
				dm = sm ^ 32'h80000000; de = se - 8'sd1;
			end else begin
				dm = sm ^ 32'h80000000; de = se + 8'sd1;
			end
		end
	endtask
	// ---------------------------------------------------------------
	// shared arithmetic unit
	// ---------------------------------------------------------------
	// one float add/sub/multiply/convert and one 32x32 signed multiplier for the
	// whole ISA. Every ISA group that used to inline its own copy now raises a
	// request and reads the result registers, so the heavy datapath exists once
	// and never sits between a register-file read and a register-file write.
	typedef enum logic [2:0] {
		FQ_NONE, FQ_ADD, FQ_SUB, FQ_MPY, FQ_IMPY, FQ_I2F, FQ_F2I, FQ_NORM
	} fqop_t;

	// request, muxed from selects decoded off raw opcode and latched at S_DECODE,
	// so S_EXEC never waits on the opcode decode; fq2 is the second
	// operation of the parallel MPY+ALU forms, run after the first
	fqop_t             fq_op, fq2_op;
	logic signed [7:0] fq_e1, fq_e2, fq2_e1, fq2_e2;
	logic [31:0]       fq_m1, fq_m2, fq2_m1, fq2_m2;
	typedef enum logic [4:0] {
		SRC_ZERO, SRC_FD, SRC_FB, SRC_FB5, SRC_IMM, SRC_OPA_F, SRC_OPB_F, SRC_WB, SRC_A24, SRC_WB24,
		SRC_FS1, SRC_S1_24, SRC_S2_24, SRC_P1, SRC_P3, SRC_OPB_I, SRC_P1_24, SRC_P3_24, SRC_OPA_24, SRC_OPB_24
	} fqsrc_t;
	typedef enum logic [3:0] {
		EXP_ZERO, EXP_FD, EXP_FB, EXP_FB5, EXP_IMM, EXP_OPA, EXP_OPB, EXP_FS1, EXP_P1, EXP_P3
	} fqexp_t;
	fqop_t  fq_op_d, fq2_op_d;
	fqsrc_t fqm1_sel_d, fqm2_sel_d, fq2m1_sel_d, fq2m2_sel_d;
	fqexp_t fqe1_sel_d, fqe2_sel_d, fq2e1_sel_d, fq2e2_sel_d;

	// latched request and pipeline state
	fqop_t              fu_op;
	logic signed [7:0]  fu_e1, fu_e2;
	logic [31:0]        fu_m1, fu_m2;
	logic [2:0]         fu_stage;
	logic               fu_slot;
	logic signed [63:0] fu_mm1, fu_mm2, fu_man, fu_prod;
	logic signed [31:0] fu_exp;
	logic [5:0]         fu_cnt;
	logic               fu_is_sub, fu_zero;

	// staged result, then the per-slot result the execute block reads
	logic signed [7:0]  fo_e;
	logic [31:0]        fo_m;
	logic               fo_v, fo_u;
	logic signed [7:0]  f1_e, f2_e;
	logic [31:0]        f1_m, f2_m;
	logic               f1_v, f1_u, f2_v, f2_u;
	logic signed [63:0] f1_p;

	// the one multiplier: float mantissas arrive as 1.1.23, integers pre-widened
	wire signed [31:0] mul_a = (fu_op == FQ_MPY) ? (asr32(fu_m1, 8) ^ 32'h00800000) : fu_m1;
	wire signed [31:0] mul_b = (fu_op == FQ_MPY) ? (asr32(fu_m2, 8) ^ 32'h00800000) : fu_m2;


	// ---------------------------------------------------------------
	// FSM
	// ---------------------------------------------------------------
	typedef enum logic [4:0] {
		S_IRQCHECK, S_TRAP_PUSH_REQ, S_TRAP_PUSH_WAIT, S_TRAP_VEC_REQ, S_TRAP_VEC_WAIT,
		S_FETCH, S_FETCH_WAIT, S_DECODE, S_ADDR,
		S_OPA_REQ, S_OPA_WAIT, S_OPB_REQ, S_OPB_WAIT,
		S_EXEC, S_FU, S_SH, S_STORE_REQ, S_STORE_WAIT, S_STORE2_REQ, S_STORE2_WAIT, S_POP_WAIT, S_WRITEBACK, S_COMMIT
	} state_t;

	state_t state, state_n;

	logic [31:0] opcode;
	logic [23:0] fetch_pc;
	// registered one cycle behind opcode, latched at the end of S_DECODE: breaks
	// the opcode -> class mux -> ALU -> register write combinational cone into
	// two shorter register-to-register paths. Exec/writeback read only these
	// _d fields; the decode-address block above still reads raw opcode since it
	// runs and gets latched into dc_opa_*/dc_opb_* during S_DECODE itself.
	logic [31:0] opcode_d;
	// AND/ANDN/NOT/OR/RPTS/TSTB/XOR take a zero-extended imm16 (MAME uint16_t), the rest sign-extend
	logic        imm_zx_d;
	// top-level exec branch, classified off raw idx11 and latched with opcode_d
	typedef enum logic [4:0] {
		CLS_TWO, CLS_THREE, CLS_LDFC, CLS_LDIC, CLS_BR, CLS_BRD, CLS_CALL, CLS_RPTB,
		CLS_BCR, CLS_BCI, CLS_DBR, CLS_DBI, CLS_CALLCR, CLS_CALLCI, CLS_TRAPC, CLS_RETIC,
		CLS_RETSC, CLS_PMPY, CLS_PST, CLS_ILLEGAL
	} cls_t;
	cls_t        cls_d, dcls;

	// decoded fields (raw opcode: decode-address block only, latched into dc_opa_*/
	// dc_opb_* during S_DECODE; the exec/writeback cone uses the _d copies below)
	wire [1:0]  g2      = opcode[22:21];
	wire [15:0] imm16   = opcode[15:0];
	wire [4:0]  ind_mod = opcode[15:11];
	wire [2:0]  ind_ar  = opcode[10:8];
	wire [7:0]  disp8   = opcode[7:0];
	wire [10:0] idx11   = opcode[31:21];
	wire [4:0]  pgroup  = (idx11 - 11'h600) >> 4;

	// three-operand fields
	wire [7:0]  s1field  = opcode[15:8];
	wire [7:0]  s2field  = opcode[7:0];

	always @* begin
		if (idx11 < 11'h0DC) dcls = CLS_TWO;
		else if (idx11 >= 11'h100 && idx11 < 11'h144) dcls = CLS_THREE;
		else if (idx11 >= 11'h200 && idx11 < 11'h254) dcls = CLS_LDFC;
		else if (idx11 >= 11'h280 && idx11 < 11'h2D4) dcls = CLS_LDIC;
		else if (idx11 >= 11'h300 && idx11 < 11'h308) dcls = CLS_BR;
		else if (idx11 >= 11'h308 && idx11 < 11'h310) dcls = CLS_BRD;
		else if (idx11 >= 11'h310 && idx11 < 11'h318) dcls = CLS_CALL;
		else if (idx11 >= 11'h320 && idx11 < 11'h328) dcls = CLS_RPTB;
		else if (idx11 == 11'h340 || idx11 == 11'h341) dcls = CLS_BCR;
		else if (idx11 == 11'h350 || idx11 == 11'h351) dcls = CLS_BCI;
		else if (idx11 >= 11'h360 && idx11 < 11'h370) dcls = CLS_DBR;
		else if (idx11 >= 11'h370 && idx11 < 11'h380) dcls = CLS_DBI;
		else if (idx11 >= 11'h380 && idx11 < 11'h388) dcls = CLS_CALLCR;
		else if (idx11 >= 11'h390 && idx11 < 11'h398) dcls = CLS_CALLCI;
		else if (idx11 >= 11'h3A0 && idx11 < 11'h3A8) dcls = CLS_TRAPC;
		else if (idx11 >= 11'h3C0 && idx11 < 11'h3C4) dcls = CLS_RETIC;
		else if (idx11 >= 11'h3C4 && idx11 < 11'h3C8) dcls = CLS_RETSC;
		else if (idx11 >= 11'h400 && idx11 < 11'h480) dcls = CLS_PMPY;
		else if (idx11 >= 11'h600 && idx11 < 11'h780) dcls = CLS_PST;
		else dcls = CLS_ILLEGAL;
	end

	// same fields, one cycle late, for the exec/writeback cone only
	wire [8:0]  mbase_d   = opcode_d[31:23];
	wire [1:0]  g2_d      = opcode_d[22:21];
	wire [4:0]  dreg_d    = opcode_d[20:16];
	wire [15:0] imm16_d   = opcode_d[15:0];
	wire signed [23:0] simm16_d = {{8{imm16_d[15]}}, imm16_d};
	wire signed [7:0] simm_exp_d = 8'($signed(imm16_d) >>> 12);
	wire [31:0]       simm_man_d = {imm16_d[11:0], 20'h0};
	wire [10:0] idx11_d   = opcode_d[31:21];
	wire [4:0]  pgroup_d  = (idx11_d - 11'h600) >> 4;
	wire [8:0]  m3base_d  = opcode_d[31:23];
	wire [4:0]  m3dreg_d  = opcode_d[20:16];
	// decode-address block reads these off opcode_d (S_ADDR runs one cycle
	// after S_DECODE latches opcode_d, see needs_addr_raw below)
	wire [4:0]  ind_mod_d = opcode_d[15:11];
	wire [2:0]  ind_ar_d  = opcode_d[10:8];
	wire [7:0]  disp8_d   = opcode_d[7:0];
	wire [7:0]  s1field_d = opcode_d[15:8];
	wire [7:0]  s2field_d = opcode_d[7:0];

	// register-file operands the exec cone needs, read during S_DECODE off the
	// raw (not yet registered) opcode so S_EXEC never chains a register-file
	// read into the ALU into a writeback in one cycle. Same bit positions the
	// _d fields above use, just one cycle earlier off raw opcode instead of
	// opcode_d.
	logic [31:0] da_val_r, db_val_r, s1_val_r;
	// fwd_reg() resolved one state before S_EXEC for a (dreg_d), b (opcode_d[4:0])
	// and s1 (opcode_d[12:8]), so the exec cone reads plain registers
	logic [31:0] a_r;
	logic [31:0] b_r, s1_r;
	// S_EXEC always precedes S_WRITEBACK and w_b/w_s1v/w_s2v hold still until
	// S_COMMIT, so writeback-only ALU arms read this copy latched at S_EXEC and
	// the operand muxes leave the S_WRITEBACK cone
	logic [31:0] x_b, x_s1, x_s2;
	logic signed [7:0] fd_e_r, fb_e_r, fs1_e_r, p1_e_r, p3_e_r, ppd1_e_r;
	logic [31:0]       fd_m_r, fb_m_r, fs1_m_r, p1_m_r, p3_m_r, ppd1_m_r;
	logic [31:0] db_ar_val_r;
	// MPYF/SUBRF's second float operand reads the full 5-bit field (unlike its
	// siblings ADDF/SUBF/CMPF which mask to [2:0]); PUSHF/STF/LDE/LDM read dreg
	// masked to [2:0] (unlike ADDF/SUBF/MPYF/SUBRF which use the full dreg).
	// Distinct latches so neither group silently reads the wrong register.
	logic signed [7:0] fb5_e_r, fd3_e_r;
	logic [31:0]       fb5_m_r, fd3_m_r;

	// mnemonic bases, two-operand table (op[31:23])
	localparam [8:0]
		M_ABSF=9'h00, M_ABSI=9'h01, M_ADDC=9'h02, M_ADDF=9'h03, M_ADDI=9'h04, M_AND=9'h05,
		M_ANDN=9'h06, M_ASH=9'h07, M_CMPF=9'h08, M_CMPI=9'h09, M_FIX=9'h0A,
		M_FLOAT=9'h0B, M_IDLE=9'h0C, M_LDE=9'h0D, M_LDF=9'h0E, M_LDI=9'h10, M_LDM=9'h12, M_LSH=9'h13,
		M_MPYF=9'h14, M_MPYI=9'h15, M_NEGB=9'h16, M_NEGF=9'h17, M_NEGI=9'h18, M_NOP=9'h19,
		M_NORM=9'h1A, M_NOT=9'h1B, M_POP=9'h1C, M_POPF=9'h1D, M_PUSH=9'h1E, M_PUSHF=9'h1F,
		M_OR=9'h20, M_RND=9'h22, M_ROL=9'h23, M_ROLC=9'h24, M_ROR=9'h25, M_RORC=9'h26, M_RPTS=9'h27,
		M_STF=9'h28, M_STI=9'h2A, M_SUBB=9'h2D, M_SUBC=9'h2E, M_SUBF=9'h2F, M_SUBI=9'h30,
		M_SUBRB=9'h31, M_SUBRF=9'h32, M_SUBRI=9'h33, M_TSTB=9'h34, M_XOR=9'h35, M_IACK=9'h36;

	localparam [8:0]
		M3_ADDC3=9'h40, M3_ADDF3=9'h41, M3_ADDI3=9'h42, M3_AND3=9'h43, M3_ANDN3=9'h44,
		M3_ASH3=9'h45, M3_CMPF3=9'h46, M3_CMPI3=9'h47, M3_LSH3=9'h48, M3_MPYF3=9'h49,
		M3_MPYI3=9'h4A, M3_OR3=9'h4B, M3_SUBB3=9'h4C, M3_SUBF3=9'h4D, M3_SUBI3=9'h4E,
		M3_TSTB3=9'h4F, M3_XOR3=9'h50;

	// shallow pre-check on raw opcode: does this instruction need S_ADDR at all, mirrors
	// the branches below that set opa_is_mem/opb_is_mem or compute pstore/qstore; register-only
	// forms and the no-read direct STI/STF/PUSH/POP family ({DP, imm16} or SP) skip S_ADDR
	wire two_nord = opcode[31:23] == M_STI || opcode[31:23] == M_STF || opcode[31:23] == M_PUSH ||
					opcode[31:23] == M_PUSHF || opcode[31:23] == M_POP || opcode[31:23] == M_POPF;
	wire needs_addr_raw =
		(idx11 < 11'h0DC && (g2 == 2'b01 || g2 == 2'b10) && !(two_nord && g2 == 2'b01)) ||
		(idx11 >= 11'h100 && idx11 < 11'h144 && g2 != 2'b00) ||
		(idx11 >= 11'h200 && idx11 < 11'h254 && (g2 == 2'b01 || g2 == 2'b10)) ||
		(idx11 >= 11'h280 && idx11 < 11'h2D4 && (g2 == 2'b01 || g2 == 2'b10)) ||
		(idx11 >= 11'h400 && idx11 < 11'h480) ||
		(idx11 >= 11'h600 && idx11 < 11'h780);

	// fq request selects off raw opcode, the same rows and operands the exec arms used
	fqop_t  dq_op, dq2_op;
	fqsrc_t dq_m1, dq_m2, dq2_m1, dq2_m2;
	fqexp_t dq_e1, dq_e2, dq2_e1, dq2_e2;
	always @* begin
		fqexp_t s2e, s2e5, me1, me2, ae1, ae2;
		fqsrc_t s2m, s2m5, mm1, mm2, mi1, mi2, am1, am2;
		dq_op = FQ_NONE; dq2_op = FQ_NONE;
		dq_e1 = EXP_ZERO; dq_m1 = SRC_ZERO; dq_e2 = EXP_ZERO; dq_m2 = SRC_ZERO;
		dq2_e1 = EXP_ZERO; dq2_m1 = SRC_ZERO; dq2_e2 = EXP_ZERO; dq2_m2 = SRC_ZERO;
		me1 = EXP_ZERO; me2 = EXP_ZERO; ae1 = EXP_ZERO; ae2 = EXP_ZERO;
		mm1 = SRC_ZERO; mm2 = SRC_ZERO; mi1 = SRC_ZERO; mi2 = SRC_ZERO; am1 = SRC_ZERO; am2 = SRC_ZERO;
		// two-operand src2: FB (full 5-bit field for MPYF/SUBRF), imm, or a memory word
		if (g2 == 2'b00) begin s2e = EXP_FB; s2m = SRC_FB; s2e5 = EXP_FB5; s2m5 = SRC_FB5; end
		else if (g2 == 2'b11) begin s2e = EXP_IMM; s2m = SRC_IMM; s2e5 = EXP_IMM; s2m5 = SRC_IMM; end
		else begin s2e = EXP_OPA; s2m = SRC_OPA_F; s2e5 = EXP_OPA; s2m5 = SRC_OPA_F; end
		if (idx11 < 11'h0DC) begin
			case (opcode[31:23])
				M_ADDF: begin dq_op = FQ_ADD; dq_e1 = EXP_FD; dq_m1 = SRC_FD; dq_e2 = s2e; dq_m2 = s2m; end
				M_CMPF, M_SUBF: begin dq_op = FQ_SUB; dq_e1 = EXP_FD; dq_m1 = SRC_FD; dq_e2 = s2e; dq_m2 = s2m; end
				M_MPYF: begin dq_op = FQ_MPY; dq_e1 = EXP_FD; dq_m1 = SRC_FD; dq_e2 = s2e5; dq_m2 = s2m5; end
				M_SUBRF: begin dq_op = FQ_SUB; dq_e1 = s2e5; dq_m1 = s2m5; dq_e2 = EXP_FD; dq_m2 = SRC_FD; end
				M_FIX: begin dq_op = FQ_F2I; dq_e1 = s2e; dq_m1 = s2m; end
				M_NORM: begin dq_op = FQ_NORM; dq_e1 = s2e; dq_m1 = s2m; end
				M_FLOAT: begin dq_op = FQ_I2F; dq_m1 = SRC_WB; end
				M_MPYI: begin dq_op = FQ_IMPY; dq_m1 = SRC_A24; dq_m2 = SRC_WB24; end
				default: ;
			endcase
		end else if (idx11 >= 11'h100 && idx11 < 11'h144) begin
			// float 3-op forms only ever use R0-R7 for a register operand (3-bit field)
			// and a memory operand is a packed 32-bit float word (LONG2FP unpack)
			if (g2[0] == 1'b0) begin me1 = EXP_FS1; mm1 = SRC_FS1; end
			else begin me1 = EXP_OPA; mm1 = SRC_OPA_F; end
			if (g2[1] == 1'b0) begin me2 = EXP_FB; mm2 = SRC_FB; end
			else begin me2 = EXP_OPB; mm2 = SRC_OPB_F; end
			case (opcode[31:23])
				M3_ADDF3: begin dq_op = FQ_ADD; dq_e1 = me1; dq_m1 = mm1; dq_e2 = me2; dq_m2 = mm2; end
				M3_MPYF3: begin dq_op = FQ_MPY; dq_e1 = me1; dq_m1 = mm1; dq_e2 = me2; dq_m2 = mm2; end
				M3_SUBF3, M3_CMPF3: begin dq_op = FQ_SUB; dq_e1 = me1; dq_m1 = mm1; dq_e2 = me2; dq_m2 = mm2; end
				M3_MPYI3: begin dq_op = FQ_IMPY; dq_m1 = SRC_S1_24; dq_m2 = SRC_S2_24; end
				default: ;
			endcase
		end else if (idx11 >= 11'h400 && idx11 < 11'h480) begin
			// mul pair (float and 24-bit integer) and add pair per pattern idx11[4:3]
			case (idx11[4:3])
				2'd0: begin
					me1 = EXP_OPA; mm1 = SRC_OPA_F; mi1 = SRC_OPA_24; me2 = EXP_OPB; mm2 = SRC_OPB_F; mi2 = SRC_OPB_24;
					ae1 = EXP_P1; am1 = SRC_P1; ae2 = EXP_P3; am2 = SRC_P3;
				end
				2'd1: begin
					me1 = EXP_OPA; mm1 = SRC_OPA_F; mi1 = SRC_OPA_24; me2 = EXP_P1; mm2 = SRC_P1; mi2 = SRC_P1_24;
					ae1 = EXP_OPB; am1 = SRC_OPB_F; ae2 = EXP_P3; am2 = SRC_P3;
				end
				2'd2: begin
					me1 = EXP_P1; mm1 = SRC_P1; mi1 = SRC_P1_24; me2 = EXP_P3; mm2 = SRC_P3; mi2 = SRC_P3_24;
					ae1 = EXP_OPA; am1 = SRC_OPA_F; ae2 = EXP_OPB; am2 = SRC_OPB_F;
				end
				default: begin
					me1 = EXP_OPA; mm1 = SRC_OPA_F; mi1 = SRC_OPA_24; me2 = EXP_P1; mm2 = SRC_P1; mi2 = SRC_P1_24;
					ae1 = EXP_P3; am1 = SRC_P3; ae2 = EXP_OPB; am2 = SRC_OPB_F;
				end
			endcase
			if (idx11[6]) begin
				dq_op = FQ_IMPY; dq_m1 = mi1; dq_m2 = mi2;
			end else begin
				dq_op = FQ_MPY; dq_e1 = me1; dq_m1 = mm1; dq_e2 = me2; dq_m2 = mm2;
				if (idx11[5]) dq2_op = FQ_SUB; else dq2_op = FQ_ADD;
				dq2_e1 = ae1; dq2_m1 = am1; dq2_e2 = ae2; dq2_m2 = am2;
			end
		end else if (idx11 >= 11'h600 && idx11 < 11'h780) begin
			case (pgroup)
				5'd6:  begin dq_op = FQ_ADD; dq_e1 = EXP_P1; dq_m1 = SRC_P1; dq_e2 = EXP_OPB; dq_m2 = SRC_OPB_F; end
				5'd10: begin dq_op = FQ_F2I; dq_e1 = EXP_OPB; dq_m1 = SRC_OPB_F; end
				5'd11: begin dq_op = FQ_I2F; dq_m1 = SRC_OPB_I; end
				5'd15: begin dq_op = FQ_MPY; dq_e1 = EXP_P1; dq_m1 = SRC_P1; dq_e2 = EXP_OPB; dq_m2 = SRC_OPB_F; end
				5'd16: begin dq_op = FQ_IMPY; dq_m1 = SRC_P1_24; dq_m2 = SRC_OPB_24; end
				5'd21: begin dq_op = FQ_SUB; dq_e1 = EXP_OPB; dq_m1 = SRC_OPB_F; dq_e2 = EXP_P1; dq_m2 = SRC_P1; end
				default: ;
			endcase
		end
	end

	// condition table (21-way OR of the flag bits, matches condition_table[])
	function automatic logic cond_true(input logic [4:0] which, input logic [13:0] stv);
		logic c,v,z,n,uf,lv,luf;
		begin
			c=stv[0]; v=stv[1]; z=stv[2]; n=stv[3]; uf=stv[4]; lv=stv[5]; luf=stv[6];
			case (which)
				5'd0:  cond_true = 1'b1;
				5'd1:  cond_true = c;
				5'd2:  cond_true = c | z;
				5'd3:  cond_true = ~c & ~z;
				5'd4:  cond_true = ~c;
				5'd5:  cond_true = z;
				5'd6:  cond_true = ~z;
				5'd7:  cond_true = n;
				5'd8:  cond_true = n | z;
				5'd9:  cond_true = ~n & ~z;
				5'd10: cond_true = ~n;
				5'd12: cond_true = ~v;
				5'd13: cond_true = v;
				5'd14: cond_true = ~uf;
				5'd15: cond_true = uf;
				5'd16: cond_true = ~lv;
				5'd17: cond_true = lv;
				5'd18: cond_true = ~luf;
				5'd19: cond_true = luf;
				5'd20: cond_true = uf | z;
				default: cond_true = 1'b0;
			endcase
		end
	endfunction

	// register read helper (0-27 valid, 28-31 read as zero)
	function automatic logic [31:0] rread(input logic [4:0] idx);
		rread = (idx == IOF[4:0]) ? iof_effective : (idx < 28) ? rmant[idx] : 32'h0;
	endfunction

	// ---------------------------------------------------------------
	// operand fetch bookkeeping
	// ---------------------------------------------------------------
	logic        opa_is_mem, opb_is_mem;
	logic [23:0] opa_addr, opb_addr;
	// address-select pipeline: which AR/mode/step the a/b memory slots use,
	// classified off raw opcode in S_DECODE (same idx11/g2 ranges the old
	// opcode_d-keyed classify block used) so S_ADDR's indaddr call only adds
	// and does the circular compare on already-registered inputs
	logic        dc_a_active, dc_b_active;   // slot needs an address this instruction
	logic        dc_a_is_mem, dc_b_is_mem;   // slot is a read (gates the operand requests)
	logic        dc_a_is_direct;             // a-slot is direct {DP,imm16}, no indaddr
	logic [23:0] dc_a_direct_addr;
	logic [4:0]  dc_a_modf, dc_b_modf;
	logic [31:0] dc_a_arval, dc_b_arval;
	logic [4:0]  dc_a_arid,  dc_b_arid;
	logic [31:0] dc_a_step,  dc_b_step;
	logic [31:0] opa_val, opb_val;
	logic [31:0] opa_ar_new; logic opa_ar_wb; logic [4:0] opa_ar_idx;
	logic [31:0] opb_ar_new; logic opb_ar_wb; logic [4:0] opb_ar_idx;
	// latched at decode. The reference computes both operand addresses before it
	// applies either AR modify (INDIRECT_1_DEF defers the first one to the very end
	// of the instruction), so recomputing them after a writeback would read the
	// wrong word whenever both fields ride the same AR.
	logic [23:0] dc_opa_addr, dc_opb_addr;
	logic [31:0] dc_opa_ar_new, dc_opb_ar_new;
	logic        dc_opa_ar_wb, dc_opb_ar_wb;
	logic [4:0]  dc_opa_ar_idx, dc_opb_ar_idx;
	logic        opa_illegal, opb_illegal;

	// the operand request/ack edges can write an AR back between the S_DECODE latch and
	// S_EXEC (indirect autoincrement/decrement on the same AR a full-width
	// opcode field also names as a plain register operand); forward the new
	// value so a latched da/db/s1_val_r still reads what the live rmant[]
	// read used to see. Opa's deferred write wins when both fields share an AR.
	function automatic logic [31:0] fwd_reg(input logic [4:0] idx, input logic [31:0] latched);
		fwd_reg = latched;
		if (dc_opb_ar_wb && dc_opb_ar_idx == idx) fwd_reg = dc_opb_ar_new;
		if (dc_opa_ar_wb && dc_opa_ar_idx == idx) fwd_reg = dc_opa_ar_new;
	endfunction

	logic        is_store, store_is_float;
	logic [23:0] store_addr;
	logic [31:0] store_data;
	logic [4:0]  store_ar_idx; logic [31:0] store_ar_new; logic store_ar_wb;

	// second store, for the parallel STx||STx forms (two memory writes, no register write)
	logic        is_store2;
	logic [23:0] store2_addr;
	logic [31:0] store2_data;
	logic [4:0]  store2_ar_idx; logic [31:0] store2_ar_new; logic store2_ar_wb;

	// store AR modifies held from the store issue edge to its ack edge
	logic [4:0]  st_ar_idx, st2_ar_idx;
	logic [31:0] st_ar_new, st2_ar_new;
	logic        st_ar_wb, st2_ar_wb;

	logic        wr_en, wr_is_float;
	logic [4:0]  wr_idx;
	logic [31:0] wr_val;
	logic signed [7:0] wr_exp;
	// second register write, for the parallel forms (LDF||LDF, LDI||LDI, and the
	// dual-destination MPY+ALU forms): committed the same cycle as wr_en, always
	// computed from pre-instruction register values so ordering never matters
	logic        wr2_en, wr2_is_float;
	logic [4:0]  wr2_idx;
	logic [31:0] wr2_val;
	logic signed [7:0] wr2_exp;
	logic        touches_flags, clears_c;
	// RND is the only op whose flag clear skips Z (CLR_NVUF, not CLR_NZVUF)
	logic        touch_z;
	logic        new_n, new_z, new_c, new_v, new_uf;
	// MPYF half of a parallel MPYF||ADDF/SUBF: its V/UF only reach the LV/LUF latches
	logic        lat_v, lat_uf;
	logic        illegal_insn;
`ifdef SIMULATION
	// counted at S_WRITEBACK only, so a decode of a word that never executes stays quiet
	logic [31:0] sim_illegal_n = 32'h0;
`endif

	logic [23:0] next_pc;
	logic        branch_taken;
	logic        is_brd, is_rptb, is_trap_now, is_reti, is_special_wr;
	wire         reti_taken = is_reti; // is_reti is already gated by cond_true where it's set
	logic        is_rpts;
	logic [31:0] rpts_count;
	// delayed conditional branch: taken/not-taken still runs 3 delay slots
	logic        is_brcd, is_dbcd;
	logic [4:0]  special_reg;
	// a write to ST/IE/IF re-runs check_irqs synchronously in the reference
	// (update_special), so a trap it triggers is part of the SAME c31_step()/
	// insn_done pulse as the write itself, not a separate one
	wire writes_irq_regs = (is_special_wr && (special_reg == ST[4:0] || special_reg == IE[4:0] || special_reg == IFR[4:0]))
							|| reti_taken; // retic_reg calls check_irqs() synchronously too
	logic pending_recheck;

	// writeback cone snapshot: S_WRITEBACK latches the live combinational cone
	// here instead of committing it directly, so the actual rmant[]/pc writes in
	// S_COMMIT are a plain mux read, not opcode->class->ALU->write in one cycle
	logic        wb_store_ar_wb, wb_is_store;
	logic [4:0]  wb_store_ar_idx;
	logic [31:0] wb_store_ar_new;
	logic        wb_wr_en, wb_wr_is_float;
	logic [4:0]  wb_wr_idx;
	logic [31:0] wb_wr_val;
	logic signed [7:0] wb_wr_exp;
	logic        wb_wr2_en, wb_wr2_is_float;
	logic [4:0]  wb_wr2_idx;
	logic [31:0] wb_wr2_val;
	logic signed [7:0] wb_wr2_exp;
	logic        wb_touches_flags, wb_touch_z, wb_clears_c;
	logic        wb_new_n, wb_new_z, wb_new_v, wb_new_uf, wb_new_c;
	logic        wb_lat_v, wb_lat_uf;
	logic        wb_retscond_taken, wb_reti_taken;
	logic        wb_branch_taken, wb_is_brd, wb_is_brcd, wb_is_dbcd;
	logic [23:0] wb_next_pc;
	logic        wb_is_rptb, wb_is_rpts, wb_is_trap_now;
	logic [31:0] wb_rpts_count;
	logic        wb_writes_irq_regs;
	logic        wb_delay_active;
	logic [1:0]  wb_delay_count;

	logic signed [63:0] mpy_prod;
	logic [2:0]  db_arsel;
	logic [31:0] db_newar;
	logic [31:0] db_arval;
	logic        db_taken;

	// ---------------------------------------------------------------
	// decode: classify instruction, compute operand addresses
	// ---------------------------------------------------------------
	// S_ADDR cone: indaddr just adds/subtracts and does the circular compare on
	// the dc_a_*/dc_b_* registers S_DECODE already selected; no more opcode
	// field classification here (that was the -0.97ns Mux452->Add20->LessThan19
	// path, now split across the S_DECODE/S_ADDR boundary instead of one cycle)
	assign opa_is_mem = dc_a_is_mem;
	assign opb_is_mem = dc_b_is_mem;
	always @* begin
		opa_addr = 24'h0; opb_addr = 24'h0;
		opa_ar_wb = 1'b0; opb_ar_wb = 1'b0;
		opa_ar_new = 32'h0; opb_ar_new = 32'h0;
		opa_ar_idx = dc_a_arid; opb_ar_idx = dc_b_arid;
		opa_illegal = 1'b0; opb_illegal = 1'b0;

		if (dc_a_active) begin
			if (dc_a_is_direct) opa_addr = dc_a_direct_addr;
			else indaddr(dc_a_modf, dc_a_arval, dc_a_step, rmant[BK], bkmask,
						 opa_addr, opa_ar_new, opa_ar_wb, opa_illegal);
		end
		if (dc_b_active)
			indaddr(dc_b_modf, dc_b_arval, dc_b_step, rmant[BK], bkmask,
					 opb_addr, opb_ar_new, opb_ar_wb, opb_illegal);
	end

	// the source operands every group shares, hoisted so the register-file read
	// muxes exist once instead of once per case arm
	// ---------------------------------------------------------------
	// shared source operands and the one barrel shifter behind every
	// ASH/LSH/ASH3/LSH3 and ||STI form
	// ---------------------------------------------------------------
	// one block, and it reads rmant[] directly: a block that only ever touches the
	// register file through rread() does not get rmant[] into Icarus's @* list
	logic [31:0] w_a, w_b, w_s1v, w_s2v;
	logic [31:0] sh_val, sh_cntsrc;
	logic        sh_arith;
	always @* begin
		w_a   = a_r;
		w_b   = (g2_d == 2'b00) ? b_r :
				(g2_d == 2'b11) ? {{16{imm16_d[15] & ~imm_zx_d}}, imm16_d} : opa_val;
		w_s1v = (g2_d[0] == 1'b0) ? s1_r : opa_val;
		w_s2v = (g2_d[1] == 1'b0) ? b_r : opb_val;
		sh_val    = (idx11_d >= 11'h600) ? opb_val :
					(idx11_d >= 11'h100) ? w_s1v : w_a;
		sh_cntsrc = (idx11_d >= 11'h600) ? p1_m_r :
					(idx11_d >= 11'h100) ? w_s2v : w_b;
		sh_arith  = (idx11_d >= 11'h600) ? (pgroup_d == 5'd9) :
					(idx11_d >= 11'h100) ? (m3base_d == M3_ASH3) : (mbase_d == M_ASH);
	end

	// each case lists only the sources the decode above can pick for that operand
	always @* begin
		fq_op = fq_op_d; fq2_op = fq2_op_d;
		fq_e1 = 8'sh0; fq_m1 = 32'h0; fq_e2 = 8'sh0; fq_m2 = 32'h0;
		fq2_e1 = 8'sh0; fq2_m1 = 32'h0; fq2_e2 = 8'sh0; fq2_m2 = 32'h0;
		case (fqe1_sel_d)
			EXP_FD:  fq_e1 = fd_e_r;
			EXP_FB:  fq_e1 = fb_e_r;
			EXP_FB5: fq_e1 = fb5_e_r;
			EXP_IMM: fq_e1 = (imm16_d == 16'h8000) ? 8'sh80 : simm_exp_d;
			EXP_OPA: fq_e1 = opa_val[31:24];
			EXP_OPB: fq_e1 = opb_val[31:24];
			EXP_FS1: fq_e1 = fs1_e_r;
			EXP_P1:  fq_e1 = p1_e_r;
			default: ;
		endcase
		case (fqm1_sel_d)
			SRC_FD:     fq_m1 = fd_m_r;
			SRC_FB:     fq_m1 = fb_m_r;
			SRC_FB5:    fq_m1 = fb5_m_r;
			SRC_IMM:    fq_m1 = (imm16_d == 16'h8000) ? 32'h0 : simm_man_d;
			SRC_OPA_F:  fq_m1 = {opa_val[23:0], 8'h0};
			SRC_OPB_F:  fq_m1 = {opb_val[23:0], 8'h0};
			SRC_WB:     fq_m1 = w_b;
			SRC_A24:    fq_m1 = {{8{a_r[23]}}, a_r[23:0]};
			SRC_FS1:    fq_m1 = fs1_m_r;
			SRC_S1_24:  fq_m1 = {{8{w_s1v[23]}}, w_s1v[23:0]};
			SRC_P1:     fq_m1 = p1_m_r;
			SRC_OPB_I:  fq_m1 = opb_val;
			SRC_P1_24:  fq_m1 = {{8{p1_m_r[23]}}, p1_m_r[23:0]};
			SRC_OPA_24: fq_m1 = {{8{opa_val[23]}}, opa_val[23:0]};
			default: ;
		endcase
		case (fqe2_sel_d)
			EXP_FD:  fq_e2 = fd_e_r;
			EXP_FB:  fq_e2 = fb_e_r;
			EXP_FB5: fq_e2 = fb5_e_r;
			EXP_IMM: fq_e2 = (imm16_d == 16'h8000) ? 8'sh80 : simm_exp_d;
			EXP_OPA: fq_e2 = opa_val[31:24];
			EXP_OPB: fq_e2 = opb_val[31:24];
			EXP_P1:  fq_e2 = p1_e_r;
			EXP_P3:  fq_e2 = p3_e_r;
			default: ;
		endcase
		case (fqm2_sel_d)
			SRC_FD:     fq_m2 = fd_m_r;
			SRC_FB:     fq_m2 = fb_m_r;
			SRC_FB5:    fq_m2 = fb5_m_r;
			SRC_IMM:    fq_m2 = (imm16_d == 16'h8000) ? 32'h0 : simm_man_d;
			SRC_OPA_F:  fq_m2 = {opa_val[23:0], 8'h0};
			SRC_OPB_F:  fq_m2 = {opb_val[23:0], 8'h0};
			SRC_WB24:   fq_m2 = {{8{w_b[23]}}, w_b[23:0]};
			SRC_S2_24:  fq_m2 = {{8{w_s2v[23]}}, w_s2v[23:0]};
			SRC_P1:     fq_m2 = p1_m_r;
			SRC_P3:     fq_m2 = p3_m_r;
			SRC_P1_24:  fq_m2 = {{8{p1_m_r[23]}}, p1_m_r[23:0]};
			SRC_P3_24:  fq_m2 = {{8{p3_m_r[23]}}, p3_m_r[23:0]};
			SRC_OPB_24: fq_m2 = {{8{opb_val[23]}}, opb_val[23:0]};
			default: ;
		endcase
		case (fq2e1_sel_d)
			EXP_OPA: fq2_e1 = opa_val[31:24];
			EXP_OPB: fq2_e1 = opb_val[31:24];
			EXP_P1:  fq2_e1 = p1_e_r;
			EXP_P3:  fq2_e1 = p3_e_r;
			default: ;
		endcase
		case (fq2m1_sel_d)
			SRC_OPA_F: fq2_m1 = {opa_val[23:0], 8'h0};
			SRC_OPB_F: fq2_m1 = {opb_val[23:0], 8'h0};
			SRC_P1:    fq2_m1 = p1_m_r;
			SRC_P3:    fq2_m1 = p3_m_r;
			default: ;
		endcase
		case (fq2e2_sel_d)
			EXP_OPB: fq2_e2 = opb_val[31:24];
			EXP_P3:  fq2_e2 = p3_e_r;
			default: ;
		endcase
		case (fq2m2_sel_d)
			SRC_OPB_F: fq2_m2 = {opb_val[23:0], 8'h0};
			SRC_P3:    fq2_m2 = p3_m_r;
			default: ;
		endcase
	end

	// true for exactly the 6 shift-class forms; opcode_d fields only, no
	// register-file read, so it is safe to test in the same cycle as decode
	wire sh_class = (cls_d == CLS_TWO && (mbase_d == M_ASH || mbase_d == M_LSH)) ||
					(cls_d == CLS_THREE && (m3base_d == M3_ASH3 || m3base_d == M3_LSH3)) ||
					(cls_d == CLS_PST && (pgroup_d == 5'd9 || pgroup_d == 5'd14));

	// shift stage: operands latch out of S_EXEC into S_SH so the barrel shift
	// itself never sits between a register-file read and a writeback in one cycle
	logic [31:0] sh_val_r, sh_cnt_r, sh_o_val;
	logic        sh_arith_r, sh_o_c;
	logic [31:0] sh_res_o;
	logic        sh_c_o;
	logic signed [31:0] sh_n_o;
	always @* begin
		// the count is the low 7 bits, sign extended
		sh_n_o = $signed({{25{sh_cnt_r[6]}}, sh_cnt_r[6:0]});
		sh_res_o = 32'h0; sh_c_o = 1'b0;
		if (sh_n_o < 0) begin
			if (sh_n_o >= -31) sh_res_o = sh_arith_r ? asr32(sh_val_r, -sh_n_o) : (sh_val_r >> (-sh_n_o));
			else sh_res_o = sh_arith_r ? {32{sh_val_r[31]}} : 32'h0;
			// the carry is one bit of the source, not a second shift
			if (sh_n_o >= -32) sh_c_o = sh_val_r[5'(-sh_n_o - 1)];
			else if (sh_arith_r) sh_c_o = sh_val_r[31];
		end else begin
			sh_res_o = (sh_n_o <= 31) ? (sh_val_r << sh_n_o) : 32'h0;
			if (sh_n_o > 0 && sh_n_o <= 32) sh_c_o = sh_val_r[5'(32 - sh_n_o)];
		end
	end

	wire needs_opa = opa_is_mem;
	wire needs_opb = opb_is_mem;

	// ---------------------------------------------------------------
	// execute: produce writeback / store / branch results
	// ---------------------------------------------------------------
	always @* begin
		logic [31:0] a, b, res, dst;
		logic signed [7:0] fe1, fe2, feo;
		logic [31:0] fm1, fmo;
		logic vf, uff, cf;
		logic [4:0] cnd;
		logic [2:0] pdreg1, psreg1, psreg3;
		logic [31:0] p_sreg1_val, p_sreg3_val;
		logic signed [7:0] p_le; logic [31:0] p_lm;
		// parallel MPY+ALU forms: src1/src2 = registers op[21:19]/op[18:16], src3/src4 = mem
		logic signed [7:0] pe1, pe2, pe3, pe4;
		logic [31:0] pm1, pm2, pm3, pm4;
		logic [31:0] pi1, pi2, pi3, pi4;
		logic signed [7:0] mul_e1, mul_e2, add_e1, add_e2;
		logic [31:0] mul_m1, mul_m2, add_m1, add_m2;
		logic [31:0] mul_i1, mul_i2, add_i1, add_i2;
		logic signed [7:0] mul_feo, add_feo;
		logic [31:0] mul_fmo, add_fmo;
		logic mul_vf, mul_uff, add_vf, add_uff;
		logic [31:0] mul_ires, add_ires;
		logic [4:0] pa_dst1, pa_dst2;

		// defaults so nothing latches: every local this block can touch gets a value here
		a = 32'h0; b = 32'h0; res = 32'h0; dst = 32'h0;
		fe1 = 8'sh0; fe2 = 8'sh0; feo = 8'sh0;
		fm1 = 32'h0; fmo = 32'h0;
		vf = 1'b0; uff = 1'b0; cf = 1'b0;
		mpy_prod = 64'h0;
		db_arsel = 3'h0; db_newar = 32'h0; db_arval = 32'h0; db_taken = 1'b0;
		cnd = 5'h0;
		pdreg1 = 3'h0; psreg1 = 3'h0; psreg3 = 3'h0;
		pe1=8'sh0; pe2=8'sh0; pe3=8'sh0; pe4=8'sh0;
		pm1=32'h0; pm2=32'h0; pm3=32'h0; pm4=32'h0;
		pi1=32'h0; pi2=32'h0; pi3=32'h0; pi4=32'h0;
		mul_e1=8'sh0; mul_e2=8'sh0; add_e1=8'sh0; add_e2=8'sh0;
		mul_m1=32'h0; mul_m2=32'h0; add_m1=32'h0; add_m2=32'h0;
		mul_i1=32'h0; mul_i2=32'h0; add_i1=32'h0; add_i2=32'h0;
		mul_feo=8'sh0; add_feo=8'sh0; mul_fmo=32'h0; add_fmo=32'h0;
		mul_vf=1'b0; mul_uff=1'b0; add_vf=1'b0; add_uff=1'b0;
		mul_ires=32'h0; add_ires=32'h0; pa_dst1=5'h0; pa_dst2=5'h0;
		p_sreg1_val = 32'h0; p_sreg3_val = 32'h0; p_le = 8'sh0; p_lm = 32'h0;

		wr_en = 1'b0; wr_is_float = 1'b0; wr_idx = 5'h0; wr_val = 32'h0; wr_exp = 8'sh80;
		wr2_en = 1'b0; wr2_is_float = 1'b0; wr2_idx = 5'h0; wr2_val = 32'h0; wr2_exp = 8'sh80;
		is_store2 = 1'b0; store2_addr = 24'h0; store2_data = 32'h0;
		store2_ar_idx = 5'h0; store2_ar_new = 32'h0; store2_ar_wb = 1'b0;
		touches_flags = 1'b0; clears_c = 1'b0; touch_z = 1'b1;
		new_n = 1'b0; new_z = 1'b0; new_c = 1'b0; new_v = 1'b0; new_uf = 1'b0;
		lat_v = 1'b0; lat_uf = 1'b0;
		is_store = 1'b0; store_is_float = 1'b0; store_addr = 24'h0; store_data = 32'h0;
		store_ar_idx = 5'h0; store_ar_new = 32'h0; store_ar_wb = 1'b0;
		illegal_insn = 1'b0;
		next_pc = fetch_pc;
		branch_taken = 1'b0;
		is_brd = 1'b0; is_rptb = 1'b0; is_trap_now = 1'b0; is_reti = 1'b0;
		is_rpts = 1'b0; rpts_count = 32'h0; is_brcd = 1'b0; is_dbcd = 1'b0;
		is_special_wr = 1'b0; special_reg = 5'h0;

		a = a_r;
		b = w_b;

		case (cls_d)
		CLS_TWO: begin
			case (mbase_d)
				M_ABSF: begin
					fe1 = (g2_d==2'b00) ? fb_e_r :
						  (g2_d==2'b11) ? ((imm16_d==16'h8000) ? 8'sh80 : simm_exp_d) : opa_val[31:24];
					fm1 = (g2_d==2'b00) ? fb_m_r :
						  (g2_d==2'b11) ? ((imm16_d==16'h8000) ? 32'h0 : simm_man_d) : {opa_val[23:0], 8'h0};
					// not fneg: ABSF is its own macro, with a 7F:80000000 overflow case
					if (fe1 == 8'sh80) begin feo = fe1; fmo = 32'h0; end
					else if (!fm1[31]) begin feo = fe1; fmo = fm1; end
					else if (fm1 != 32'h80000000) begin feo = fe1; fmo = 32'h0 - fm1; end
					else if (fe1 == 8'sd127) begin feo = fe1; fmo = 32'h7fffffff; vf = 1'b1; end
					else begin feo = fe1 + 8'sd1; fmo = 32'h0; end
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf;
				end
				M_ABSI: begin
					res = $signed(x_b) < 0 ? (32'h0 - x_b) : x_b;
					if (st[7] && res == 32'h80000000) begin res = 32'h7fffffff; new_v=1'b1; new_uf=1'b0; end
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
					if (res == 32'h80000000) new_v = 1'b1;
				end
				M_ADDF: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M_ADDI: begin
					res = a + x_b;
					if (st[7] && (((a^res) & (x_b^res)) >> 31)) res = a[31] ? 32'h80000000 : 32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c = (a > res); // unsigned a > res => carry out
					new_v = ((a^res) & (x_b^res)) >> 31;
				end
				M_AND: begin res=a&x_b; wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M_ANDN: begin res=a&(~x_b); wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M_ASH: begin
					res = sh_o_val;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; clears_c=1'b1;
					new_c=sh_o_c; new_n=res[31]; new_z=(res==0);
				end
				M_CMPI: begin
					res = a - x_b;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c = (x_b > a); new_v = ((a^x_b) & (a^res)) >> 31;
				end
				M_CMPF: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M_FIX: begin
					res=f1_m; vf=f1_v;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					if (dreg_d < 8) begin touches_flags=1'b1; new_n=res[31]; new_z=(res==0); new_v=vf; end
				end
				M_FLOAT: begin
					feo=f1_e; fmo=f1_m;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80);
				end
				M_IDLE: ; // handled specially in WRITEBACK (idle-until-interrupt)
				M_LDF: begin
					if (g2_d==2'b00) begin feo=fb_e_r; fmo=fb_m_r; end
					else if (g2_d==2'b11) begin
						if (imm16_d==16'h8000) begin feo=8'sh80; fmo=32'h0; end
						else begin fmo=simm_man_d; feo=simm_exp_d; end
					end else begin feo=opa_val[31:24]; fmo={opa_val[23:0], 8'h0}; end
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80);
				end
				M_LDI: begin
					wr_en=1'b1; wr_idx=dreg_d; wr_val=x_b;
					if (dreg_d<8) begin touches_flags=1'b1; new_n=x_b[31]; new_z=(x_b==0); end
					else if (dreg_d>=BK) begin is_special_wr=1'b1; special_reg=dreg_d; end
				end
				M_LSH: begin
					res = sh_o_val;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; clears_c=1'b1;
					new_c=sh_o_c; new_n=res[31]; new_z=(res==0);
				end
				M_MPYF: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M_MPYI: begin
					mpy_prod=f1_p;
					res = mpy_prod[31:0];
					if (st[7] && (mpy_prod < -64'sd2147483648 || mpy_prod > 64'sd2147483647))
						res = (mpy_prod < 0) ? 32'h80000000 : 32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
					if (mpy_prod < -64'sd2147483648 || mpy_prod > 64'sd2147483647) new_v=1'b1;
				end
				M_NEGF: begin
					fe1 = (g2_d==2'b00) ? fb_e_r :
						  (g2_d==2'b11) ? ((imm16_d==16'h8000) ? 8'sh80 : simm_exp_d) : opa_val[31:24];
					fm1 = (g2_d==2'b00) ? fb_m_r :
						  (g2_d==2'b11) ? ((imm16_d==16'h8000) ? 32'h0 : simm_man_d) : {opa_val[23:0], 8'h0};
					fneg(fe1, fm1, feo, fmo);
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80);
				end
				M_NEGI: begin
					res = 32'h0 - x_b;
					if (st[7] && (((32'h0^x_b)&(32'h0^res))>>31)) res = x_b[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(x_b>32'h0); new_v=((32'h0^x_b)&(32'h0^res))>>31;
				end
				M_NOP: ; // no register effect
				M_NOT: begin res=~x_b; wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M_POP: begin end // handled via memory read path in WRITEBACK
				M_POPF: begin end
				M_PUSH: begin
					is_store=1'b1; store_data=a_r; store_addr=rmant[SP]+1;
					store_ar_idx=SP; store_ar_new=rmant[SP]+1; store_ar_wb=1'b1;
				end
				M_PUSHF: begin
					is_store=1'b1; store_is_float=1'b1;
					store_data = {fd3_e_r, fd3_m_r[31:8]};
					store_addr=rmant[SP]+1;
					store_ar_idx=SP; store_ar_new=rmant[SP]+1; store_ar_wb=1'b1;
				end
				M_OR: begin res=a|x_b; wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M_STF: begin
					is_store=1'b1; store_is_float=1'b1;
					store_data = {fd3_e_r, fd3_m_r[31:8]};
					store_addr = (g2_d==2'b01) ? {rmant[DP][7:0], imm16_d} : dc_opa_addr;
					store_ar_idx = dc_opa_ar_idx; store_ar_new = dc_opa_ar_new; store_ar_wb = dc_opa_ar_wb && g2_d == 2'b10;
				end
				M_STI: begin
					is_store=1'b1;
					// the reference stores the register after its own AR modify
					store_data = (g2_d == 2'b10 && dc_opa_ar_wb && dc_opa_ar_idx == dreg_d) ? dc_opa_ar_new : a_r;
					store_addr = (g2_d==2'b01) ? {rmant[DP][7:0], imm16_d} : dc_opa_addr;
					store_ar_idx = dc_opa_ar_idx; store_ar_new = dc_opa_ar_new; store_ar_wb = dc_opa_ar_wb && g2_d == 2'b10;
				end
				M_SUBF: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M_SUBI: begin
					res = a - x_b;
					if (st[7] && (((a^x_b)&(a^res))>>31)) res = a[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(x_b>a); new_v=((a^x_b)&(a^res))>>31;
				end
				M_XOR: begin res=a^x_b; wr_en=1'b1; wr_idx=dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M_ADDC: begin
					{cf,res} = {1'b0,a} + {1'b0,x_b} + {32'b0,st[0]};
					if (st[7] && (((a^res)&(x_b^res))>>31)) res = a[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=cf; new_v=((a^res)&(x_b^res))>>31;
					new_n=res[31]; new_z=(res==0);
				end
				M_SUBB: begin
					res = a - x_b - {31'b0,st[0]};
					if (st[7] && (((a^x_b)&(a^res))>>31)) res = a[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=(x_b>a) | (st[0] && x_b==a);
					new_v=((a^x_b)&(a^res))>>31; new_n=res[31]; new_z=(res==0);
				end
				M_SUBC: begin
					res = (a >= x_b) ? (((a-x_b)<<1)|32'h1) : (a<<1);
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
				end
				M_SUBRB: begin
					res = x_b - a - {31'b0,st[0]};
					if (st[7] && (((x_b^a)&(x_b^res))>>31)) res = x_b[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=(a>x_b) | (st[0] && a==x_b);
					new_v=((x_b^a)&(x_b^res))>>31; new_n=res[31]; new_z=(res==0);
				end
				M_SUBRF: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u; // reversed: src - dst
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M_SUBRI: begin
					res = x_b - a;
					if (st[7] && (((x_b^a)&(x_b^res))>>31)) res = x_b[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(a>x_b); new_v=((x_b^a)&(x_b^res))>>31;
				end
				M_TSTB: begin
					res = a & x_b;
					touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
				end
				M_NEGB: begin
					res = 32'h0 - x_b - {31'b0,st[0]};
					if (st[7] && (((32'h0^x_b)&(32'h0^res))>>31)) res = x_b[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=(x_b!=32'h0) | st[0];
					new_v=((32'h0^x_b)&(32'h0^res))>>31; new_n=res[31]; new_z=(res==0);
				end
				M_ROL: begin
					res = {a[30:0], a[31]};
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=a[31]; new_n=res[31]; new_z=(res==0);
				end
				M_ROLC: begin
					res = {a[30:0], st[0]};
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=a[31]; new_n=res[31]; new_z=(res==0);
				end
				M_ROR: begin
					res = {a[0], a[31:1]};
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=a[0]; new_n=res[31]; new_z=(res==0);
				end
				M_RORC: begin
					res = {st[0], a[31:1]};
					wr_en=1'b1; wr_idx=dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=a[0]; new_n=res[31]; new_z=(res==0);
				end
				M_RPTS: begin
					is_rpts = 1'b1; rpts_count = x_b;
				end
				M_LDE: begin
					fe2 = (g2_d==2'b00) ? fb_e_r :
						  (g2_d==2'b11) ? ((imm16_d==16'h8000) ? 8'sh80 : (simm_exp_d)) : opa_val[31:24];
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=fe2;
					wr_val = (fe2 == 8'sh80) ? 32'h0 : fd3_m_r;
				end
				M_LDM: begin
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=fd3_e_r;
					wr_val=(g2_d==2'b11) ? ((imm16_d==16'h8000) ? 32'h0 : simm_man_d) : b;
				end
				M_NORM: begin
					feo=f1_e; fmo=f1_m; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_uf=uff;
				end
				M_RND: begin
					if (g2_d==2'b00) begin feo=fb_e_r; fmo=fb_m_r; end
					else if (g2_d==2'b11) begin
						if (imm16_d==16'h8000) begin feo=8'sh80; fmo=32'h0; end
						else begin fmo=simm_man_d; feo=simm_exp_d; end
					end else begin feo=opa_val[31:24]; fmo={opa_val[23:0], 8'h0}; end
					if ($signed(fmo) < 32'sh7fffff80) begin
						fmo = (fmo + 32'h80) & 32'hffffff00;
					end else if ($signed(feo) < 127) begin
						fmo = (fmo + 32'h80) & 32'h7fffff00; feo = feo + 8'sd1;
					end else begin
						fmo = 32'h7fffff00; vf = 1'b1;
					end
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; touch_z=1'b0; new_n=fmo[31]; new_v=vf;
					new_uf = !vf && (feo == 8'sh80);
				end
				M_IACK: ; // bus ack cycle, no architectural state change
				default: illegal_insn = 1'b1;
			endcase
		end
		CLS_THREE: begin
			// three-operand forms
			case (m3base_d)
				M3_ADDF3: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=m3dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M3_ADDI3: begin
					res = x_s1 + x_s2;
					if (st[7] && (((x_s1^res)&(x_s2^res))>>31)) res = x_s1[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(x_s1>res); new_v=((x_s1^res)&(x_s2^res))>>31;
				end
				M3_AND3: begin res=x_s1&x_s2; wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M3_ANDN3: begin res=x_s1&(~x_s2); wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M3_ASH3: begin
					res = sh_o_val;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; clears_c=1'b1;
					new_c=sh_o_c; new_n=res[31]; new_z=(res==0);
				end
				M3_CMPI3: begin
					res = x_s1 - x_s2;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(x_s2>x_s1); new_v=((x_s1^x_s2)&(x_s1^res))>>31;
				end
				M3_LSH3: begin
					res = sh_o_val;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; clears_c=1'b1;
					new_c=sh_o_c; new_n=res[31]; new_z=(res==0);
				end
				M3_MPYF3: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=m3dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M3_MPYI3: begin
					mpy_prod=f1_p;
					res = mpy_prod[31:0];
					if (st[7] && (mpy_prod < -64'sd2147483648 || mpy_prod > 64'sd2147483647))
						res = (mpy_prod<0) ? 32'h80000000 : 32'h7fffffff;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res;
					touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
					if (mpy_prod < -64'sd2147483648 || mpy_prod > 64'sd2147483647) new_v=1'b1;
				end
				M3_OR3: begin res=x_s1|x_s2; wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M3_SUBF3: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					wr_en=1'b1; wr_is_float=1'b1; wr_idx=m3dreg_d; wr_exp=feo; wr_val=fmo;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				M3_SUBI3: begin
					res = x_s1 - x_s2;
					if (st[7] && (((x_s1^x_s2)&(x_s1^res))>>31)) res = x_s1[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
					new_c=(x_s2>x_s1); new_v=((x_s1^x_s2)&(x_s1^res))>>31;
				end
				M3_XOR3: begin res=x_s1^x_s2; wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res; touches_flags=1'b1; new_n=res[31]; new_z=(res==0); end
				M3_ADDC3: begin
					{cf,res} = {1'b0,x_s1} + {1'b0,x_s2} + {32'b0,st[0]};
					if (st[7] && (((x_s1^res)&(x_s2^res))>>31)) res = x_s1[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=cf; new_v=((x_s1^res)&(x_s2^res))>>31;
					new_n=res[31]; new_z=(res==0);
				end
				M3_SUBB3: begin
					res = x_s1 - x_s2 - {31'b0,st[0]};
					if (st[7] && (((x_s1^x_s2)&(x_s1^res))>>31)) res = x_s1[31]?32'h80000000:32'h7fffffff;
					wr_en=1'b1; wr_idx=m3dreg_d; wr_val=res;
					touches_flags=1'b1; clears_c=1'b1; new_c=(x_s2>x_s1) | (st[0] && x_s2==x_s1);
					new_v=((x_s1^x_s2)&(x_s1^res))>>31; new_n=res[31]; new_z=(res==0);
				end
				M3_TSTB3: begin
					res = x_s1 & x_s2;
					touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
				end
				M3_CMPF3: begin
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
					touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
				end
				default: illegal_insn = 1'b1;
			endcase
		end
		CLS_LDFC: begin
			// LDFcond: cond = (idx11_d-0x200)>>2
			cnd = (idx11_d - 11'h200) >> 2;
			if (cond_true(cnd, st[13:0])) begin
				if (g2_d==2'b00) begin feo=fb_e_r; fmo=fb_m_r; end
				else if (g2_d==2'b11) begin
					if (imm16_d==16'h8000) begin feo=8'sh80; fmo=32'h0; end
					else begin fmo=simm_man_d; feo=simm_exp_d; end
				end else begin feo=opa_val[31:24]; fmo={opa_val[23:0], 8'h0}; end
				wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,dreg_d[2:0]}; wr_exp=feo; wr_val=fmo;
			end
		end
		CLS_LDIC: begin
			// LDIcond
			cnd = (idx11_d - 11'h280) >> 2;
			if (cond_true(cnd, st[13:0])) begin
				b = (g2_d==2'b00) ? b_r :
					(g2_d==2'b11) ? {{16{imm16_d[15]}}, imm16_d} : opa_val;
				wr_en=1'b1; wr_idx=dreg_d; wr_val=b;
				if (dreg_d>=BK) begin is_special_wr=1'b1; special_reg=dreg_d; end
			end
		end
		CLS_BR: begin
			next_pc = opcode_d[23:0]; branch_taken = 1'b1; // BR
		end
		CLS_BRD: begin
			is_brd = 1'b1; next_pc = opcode_d[23:0]; // BRD
		end
		CLS_CALL: begin
			is_store = 1'b1; store_data = fetch_pc; store_addr = rmant[SP] + 1;
			store_ar_idx = SP; store_ar_new = rmant[SP] + 1; store_ar_wb = 1'b1;
			next_pc = opcode_d[23:0]; branch_taken = 1'b1; // CALL
		end
		CLS_RPTB: begin
			is_rptb = 1'b1; // RPTB: RE = imm, RS = pc after fetch
		end
		CLS_BCR: begin
			// Bcond reg: 0x340 plain, 0x341 delayed (runs 3 delay slots either way)
			is_brcd = idx11_d[0];
			if (cond_true(dreg_d, st[13:0])) begin next_pc = b_r; branch_taken = 1'b1; end
		end
		CLS_BCI: begin
			// Bcond imm: delayed target is m_pc+2+disp (m_pc = fetch_pc here), plain is m_pc+disp
			is_brcd = idx11_d[0];
			if (cond_true(dreg_d, st[13:0])) begin
				next_pc = is_brcd ? (fetch_pc + 24'd2 + simm16_d) : (fetch_pc + simm16_d);
				branch_taken = 1'b1;
			end
		end
		CLS_DBR: begin
			// DBcond reg: AR select in op[24:22] = idx11_d[3:1], delayed = idx11_d[0]
			is_dbcd = idx11_d[0];
			db_arsel = idx11_d[3:1];
			db_arval = db_ar_val_r;
			db_newar = (db_arval - 1) & 24'hffffff;
			store_ar_idx = AR0+db_arsel; store_ar_new = {db_arval[31:24], db_newar[23:0]}; store_ar_wb = 1'b1;
			db_taken = cond_true(dreg_d, st[13:0]) && !db_newar[23];
			if (db_taken) begin next_pc = b_r; branch_taken = 1'b1; end
		end
		CLS_DBI: begin
			// DBcond imm
			is_dbcd = idx11_d[0];
			db_arsel = idx11_d[3:1];
			db_arval = db_ar_val_r;
			db_newar = (db_arval - 1) & 24'hffffff;
			store_ar_idx = AR0+db_arsel; store_ar_new = {db_arval[31:24], db_newar[23:0]}; store_ar_wb = 1'b1;
			db_taken = cond_true(dreg_d, st[13:0]) && !db_newar[23];
			if (db_taken) begin
				next_pc = is_dbcd ? (fetch_pc + 24'd2 + simm16_d) : (fetch_pc + simm16_d);
				branch_taken = 1'b1;
			end
		end
		CLS_CALLCR: begin
			// CALLcond reg
			if (cond_true(dreg_d, st[13:0])) begin
				is_store=1'b1; store_data=fetch_pc; store_addr=rmant[SP]+1;
				store_ar_idx=SP; store_ar_new=rmant[SP]+1; store_ar_wb=1'b1;
				next_pc = b_r; branch_taken = 1'b1;
			end
		end
		CLS_CALLCI: begin
			// CALLcond imm
			if (cond_true(dreg_d, st[13:0])) begin
				is_store=1'b1; store_data=fetch_pc; store_addr=rmant[SP]+1;
				store_ar_idx=SP; store_ar_new=rmant[SP]+1; store_ar_wb=1'b1;
				next_pc = fetch_pc + simm16_d; branch_taken = 1'b1;
			end
		end
		CLS_TRAPC: begin
			// TRAPcond
			if (cond_true(dreg_d, st[13:0])) is_trap_now = 1'b1;
		end
		CLS_RETIC: begin
			// RETIcond
			if (cond_true(dreg_d, st[13:0])) is_reti = 1'b1;
		end
		CLS_RETSC: begin
			// RETScond
			if (cond_true(dreg_d, st[13:0])) begin
				next_pc = 24'hffffff; // placeholder, real value comes from RMEM(SP) in WRITEBACK via opa
				branch_taken = 1'b1;
				// opa_is_mem stays 0 here (RETScond is outside the decode block's ranges);
				// the pop itself is driven explicitly from S_EXEC, not through needs_opa
			end
		end
		CLS_PMPY: begin
			// parallel MPY+ALU forms: family = idx11_d[6:5] (0 mpyf+addf, 1 mpyf+subf,
			// 2 mpyi+addi, 3 mpyi+subi), pattern = idx11_d[4:3], dst1 = idx11_d[2] (R0/R1),
			// dst2 = idx11_d[1] (R2/R3). src1/src2 are registers op[21:19]/op[18:16],
			// src3/src4 are the two memory operands (opa_val/opb_val), always pre-instruction.
			pe1 = p1_e_r; pm1 = p1_m_r; pi1 = p1_m_r;
			pe2 = p3_e_r; pm2 = p3_m_r; pi2 = p3_m_r;
			pe3 = opa_val[31:24]; pm3 = {opa_val[23:0], 8'h0}; pi3 = opa_val;
			pe4 = opb_val[31:24]; pm4 = {opb_val[23:0], 8'h0}; pi4 = opb_val;
			pa_dst1 = {4'b0, idx11_d[2]};
			pa_dst2 = {3'b0, 1'b1, idx11_d[1]};
			case (idx11_d[4:3])
				2'd0: begin
					mul_e1=pe3; mul_m1=pm3; mul_e2=pe4; mul_m2=pm4; mul_i1=pi3; mul_i2=pi4;
					add_e1=pe1; add_m1=pm1; add_e2=pe2; add_m2=pm2; add_i1=pi1; add_i2=pi2;
				end
				2'd1: begin
					mul_e1=pe3; mul_m1=pm3; mul_e2=pe1; mul_m2=pm1; mul_i1=pi3; mul_i2=pi1;
					add_e1=pe4; add_m1=pm4; add_e2=pe2; add_m2=pm2; add_i1=pi4; add_i2=pi2;
				end
				2'd2: begin
					mul_e1=pe1; mul_m1=pm1; mul_e2=pe2; mul_m2=pm2; mul_i1=pi1; mul_i2=pi2;
					add_e1=pe3; add_m1=pm3; add_e2=pe4; add_m2=pm4; add_i1=pi3; add_i2=pi4;
				end
				2'd3: begin
					mul_e1=pe3; mul_m1=pm3; mul_e2=pe1; mul_m2=pm1; mul_i1=pi3; mul_i2=pi1;
					add_e1=pe2; add_m1=pm2; add_e2=pe4; add_m2=pm4; add_i1=pi2; add_i2=pi4;
				end
			endcase
			if (idx11_d[6]) begin
				// mpyi + addi/subi: 24x24 signed multiply, plain 32-bit add/sub; flags are
				// only ever cleared (CLR_NZVUF, no OR_NZ/OR_V call in mpyaddi/mpysubi)
				mpy_prod=f1_p;
				mul_ires = mpy_prod[31:0];
				add_ires = idx11_d[5] ? (add_i1 - add_i2) : (add_i1 + add_i2);
				if (st[7]) begin
					if (mpy_prod < -64'sd2147483648) mul_ires = 32'h80000000;
					else if (mpy_prod > 64'sd2147483647) mul_ires = 32'h7fffffff;
					if (idx11_d[5]) begin
						if (((add_i1^add_i2) & (add_i1^add_ires)) >> 31)
							add_ires = add_i1[31] ? 32'h80000000 : 32'h7fffffff;
					end else begin
						if (((add_i1^add_ires) & (add_i2^add_ires)) >> 31)
							add_ires = add_i1[31] ? 32'h80000000 : 32'h7fffffff;
					end
				end
				wr_en=1'b1; wr_idx=pa_dst2; wr_val=add_ires;
				wr2_en=1'b1; wr2_idx=pa_dst1; wr2_val=mul_ires;
				touches_flags=1'b1; new_n=1'b0; new_z=1'b0; new_v=1'b0; new_uf=1'b0;
			end else begin
				// mpyf then addf/subf, each with its own CLR_NZVUF/OR_NZF, so the
				// architectural flags are the ones the second operation leaves behind;
				// LV/LUF are never cleared, so they also keep the MPYF V/UF
				mul_feo=f1_e; mul_fmo=f1_m; mul_vf=f1_v; mul_uff=f1_u;
				add_feo=f2_e; add_fmo=f2_m; add_vf=f2_v; add_uff=f2_u;
				wr_en=1'b1; wr_is_float=1'b1; wr_idx=pa_dst2; wr_exp=add_feo; wr_val=add_fmo;
				wr2_en=1'b1; wr2_is_float=1'b1; wr2_idx=pa_dst1; wr2_exp=mul_feo; wr2_val=mul_fmo;
				touches_flags=1'b1; new_n=add_fmo[31]; new_z=(add_feo==8'sh80);
				new_v=add_vf; new_uf=add_uff;
				lat_v=mul_vf; lat_uf=mul_uff;
			end
		end
		CLS_PST: begin
			// parallel store forms: dreg1=op[24:22], sreg1=op[21:19], sreg3=op[18:16]
			pdreg1 = opcode_d[24:22]; psreg1 = opcode_d[21:19]; psreg3 = opcode_d[18:16];
			p_sreg1_val = p1_m_r;
			p_sreg3_val = p3_m_r;
			p_le = opb_val[31:24]; p_lm = {opb_val[23:0], 8'h0};
			case (pgroup_d)
				5'd0: begin // STF||STF
					is_store=1'b1; store_data={p3_e_r, p3_m_r[31:8]}; store_addr=dc_opa_addr;
					store_ar_idx=dc_opa_ar_idx; store_ar_new=dc_opa_ar_new; store_ar_wb=dc_opa_ar_wb;
					is_store2=1'b1; store2_data={ppd1_e_r, ppd1_m_r[31:8]}; store2_addr=dc_opb_addr;
					store2_ar_idx=dc_opb_ar_idx; store2_ar_new=dc_opb_ar_new; store2_ar_wb=dc_opb_ar_wb;
				end
				5'd1: begin // STI||STI
					is_store=1'b1; store_data=p_sreg3_val; store_addr=dc_opa_addr;
					store_ar_idx=dc_opa_ar_idx; store_ar_new=dc_opa_ar_new; store_ar_wb=dc_opa_ar_wb;
					is_store2=1'b1; store2_data=ppd1_m_r; store2_addr=dc_opb_addr;
					store2_ar_idx=dc_opb_ar_idx; store2_ar_new=dc_opb_ar_new; store2_ar_wb=dc_opb_ar_wb;
				end
				5'd2: begin // LDF||LDF
					wr2_en=1'b1; wr2_is_float=1'b1; wr2_idx={2'b0,psreg1}; wr2_exp=opa_val[31:24]; wr2_val={opa_val[23:0],8'h0};
					wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=opb_val[31:24]; wr_val={opb_val[23:0],8'h0};
				end
				5'd3: begin // LDI||LDI
					wr2_en=1'b1; wr2_idx={2'b0,psreg1}; wr2_val=opa_val;
					wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=opb_val;
				end
				default: begin
					// the 20 ALU||STx forms: store psreg3 at dc_opa_addr, apply an op
					// to the loaded dc_opb_addr value and write pdreg1
					is_store=1'b1; store_addr=dc_opa_addr;
					store_ar_idx=dc_opa_ar_idx; store_ar_new=dc_opa_ar_new; store_ar_wb=dc_opa_ar_wb;
					case (pgroup_d)
						5'd4: begin // ABSF||STF
							store_data={p3_e_r, p3_m_r[31:8]};
							if (p_le == 8'sh80) begin feo=p_le; fmo=32'h0; end
							else if (!p_lm[31]) begin feo=p_le; fmo=p_lm; end
							else if (p_lm != 32'h80000000) begin feo=p_le; fmo=32'h0-p_lm; end
							else if (p_le == 8'sd127) begin feo=p_le; fmo=32'h7fffffff; vf=1'b1; end
							else begin feo=p_le+8'sd1; fmo=32'h0; end
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf;
						end
						5'd5: begin // ABSI||STI
							store_data=p_sreg3_val;
							res = $signed(opb_val)<0 ? (32'h0-opb_val) : opb_val;
							if (st[7] && res==32'h80000000) res=32'h7fffffff;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
							if (res==32'h80000000) new_v=1'b1;
						end
						5'd6: begin // ADDF3||STF
							store_data={p3_e_r, p3_m_r[31:8]};
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
						end
						5'd7: begin // ADDI3||STI
							store_data=p_sreg3_val;
							res = p_sreg1_val + opb_val;
							if (st[7] && (((p_sreg1_val^res)&(opb_val^res))>>31)) res=p_sreg1_val[31]?32'h80000000:32'h7fffffff;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
							new_c=(p_sreg1_val>res); new_v=((p_sreg1_val^res)&(opb_val^res))>>31;
						end
						5'd8: begin // AND3||STI
							store_data=p_sreg3_val; res=p_sreg1_val & opb_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
						end
						5'd9: begin // ASH3||STI
							store_data=p_sreg3_val;
							res = sh_o_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; clears_c=1'b1; new_c=sh_o_c;
							new_n=res[31]; new_z=(res==0);
						end
						5'd10: begin // FIX||STI
							store_data=p_sreg3_val;
							res=f1_m; vf=f1_v;
							// this form converts in place in the destination, so the
							// loaded word's exponent stays behind next to the integer
							wr_en=1'b1; wr_is_float=1'b1; wr_exp=p_le;
							wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0); new_v=vf;
						end
						5'd11: begin // FLOAT||STF
							store_data={p3_e_r, p3_m_r[31:8]};
					feo=f1_e; fmo=f1_m;
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80);
						end
						5'd12: begin // LDF||STF
							store_data={p3_e_r, p3_m_r[31:8]};
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=p_le; wr_val=p_lm;
						end
						5'd13: begin // LDI||STI
							store_data=p_sreg3_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=opb_val;
						end
						5'd14: begin // LSH3||STI
							store_data=p_sreg3_val;
							res = sh_o_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; clears_c=1'b1; new_c=sh_o_c;
							new_n=res[31]; new_z=(res==0);
						end
						5'd15: begin // MPYF3||STF
							store_data={p3_e_r, p3_m_r[31:8]};
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
						end
						5'd16: begin // MPYI3||STI
							store_data=p_sreg3_val;
							mpy_prod=f1_p;
							res = mpy_prod[31:0];
							if (st[7] && (mpy_prod<-64'sd2147483648 || mpy_prod>64'sd2147483647)) res=(mpy_prod<0)?32'h80000000:32'h7fffffff;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
							if (mpy_prod<-64'sd2147483648 || mpy_prod>64'sd2147483647) new_v=1'b1;
						end
						5'd17: begin // NEGF||STF
							store_data={p3_e_r, p3_m_r[31:8]};
							fneg(p_le, p_lm, feo, fmo);
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80);
						end
						5'd18: begin // NEGI||STI
							store_data=p_sreg3_val;
							res = 32'h0 - opb_val;
							if (st[7] && (((32'h0^opb_val)&(32'h0^res))>>31)) res=opb_val[31]?32'h80000000:32'h7fffffff;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
							new_c=(opb_val>32'h0); new_v=((32'h0^opb_val)&(32'h0^res))>>31;
						end
						5'd19: begin // NOT||STI
							store_data=p_sreg3_val; res=~opb_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
						end
						5'd20: begin // OR3||STI
							store_data=p_sreg3_val; res=p_sreg1_val | opb_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
						end
						5'd21: begin // SUBF3||STF: loaded - sreg1
							store_data={p3_e_r, p3_m_r[31:8]};
					feo=f1_e; fmo=f1_m; vf=f1_v; uff=f1_u;
							wr_en=1'b1; wr_is_float=1'b1; wr_idx={2'b0,pdreg1}; wr_exp=feo; wr_val=fmo;
							touches_flags=1'b1; new_n=fmo[31]; new_z=(feo==8'sh80); new_v=vf; new_uf=uff;
						end
						5'd22: begin // SUBI3||STI: loaded - sreg1
							store_data=p_sreg3_val;
							res = opb_val - p_sreg1_val;
							if (st[7] && (((opb_val^p_sreg1_val)&(opb_val^res))>>31)) res=opb_val[31]?32'h80000000:32'h7fffffff;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; clears_c=1'b1; new_n=res[31]; new_z=(res==0);
							new_c=(p_sreg1_val>opb_val); new_v=((opb_val^p_sreg1_val)&(opb_val^res))>>31;
						end
						5'd23: begin // XOR3||STI
							store_data=p_sreg3_val; res=p_sreg1_val ^ opb_val;
							wr_en=1'b1; wr_idx={2'b0,pdreg1}; wr_val=res;
							touches_flags=1'b1; new_n=res[31]; new_z=(res==0);
						end
						default: illegal_insn = 1'b1;
					endcase
				end
			endcase
		end
		default: illegal_insn = 1'b1;
		endcase

		// integer register-writing ALU ops only touch flags when the dest is R0-R7
		// (matches every ADDI/SUBI/AND/... macro's "if (dreg_d < 8) {flags} else if
		// (dreg_d >= TMR_BK) update_special" split); ops with no register write of
		// their own (CMPI, TSTB, CMPF3, ...) are unaffected since wr_en stays 0.
		if (wr_en && !wr_is_float && wr_idx >= 5'd8) begin
			touches_flags = 1'b0;
			if (wr_idx >= 5'd19) begin is_special_wr = 1'b1; special_reg = wr_idx; end
		end
	end

	// RETScond needs a memory read (pop): route it through the opa fetch path
	wire is_retscond = (cls_d == CLS_RETSC);
	wire retscond_taken = is_retscond && cond_true(dreg_d, st[13:0]);

	// ---------------------------------------------------------------
	// interrupt levels -> IF bits (level, OR'd continuously like INT0-3)
	// ---------------------------------------------------------------
	// bit layout matches tms320c3x.h: 0-3 INT0-3, 4 XINT0, 5 RINT0, 8 TINT0, 9 TINT1, 10 DINT0
	wire [11:0] irq_levels = {1'b0, dint, tint1, tint0, 1'b0, 1'b0, rint, xint, ~int_n};
	// matches c31_set_irq: IF is latched the moment a line asserts (software must be
	// able to read it even with GIE/IE off), not just used combinationally for
	// trap arbitration. The post-trap re-arm for bits 0-3 already covers the
	// level-held-after-clear case; this catches the initial assert edge for all 12.
	logic [11:0] irq_levels_prev;
	wire  [11:0] irq_rising = irq_levels & ~irq_levels_prev;
	// c31_set_irq (any line change) and the post-trap step of check_irqs both OR the
	// held INT0-3 levels back into IF, so a bit software cleared returns only then
	wire        irq_event = |irq_rising || |(irq_levels_prev[3:0] & ~irq_levels[3:0]);
	wire [11:0] irq_set = irq_event ? (irq_rising | {8'h0, irq_levels[3:0]}) : 12'h0;

	// the unconditional "rmant[IFR] <= rmant[IFR] | irq_set" below and an
	// S_DECODE register latch are the same nonblocking edge: both read the
	// pre-edge rmant[IFR], so a latch of IFR taken the same cycle an IRQ rises
	// would otherwise miss the bit the live S_EXEC read used to already see.
	function automatic logic [31:0] rread_fwd_irq(input logic [4:0] idx);
		rread_fwd_irq = rread(idx);
		if (idx == IFR[4:0] && irq_event) rread_fwd_irq[11:0] = rread_fwd_irq[11:0] | irq_set;
	endfunction

	function automatic logic [31:0] rmant_fwd_irq(input logic [4:0] idx);
		rmant_fwd_irq = (idx < 28) ? rmant[idx] : 32'h0;
		if (idx == IFR[4:0] && irq_event) rmant_fwd_irq[11:0] = rmant_fwd_irq[11:0] | irq_set;
	endfunction

	// ---------------------------------------------------------------
	// sequencer
	// ---------------------------------------------------------------
	// set only when S_COMMIT issued the fetch early (S_IRQCHECK skipped for this
	// instruction), so S_FETCH_WAIT/S_DECODE know they still owe the irq_ready check
	// that instruction would otherwise have had at its own S_IRQCHECK
	logic fetch_irq_recheck;
	// prefetch of pc issued on entry to S_WRITEBACK; pf_want 0 means dropped,
	// the capture below still retires its ack
	logic pf_valid, pf_busy, pf_want;
	logic [31:0] pf_opcode;
	logic [23:0] pf_addr;
	logic [4:0] whichtrap;
	logic [11:0] validints;
	logic [23:0] trap_pc_save;
	logic [4:0]  trap_num; // whichtrap latched at dispatch: IFR's bit is cleared
						   // immediately in S_IRQCHECK, so whichtrap (combinational
						   // from IFR) is no longer valid by the time the vector is fetched

	always @* begin
		// level-sensitive: a live line contributes even before it is latched into IF
		// IF is latched by irq_set (below) for software visibility, but that
		// register write lags the edge by one clock; c31_set_irq's IF write is
		// atomic with the reference's very next check_irqs, so trap readiness
		// also ORs in irq_set itself (not the raw held level) to be ready the
		// same cycle a line changes; a held level software cleared stays out of
		// IF until the next irq_event or interrupt trap, as in the reference
		validints = (rmant[IFR][11:0] | irq_set) & rmant[IE][11:0];
		whichtrap = 5'h0;
		if (validints[0]) whichtrap = 5'd1;
		else if (validints[1]) whichtrap = 5'd2;
		else if (validints[2]) whichtrap = 5'd3;
		else if (validints[3]) whichtrap = 5'd4;
		else if (validints[4]) whichtrap = 5'd5;
		else if (validints[5]) whichtrap = 5'd6;
		else if (validints[6]) whichtrap = 5'd7;
		else if (validints[7]) whichtrap = 5'd8;
		else if (validints[8]) whichtrap = 5'd9;
		else if (validints[9]) whichtrap = 5'd10;
		else if (validints[10]) whichtrap = 5'd11;
		else if (validints[11]) whichtrap = 5'd12;
	end

	wire irq_ready = (validints != 12'h0) && rmant[ST][13] && !delay_active;

	// commit-time early fetch: if this instruction does not redirect pc, isn't a
	// repeat-block boundary, and no irq is ready, pc already holds the correct
	// next-fetch address (set back in S_DECODE), so the fetch can be issued
	// straight from S_COMMIT and S_IRQCHECK/S_FETCH skipped entirely.
	wire commit_redirects = wb_retscond_taken || wb_reti_taken || wb_branch_taken ||
							 wb_is_brd || wb_is_brcd || wb_is_dbcd || wb_delay_active ||
							 wb_is_rptb || wb_is_rpts || wb_is_trap_now || wb_writes_irq_regs;
	wire rptb_boundary = rmant[ST][8] && (pc == (rmant[RE] + 24'h1));
	wire early_fetch_ok = !commit_redirects && !rptb_boundary && !irq_ready;
	wire pf_seq_ok = (cls_d == CLS_TWO || cls_d == CLS_THREE || cls_d == CLS_LDFC || cls_d == CLS_LDIC ||
					  cls_d == CLS_PMPY || cls_d == CLS_PST || cls_d == CLS_ILLEGAL) &&
					 !(cls_d == CLS_TWO && mbase_d == M_RPTS) && !delay_active;

	always_ff @(posedge clk) begin
		logic signed [63:0] mm1, mm2, manx;
		logic signed [31:0] expx, cntx;
		logic [5:0] cnt6;
		logic signed [7:0] nege;
		logic [31:0] negm;
		logic [23:0] fetch_addr;
		logic pf_fire;
		mm1 = 64'h0; mm2 = 64'h0; manx = 64'h0;
		expx = 32'sh0; cntx = 32'sh0; cnt6 = 6'h0;
		nege = 8'sh0; negm = 32'h0; fetch_addr = 24'h0; pf_fire = 1'b0;
		if (reset) begin
			state <= S_IRQCHECK;
			pc <= boot_pc;
			insn_done <= 1'b0;
			mem_req <= 1'b0;
			delay_active <= 1'b0;
			delay_count <= 2'h0;
			delay_has_target <= 1'b0;
			bkmask <= 32'h0;
			for (int i = 0; i < 28; i = i + 1) rmant[i] <= 32'h0;
			for (int i = 0; i < 8; i = i + 1) rexp[i] <= 8'h0;
			irq_levels_prev <= 12'h0;
			pending_recheck <= 1'b0;
			fetch_irq_recheck <= 1'b0;
			pf_valid <= 1'b0; pf_busy <= 1'b0; pf_want <= 1'b0;
			pf_opcode <= 32'h0; pf_addr <= 24'h0;
		end else if (run) begin
			insn_done <= 1'b0;
			mem_req <= 1'b0;
			// one cycle behind the BK write it tracks: cuts the opcode->ALU->bkmask
			// path, safe since the next indaddr use is S_DECODE of a later instruction
			bkmask <= rmant[BK] | (rmant[BK]>>1) | (rmant[BK]>>2) | (rmant[BK]>>4) |
					  (rmant[BK]>>8) | (rmant[BK]>>16);
			irq_levels_prev <= irq_levels;
			// latch newly-asserted lines into IF unconditionally; a same-cycle trap
			// dispatch or software write to IF (below, in the state case) overrides
			// this for the bits it touches, same as the reference applying set_irq
			// and check_irqs as distinct sequential calls rather than a true merge.
			if (irq_event) rmant[IFR][11:0] <= rmant[IFR][11:0] | irq_set;
			// only one request is ever outstanding, so any ack while pf_busy is the prefetch's
			if (pf_busy && mem_ack) begin
				pf_busy <= 1'b0;
				if (pf_want) begin pf_valid <= 1'b1; pf_opcode <= mem_rdata; end
			end
			case (state)
				S_IRQCHECK: begin
					if (irq_ready) begin
						rmant[IFR][11:0] <= ((rmant[IFR][11:0] | irq_set) & ~(12'h1 << (whichtrap-1))) | {8'h0, irq_levels[3:0]};
						trap_pc_save <= pc;
						trap_num <= whichtrap;
						// pending_recheck survives the trap: it says whether this trap
						// belongs to the step of the instruction that enabled it
						state <= S_TRAP_PUSH_REQ;
					end else begin
						// no trap: if we got here owing to a deferred ST/IE/IF write,
						// that write's own insn_done pulse is due now
						if (pending_recheck) begin
							insn_done <= 1'b1;
							pending_recheck <= 1'b0;
						end
						state <= S_FETCH;
					end
				end
				S_TRAP_PUSH_REQ: begin
					// a dropped prefetch must land before the next request
					if (!pf_busy) begin
						mem_req <= 1'b1; mem_we <= 1'b1;
						mem_addr <= (rmant[SP] + 1) & 24'hffffff;
						mem_wdata <= trap_pc_save;
						rmant[SP] <= rmant[SP] + 1;
						rmant[ST][13] <= 1'b0; // clear GIE
						state <= S_TRAP_PUSH_WAIT;
					end
				end
				S_TRAP_PUSH_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin mem_req <= 1'b0; state <= S_TRAP_VEC_REQ; end
				end
				S_TRAP_VEC_REQ: begin
					mem_req <= 1'b1; mem_we <= 1'b0;
					mem_addr <= {18'h0, trap_num};
					state <= S_TRAP_VEC_WAIT;
				end
				S_TRAP_VEC_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0;
						pc <= mem_rdata[23:0];
						// a trap that update_special/retic triggered inside an
						// instruction ends that step here; a trap taken at the top of a
						// step costs no step of its own in c31_step(), so run on and
						// commit with the instruction at the vector
						insn_done <= pending_recheck;
						pending_recheck <= 1'b0;
						state <= S_FETCH;
					end
				end
				S_FETCH: begin
					// fold of the old S_REPEATFIX: fix pc/RC/ST[8] here, then issue
					// the fetch with the fixed pc the same cycle (always unconditionally
					// followed S_REPEATFIX before, so folding costs nothing)
					if (!pf_busy) begin
						fetch_addr = pc;
						if (rmant[ST][8] && pc == rmant[RE] + 1) begin
							// pre-decrement then test, matching --IREG(TMR_RC) >= 0
							if ($signed(rmant[RC] - 1) >= 0) begin
								rmant[RC] <= rmant[RC] - 1;
								pc <= rmant[RS];
								fetch_addr = rmant[RS];
							end else begin
								rmant[RC] <= rmant[RC] - 1;
								rmant[ST][8] <= 1'b0;
							end
						end
						mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= fetch_addr;
						fetch_irq_recheck <= 1'b0;
						state <= S_FETCH_WAIT;
					end
				end
				S_FETCH_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0;
						if (fetch_irq_recheck && irq_ready) begin
							// an irq latched in between S_COMMIT's early fetch and this ack:
							// discard the fetched opcode, do not advance pc, take the trap
							// the same way S_IRQCHECK would have for this instruction
							rmant[IFR][11:0] <= ((rmant[IFR][11:0] | irq_set) & ~(12'h1 << (whichtrap-1))) | {8'h0, irq_levels[3:0]};
							trap_pc_save <= pc;
							trap_num <= whichtrap;
							state <= S_TRAP_PUSH_REQ;
						end else begin
							opcode <= mem_rdata;
							pf_valid <= 1'b0; pf_want <= 1'b0;
							state <= S_DECODE;
						end
					end
				end
				S_DECODE: begin
					opcode_d <= opcode;
					cls_d <= dcls;
					imm_zx_d <= opcode[31:23] == M_AND || opcode[31:23] == M_ANDN || opcode[31:23] == M_NOT ||
								opcode[31:23] == M_OR || opcode[31:23] == M_RPTS || opcode[31:23] == M_TSTB ||
								opcode[31:23] == M_XOR;
					fq_op_d <= dq_op; fq2_op_d <= dq2_op;
					fqe1_sel_d <= dq_e1; fqm1_sel_d <= dq_m1; fqe2_sel_d <= dq_e2; fqm2_sel_d <= dq_m2;
					fq2e1_sel_d <= dq2_e1; fq2m1_sel_d <= dq2_m1; fq2e2_sel_d <= dq2_e2; fq2m2_sel_d <= dq2_m2;
					// address-select: classify which AR/mode/step the a/b memory
					// slots need, off raw opcode (mirrors the old opcode_d-keyed
					// classify block, one cycle earlier); S_ADDR's indaddr call
					// then only sees already-registered inputs
					dc_a_active <= 1'b0; dc_a_is_mem <= 1'b0; dc_a_is_direct <= 1'b0;
					dc_b_active <= 1'b0; dc_b_is_mem <= 1'b0;
					if (idx11 < 11'h0DC) begin
						// STI/STF/PUSH/POP never read their operand: direct forms skip S_ADDR,
						// STI/STF indirect still need the address and the AR modify
						if (g2 == 2'b01 && !two_nord) begin
							dc_a_active <= 1'b1; dc_a_is_mem <= 1'b1; dc_a_is_direct <= 1'b1;
							dc_a_direct_addr <= {rmant[DP][7:0], imm16};
						end else if (g2 == 2'b10) begin
							dc_a_active <= 1'b1; dc_a_is_mem <= !(opcode[31:23] == M_STI || opcode[31:23] == M_STF);
							dc_a_modf <= ind_mod; dc_a_arval <= rmant[AR0+ind_ar]; dc_a_arid <= AR0+ind_ar;
							dc_a_step <= (ind_mod[4:3]==2'b00) ? {24'b0,disp8} : (ind_mod[4:3]==2'b01) ? rmant[IR0] : rmant[IR1];
						end
					end else if (idx11 >= 11'h100 && idx11 < 11'h144) begin
						if (g2 == 2'b01 || g2 == 2'b11) begin
							dc_a_active <= 1'b1; dc_a_is_mem <= 1'b1;
							dc_a_modf <= {s1field[7:3]}; dc_a_arval <= rmant[AR0+s1field[2:0]]; dc_a_arid <= AR0+s1field[2:0];
							dc_a_step <= (s1field[7:6]==2'b00) ? 32'h1 : (s1field[7:6]==2'b01) ? rmant[IR0] : rmant[IR1];
						end
						if (g2 == 2'b10 || g2 == 2'b11) begin
							dc_b_active <= 1'b1; dc_b_is_mem <= 1'b1;
							dc_b_modf <= {s2field[7:3]}; dc_b_arval <= rmant[AR0+s2field[2:0]]; dc_b_arid <= AR0+s2field[2:0];
							dc_b_step <= (s2field[7:6]==2'b00) ? 32'h1 : (s2field[7:6]==2'b01) ? rmant[IR0] : rmant[IR1];
						end
					end else if ((idx11 >= 11'h200 && idx11 < 11'h254) || (idx11 >= 11'h280 && idx11 < 11'h2D4)) begin
						// LDFcond / LDIcond
						if (g2 == 2'b01) begin
							dc_a_active <= 1'b1; dc_a_is_mem <= 1'b1; dc_a_is_direct <= 1'b1;
							dc_a_direct_addr <= {rmant[DP][7:0], imm16};
						end else if (g2 == 2'b10) begin
							dc_a_active <= 1'b1; dc_a_is_mem <= 1'b1;
							dc_a_modf <= ind_mod; dc_a_arval <= rmant[AR0+ind_ar]; dc_a_arid <= AR0+ind_ar;
							dc_a_step <= (ind_mod[4:3]==2'b00) ? {24'b0,disp8} : (ind_mod[4:3]==2'b01) ? rmant[IR0] : rmant[IR1];
						end
					end else if (idx11 >= 11'h400 && idx11 < 11'h480) begin
						// parallel MPY+ALU forms: src3 = mem(op[15:8]), src4 = mem(op[7:0]), both reads
						dc_a_active <= 1'b1; dc_a_is_mem <= 1'b1;
						dc_a_modf <= {opcode[15:11]}; dc_a_arval <= rmant[AR0+opcode[10:8]]; dc_a_arid <= AR0+opcode[10:8];
						dc_a_step <= (opcode[15:14]==2'b00) ? 32'h1 : (opcode[15:14]==2'b01) ? rmant[IR0] : rmant[IR1];
						dc_b_active <= 1'b1; dc_b_is_mem <= 1'b1;
						dc_b_modf <= {opcode[7:3]}; dc_b_arval <= rmant[AR0+opcode[2:0]]; dc_b_arid <= AR0+opcode[2:0];
						dc_b_step <= (opcode[7:6]==2'b00) ? 32'h1 : (opcode[7:6]==2'b01) ? rmant[IR0] : rmant[IR1];
					end else if (idx11 >= 11'h600 && idx11 < 11'h780) begin
						// parallel store forms: op[15:8] -> a-slot, op[7:0] -> b-slot, both
						// implied displacement 1; pgroup selects which of the 24 forms
						dc_a_active <= 1'b1;
						dc_a_modf <= {opcode[15:11]}; dc_a_arval <= rmant[AR0+opcode[10:8]]; dc_a_arid <= AR0+opcode[10:8];
						dc_a_step <= (opcode[15:14]==2'b00) ? 32'h1 : (opcode[15:14]==2'b01) ? rmant[IR0] : rmant[IR1];
						dc_b_active <= 1'b1;
						dc_b_modf <= {opcode[7:3]}; dc_b_arval <= rmant[AR0+opcode[2:0]]; dc_b_arid <= AR0+opcode[2:0];
						dc_b_step <= (opcode[7:6]==2'b00) ? 32'h1 : (opcode[7:6]==2'b01) ? rmant[IR0] : rmant[IR1];
						// groups 0,1 (STF||STF, STI||STI) are two stores, no read; 2,3 load
						// both fields; the 20 ALU||STx forms load only the q (b) field
						if (pgroup == 5'd2 || pgroup == 5'd3) begin
							dc_a_is_mem <= 1'b1; dc_b_is_mem <= 1'b1;
						end else if (pgroup != 5'd0 && pgroup != 5'd1) begin
							dc_b_is_mem <= 1'b1;
						end
					end
					// exec-cone register operands, read off raw opcode here instead of
					// live in S_EXEC (see fwd_reg for the AR-writeback-in-between case)
					da_val_r <= rread_fwd_irq(opcode[20:16]);
					db_val_r <= rread_fwd_irq(opcode[4:0]);
					s1_val_r <= rread_fwd_irq(opcode[12:8]);
					fd_e_r <= rexp[opcode[20:16]]; fd_m_r <= rmant_fwd_irq(opcode[20:16]);
					fb_e_r <= rexp[opcode[2:0]];   fb_m_r <= rmant[opcode[2:0]];
					fs1_e_r <= rexp[opcode[10:8]]; fs1_m_r <= rmant[opcode[10:8]];
					p1_e_r <= rexp[opcode[21:19]]; p1_m_r <= rmant[opcode[21:19]];
					p3_e_r <= rexp[opcode[18:16]]; p3_m_r <= rmant[opcode[18:16]];
					ppd1_e_r <= rexp[opcode[24:22]]; ppd1_m_r <= rmant[opcode[24:22]];
					db_ar_val_r <= rread(AR0[4:0] + {2'b0, opcode[24:22]});
					fb5_e_r <= rexp[opcode[4:0]]; fb5_m_r <= rmant[opcode[4:0]];
					fd3_e_r <= rexp[opcode[18:16]]; fd3_m_r <= rmant[opcode[18:16]];
					// register-only forms have no indirect/store address to compute,
					// so they skip S_ADDR and their cycle count is unchanged
					if (needs_addr_raw) state <= S_ADDR;
					else begin
						// S_ADDR is skipped, so fwd_reg still sees the previous S_ADDR
						// instruction's AR forward; clear it after this read so a later
						// direct exit never forwards an AR a register write has overtaken
						a_r <= fwd_reg(opcode[20:16], rread_fwd_irq(opcode[20:16]));
						b_r <= fwd_reg(opcode[4:0], rread_fwd_irq(opcode[4:0]));
						s1_r <= fwd_reg(opcode[12:8], rread_fwd_irq(opcode[12:8]));
						dc_opa_ar_wb <= 1'b0; dc_opb_ar_wb <= 1'b0;
						state <= S_EXEC;
					end
					// the irq check owed by a commit-side fetch; the latches above are
					// harmless on a trap, only state/pc/trap registers depend on it;
					// this can be the irq_event cycle: keep the bits the IFR latch at the top adds
					if (fetch_irq_recheck && irq_ready) begin
						rmant[IFR][11:0] <= ((rmant[IFR][11:0] | irq_set) & ~(12'h1 << (whichtrap-1))) | {8'h0, irq_levels[3:0]};
						trap_pc_save <= pc;
						trap_num <= whichtrap;
						state <= S_TRAP_PUSH_REQ;
					end else begin
						fetch_pc <= pc + 24'h1;
						pc <= pc + 24'h1;
					end
					fetch_irq_recheck <= 1'b0;
				end
				S_ADDR: begin
					// same computation S_DECODE used to do inline, now off opcode_d
					// (registered last cycle) so it is its own short cone
					dc_opa_addr <= opa_addr; dc_opa_ar_idx <= opa_ar_idx;
					dc_opa_ar_new <= opa_ar_new; dc_opa_ar_wb <= opa_ar_wb;
					dc_opb_addr <= opb_addr; dc_opb_ar_idx <= opb_ar_idx;
					dc_opb_ar_new <= opb_ar_new; dc_opb_ar_wb <= opb_ar_wb;
					if (needs_opa) begin
						if (opa_illegal) state <= S_OPA_REQ;
						else begin
							mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= opa_addr;
							state <= S_OPA_WAIT;
						end
					end else if (needs_opb) begin
						if (opb_illegal) state <= S_OPB_REQ;
						else begin
							mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= opb_addr;
							state <= S_OPB_WAIT;
						end
					end else begin
						// only STF||STF, STI||STI (arms read p3/ppd1/dc_op* only) and STI/STF
						// indirect (STI forwards its own AR modify in store_data) get here,
						// so no AR-forward of a_r/b_r/s1_r is needed
						a_r <= da_val_r;
						b_r <= db_val_r;
						s1_r <= s1_val_r;
`ifdef SIMULATION
						if (!((cls_d == CLS_PST && (pgroup_d == 5'd0 || pgroup_d == 5'd1)) ||
							  (cls_d == CLS_TWO && (mbase_d == M_STI || mbase_d == M_STF) && g2_d == 2'b10)))
							$display("C31 BUG: S_ADDR direct exec %08x", opcode_d);
`endif
						state <= S_EXEC;
					end
				end
				S_OPA_REQ: begin
					if (opa_illegal) begin
						a_r <= fwd_reg(dreg_d, da_val_r);
						b_r <= fwd_reg(opcode_d[4:0], db_val_r);
						s1_r <= fwd_reg(opcode_d[12:8], s1_val_r);
						state <= S_EXEC;
					end else begin
						mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= dc_opa_addr;
						// with a second operand the first field's AR modify is the
						// deferred one and must land after the second field's
						if (dc_opa_ar_wb && !needs_opb) rmant[dc_opa_ar_idx] <= dc_opa_ar_new;
						state <= S_OPA_WAIT;
					end
				end
				S_OPA_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0; opa_val <= mem_rdata;
						if (dc_opa_ar_wb && !needs_opb) rmant[dc_opa_ar_idx] <= dc_opa_ar_new;
						if (needs_opb) begin
							mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= dc_opb_addr;
							if (dc_opb_ar_wb) rmant[dc_opb_ar_idx] <= dc_opb_ar_new;
							if (dc_opa_ar_wb) rmant[dc_opa_ar_idx] <= dc_opa_ar_new;
							state <= S_OPB_WAIT;
						end else begin
							a_r <= fwd_reg(dreg_d, da_val_r);
							b_r <= fwd_reg(opcode_d[4:0], db_val_r);
							s1_r <= fwd_reg(opcode_d[12:8], s1_val_r);
							state <= S_EXEC;
						end
					end
				end
				S_OPB_REQ: begin
					if (opb_illegal) begin
						a_r <= fwd_reg(dreg_d, da_val_r);
						b_r <= fwd_reg(opcode_d[4:0], db_val_r);
						s1_r <= fwd_reg(opcode_d[12:8], s1_val_r);
						state <= S_EXEC;
					end else begin
						mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= dc_opb_addr;
						if (dc_opb_ar_wb) rmant[dc_opb_ar_idx] <= dc_opb_ar_new;
						// deferred first-field modify wins when both fields share an AR
						if (dc_opa_ar_wb && needs_opa) rmant[dc_opa_ar_idx] <= dc_opa_ar_new;
						state <= S_OPB_WAIT;
					end
				end
				S_OPB_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0; opb_val <= mem_rdata;
						if (dc_opb_ar_wb && !needs_opa) rmant[dc_opb_ar_idx] <= dc_opb_ar_new;
						a_r <= fwd_reg(dreg_d, da_val_r);
						b_r <= fwd_reg(opcode_d[4:0], db_val_r);
						s1_r <= fwd_reg(opcode_d[12:8], s1_val_r);
						state <= S_EXEC;
					end
				end
				S_EXEC: begin
					x_b <= w_b; x_s1 <= w_s1v; x_s2 <= w_s2v;
					if (retscond_taken || reti_taken || (mbase_d == M_POP && cls_d == CLS_TWO && g2_d==2'b01) ||
						(mbase_d == M_POPF && cls_d == CLS_TWO && g2_d==2'b01)) begin
						// pop-style memory read: a dedicated wait state so this never
						// loops back through S_EXEC's own dispatch and re-issues itself
						mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= rmant[SP];
						rmant[SP] <= rmant[SP] - 1;
						state <= S_POP_WAIT;
					end else if (fq_op != FQ_NONE) begin
						fu_op <= fq_op; fu_e1 <= fq_e1; fu_m1 <= fq_m1;
						fu_e2 <= fq_e2; fu_m2 <= fq_m2;
						fu_slot <= 1'b0; fu_stage <= 3'd0;
						state <= S_FU;
					end else if (sh_class) begin
						sh_val_r <= sh_val; sh_cnt_r <= sh_cntsrc; sh_arith_r <= sh_arith;
						state <= S_SH;
					end else if (is_store) begin
						st_ar_idx <= store_ar_idx; st_ar_new <= store_ar_new; st_ar_wb <= store_ar_wb;
						mem_req <= 1'b1; mem_we <= 1'b1; mem_addr <= store_addr; mem_wdata <= store_data;
						state <= S_STORE_WAIT;
					end else begin
						pf_fire = 1'b1; state <= S_WRITEBACK;
					end
				end
				S_SH: begin
					sh_o_val <= sh_res_o; sh_o_c <= sh_c_o;
					if (is_store) begin
						st_ar_idx <= store_ar_idx; st_ar_new <= store_ar_new; st_ar_wb <= store_ar_wb;
						mem_req <= 1'b1; mem_we <= 1'b1; mem_addr <= store_addr; mem_wdata <= store_data;
						state <= S_STORE_WAIT;
					end else begin pf_fire = 1'b1; state <= S_WRITEBACK; end
				end
				S_FU: begin
					case (fu_stage)
						3'd0: begin // align (add/sub), multiply, or a whole conversion
							case (fu_op)
								FQ_ADD, FQ_SUB: begin
									if (fu_op == FQ_ADD && fu_e1 == 8'sh80) begin
										fo_e <= fu_e2; fo_m <= fu_m2; fo_v <= 1'b0; fo_u <= 1'b0;
										fu_stage <= 3'd7;
									end else if (fu_e2 == 8'sh80) begin
										fo_e <= fu_e1; fo_m <= fu_m1; fo_v <= 1'b0; fo_u <= 1'b0;
										fu_stage <= 3'd7;
									end else begin
										mm1 = ({{32{fu_m1[31]}}, fu_m1}) ^ 64'h0000000080000000;
										mm2 = ({{32{fu_m2[31]}}, fu_m2}) ^ 64'h0000000080000000;
										if (fu_e1 > fu_e2) begin
											cntx = fu_e1 - fu_e2;
											if (cntx >= 32) begin
												fo_e <= fu_e1; fo_m <= fu_m1; fo_v <= 1'b0; fo_u <= 1'b0;
												fu_stage <= 3'd7;
											end else begin
												fu_mm1 <= mm1; fu_mm2 <= mm2 >>> cntx;
												fu_exp <= fu_e1; fu_is_sub <= (fu_op == FQ_SUB);
												fu_stage <= 3'd1;
											end
										end else begin
											cntx = fu_e2 - fu_e1;
											if (cntx >= 32) begin
												if (fu_op == FQ_ADD) begin fo_e <= fu_e2; fo_m <= fu_m2; end
												else begin
													fneg(fu_e2, fu_m2, nege, negm);
													fo_e <= nege; fo_m <= negm;
												end
												fo_v <= 1'b0; fo_u <= 1'b0;
												fu_stage <= 3'd7;
											end else begin
												fu_mm1 <= mm1 >>> cntx; fu_mm2 <= mm2;
												fu_exp <= fu_e2; fu_is_sub <= 1'b0;
												fu_stage <= 3'd1;
											end
										end
									end
								end
								FQ_MPY: begin
									if (fu_e1 == 8'sh80 || fu_e2 == 8'sh80) begin
										fo_e <= 8'sh80; fo_m <= 32'h0; fo_v <= 1'b0; fo_u <= 1'b0;
										fu_stage <= 3'd7;
									end else begin
										fu_prod <= mul_a * mul_b;
										fu_exp <= fu_e1 + fu_e2;
										fu_stage <= 3'd1;
									end
								end
								FQ_IMPY: begin
									fu_prod <= mul_a * mul_b;
									fu_stage <= 3'd7;
								end
								FQ_I2F: begin
									if (fu_m1 == 32'h0) begin
										fo_m <= 32'h0; fo_e <= 8'sh80;
										fo_v <= 1'b0; fo_u <= 1'b0; fu_stage <= 3'd7;
									end else if (fu_m1 == 32'hffffffff) begin
										fo_m <= 32'h80000000; fo_e <= -8'sd1;
										fo_v <= 1'b0; fo_u <= 1'b0; fu_stage <= 3'd7;
									end else begin
										fu_cnt <= 6'(clz32(fu_m1 ^ {32{fu_m1[31]}}));
										fu_stage <= 3'd4;
									end
								end
								FQ_NORM: begin
									if (fu_e1 == 8'sh80 || fu_m1 == 0) begin
										// MAME zeroes dst in this branch but then overwrites it on the
										// way out with the locals it never touched, so src passes through
										fo_e <= fu_e1; fo_m <= fu_m1; fo_v <= 1'b0; fo_u <= (fu_m1 != 0);
										fu_stage <= 3'd7;
									end else begin
										fu_cnt <= 6'(clz32(fu_m1 ^ {32{fu_m1[31]}}));
										fu_exp <= fu_e1;
										fu_stage <= 3'd5;
									end
								end
								default: begin // FQ_F2I
									cntx = 32'sd31 - fu_e1;
									if (cntx <= 0) begin
										fo_m <= fu_m1[31] ? 32'h80000000 : 32'h7fffffff; fo_v <= 1'b1;
									end else if (cntx > 31) begin
										fo_m <= fu_m1[31] ? 32'hffffffff : 32'h0; fo_v <= 1'b0;
									end else begin
										fo_m <= asr32(fu_m1, cntx) ^ (32'h1 << (31 - cntx)); fo_v <= 1'b0;
									end
									fo_e <= 8'sh0; fo_u <= 1'b0;
									fu_stage <= 3'd7;
								end
							endcase
						end
						3'd1: begin // add the aligned mantissas, or chop the product back to 1.2.31
							if (fu_op == FQ_ADD) begin
								fu_man <= fu_mm1 + fu_mm2; fu_stage <= 3'd2;
							end else if (fu_op == FQ_SUB) begin
								fu_man <= fu_mm1 - fu_mm2; fu_stage <= 3'd2;
							end else begin
								manx = fu_prod >>> 15;
								expx = fu_exp;
								if (manx == 0) begin expx = -32'sd128; manx = 64'h0000000080000000; end
								else if (manx >= 64'sh0000000100000000) begin
									manx = manx >>> 1; expx = expx + 1;
									if (manx >= 64'sh0000000100000000) begin manx = manx >>> 1; expx = expx + 1; end
								end else if (manx < -64'sh0000000100000000) begin
									manx = manx >>> 1; expx = expx + 1;
								end
								// the multiply has no leading-zero step, so skip straight to the clamp
								fu_man <= manx; fu_exp <= expx; fu_cnt <= 6'd0;
								fu_is_sub <= 1'b0; fu_zero <= 1'b0;
								fu_stage <= 3'd3;
							end
						end
						3'd2: begin // leading zero/one count for the add/sub normalize
							manx = fu_man; expx = fu_exp; cnt6 = 6'd0;
							fu_zero <= (fu_man == 0);
							if (fu_man == 0 || fu_exp == -32'sd128) begin
								manx = 64'h0000000080000000; expx = -32'sd128;
							end else if (fu_man >= 64'sh0000000100000000 || fu_man < -64'sh0000000100000000) begin
								manx = fu_man >>> 1; expx = fu_exp + 1;
							end else if (fu_man < 64'sh0000000080000000 && fu_man >= -64'sh0000000080000000) begin
								cnt6 = (fu_man > 0) ? 6'(clz32(fu_man[31:0])) : 6'(clo32(fu_man[31:0]));
							end
							fu_man <= manx; fu_exp <= expx; fu_cnt <= cnt6;
							fu_stage <= 3'd3;
						end
						3'd3: begin // normalize shift and the over/underflow clamps
							manx = fu_man <<< fu_cnt;
							expx = fu_exp - $signed({26'b0, fu_cnt});
							fo_v <= 1'b0; fo_u <= 1'b0;
							if (expx <= -32'sd128) begin
								if (!fu_is_sub || !fu_zero || expx < -32'sd128) fo_u <= 1'b1;
								manx = 64'h0000000080000000; expx = -32'sd128;
							end else if (expx > 32'sd127) begin
								manx = manx[63] ? 64'h0 : 64'h00000000ffffffff;
								expx = 32'sd127; fo_v <= 1'b1;
							end
							fo_e <= expx[7:0];
							fo_m <= manx[31:0] ^ 32'h80000000;
							fu_stage <= 3'd7;
						end
						3'd4: begin // int-to-float shift
							fo_m <= (fu_m1 << fu_cnt) ^ 32'h80000000;
							fo_e <= 8'(32'sd31 - $signed({26'b0, fu_cnt}));
							fo_v <= 1'b0; fo_u <= 1'b0;
							fu_stage <= 3'd7;
						end
						3'd5: begin // norm: shift mantissa, subtract exponent by the leading count
							// count 32 (an all-ones mantissa) is a C shift-count UB that x86
							// resolves as count mod 32, leaving the mantissa untouched
							fu_man <= 64'(fu_m1 << fu_cnt[4:0]);
							fu_exp <= fu_exp - $signed({26'b0, fu_cnt});
							fu_stage <= 3'd6;
						end
						3'd6: begin // norm: underflow clamp and flags
							// the underflow test must see the unwrapped exponent: -127-2 is
							// +127 once truncated to 8 bits and would sail past the clamp
							fo_v <= 1'b0;
							if ($signed(fu_exp) <= -32'sd128) begin
								fo_m <= 32'h0; fo_e <= 8'sh80; fo_u <= 1'b1;
							end else begin
								fo_m <= fu_man[31:0]; fo_e <= fu_exp[7:0]; fo_u <= 1'b0;
							end
							fu_stage <= 3'd7;
						end
						default: begin // 7: hand the result over, then run the second request if any
							if (fu_slot == 1'b0) begin
								f1_e <= fo_e; f1_m <= fo_m; f1_v <= fo_v; f1_u <= fo_u; f1_p <= fu_prod;
								if (fq2_op != FQ_NONE) begin
									fu_op <= fq2_op; fu_e1 <= fq2_e1; fu_m1 <= fq2_m1;
									fu_e2 <= fq2_e2; fu_m2 <= fq2_m2;
									fu_slot <= 1'b1; fu_stage <= 3'd0;
								end else begin
									if (is_store) begin
										st_ar_idx <= store_ar_idx; st_ar_new <= store_ar_new; st_ar_wb <= store_ar_wb;
										mem_req <= 1'b1; mem_we <= 1'b1; mem_addr <= store_addr; mem_wdata <= store_data;
										state <= S_STORE_WAIT;
									end else begin pf_fire = 1'b1; state <= S_WRITEBACK; end
								end
							end else begin
								f2_e <= fo_e; f2_m <= fo_m; f2_v <= fo_v; f2_u <= fo_u;
								if (is_store) begin
									st_ar_idx <= store_ar_idx; st_ar_new <= store_ar_new; st_ar_wb <= store_ar_wb;
									mem_req <= 1'b1; mem_we <= 1'b1; mem_addr <= store_addr; mem_wdata <= store_data;
									state <= S_STORE_WAIT;
								end else begin pf_fire = 1'b1; state <= S_WRITEBACK; end
							end
						end
					endcase
				end
				S_POP_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin mem_req <= 1'b0; opa_val <= mem_rdata; pf_fire = 1'b1; state <= S_WRITEBACK; end
				end
				S_STORE_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0;
						if (st_ar_wb) rmant[st_ar_idx] <= st_ar_new;
						if (is_store2) begin
							st2_ar_idx <= store2_ar_idx; st2_ar_new <= store2_ar_new; st2_ar_wb <= store2_ar_wb;
							mem_req <= 1'b1; mem_we <= 1'b1; mem_addr <= store2_addr; mem_wdata <= store2_data;
							state <= S_STORE2_WAIT;
						end else begin pf_fire = 1'b1; state <= S_WRITEBACK; end
					end
				end
				S_STORE2_WAIT: begin
					mem_req <= 1'b0; // pulse: deassert one cycle after the request, before ack is checked
					if (mem_ack) begin
						mem_req <= 1'b0;
						if (st2_ar_wb) rmant[st2_ar_idx] <= st2_ar_new;
						pf_fire = 1'b1; state <= S_WRITEBACK;
					end
				end
				S_WRITEBACK: begin
					// snapshot the writeback cone: breaks opcode->class->ALU->write into
					// two shorter cones (this latch, then S_COMMIT's plain mux+write).
					// delay_active/delay_count are read here pre-edge, same as the
					// combinational read the old single-state version used to do.
					wb_store_ar_wb <= store_ar_wb; wb_is_store <= is_store;
					wb_store_ar_idx <= store_ar_idx; wb_store_ar_new <= store_ar_new;
					wb_wr_en <= wr_en; wb_wr_is_float <= wr_is_float;
					wb_wr_idx <= wr_idx; wb_wr_val <= wr_val; wb_wr_exp <= wr_exp;
					wb_wr2_en <= wr2_en; wb_wr2_is_float <= wr2_is_float;
					wb_wr2_idx <= wr2_idx; wb_wr2_val <= wr2_val; wb_wr2_exp <= wr2_exp;
					wb_touches_flags <= touches_flags; wb_touch_z <= touch_z;
					wb_clears_c <= clears_c;
					wb_new_n <= new_n; wb_new_z <= new_z; wb_new_v <= new_v;
					wb_new_uf <= new_uf; wb_new_c <= new_c;
					wb_lat_v <= lat_v; wb_lat_uf <= lat_uf;
					wb_retscond_taken <= retscond_taken; wb_reti_taken <= reti_taken;
					wb_branch_taken <= branch_taken; wb_next_pc <= next_pc;
					wb_is_brd <= is_brd; wb_is_brcd <= is_brcd; wb_is_dbcd <= is_dbcd;
					wb_is_rptb <= is_rptb; wb_is_rpts <= is_rpts; wb_rpts_count <= rpts_count;
					wb_is_trap_now <= is_trap_now;
					wb_writes_irq_regs <= writes_irq_regs;
					wb_delay_active <= delay_active; wb_delay_count <= delay_count;
`ifdef SIMULATION
					if (illegal_insn) begin
						if (sim_illegal_n < 32'd16)
							$display("C31 ILLEGAL opcode %08x at pc %06x", opcode_d, fetch_pc - 24'h1);
						sim_illegal_n <= sim_illegal_n + 32'h1;
					end
`endif
					state <= S_COMMIT;
				end
				S_COMMIT: begin
					// exactly the writes S_WRITEBACK used to do, off the wb_* snapshot;
					// mbase_d/idx11_d/dreg_d/opa_val/fetch_pc/opcode_d are still this
					// instruction's values (stable since S_DECODE), no need to snapshot
					if (wb_store_ar_wb && !wb_is_store) rmant[wb_store_ar_idx] <= wb_store_ar_new;
					if (wb_wr_en) begin
						if (wb_wr_is_float) begin
							rmant[wb_wr_idx[2:0]] <= wb_wr_val;
							rexp[wb_wr_idx[2:0]] <= wb_wr_exp;
						end else begin
							rmant[wb_wr_idx] <= wb_wr_val;
						end
					end
					if (wb_wr2_en) begin
						if (wb_wr2_is_float) begin
							rmant[wb_wr2_idx[2:0]] <= wb_wr2_val;
							rexp[wb_wr2_idx[2:0]] <= wb_wr2_exp;
						end else begin
							rmant[wb_wr2_idx] <= wb_wr2_val;
						end
					end
					if (wb_touches_flags) begin
						rmant[ST][3] <= wb_new_n;
						if (wb_touch_z) rmant[ST][2] <= wb_new_z;
						rmant[ST][1] <= wb_new_v;
						rmant[ST][4] <= wb_new_uf;
						if (wb_clears_c) rmant[ST][0] <= wb_new_c;
						if (wb_new_v || wb_lat_v) rmant[ST][5] <= 1'b1;
						if (wb_new_uf || wb_lat_uf) rmant[ST][6] <= 1'b1;
					end
					if (mbase_d == M_POP && cls_d == CLS_TWO) begin
						rmant[dreg_d] <= opa_val;
						if (dreg_d < 8) begin
							rmant[ST][3] <= opa_val[31]; rmant[ST][2] <= (opa_val==0);
							rmant[ST][1] <= 1'b0; rmant[ST][4] <= 1'b0;
						end
					end
					if (mbase_d == M_POPF && cls_d == CLS_TWO) begin
						rmant[dreg_d[2:0]] <= {opa_val[31:0]<<8};
						rexp[dreg_d[2:0]] <= opa_val[31:24];
						rmant[ST][3] <= opa_val[23]; // approx sign bit after shift; harmless best effort
						rmant[ST][2] <= (opa_val[31:24]==8'h80 && opa_val[23:0]==0);
					end
					if (wb_retscond_taken) begin
						pc <= opa_val[23:0];
					end else if (wb_reti_taken) begin
						// RETI pops PC (same mechanism as RETS) and sets GIE
						pc <= opa_val[23:0];
						rmant[ST][13] <= 1'b1;
					end
					if (wb_branch_taken && !wb_retscond_taken && !wb_is_brcd && !wb_is_dbcd) pc <= wb_next_pc;
					if (wb_is_brd || wb_is_brcd || wb_is_dbcd) begin
						delay_active <= 1'b1;
						delay_count <= 2'd3;
						delay_target <= wb_next_pc;
						delay_has_target <= wb_is_brd || wb_branch_taken;
					end else if (wb_delay_active) begin
						if (wb_delay_count == 2'd1) begin
							delay_active <= 1'b0;
							if (delay_has_target) pc <= delay_target;
						end else begin
							delay_count <= wb_delay_count - 2'd1;
						end
					end
					if (wb_is_rptb) begin
						rmant[RE] <= opcode_d[23:0];
						rmant[RS] <= fetch_pc;
						rmant[ST][8] <= 1'b1;
					end
					if (wb_is_rpts) begin
						rmant[RC] <= wb_rpts_count;
						rmant[RS] <= fetch_pc;
						rmant[RE] <= fetch_pc;
						rmant[ST][8] <= 1'b1;
					end
					if (wb_is_trap_now) begin
						// TRAPcond: push current pc and jump, single extra cycle here
						rmant[SP] <= rmant[SP] + 1;
						rmant[ST][13] <= 1'b0;
					end
					// insn_done marks one architectural step, matching c31_step(): a delayed
					// branch and its 3 delay slots are ONE step, so only the last delay slot
					// (or a plain instruction outside any delay sequence) pulses it. Moved
					// here with the commit so it still pulses one state after the writes land.
					if (wb_is_brd || wb_is_brcd || wb_is_dbcd) begin
						insn_done <= 1'b0;
					end else if (wb_delay_active) begin
						insn_done <= (wb_delay_count == 2'd1);
					end else if (wb_writes_irq_regs) begin
						// deferred: pulses in S_IRQCHECK, either as part of the trap
						// this write may itself trigger, or on its own if it doesn't
						insn_done <= 1'b0;
						pending_recheck <= 1'b1;
					end else begin
						insn_done <= 1'b1;
					end
					// pc is already the confirmed next-fetch address: take the prefetch
					// off the bus (every memory model acks it in this cycle), from the
					// buffer, wait for it, or issue the fetch now; a mismatch goes the slow way
					if (wb_delay_active || wb_is_brd || wb_is_brcd || wb_is_dbcd) begin
						state <= S_FETCH;
					end else if (early_fetch_ok && pf_addr == pc && pf_busy && pf_want && mem_ack) begin
						opcode <= mem_rdata;
						fetch_irq_recheck <= 1'b1;
						state <= S_DECODE;
					end else if (early_fetch_ok && pf_addr == pc && pf_valid) begin
						opcode <= pf_opcode;
						fetch_irq_recheck <= 1'b1;
						state <= S_DECODE;
					end else if (early_fetch_ok && pf_addr == pc && pf_busy && pf_want) begin
						fetch_irq_recheck <= 1'b1;
						state <= S_FETCH_WAIT;
					end else if (early_fetch_ok && !pf_busy) begin
						mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= pc;
`ifdef SIMULATION
						if (pc[23:8] == 16'h8080 || pc == 24'hA00000) $display("C31 BUG: prefetch %06x", pc);
`endif
						fetch_irq_recheck <= 1'b1;
						state <= S_FETCH_WAIT;
					end else state <= S_IRQCHECK;
					pf_valid <= 1'b0; pf_want <= 1'b0;
				end
				default: state <= S_IRQCHECK;
			endcase
			if (pf_fire && pf_seq_ok) begin
				mem_req <= 1'b1; mem_we <= 1'b0; mem_addr <= pc;
				pf_busy <= 1'b1; pf_want <= 1'b1; pf_valid <= 1'b0; pf_addr <= pc;
`ifdef SIMULATION
				if (pc[23:8] == 16'h8080 || pc == 24'hA00000 || pf_busy) $display("C31 BUG: prefetch %06x", pc);
`endif
			end
		end
	end

endmodule
