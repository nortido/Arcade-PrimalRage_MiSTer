module tdp_ram #(parameter AW = 11, DW = 8)
(
	input               clk_a,
	input               clk_b,
	input      [AW-1:0] addr_a,
	input      [DW-1:0] din_a,
	input               we_a,
	output reg [DW-1:0] q_a,
	input      [AW-1:0] addr_b,
	input      [DW-1:0] din_b,
	input               we_b,
	output reg [DW-1:0] q_b
);

`ifdef ALTERA_RESERVED_QIS
// port A and port B may run on different (related) clocks: the CPU domain and clk_sys
// Quartus 17 duplicates an inferred array that has two read ports, so the
// true dual-port M10K is instantiated directly for synthesis
wire [DW-1:0] qa_w, qb_w;
altsyncram #(
	.operation_mode("BIDIR_DUAL_PORT"),
	.ram_block_type("M10K"),
	.width_a(DW), .widthad_a(AW), .numwords_a(1 << AW),
	.width_b(DW), .widthad_b(AW), .numwords_b(1 << AW),
	.outdata_reg_a("UNREGISTERED"), .outdata_reg_b("UNREGISTERED"),
	.address_reg_b("CLOCK1"), .indata_reg_b("CLOCK1"), .wrcontrol_wraddress_reg_b("CLOCK1"),
	.read_during_write_mode_mixed_ports("DONT_CARE"),
	.read_during_write_mode_port_a("NEW_DATA_NO_NBE_READ"),
	.read_during_write_mode_port_b("NEW_DATA_NO_NBE_READ"),
	.power_up_uninitialized("FALSE"),
	.intended_device_family("Cyclone V"),
	.lpm_type("altsyncram")
) ram (
	.clock0(clk_a), .clock1(clk_b), .clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
	.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .eccstatus(),
	.address_a(addr_a), .data_a(din_a), .wren_a(we_a), .rden_a(1'b1), .byteena_a(1'b1), .q_a(qa_w),
	.address_b(addr_b), .data_b(din_b), .wren_b(we_b), .rden_b(1'b1), .byteena_b(1'b1), .q_b(qb_w)
);
always @* begin
	q_a = qa_w;
	q_b = qb_w;
end
`else
reg [DW-1:0] mem [0:(1<<AW)-1];
always @(posedge clk_a) begin
	if (we_a) mem[addr_a] <= din_a;
	q_a <= mem[addr_a];
end
always @(posedge clk_b) begin
	if (we_b) mem[addr_b] <= din_b;
	q_b <= mem[addr_b];
end
`endif

endmodule
