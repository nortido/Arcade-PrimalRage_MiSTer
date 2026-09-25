module nvram_hps #(parameter [7:0] INDEX = 8'd4)
(
	input        clk,
	input        ioctl_download,
	input [15:0] ioctl_index,
	input        ioctl_wr,
	input [26:0] ioctl_addr,
	input  [7:0] ioctl_dout,
	input        wr_toggle,
	input        autosave,
	input        save_req,

	output [10:0] nv_addr,
	output  [7:0] nv_din,
	output        nv_we,
	output reg    upload_req
);

assign nv_addr = ioctl_addr[10:0];
assign nv_din  = ioctl_dout;
assign nv_we   = ioctl_download & ioctl_wr & (ioctl_index[7:0] == INDEX) & (ioctl_addr[26:11] == 16'd0);

// wr_toggle flips once per CPU EEPROM write (clk_cpu domain); two flops
// synchronise it into clk, t1^t2 is a one-clock pulse per change
reg t1, t2;
always @(posedge clk) begin
	t1 <= wr_toggle;
	t2 <= t1;
end
wire toggle_pulse = t1 ^ t2;

// autosave and save_req are quasi-static status[] levels, already in clk;
// save_req is a "T" menu item, so treat it as a level and edge-detect it here
reg autosave_d, save_req_d, dirty;
always @(posedge clk) begin
	autosave_d <= autosave;
	save_req_d <= save_req;
end
wire autosave_edge = autosave & ~autosave_d;
wire save_edge      = save_req & ~save_req_d;

wire pulse_now = (toggle_pulse & autosave) | save_edge | (autosave_edge & dirty);

always @(posedge clk) begin
	upload_req <= pulse_now;
	if (pulse_now)
		dirty <= 1'b0;
	else if (toggle_pulse & ~autosave)
		dirty <= 1'b1;
end

endmodule
