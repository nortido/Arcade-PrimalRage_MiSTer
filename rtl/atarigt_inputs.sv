module atarigt_inputs
(
	input  [31:0] joystick_0,
	input  [31:0] joystick_1,
	input         set_jan,

	output [31:0] p1p2,
	output [15:0] coin
);

// primrageo (Dec 1994): no start bits, buttons 1-4 fill the start/button1-3
// slots of the common port at bits 8-11 (P2) and 24-27 (P1); the game starts
// on button 1, so the Start button presses it too
wire [31:0] p1p2_dec;
assign p1p2_dec[7:0]   = 8'hff;
assign p1p2_dec[8]     = ~(joystick_1[4] | joystick_1[9]);
assign p1p2_dec[9]     = ~joystick_1[5];
assign p1p2_dec[10]    = ~joystick_1[6];
assign p1p2_dec[11]    = ~joystick_1[7];
assign p1p2_dec[12]    = ~joystick_1[0];
assign p1p2_dec[13]    = ~joystick_1[1];
assign p1p2_dec[14]    = ~joystick_1[2];
assign p1p2_dec[15]    = ~joystick_1[3];
assign p1p2_dec[23:16] = 8'hff;
assign p1p2_dec[24]    = ~(joystick_0[4] | joystick_0[9]);
assign p1p2_dec[25]    = ~joystick_0[5];
assign p1p2_dec[26]    = ~joystick_0[6];
assign p1p2_dec[27]    = ~joystick_0[7];
assign p1p2_dec[28]    = ~joystick_0[0];
assign p1p2_dec[29]    = ~joystick_0[1];
assign p1p2_dec[30]    = ~joystick_0[2];
assign p1p2_dec[31]    = ~joystick_0[3];

// primrage (Jan 1995): common port unmodified except a dedicated start at
// bit 8/24 and button4 pulled out to bit 1 (P1) / bit 3 (P2)
wire [31:0] p1p2_jan;
assign p1p2_jan[0]     = 1'b1;
assign p1p2_jan[1]     = ~joystick_0[7];
assign p1p2_jan[2]     = 1'b1;
assign p1p2_jan[3]     = ~joystick_1[7];
assign p1p2_jan[7:4]   = 4'hf;
assign p1p2_jan[8]     = ~joystick_1[9];
assign p1p2_jan[9]     = ~joystick_1[4];
assign p1p2_jan[10]    = ~joystick_1[5];
assign p1p2_jan[11]    = ~joystick_1[6];
assign p1p2_jan[12]    = ~joystick_1[0];
assign p1p2_jan[13]    = ~joystick_1[1];
assign p1p2_jan[14]    = ~joystick_1[2];
assign p1p2_jan[15]    = ~joystick_1[3];
assign p1p2_jan[23:16] = 8'hff;
assign p1p2_jan[24]    = ~joystick_0[9];
assign p1p2_jan[25]    = ~joystick_0[4];
assign p1p2_jan[26]    = ~joystick_0[5];
assign p1p2_jan[27]    = ~joystick_0[6];
assign p1p2_jan[28]    = ~joystick_0[0];
assign p1p2_jan[29]    = ~joystick_0[1];
assign p1p2_jan[30]    = ~joystick_0[2];
assign p1p2_jan[31]    = ~joystick_0[3];

assign p1p2 = set_jan ? p1p2_jan : p1p2_dec;

// coin port is the same "COIN" region in both sets: bit7 = COINL (P1), bit6 = COINR (P2)
assign coin = {8'hff, ~joystick_0[8], ~joystick_1[8], 6'h3f};

endmodule
