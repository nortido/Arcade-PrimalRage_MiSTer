module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] cage_audio_l, cage_audio_r;
assign AUDIO_S   = 1;
assign AUDIO_L   = aud_l;
assign AUDIO_R   = aud_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[2:1];

`include "build_id.v"
localparam CONF_STR = {
	"PrimalRage;;",
	"-;",
	"P1,Video Settings;",
	"P1O[2:1],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[5:3],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P1O[9:8],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1-;",
	"P1O[25:20],Analog Video H-Pos,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[30:26],Analog Video V-Pos,0,+1,+2,+3,+4,+5,+6,+7,+8,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"O[19:18],Volume,100%,50%,25%,12.5%;",
	"O[7],Service Mode,Off,On;",
	"O[31],Autosave NVRAM,Off,On;",
	"T[32],Save NVRAM;",
	"-;",
	"R0,Reset;",
	"J1,High Quick,High Fierce,Low Quick,Low Fierce,Coin,Start;",
	"V,v",`BUILD_DATE
};

// ioctl_index 4 is the game EEPROM NVRAM stream, used by hps_io upload below
localparam [7:0] NVRAM_INDEX = 8'd4;

wire forced_scandoubler;
wire [1:0] buttons;
wire [127:0] status;
wire [31:0] joystick_0, joystick_1;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_wait;

wire        nv_upload_req;
wire [10:0] nv_addr;
wire  [7:0] nv_din;
wire        nv_we;
wire  [7:0] nv_q;
wire        nv_wr_toggle;

wire [21:0] gamma_bus;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({status[5]}),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(),
	.ioctl_upload_req(nv_upload_req),
	.ioctl_upload_index(NVRAM_INDEX),
	.ioctl_din(nv_q),
	.ioctl_rd()
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire clk_cpu;
wire pll_locked;
wire [63:0] reconfig_to_pll = 64'd0;
wire [63:0] reconfig_from_pll;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_cpu),
	.locked(pll_locked),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// only ioctl_index 0 is ROM data; anything else (savestates etc) is
// not a ROM download and must not reset the CPU or trigger a prescan
wire rom_download = ioctl_download & (ioctl_index[7:0] == 8'd0);

// mod byte (MRA rom index 1, single byte at offset 0): 0 selects the
// primrageo input layout, 1 selects primrage. Power-up default 0; must
// survive the game reset and status[0], so no if(reset) clause here
reg set_jan = 1'b0;
always @(posedge clk_sys)
	if (ioctl_download && (ioctl_index[7:0] == 8'd1) && ioctl_wr && (ioctl_addr == 27'd0))
		set_jan <= ioctl_dout[0];

// a late NVRAM load must reset the CPU and CAGE the same way an OSD
// reset would, so the write lands cleanly
wire nvram_download = ioctl_download & (ioctl_index[7:0] == NVRAM_INDEX);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked | nvram_download;

// clk_sys/clk_cpu are related (clk_cpu = clk_sys/2, phase aligned), but
// these resets are generated combinationally, not off either clock, so
// still double-flop them into the clk_cpu domain before use there
reg reset_cpu_m, reset_cpu;
always @(posedge clk_cpu) {reset_cpu, reset_cpu_m} <= {reset_cpu_m, reset};

// the framework holds RESET while it uploads the ROM, so the loader and
// rle_objects (prescan) must not reset from it: PLL lock and a new download only
wire rle_reset = ~pll_locked | rom_download;
wire loader_reset = ~pll_locked;

// prescan_start fires once, on the falling edge of a ROM download
reg rom_download_d;
always @(posedge clk_sys) rom_download_d <= rom_download;
wire prescan_start = rom_download_d & ~rom_download;

wire prescan_done;
wire ram_clear_done;
wire cpu_reset = reset | rom_download | ~prescan_done | ~ram_clear_done;

reg cpu_reset_cpu_m, cpu_reset_cpu;
always @(posedge clk_cpu) {cpu_reset_cpu, cpu_reset_cpu_m} <= {cpu_reset_cpu_m, cpu_reset};

assign LED_USER = ioctl_download;

///////////////////////   250 Hz SCANLINE TICK   //////////////////
// runs on clk_cpu now, alongside cpu_bus: 42954545/250 clocks

wire scan_irq_tick;
scan_tick scan_tick
(
	.clk(clk_cpu),
	.reset(reset_cpu),
	.tick(scan_irq_tick)
);

///////////////////////   ROM LOADER   /////////////////////////////

wire [24:1] ld_sdram_addr;
wire [15:0] ld_sdram_din;
wire        ld_sdram_wrl, ld_sdram_wrh;
wire        ld_sdram_req;
wire        ld_sdram_ack;

wire [28:0] ld_ddr_addr;
wire [63:0] ld_ddr_din;
wire  [7:0] ld_ddr_be;
wire        ld_ddr_we;
wire        ld_ddr_busy;
wire        ld_ddr_pending;
wire        ld_dbg_active;
wire [15:0] cage_dsp_we_count;

rom_loader rom_loader
(
	.clk(clk_sys),
	.reset(loader_reset),
	.soft_reset(RESET | status[0] | buttons[1] | nvram_download),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.sdram_addr(ld_sdram_addr),
	.sdram_din(ld_sdram_din),
	.sdram_wrl(ld_sdram_wrl),
	.sdram_wrh(ld_sdram_wrh),
	.sdram_req(ld_sdram_req),
	.sdram_ack(ld_sdram_ack),

	.ddr_addr(ld_ddr_addr),
	.ddr_din(ld_ddr_din),
	.ddr_be(ld_ddr_be),
	.ddr_we(ld_ddr_we),
	.ddr_busy(ld_ddr_busy),
	.ddr_pending(ld_ddr_pending),
	.dbg_active(ld_dbg_active),
	.ram_clear_done(ram_clear_done),

	.RESET_in(RESET),
	.status0(status[0]),
	.buttons1(buttons[1]),
	.pll_locked(pll_locked),
	.prescan_done(prescan_done),
	.cpu_reset_in(cpu_reset),
	.loader_sel_in(loader_sel),
	.ddr_busy_in(DDRAM_BUSY),
	.dsp_we_count_in(cage_dsp_we_count)
);

///////////////////////   DDR3 ARBITER   ///////////////////////////

wire [28:0] rle_ddr_addr;
wire        rle_ddr_rd;
wire        rle_ddr_wr;
wire  [7:0] rle_ddr_burstcnt;
wire [63:0] rle_ddr_din;
wire  [7:0] rle_ddr_be;
wire [63:0] rle_ddr_dout;
wire        rle_ddr_dout_ready;
wire        rle_ddr_busy;

// keep the bus with the loader until its last write is actually accepted
// (even after ioctl_download has already dropped), and while the
// periodic debug block is writing its qwords
wire loader_sel = rom_download | ld_ddr_pending | ld_dbg_active;

wire [28:0] cage_ddr_addr;
wire        cage_ddr_rd, cage_ddr_we;
wire  [7:0] cage_ddr_burstcnt;
wire [63:0] cage_ddr_din;
wire  [7:0] cage_ddr_be;
wire [63:0] cage_ddr_dout;
wire        cage_ddr_dout_ready;
wire        cage_ddr_busy;

ddr_arbiter ddr_arbiter
(
	.clk(clk_sys),
	.reset(~pll_locked),

	.loader_sel(loader_sel),

	.loader_addr(ld_ddr_addr),
	.loader_din(ld_ddr_din),
	.loader_be(ld_ddr_be),
	.loader_we(ld_ddr_we),
	.loader_busy(ld_ddr_busy),

	.rle_addr(rle_ddr_addr),
	.rle_rd(rle_ddr_rd),
	.rle_wr(rle_ddr_wr),
	.rle_burstcnt(rle_ddr_burstcnt),
	.rle_din(rle_ddr_din),
	.rle_be(rle_ddr_be),
	.rle_dout(rle_ddr_dout),
	.rle_dout_ready(rle_ddr_dout_ready),
	.rle_busy(rle_ddr_busy),

	.cage_addr(cage_ddr_addr),
	.cage_rd(cage_ddr_rd),
	.cage_we(cage_ddr_we),
	.cage_burstcnt(cage_ddr_burstcnt),
	.cage_din(cage_ddr_din),
	.cage_be(cage_ddr_be),
	.cage_dout(cage_ddr_dout),
	.cage_dout_ready(cage_ddr_dout_ready),
	.cage_busy(cage_ddr_busy),

	.DDRAM_CLK(DDRAM_CLK),
	.DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN),
	.DDRAM_BE(DDRAM_BE),
	.DDRAM_WE(DDRAM_WE)
);

///////////////////////   SDRAM   //////////////////////////////////

wire [24:1] cpu_sdram_addr;
wire [15:0] cpu_sdram_din, cpu_sdram_dout;
wire        cpu_sdram_wrl, cpu_sdram_wrh;
wire        cpu_sdram_req, cpu_sdram_ack;

wire [24:1] video_rom_addr;
wire [15:0] video_rom_dout;
wire        video_rom_req, video_rom_ack;

sdram sdram
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),

	.addr0(cpu_sdram_addr),
	.wrl0(cpu_sdram_wrl),
	.wrh0(cpu_sdram_wrh),
	.din0(cpu_sdram_din),
	.dout0(cpu_sdram_dout),
	.req0(cpu_sdram_req),
	.ack0(cpu_sdram_ack),

	.addr1(video_rom_addr),
	.wrl1(1'b0),
	.wrh1(1'b0),
	.din1(16'd0),
	.dout1(video_rom_dout),
	.req1(video_rom_req),
	.ack1(video_rom_ack),

	.addr2(ld_sdram_addr),
	.wrl2(ld_sdram_wrl),
	.wrh2(ld_sdram_wrh),
	.din2(ld_sdram_din),
	.dout2(),
	.req2(ld_sdram_req),
	.ack2(ld_sdram_ack)
);

///////////////////////   NVRAM   //////////////////////////////////

nvram_hps #(.INDEX(NVRAM_INDEX)) nvram_hps
(
	.clk(clk_sys),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.wr_toggle(nv_wr_toggle),
	.autosave(status[31]),
	.save_req(status[32]),

	.nv_addr(nv_addr),
	.nv_din(nv_din),
	.nv_we(nv_we),
	.upload_req(nv_upload_req)
);

///////////////////////   CORE   ///////////////////////////////////

wire [15:1] vram_addr;
wire [15:0] vram_din, vram_dout;
wire  [1:0] vram_we;

wire [11:1] oram_addr;
wire [15:0] oram_din, oram_dout;
wire  [1:0] oram_we;

wire [18:1] cram_addr;
wire [15:0] cram_din, cram_dout;
wire  [1:0] cram_we;
wire        cram_rd;

wire [18:1] xga_addr;
wire [15:0] xga_din;
wire        xga_we, xga_rd;
wire        xga_override;
wire        xga_busy;
wire [15:0] xga_dout;

wire [15:0] latch;
wire  [2:0] mo_control;
wire [15:0] mo_command;
wire        irq_scanline, irq_vblank;
wire        vblank, hblank, vsync, hsync, vblank_rise;
wire  [8:0] hcnt, vcnt;
wire        ce_pix;
wire  [7:0] vid_r, vid_g, vid_b;

wire [31:0] p1p2;
wire [15:0] service;
wire [15:0] coin;

cpu_bus cpu_bus
(
	.clk_cpu(clk_cpu),
	.reset(reset_cpu),
	.cpu_reset(cpu_reset_cpu),

	.sdram_addr(cpu_sdram_addr),
	.sdram_din(cpu_sdram_din),
	.sdram_wrl(cpu_sdram_wrl),
	.sdram_wrh(cpu_sdram_wrh),
	.sdram_req(cpu_sdram_req),
	.sdram_ack(cpu_sdram_ack),
	.sdram_dout(cpu_sdram_dout),

	.vram_addr(vram_addr),
	.vram_din(vram_din),
	.vram_we(vram_we),
	.vram_dout(vram_dout),

	.oram_addr(oram_addr),
	.oram_din(oram_din),
	.oram_we(oram_we),
	.oram_dout(oram_dout),

	.cram_addr(cram_addr),
	.cram_din(cram_din),
	.cram_we(cram_we),
	.cram_rd(cram_rd),
	.cram_dout(cram_dout),

	.xga_addr(xga_addr),
	.xga_din(xga_din),
	.xga_we(xga_we),
	.xga_rd(xga_rd),
	.xga_override(xga_override),
	.xga_dout(xga_dout),

	.latch(latch),
	.mo_control(mo_control),
	.mo_command(mo_command),

	.scan_tick(scan_irq_tick),
	.irq_scanline(irq_scanline),
	.irq_vblank(irq_vblank),

	.p1p2(p1p2),
	.service(service),
	.coin(coin),
	.vblank(vblank),
	.xga_busy(xga_busy),

	.clk_nv(clk_sys),
	.nv_addr(nv_addr),
	.nv_din(nv_din),
	.nv_we(nv_we),
	.nv_q(nv_q),
	.nv_wr_toggle(nv_wr_toggle),

	.cage_data_wr(cage_data_wr),
	.cage_ctrl_wr(cage_ctrl_wr),
	.cage_data_rd(cage_data_rd),
	.cage_wdata(cage_wdata),
	.cage_data_rdata(cage_data_rdata),
	.cage_ctrl_rdata(cage_ctrl_rdata),
	.cage_data_rd_ack(cage_data_rd_ack),
	.irq3(cage_irq3)
);

wire        cage_data_wr, cage_ctrl_wr, cage_data_rd;
wire [15:0] cage_wdata;
wire [15:0] cage_data_rdata, cage_ctrl_rdata;
wire        cage_data_rd_ack;
wire        cage_irq3;

cage cage
(
	.clk_sys(clk_sys),
	.reset(reset),
	.rom_download(rom_download),
	.prescan_done(prescan_done),

	.clk_cpu(clk_cpu),
	.cage_data_wr(cage_data_wr),
	.cage_ctrl_wr(cage_ctrl_wr),
	.cage_data_rd(cage_data_rd),
	.cage_wdata(cage_wdata),
	.cage_data_rdata(cage_data_rdata),
	.cage_ctrl_rdata(cage_ctrl_rdata),
	.cage_data_rd_ack(cage_data_rd_ack),
	.dsp_we_count(cage_dsp_we_count),
	.irq3(cage_irq3),

	.ddr_addr(cage_ddr_addr),
	.ddr_rd(cage_ddr_rd),
	.ddr_we(cage_ddr_we),
	.ddr_burstcnt(cage_ddr_burstcnt),
	.ddr_din(cage_ddr_din),
	.ddr_be(cage_ddr_be),
	.ddr_dout(cage_ddr_dout),
	.ddr_dout_ready(cage_ddr_dout_ready),
	.ddr_busy(cage_ddr_busy),

	.audio_l(cage_audio_l),
	.audio_r(cage_audio_r)
);

///////////////////////   AUDIO   //////////////////////////////////

// arithmetic shift only attenuates (0/-6/-12/-18 dB), cannot clip
reg signed [15:0] aud_l, aud_r;
always @(posedge clk_sys) begin
	aud_l <= cage_audio_l >>> status[19:18];
	aud_r <= cage_audio_r >>> status[19:18];
end

xga_prot xga_prot
(
	.clk_cpu(clk_cpu),
	.reset(reset_cpu),

	.addr(xga_addr),
	.din(xga_din),
	.we(xga_we),
	.rd(xga_rd),

	.override(xga_override),
	.dout(xga_dout),
	.busy(xga_busy)
);

wire  [8:0] fb_rd_x;
wire  [7:0] fb_rd_y;
wire [15:0] fb_rd_data;

atarigt_video atarigt_video
(
	.clk(clk_sys),
	.clk_cpu(clk_cpu),
	.reset(~pll_locked), // video timing keeps running through the OSD reset and the ROM upload, so the framework can draw the loading bar

	.ce_pix(ce_pix),
	.hblank(hblank),
	.vblank(vblank),
	.hsync(hsync),
	.vsync(vsync),
	.hcnt(hcnt),
	.vcnt(vcnt),
	.vblank_rise(vblank_rise),

	.vram_addr(vram_addr),
	.vram_din(vram_din),
	.vram_we(vram_we),
	.vram_dout(vram_dout),

	.cram_addr(cram_addr),
	.cram_din(cram_din),
	.cram_we(cram_we),
	.cram_dout(cram_dout),

	.rom_addr(video_rom_addr),
	.rom_req(video_rom_req),
	.rom_ack(video_rom_ack),
	.rom_dout(video_rom_dout),

	.fb_rd_x(fb_rd_x),
	.fb_rd_y(fb_rd_y),
	.fb_rd_data(fb_rd_data),
	.frame_sel(mo_control[2]),

	.latch(latch),

	.r(vid_r),
	.g(vid_g),
	.b(vid_b)
);

wire [7:0] mo_next_line = (vcnt >= 9'd261) ? 8'd0 : vcnt[7:0] + 8'd1;

rle_objects rle_objects
(
	.clk(clk_sys),
	.clk_cpu(clk_cpu),
	.reset(rle_reset),

	// the mo line prefetch runs in hblank for the line the mixer reads next
	.hblank(hblank),
	.vblank(vblank),
	.next_line(mo_next_line),

	.control(mo_control),
	.mo_command(mo_command),
	.vblank_rise(vblank_rise),

	.oram_addr(oram_addr),
	.oram_din(oram_din),
	.oram_dout(oram_dout),
	.oram_we(oram_we),

	.fb_rd_x(fb_rd_x),
	.fb_rd_y(fb_rd_y),
	.fb_rd_data(fb_rd_data),
	.fb_rd_frame(mo_control[2]),

	.ddr_addr(rle_ddr_addr),
	.ddr_rd(rle_ddr_rd),
	.ddr_we(rle_ddr_wr),
	.ddr_burstcnt(rle_ddr_burstcnt),
	.ddr_din(rle_ddr_din),
	.ddr_be(rle_ddr_be),
	.ddr_dout(rle_ddr_dout),
	.ddr_dout_ready(rle_ddr_dout_ready),
	.ddr_busy(rle_ddr_busy),

	.prescan_start(prescan_start),
	.prescan_done(prescan_done)
);

///////////////////////   INPUTS   ///////////////////////////////

// joystick_x bits: 0 right,1 left,2 down,3 up,4..7 buttons1-4,8 coin,9 start
// set_jan picks the primrage (1) or primrageo (0) button/start layout;
// see rtl/atarigt_inputs.sv for the MAME atarigt.cpp port mapping.
atarigt_inputs inputs
(
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.set_jan(set_jan),
	.p1p2(p1p2),
	.coin(coin)
);

// bit7 = VBLANK (active high), bit6 = SELFTEST (active low, status[7] set = test menu)
assign service = {8'hff, vblank, ~status[7], 6'h3f};

///////////////////////   VIDEO   /////////////////////////////////

// H-Pos and V-Pos are indices, not signed values: idx0 is 0, then the
// positive labels count up, then the negative labels count up from the
// most negative; out-of-range idx (unused field values) decodes to 0
function automatic signed [5:0] h_pos_decode(input [5:0] idx);
	case (idx)
		6'd0:  h_pos_decode = 6'sd0;
		6'd1:  h_pos_decode = 6'sd1;
		6'd2:  h_pos_decode = 6'sd2;
		6'd3:  h_pos_decode = 6'sd3;
		6'd4:  h_pos_decode = 6'sd4;
		6'd5:  h_pos_decode = 6'sd5;
		6'd6:  h_pos_decode = 6'sd6;
		6'd7:  h_pos_decode = 6'sd7;
		6'd8:  h_pos_decode = 6'sd8;
		6'd9:  h_pos_decode = 6'sd9;
		6'd10: h_pos_decode = 6'sd10;
		6'd11: h_pos_decode = 6'sd11;
		6'd12: h_pos_decode = 6'sd12;
		6'd13: h_pos_decode = 6'sd13;
		6'd14: h_pos_decode = 6'sd14;
		6'd15: h_pos_decode = 6'sd15;
		6'd16: h_pos_decode = 6'sd16;
		6'd17: h_pos_decode = 6'sb110000; // -16, avoid constant overflow warning
		6'd18: h_pos_decode = -6'sd15;
		6'd19: h_pos_decode = -6'sd14;
		6'd20: h_pos_decode = -6'sd13;
		6'd21: h_pos_decode = -6'sd12;
		6'd22: h_pos_decode = -6'sd11;
		6'd23: h_pos_decode = -6'sd10;
		6'd24: h_pos_decode = -6'sd9;
		6'd25: h_pos_decode = -6'sd8;
		6'd26: h_pos_decode = -6'sd7;
		6'd27: h_pos_decode = -6'sd6;
		6'd28: h_pos_decode = -6'sd5;
		6'd29: h_pos_decode = -6'sd4;
		6'd30: h_pos_decode = -6'sd3;
		6'd31: h_pos_decode = -6'sd2;
		6'd32: h_pos_decode = -6'sd1;
		default: h_pos_decode = 6'sd0;
	endcase
endfunction

// same vsync also starts a frame for the HDMI scaler (sys/ascal.vhd): for
// voff>=2 sync_shift also blanks the trailing voff-1 active lines, so the
// shifted vsync still falls inside vblank_o instead of rolling the picture
function automatic signed [4:0] v_pos_decode(input [4:0] idx);
	case (idx)
		5'd0: v_pos_decode = 5'sd0;
		5'd1: v_pos_decode = 5'sd1;
		5'd2: v_pos_decode = 5'sd2;
		5'd3: v_pos_decode = 5'sd3;
		5'd4: v_pos_decode = 5'sd4;
		5'd5: v_pos_decode = 5'sd5;
		5'd6: v_pos_decode = 5'sd6;
		5'd7: v_pos_decode = 5'sd7;
		5'd8: v_pos_decode = 5'sd8;
		5'd9: v_pos_decode = -5'sd8;
		5'd10: v_pos_decode = -5'sd7;
		5'd11: v_pos_decode = -5'sd6;
		5'd12: v_pos_decode = -5'sd5;
		5'd13: v_pos_decode = -5'sd4;
		5'd14: v_pos_decode = -5'sd3;
		5'd15: v_pos_decode = -5'sd2;
		5'd16: v_pos_decode = -5'sd1;
		default: v_pos_decode = 5'sd0;
	endcase
endfunction

wire signed [5:0] hoff = h_pos_decode(status[25:20]);
wire signed [4:0] voff = v_pos_decode(status[30:26]);
wire hsync_sh, vsync_sh, vblank_sh;

sync_shift u_sync_shift
(
	.clk(clk_sys),
	.ce_pix(ce_pix),
	.hblank(hblank),
	.vblank(vblank),
	.hsync(hsync),
	.vsync(vsync),
	.hoff(hoff),
	.voff(voff),
	.hsync_o(hsync_sh),
	.vsync_o(vsync_sh),
	.vblank_o(vblank_sh)
);

wire vga_de_mixer;

arcade_video #(.WIDTH(336), .DW(24)) arcade_video
(
	.clk_video(clk_sys),
	.ce_pix(ce_pix),

	.RGB_in({vid_r, vid_g, vid_b}),
	.HBlank(hblank),
	.VBlank(vblank_sh),
	.HSync(hsync_sh),
	.VSync(vsync_sh),

	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_HS(VGA_HS),
	.VGA_VS(VGA_VS),
	.VGA_DE(vga_de_mixer),
	.VGA_SL(VGA_SL),

	.fx(status[5:3]),
	.forced_scandoubler(forced_scandoubler),
	.gamma_bus(gamma_bus)
);

video_freak video_freak
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_VS(VGA_VS),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(vga_de_mixer),
	.ARX((!ar) ? 12'd4 : 12'(ar - 1'd1)),
	.ARY((!ar) ? 12'd3 : 12'd0),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE({1'b0, status[9:8]})
);

endmodule


// ---------------------------------------------------------------------
// rom_loader: ioctl byte stream -> SDRAM port2 words / DDR3 qwords.
// Every ioctl_wr is accepted unconditionally into a 256-entry FIFO
// ({addr,data}); ioctl_wait rises once the FIFO holds 32+ entries, giving
// slack against HPS bursts that do not react to wait immediately, plus a
// margin against a debug beat that was already in flight when a download
// began (see dbg_hold below). The FIFO drains into the same SDRAM/DDR
// engines as before, one pending write per engine, held until not busy
// per the Avalon waitrequest convention (see mdp_audio.sv DDR_REQ). At
// download end, once the FIFO has drained and any half-word/partial-group
// is flushed, the loader moves on to whatever the debug block below wants next.
//
// Debug block: does not depend on download_end (that trigger was lost
// on the board), so it fires periodically (every 2^25 clk_sys cycles)
// and also once on a download-end edge, writing six qwords to DDR3
// 0x33000000. Kept as board diagnostics (devmem), six DDR3 writes per
// ~0.4 s. q4 bits [31:16] of its low word carry the DSP host-latch
// write counter.
// ---------------------------------------------------------------------
module rom_loader
(
	input             clk,
	input             reset,
	input             soft_reset,

	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	output            ioctl_wait,

	output reg [24:1] sdram_addr,
	output reg [15:0] sdram_din,
	output reg        sdram_wrl,
	output reg        sdram_wrh,
	output reg        sdram_req,
	input             sdram_ack,

	output reg [28:0] ddr_addr,
	output reg [63:0] ddr_din,
	output reg  [7:0] ddr_be,
	output reg        ddr_we,
	input             ddr_busy,
	output            ddr_pending,
	output            dbg_active,
	output            ram_clear_done,

	// brought in just for the debug status word (q4)
	input             RESET_in,
	input             status0,
	input             buttons1,
	input             pll_locked,
	input             prescan_done,
	input             cpu_reset_in,
	input             loader_sel_in,
	input             ddr_busy_in,
	input      [15:0] dsp_we_count_in
);

// declared here, ahead of do_pop's use below, so a plain iverilog -g2012
// elaboration of this module does not need a forward reference
reg pend_sdram, pend_ddr;

localparam PROGRAM_END = 27'h200000;
localparam TILES_END   = 27'h500000;
localparam ALPHA_END   = 27'h520000;
// object ROM, then C31 boot ROM, then sound ROM, back to back in the stream
localparam OBJECT_END  = 27'h2520000; // 0x520000 + 0x2000000
localparam BOOT_END    = 27'h25a0000; // + 0x80000 boot ROM
localparam SOUND_END   = 27'h29a0000; // + 0x400000 sound ROM
localparam DEBUG_ADDR  = 29'h06600000;

wire index0        = (ioctl_index[7:0] == 8'd0);
wire download      = ioctl_download & index0;

function automatic [23:0] sdram_word_addr(input [26:0] a);
	if (a < PROGRAM_END)
		sdram_word_addr = a[24:1];
	else if (a < TILES_END)
		sdram_word_addr = 24'(27'h200000 + ((a - PROGRAM_END) >> 1));
	else
		sdram_word_addr = 24'(27'h380000 + ((a - TILES_END) >> 1));
endfunction

// object ROM -> DDR3 0x30000000, boot ROM -> 0x35100000, sound ROM -> 0x36000000
function automatic [28:0] ddr_qword_addr(input [26:0] a);
	if (a < OBJECT_END)
		ddr_qword_addr = 29'h06000000 + ((29'(a) - 29'(ALPHA_END)) >> 3);
	else if (a < BOOT_END)
		ddr_qword_addr = 29'h06a20000 + ((29'(a) - 29'(OBJECT_END)) >> 3);
	else
		ddr_qword_addr = 29'h06c00000 + ((29'(a) - 29'(BOOT_END)) >> 3);
endfunction

// ---- input FIFO: never drop a byte handed to us by hps_io ----
// 256 entries for headroom against a DDR3-busy stall of the debug block;
// the read is registered (pop_addr_r/pop_data_r), one cycle behind fifo_rptr,
// so Quartus maps fifo_mem to M10K/MLAB instead of ~9000 flip-flops
localparam FIFO_DEPTH = 256;
reg [34:0] fifo_mem [0:FIFO_DEPTH-1]; // {ioctl_addr[26:0], ioctl_dout[7:0]}
reg  [7:0] fifo_wptr, fifo_rptr;
reg  [8:0] fifo_count;

wire fifo_full  = (fifo_count == FIFO_DEPTH);
wire fifo_empty = (fifo_count == 0);
assign ioctl_wait = (fifo_count >= 9'd32); // the HPS sends more than 16 bytes after wait rises (board saw overflow_cnt=1), keep 32 spare

wire do_push = download & ioctl_wr & ~fifo_full;

reg        pop_pending; // one registered fifo_mem read in flight, not yet applied
reg [26:0] pop_addr_r;
reg  [7:0] pop_data_r;
wire       pop_object_region_r = pop_addr_r >= ALPHA_END;

// no ~dbg_active term: a debug beat parked on dbg_hold must not stall pops for
// the whole download; ~pend_ddr alone keeps the ddr bus race-free
wire do_issue = ~fifo_empty & ~pend_sdram & ~pend_ddr & ~pop_pending;

reg        hi_valid;
reg  [7:0] hi_byte;
reg [26:0] hi_addr;
reg  [2:0] ddr_cnt;
reg [55:0] ddr_acc;
reg [26:0] ddr_group_addr; // stream address of byte 0 of the in-progress group

assign ddr_pending = pend_ddr;

reg download_d;
always @(posedge clk) download_d <= download;
wire download_end   = download_d & ~download;
wire download_start = download & ~download_d;
reg ended;
// the game tells a cold boot from a watchdog reset by what it finds in work
// RAM, so zero it after the download like MAME does at power-up
reg [17:0] clr_cnt;
reg        clr_done;
assign ram_clear_done = clr_done;

wire drain_complete = fifo_empty & ~pop_pending & ~pend_sdram & ~pend_ddr & ~hi_valid & (ddr_cnt == 0);

// the debug sequence may only start or step while the loader is otherwise
// idle: an index-0 download in progress, or anything still queued/in flight
// from one, defers it (board bug: it used to race an active download)
wire dbg_hold = download | ~drain_complete;

// ---- debug counters (never gated by 'ended'/drain, always live) ----
reg [31:0] wr_any_cnt, wr_idx0_cnt, overflow_cnt, ddr_wr_cnt, sdram_wr_cnt;
reg [31:0] dl_start_cnt, dl_end_cnt;
reg [63:0] first8_bytes;
reg  [3:0] first8_cnt;
reg [26:0] last_addr;

reg dl_raw_d;
always @(posedge clk) dl_raw_d <= ioctl_download;
wire dl_start_edge = ioctl_download & ~dl_raw_d;
wire dl_end_edge   = ~ioctl_download & dl_raw_d;

// free-running slow tick, bits[63:32] of q4, so a periodic re-write is
// visible even when nothing else in the block changes
reg [25:0] hb_div;
reg [31:0] heartbeat;

localparam DBG_IDLE=0, DBG_Q0=1, DBG_Q1=2, DBG_Q2=3, DBG_Q3=4, DBG_Q4=5, DBG_Q5=6;
reg [2:0] dbg_state;
assign dbg_active = (dbg_state != DBG_IDLE);

reg [24:0] per_cnt;
wire trig_periodic = (per_cnt == 25'h1ffffff);
wire dbg_trig = trig_periodic | dl_end_edge;
reg  dbg_pending; // sticky: set by dbg_trig, held until the arm cycle wins arbitration

wire [15:0] dbg_flags = {ioctl_wait, ioctl_download, (ioctl_index[7:0] == 8'd0), ddr_busy_in, loader_sel_in, dbg_active,
						 fifo_empty, pend_sdram, pend_ddr, (sdram_req == sdram_ack),
						 cpu_reset_in, prescan_done, pll_locked, buttons1, status0, RESET_in};

always @(posedge clk) begin
	if (reset) begin
		sdram_req    <= 0;
		sdram_wrl    <= 0;
		sdram_wrh    <= 0;
		ddr_we       <= 0;
		pend_sdram   <= 0;
		pend_ddr     <= 0;
		hi_valid     <= 0;
		ddr_cnt      <= 0;
		fifo_wptr    <= 0;
		fifo_rptr    <= 0;
		fifo_count   <= 0;
		pop_pending  <= 0;
		ended        <= 0;
		clr_cnt      <= 0;
		clr_done     <= 0;
		wr_any_cnt   <= 0;
		wr_idx0_cnt  <= 0;
		overflow_cnt <= 0;
		ddr_wr_cnt   <= 0;
		sdram_wr_cnt <= 0;
		dl_start_cnt <= 0;
		dl_end_cnt   <= 0;
		first8_cnt   <= 0;
		per_cnt      <= 0;
		hb_div       <= 0;
		heartbeat    <= 0;
		dbg_state    <= DBG_IDLE;
		dbg_pending  <= 0;
	end else begin
		per_cnt <= per_cnt + 1'd1;
		if (trig_periodic) per_cnt <= 0;

		hb_div <= hb_div + 1'd1;
		if (&hb_div) heartbeat <= heartbeat + 1'd1;

		if (dbg_trig) dbg_pending <= 1'b1;

		if (dl_start_edge) begin
			dl_start_cnt <= dl_start_cnt + 1'd1;
			first8_cnt   <= 0;
		end
		if (dl_end_edge) dl_end_cnt <= dl_end_cnt + 1'd1;

		if (pend_sdram && (sdram_ack == sdram_req)) begin
			sdram_wrl  <= 0;
			sdram_wrh  <= 0;
			pend_sdram <= 0;
		end

		// hold ddr_we/addr/din/be steady while busy; accept once low
		if (pend_ddr && !ddr_busy) begin
			ddr_we   <= 0;
			pend_ddr <= 0;
		end

		if (download_end) ended <= 1'b1;

		// ---- raw counters: every ioctl_wr, any index ----
		if (ioctl_wr) begin
			wr_any_cnt <= wr_any_cnt + 1'd1;
			last_addr  <= ioctl_addr;
			if (index0) wr_idx0_cnt <= wr_idx0_cnt + 1'd1;
			if (ioctl_download && first8_cnt < 4'd8) begin
				first8_bytes[8*first8_cnt +: 8] <= ioctl_dout;
				first8_cnt <= first8_cnt + 1'd1;
			end
		end

		// ---- push: never dropped, just counted as overflow if full ----
		if (download & ioctl_wr) begin
			if (fifo_full) begin
				overflow_cnt <= overflow_cnt + 1'd1;
			end else begin
				fifo_mem[fifo_wptr] <= {ioctl_addr, ioctl_dout};
				fifo_wptr <= fifo_wptr + 1'd1;
			end
		end

		fifo_count <= fifo_count + (do_push ? 9'd1 : 9'd0) - (do_issue ? 9'd1 : 9'd0);

		// ---- pop: registered fifo_mem read (do_issue), applied one cycle
		// later (pop_pending) so Quartus can map fifo_mem to block RAM ----
		if (pop_pending) begin
			if (!pop_object_region_r) begin
				if (!pop_addr_r[0]) begin
					hi_byte  <= pop_data_r;
					hi_addr  <= pop_addr_r;
					hi_valid <= 1'b1;
				end else begin
					sdram_din    <= {hi_byte, pop_data_r};
					sdram_wrl    <= 1'b1;
					sdram_wrh    <= 1'b1;
					sdram_addr   <= sdram_word_addr(pop_addr_r);
					sdram_req    <= ~sdram_req;
					pend_sdram   <= 1'b1;
					sdram_wr_cnt <= sdram_wr_cnt + 1'd1;
					hi_valid     <= 1'b0;
				end
			end else begin
				if (ddr_cnt == 3'd0) ddr_group_addr <= pop_addr_r;
				ddr_acc[8*pop_addr_r[2:0] +: 8] <= pop_data_r;
				ddr_cnt <= pop_addr_r[2:0] + 3'd1;
				if (pop_addr_r[2:0] == 3'd7) begin
					ddr_addr   <= ddr_qword_addr(ddr_cnt == 3'd0 ? pop_addr_r : ddr_group_addr);
					ddr_din    <= {pop_data_r, ddr_acc[55:0]};
					ddr_be     <= 8'hff;
					ddr_we     <= 1'b1;
					pend_ddr   <= 1'b1;
					ddr_wr_cnt <= ddr_wr_cnt + 1'd1;
					ddr_cnt    <= 0;
				end
			end
			pop_pending <= 1'b0;
		end else if (do_issue) begin
			fifo_rptr   <= fifo_rptr + 1'd1;
			pop_addr_r  <= fifo_mem[fifo_rptr][34:8];
			pop_data_r  <= fifo_mem[fifo_rptr][7:0];
			pop_pending <= 1'b1;
		end else if (ended && drain_complete && !dbg_active && (hi_valid || ddr_cnt != 0)) begin
			// flush a half-assembled SDRAM word (low byte never arrived)
			if (hi_valid) begin
				sdram_din    <= {hi_byte, 8'h00};
				sdram_wrl    <= 1'b0;
				sdram_wrh    <= 1'b1;
				sdram_addr   <= sdram_word_addr(hi_addr);
				sdram_req    <= ~sdram_req;
				pend_sdram   <= 1'b1;
				sdram_wr_cnt <= sdram_wr_cnt + 1'd1;
				hi_valid     <= 1'b0;
			end
			// flush a partial DDR3 group (fewer than 8 bytes seen); use the
			// group's own base address, not whatever a prior completed
			// group last left in ddr_addr
			else if (ddr_cnt != 0) begin
				ddr_addr   <= ddr_qword_addr(ddr_group_addr);
				ddr_din    <= {8'h00, ddr_acc};
				ddr_be     <= (8'h01 << ddr_cnt) - 8'h01;
				ddr_we     <= 1'b1;
				pend_ddr   <= 1'b1;
				ddr_wr_cnt <= ddr_wr_cnt + 1'd1;
				ddr_cnt    <= 0;
			end
		end else if (ended && drain_complete && !dbg_active && !clr_done) begin
			sdram_din  <= 16'h0000;
			sdram_wrl  <= 1'b1;
			sdram_wrh  <= 1'b1;
			sdram_addr <= 24'h100000 + {6'b0, clr_cnt};
			sdram_req  <= ~sdram_req;
			pend_sdram <= 1'b1;
			clr_cnt    <= clr_cnt + 1'd1;
			if (clr_cnt == 18'h3ffff) clr_done <= 1'b1;
		end else if (!pend_ddr && !dbg_hold && dbg_state == DBG_IDLE && dbg_pending) begin
			// arm only while the loader is idle: dbg_hold defers a trigger that
			// lands during a download, where it stalled the pops (board bug)
			ddr_addr    <= DEBUG_ADDR;
			ddr_din     <= {wr_any_cnt, wr_idx0_cnt};
			ddr_be      <= 8'hff;
			ddr_we      <= 1'b1;
			pend_ddr    <= 1'b1;
			dbg_state   <= DBG_Q0;
			dbg_pending <= 1'b0;
		end else if (!pend_ddr && !dbg_hold && dbg_state != DBG_IDLE) begin
			// dbg_hold keeps this from firing during/just after a download; a
			// pop can still interleave between beats (~pend_ddr keeps the ddr
			// bus race-free), the flush/clear paths above stay gated by dbg_active
			case (dbg_state)
				DBG_Q0: begin
					ddr_addr <= DEBUG_ADDR + 29'd1;
					ddr_din  <= {ddr_wr_cnt, sdram_wr_cnt};
					ddr_be   <= 8'hff;
					ddr_we   <= 1'b1;
					pend_ddr <= 1'b1;
					dbg_state <= DBG_Q1;
				end
				DBG_Q1: begin
					ddr_addr <= DEBUG_ADDR + 29'd2;
					ddr_din  <= {ioctl_download, ioctl_index[14:0],
								 overflow_cnt[15:0], 5'b0, last_addr};
					ddr_be   <= 8'hff;
					ddr_we   <= 1'b1;
					pend_ddr <= 1'b1;
					dbg_state <= DBG_Q2;
				end
				DBG_Q2: begin
					ddr_addr <= DEBUG_ADDR + 29'd3;
					ddr_din  <= {dl_start_cnt, dl_end_cnt};
					ddr_be   <= 8'hff;
					ddr_we   <= 1'b1;
					pend_ddr <= 1'b1;
					dbg_state <= DBG_Q3;
				end
				DBG_Q3: begin
					ddr_addr <= DEBUG_ADDR + 29'd4;
					ddr_din  <= {heartbeat, dsp_we_count_in, dbg_flags};
					ddr_be   <= 8'hff;
					ddr_we   <= 1'b1;
					pend_ddr <= 1'b1;
					dbg_state <= DBG_Q4;
				end
				DBG_Q4: begin
					ddr_addr <= DEBUG_ADDR + 29'd5;
					ddr_din  <= first8_bytes;
					ddr_be   <= 8'hff;
					ddr_we   <= 1'b1;
					pend_ddr <= 1'b1;
					dbg_state <= DBG_Q5;
				end
				DBG_Q5: dbg_state <= DBG_IDLE;
				default: dbg_state <= DBG_IDLE;
			endcase
		end

		// a fresh index-0 download must not inherit stale state; last in
		// program order so it wins over a pop of the previous tail this cycle
		if (download_start) begin
			fifo_wptr   <= 0;
			fifo_rptr   <= 0;
			fifo_count  <= 0;
			pop_pending <= 0;
			hi_valid    <= 0;
			ddr_cnt     <= 0;
			ended       <= 0;
		end

		// re-run the zero-fill on every reset: the game tells a cold boot from
		// a watchdog reset by work RAM contents, so a reset must look like power-on
		if (soft_reset && ended) begin
			clr_cnt  <= 0;
			clr_done <= 0;
		end
	end
end

endmodule


// ---------------------------------------------------------------------
// ddr_arbiter: priority mux, three masters, priority rle > cage > loader.
// The bus changes owner only when the current owner has no burst in
// flight (a per-owner outstanding-words counter), so a lower-priority
// write can never steal words out of a running read. loader is lowest
// priority and never bursts (one qword at a time); cage is not
// instantiated yet, its ports are tied off at the Arcade-PrimalRage.sv instance.
// ---------------------------------------------------------------------
module ddr_arbiter
(
	input clk,
	input reset,

	input        loader_sel,

	input [28:0] loader_addr,
	input [63:0] loader_din,
	input  [7:0] loader_be,
	input        loader_we,
	output       loader_busy,

	input [28:0] rle_addr,
	input        rle_rd,
	input        rle_wr,
	input  [7:0] rle_burstcnt,
	input [63:0] rle_din,
	input  [7:0] rle_be,
	output [63:0] rle_dout,
	output        rle_dout_ready,
	output        rle_busy,

	input [28:0] cage_addr,
	input        cage_rd,
	input        cage_we,
	input  [7:0] cage_burstcnt,
	input [63:0] cage_din,
	input  [7:0] cage_be,
	output [63:0] cage_dout,
	output        cage_dout_ready,
	output        cage_busy,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE
);

assign DDRAM_CLK = clk;

reg [7:0] rle_words_left, cage_words_left;
wire rle_active  = (rle_words_left  != 0);
wire cage_active = (cage_words_left != 0);

// priority rle > cage > loader, but an owner mid-burst keeps the bus
// regardless of who else wants it
wire grant_rle    = rle_active  | (~cage_active & (rle_rd | rle_wr));
wire grant_cage   = ~grant_rle  & (cage_active | cage_rd | cage_we);
wire grant_loader = ~grant_rle  & ~grant_cage & loader_sel;

wire rle_rd_acc  = grant_rle  & rle_rd  & ~rle_active  & ~DDRAM_BUSY;
wire cage_rd_acc = grant_cage & cage_rd & ~cage_active & ~DDRAM_BUSY;

always @(posedge clk) begin
	if (reset) begin
		rle_words_left  <= 0;
		cage_words_left <= 0;
	end else begin
		if (rle_rd_acc) rle_words_left <= rle_burstcnt;
		else if (DDRAM_DOUT_READY && rle_active) rle_words_left <= rle_words_left - 1'd1;

		if (cage_rd_acc) cage_words_left <= cage_burstcnt;
		else if (DDRAM_DOUT_READY && cage_active) cage_words_left <= cage_words_left - 1'd1;
	end
end

wire is_write = grant_rle ? rle_wr : grant_cage ? cage_we : loader_we;

assign DDRAM_ADDR     = grant_rle ? rle_addr : grant_cage ? cage_addr : loader_addr;
assign DDRAM_DIN      = grant_rle ? rle_din  : grant_cage ? cage_din  : loader_din;
assign DDRAM_WE       = is_write;
assign DDRAM_RD       = grant_rle ? rle_rd : grant_cage ? cage_rd : 1'b0;
// writes never burst (one qword at a time); BE only matters for writes
assign DDRAM_BURSTCNT = is_write ? 8'd1 : (grant_rle ? rle_burstcnt : cage_burstcnt);
assign DDRAM_BE       = is_write ? (grant_rle ? rle_be : grant_cage ? cage_be : loader_be) : 8'hff;

// busy to a non-granted master is 1, so it cannot mistake a wait for
// another master as its own request having been accepted
assign loader_busy    = DDRAM_BUSY | ~grant_loader;
assign rle_busy        = DDRAM_BUSY | ~grant_rle;
assign cage_busy        = DDRAM_BUSY | ~grant_cage;

// dout_ready routed to the owner only; grant_rle/grant_cage are mutually
// exclusive so at most one of these is ever high on a given cycle
assign rle_dout         = DDRAM_DOUT;
assign rle_dout_ready    = DDRAM_DOUT_READY & rle_active;
assign cage_dout        = DDRAM_DOUT;
assign cage_dout_ready   = DDRAM_DOUT_READY & cage_active;

endmodule


// ---------------------------------------------------------------------
// scan_tick: 250 Hz pulse generator, clk_cpu / (42954545/250)
// ---------------------------------------------------------------------
module scan_tick
(
	input  clk,
	input  reset,
	output reg tick
);

localparam DIV = 171818;

reg [18:0] cnt;

always @(posedge clk) begin
	tick <= 1'b0;
	if (reset) begin
		cnt <= 0;
	end else if (cnt == DIV-1) begin
		cnt  <= 0;
		tick <= 1'b1;
	end else begin
		cnt <= cnt + 1'd1;
	end
end

endmodule
