//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  reg     [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  reg             video_de,
output  reg             video_skip,
output  reg             video_vs,
output  reg             video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:28] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

wire    [31:0]  cont1_key_s;
wire    [31:0]  cont2_key_s;
wire    [31:0]  cont1_joy_s;
wire    [31:0]  cont2_joy_s;
wire    [31:0]  cont3_joy_s;
wire    [31:0]  cont4_joy_s;

synch_3 #(.WIDTH(32)) controller_key1_sync(cont1_key, cont1_key_s, clk_sys);
synch_3 #(.WIDTH(32)) controller_key2_sync(cont2_key, cont2_key_s, clk_sys);
synch_3 #(.WIDTH(32)) controller_joy1_sync(cont1_joy, cont1_joy_s, clk_sys);
synch_3 #(.WIDTH(32)) controller_joy2_sync(cont2_joy, cont2_joy_s, clk_sys);
synch_3 #(.WIDTH(32)) controller_joy3_sync(cont3_joy, cont3_joy_s, clk_sys);
synch_3 #(.WIDTH(32)) controller_joy4_sync(cont4_joy, cont4_joy_s, clk_sys);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;            // these pins are not used, make them inputs
 assign cart_tran_bank3_dir = 1'b0;
 
 assign cart_tran_bank2 = 8'hzz;            // these pins are not used, make them inputs
 assign cart_tran_bank2_dir = 1'b0;
 assign cart_tran_bank1 = 8'hzz;            // these pins are not used, make them inputs
 assign cart_tran_bank1_dir = 1'b0;
 
 assign cart_tran_bank0 = {1'b0, TXDATA, LED, 1'b0};    // LED and TXD hook up here
 assign cart_tran_bank0_dir = 1'b1;
 
 assign cart_tran_pin30 = 1'bz;            // this pin is not used, make it an input
 assign cart_tran_pin30_dir = 1'b0;
 assign cart_pin30_pwroff_reset = 1'b1;    
 
 assign cart_tran_pin31 = 1'bz;            // this pin is an input
 assign cart_tran_pin31_dir = 1'b0;        // input
 // UART
 wire TXDATA;                        // your UART transmit data hooks up here
 wire RXDATA = cart_tran_pin31;        // your UART RX data shows up here
 
 // button/LED
 wire LED;                    // LED hooks up here.  HIGH = light up, LOW = off
 wire BUTTON = cart_tran_bank3[0];    // button data comes out here.  LOW = pressed, HIGH = unpressed

// link port is unused, set to input only to be safe
// each bit may be bidirectional in some applications
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;

wire [31:0] fpga_bridge_rd_data;
wire [31:0] mpu_reg_bridge_rd_data; 
wire [31:0] mpu_ram_bridge_rd_data;
reg  [31:0] vga_bridge_rd_data;

wire clk_mpu;


// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        // all unmapped addresses are zero
        bridge_rd_data <= 0;
    end
	 32'h10xxxxxx: begin
        bridge_rd_data <= fpga_bridge_rd_data;
    end
	 32'h200000xx: begin
        bridge_rd_data <= vga_bridge_rd_data;
    end
	 32'h8000xxxx: begin
        bridge_rd_data <= mpu_ram_bridge_rd_data;
    end
	 32'h810000xx: begin
        bridge_rd_data <= mpu_reg_bridge_rd_data;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end

//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
	 wire					pll_core_locked;
    wire            status_boot_done = pll_core_locked; 
    wire            status_setup_done = pll_core_locked; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire    [31:0]  dataslot_requestwrite_size;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_update;
    wire    [15:0]  dataslot_update_id;
    wire    [31:0]  dataslot_update_size;
    
    wire            dataslot_allcomplete;

    wire     [31:0] rtc_epoch_seconds;
    wire     [31:0] rtc_date_bcd;
    wire     [31:0] rtc_time_bcd;
    wire            rtc_valid;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a

    wire             target_dataslot_read;       
    wire             target_dataslot_write;

    wire            target_dataslot_ack;        
    wire            target_dataslot_done;
    wire    [2:0]   target_dataslot_err;

    wire     [15:0]  target_dataslot_id;
    wire     [31:0]  target_dataslot_slotoffset;
    wire     [31:0]  target_dataslot_bridgeaddr;
    wire     [31:0]  target_dataslot_length;
    
// bridge data slot access
// synchronous to clk_74a

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire            datatable_rden;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                    ( clk_74a ),
    .reset_n                ( reset_n ),
	 .clk_sys					 ( clk_mpu ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_update            ( dataslot_update ),
    .dataslot_update_id         ( dataslot_update_id ),
    .dataslot_update_size       ( dataslot_update_size ),
    
    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .rtc_epoch_seconds      ( rtc_epoch_seconds ),
    .rtc_date_bcd           ( rtc_date_bcd ),
    .rtc_time_bcd           ( rtc_time_bcd ),
    .rtc_valid              ( rtc_valid ),
    
    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .target_dataslot_read       ( target_dataslot_read ),
    .target_dataslot_write      ( target_dataslot_write ),

    .target_dataslot_ack        ( target_dataslot_ack ),
    .target_dataslot_done       ( target_dataslot_done ),
    .target_dataslot_err        ( target_dataslot_err ),

    .target_dataslot_id         ( target_dataslot_id ),
    .target_dataslot_slotoffset ( target_dataslot_slotoffset ),
    .target_dataslot_bridgeaddr ( target_dataslot_bridgeaddr ),
    .target_dataslot_length     ( target_dataslot_length ),

    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_rden         ( datatable_rden ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q )

);



////////////////////////////////////////////////////////////////////////////////////////

    
    reg [9:0]   work_x;
    reg [9:0]   work_y;
    reg [9:0]   target_x;
    reg [9:0]   target_y;
    reg         fifo_cleared;
    reg         reset_n_last;
    reg [3:0]   bootup_clearing;
    
	 wire [15:0]	ldata, rdata;

	wire 			clk_114;
	wire 			clk_sys;
	wire        cpu_rst;
	
	// video
	
	wire			hblank_i;
	wire 			vblank_i;
	wire			hs;
	wire 			vs;
	wire [2:0]	res;
	wire 			lace;
	wire 			field1;
	wire [7:0]	r;
	wire [7:0]	g;
	wire [7:0]	b;

	wire [15:0] JOY0  =  {cont1_key_s[14], cont1_key_s[9], cont1_key_s[8], cont1_key_s[7], cont1_key_s[6], 
								 cont1_key_s[5],  cont1_key_s[4], cont1_key_s[0], cont1_key_s[1], cont1_key_s[2], cont1_key_s[3]};
	// joystick 2 [fire4,fire3,fire2,fire,up,down,left,right] (default joystick port)
	wire [15:0] JOY1 = 	{cont2_key_s[14], cont2_key_s[9], cont2_key_s[8], cont2_key_s[7], cont2_key_s[6], 
								 cont2_key_s[5],  cont2_key_s[4], cont2_key_s[0], cont2_key_s[1], cont2_key_s[2], cont2_key_s[3]};
	wire [15:0] JOY2;// = {16{joystick_enable[2]}} & {cont1_key[7], cont1_key[6], cont1_key[4], cont1_key[5], cont1_key[0], cont1_key[1], cont1_key[2], cont1_key[3]};
	wire [15:0] JOY3;// = {16{joystick_enable[3]}} & {cont1_key[7], cont1_key[6], cont1_key[4], cont1_key[5], cont1_key[0], cont1_key[1], cont1_key[2], cont1_key[3]};
	wire [15:0] JOYA0;
	wire [15:0] JOYA1;

	wire        io_strobe;
	wire        io_wait;
	wire        io_fpga;
	wire        io_uio;
	wire [15:0] io_din, io_dout;
	wire [15:0] fpga_dout;
	
	
pll pll
(
	.refclk(clk_74a),
	.outclk_0(clk_114),
	.outclk_1(clk_sys),
	.outclk_2(video_rgb_clock),
	.outclk_3(video_rgb_clock_90),
	.locked(pll_core_locked)
);



wire reset_mpu_l;
substitute_mcu_apf_mister substitute_mcu_apf_mister(
		// Controls for the MPU
		.clk_mpu								( clk_74a ), 							// Clock of the MPU itself
		.clk_sys								( clk_sys ),
		.clk_74a								( clk_74a ),							// Clock of the APF Bus
		.reset_n								( reset_n ),							// Reset from the APF System
		.reset_out							( reset_mpu_l ),						// Able to restart the core from the MPU if required
		
		// APF Bus controll
		.bridge_addr            		( bridge_addr ),
		.bridge_rd              		( bridge_rd ),
		.mpu_reg_bridge_rd_data       ( mpu_reg_bridge_rd_data ),		// Used for interactions
		.mpu_ram_bridge_rd_data       ( mpu_ram_bridge_rd_data ),		// Used for ram up/download
		.bridge_wr              		( bridge_wr ),
		.bridge_wr_data         		( bridge_wr_data ),
	  
	   // Debugging to the Cart	
		.rxd									( RXDATA ),
		.txd									( TXDATA ),
		
		// APF Controller access if required
		
		.cont1_key          				( cont1_key ),
		.cont2_key          				( cont2_key ),
		.cont3_key          				( cont3_key ),
		.cont4_key          				( cont4_key ),
		.cont1_joy          				( cont1_joy ),
		.cont2_joy          				( cont2_joy ),
		.cont3_joy          				( cont3_joy ),
		.cont4_joy          				( cont4_joy ),
		.cont1_trig         				( cont1_trig ),
		.cont2_trig         				( cont2_trig ),
		.cont3_trig         				( cont3_trig ),
		.cont4_trig         				( cont4_trig ),
		
		// MPU Controlls to the APF
		
		.dataslot_update            	( dataslot_update ),
		.dataslot_update_id         	( dataslot_update_id ),
		.dataslot_update_size       	( dataslot_update_size ),
	  
		.target_dataslot_read       	( target_dataslot_read ),
		.target_dataslot_write      	( target_dataslot_write ),

		.target_dataslot_ack        	( target_dataslot_ack ),
		.target_dataslot_done       	( target_dataslot_done ),
		.target_dataslot_err        	( target_dataslot_err ),

		.target_dataslot_id         	( target_dataslot_id ),
		.target_dataslot_slotoffset 	( target_dataslot_slotoffset ),
		.target_dataslot_bridgeaddr 	( target_dataslot_bridgeaddr ),
		.target_dataslot_length     	( target_dataslot_length ),

		.datatable_addr         		( datatable_addr ),
		.datatable_wren         		( datatable_wren ),
		.datatable_rden         		( datatable_rden ),
		.datatable_data         		( datatable_data ),
		.datatable_q            		( datatable_q ),
		
		// Core interactions
		.IO_UIO       						( io_uio ),
		.IO_FPGA      						( io_fpga ),
		.IO_STROBE    						( io_strobe ),
		.IO_WAIT      						( io_wait ),
		.IO_DIN       						( io_dout ),
		.IO_DOUT      						( io_din ),
		.IO_WIDE								( 1'b1 )
	 
	 );

	 
	 
	 
	 
/***************************************************************************


	Video Core - THis does the decoding of the Vs and HS signals and the coding
	for the interlacing.
	
*****************************************************************************/


reg hs_reg, vs_reg, hblank_i_reg;
reg hs_delay0, hs_delay1, hs_delay2, hs_delay3;

// lets get some video offsets working

reg [7:0] x_offset = 0;
reg [7:0] y_offset = 0;
wire [7:0] x_offset_s;
wire [7:0] y_offset_s;
reg [31:0] vga_bridge_rd_data_reg;

always @(posedge clk_74a) begin
	if (bridge_wr && bridge_addr[31:8] == 24'h200000) begin
		case (bridge_addr[7:0])
			8'h00 : begin
				x_offset <= bridge_wr_data;
			end
			8'h04 : begin
				y_offset <= bridge_wr_data;
			end
		endcase
	end
end

always @(posedge clk_74a) begin
	if (bridge_rd) begin
		case (bridge_addr[7:0])
			8'h00 : begin
				vga_bridge_rd_data_reg <= x_offset;
			end
			8'h04 : begin
				vga_bridge_rd_data_reg <= y_offset;
			end
		endcase
	end
	vga_bridge_rd_data <= vga_bridge_rd_data_reg;
end

synch_3 #(.WIDTH(8)) x_offset_sync(x_offset, x_offset_s, video_rgb_clock);
synch_3 #(.WIDTH(8)) y_offset_sync(y_offset, y_offset_s, video_rgb_clock);

reg [7:0] x_offset_vga; // these are the counters for the offset when the DE or each HS wit DE happens
reg [7:0] y_offset_vga;
reg [1:0] res_reg;

reg [9:0] x_counter_vga;
reg [9:0] y_counter_vga;
reg 		 video_de_reg;

always @(posedge video_rgb_clock) begin
	video_rgb 	<= 'b0;
	video_de 	<= 'b0;
	video_skip 	<= 'b0;
	video_vs 	<= 'b0;
	hs_delay0 	<= 'b0;
	hblank_i_reg <= hblank_i;
	hs_delay1 <= hs_delay0;
	hs_delay2 <= hs_delay1;
	hs_delay3 <= hs_delay2;
	video_hs <= hs_delay3;
	video_de_reg <= video_de;
	if (video_de) x_counter_vga <= x_counter_vga + 1;
	if (video_de && ~video_de_reg) y_counter_vga <= y_counter_vga + 1;
	
	hs_reg <= hs;
	
	if (~hs && hs_reg)  begin
		hs_delay0 	<= 'b1;
		x_counter_vga <= 0;
		x_offset_vga <= x_offset_s;
		if (y_offset_vga != 0) y_offset_vga <= y_offset_vga - 1;
	end
	
	vs_reg <= vs;
	
	if (~vs && vs_reg) begin
		res_reg <= {res[1], res[0] && lace};
		y_offset_vga <= y_offset_s;
		y_counter_vga <= 0;
		video_vs 	<= 'b1;
		video_rgb 	<= {21'd0, 1'b0, field1 && lace, lace, 1'b0}; // This is the interlace part for the core.
	end
	
	
	
	else if (~hblank_i && ~vblank_i) begin
		video_rgb 	<= {r, g, b};
		if (y_offset_vga == 0 && x_offset_vga == 0 ) video_de 	<= 'b1;
		if (x_offset_vga != 0) x_offset_vga <= x_offset_vga - 1;
	end
	
	else if (hblank_i && ~hblank_i_reg) begin
		case (res_reg)
			3'd7		: video_rgb 	<= {10'd0, 3'h7, 13'd0};
			3'd6		: video_rgb 	<= {10'd0, 3'h6, 13'd0}; 
			3'd5		: video_rgb 	<= {10'd0, 3'h5, 13'd0};
			3'd4		: video_rgb 	<= {10'd0, 3'h4, 13'd0};
			3'd3		: video_rgb 	<= {10'd0, 3'h3, 13'd0}; 
			3'd2		: video_rgb 	<= {10'd0, 3'h2, 13'd0};
			3'd1		: video_rgb 	<= {10'd0, 3'h1, 13'd0};
			default  : video_rgb 	<= 24'h0;
		endcase
	end
	
end

i2s i2s (
.clk_74a			(clk_74a),
.left_audio		(ldata),
.right_audio	(rdata),

.audio_mclk		(audio_mclk),
.audio_dac		(audio_dac),
.audio_lrck		(audio_lrck)

);
    
endmodule
