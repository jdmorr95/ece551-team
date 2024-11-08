//////////////////////////////////////////////////////
// Interfaces with ST 6-axis inertial sensor.  In  //
// this application we only use Z-axis gyro for   //
// heading of robot.  Fusion correction comes    //
// from "gaurdrail" signals lftIR/rghtIR.       //
/////////////////////////////////////////////////
module inert_intf(clk,rst_n,strt_cal,cal_done,heading,rdy,lftIR,
                  rghtIR,SS_n,SCLK,MOSI,MISO,INT,moving);

  parameter FAST_SIM = 1;	// used to speed up simulation

  input clk, rst_n;
  input MISO;					// SPI input from inertial sensor
  input INT;					// goes high when measurement ready
  input strt_cal;				// initiate claibration of yaw readings
  input moving;					// Only integrate yaw when going
  input lftIR,rghtIR;			// gaurdrail sensors

  output cal_done;				// pulses high for 1 clock when calibration done
  output signed [11:0] heading;	// heading of robot.  000 = Orig dir 3FF = 90 CCW 7FF = 180 CCW
  output rdy;					// goes high for 1 clock when new outputs ready (from inertial_integrator)
  output SS_n,SCLK,MOSI;		// SPI outputs


  //////////////////////////////////
  // Declare any internal signal //
  ////////////////////////////////
    typedef enum logic [15:0]
    {
        CMD_ENABLE_INTERRUPTS = 16'h0D02,
        CMD_SETUP_GYRO = 16'h1160,
        CMD_ENABLE_ROUNDING = 16'h1440,
        CMD_READ_YAW_HIGH = 16'hA7xx,
        CMD_READ_YAW_LOW = 16'hA6xx
    } inertial_int_cmd_t;

    typedef enum logic [3:0] {
        INIT_ENABLE_INTERRUPTS,
        INIT_ENABLE_GYRO,
        INIT_ENABLE_ROUNDING,
        IDLE,
        READ_YAW_LOW,
        READ_YAW_HIGH
    } state_t;


    logic vld;		// vld yaw_rt provided to inertial_integrator
    logic [7:0] resp; // response from the iNEMO to the SPI_mnrch
    logic done; // asserted when the SPI transaction is complete
    logic enough_smpls; // asserted when we have obtained enough samples.
    state_t state;
    state_t next_state;
    logic [15:0] cmd; // the cmd to send to the NEMO.
    logic snd; // asserted when the cmd is ready to send.


SPI_mnrch SPI_mnrch(
    /****************************************/
    /*             serf controls            */
    /****************************************/

    .MOSI(MOSI), // serial output to the serf.
    .SS_n(SS_n), // active low serf select signal.
    .SCLK(SCLK), // 1/32nd of our clock signal @ 1.5625 Mhz

    /****************************************/
    /*             serf response            */
    /****************************************/

    .MISO(MISO), // serial input from the serf.

    /****************************************/
    /* output for the consumers of our data */
    /****************************************/

    .resp(resp), // data from SPI serf.
    .done(done), // asserted when SPI transaction is complete.

    /****************************************/
    /* input for the consumers of our data  */
    /****************************************/

    .cmd(cmd), // data sent to inertial sensor
    .snd(snd), // high for one clock period when cmd is sent.

    // clock and reset @ 50MHz
    .clk(clk),
    .rst_n(rst_n)
);

/////////////////////
// Double Flop INT //
/////////////////////
logic INT_q1;
logic INT_q2;
always_ff @(posedge clk, negedge rst_n) begin : INT_metastability_ff
    if (!rst_n) begin
        INT_q1 <= 1'b0;
        INT_q2 <= 1'b0;
    end else begin
        INT_q1 <= INT;
        INT_q2 <= INT_q1;
    end
end


///////////////////////////////////////////////
// 16-bit timer to wait for NEMO to be reset //
///////////////////////////////////////////////
reg [15:0] timer;
logic timer_done;
always_ff @(posedge clk, negedge rst_n) begin : sixteen_bit_counter
    if (!rst_n)
        timer <= '0;
    else
        timer <= (timer + 1);
end

assign timer_done = &timer;


///////////////////////////////////////////////////////////
// Registers to store the yaw data from SPI transactions //
///////////////////////////////////////////////////////////

logic C_Y_H;
logic C_Y_L;
reg [15:0] yaw_rt;

always_ff @ (posedge clk) begin : c_y_hl_ff
    if (C_Y_H)
        yaw_rt[15:8] <= resp;
    else if (C_Y_L)
        yaw_rt[7:0] <= resp;
end

/////////////////
// vld SR flop //
/////////////////
logic set_vld;
logic clr_vld;

always_ff @(posedge clk, negedge rst_n) begin : vld_sr_ff
    if (!rst_n)
        vld <= 1'b0;
    else if (set_vld)
        vld <= 1'b1;
    else if (clr_vld)
        vld = 1'b0;
end


///////////////////
// STATE MACHINE //
///////////////////
always_ff @(posedge clk, negedge rst_n) begin : next_state_ff
    if (!rst_n)
        state <= INIT_ENABLE_INTERRUPTS;
    else
        state <= next_state;
end

always_comb begin : next_state_logic

    /////////////////////////
    // default assignments //
    /////////////////////////
    next_state = state;
    C_Y_H = 1'b0;
    C_Y_L = 1'b0;
    snd = 1'b0;
    set_vld = 1'b0;


    case (state)

        /////////////////////////////////////////////////////////////////////////////////////////
        // These next three states send setup commands to enable certain settings in the NEMO. //
        /////////////////////////////////////////////////////////////////////////////////////////
        INIT_ENABLE_INTERRUPTS: begin
            cmd = CMD_ENABLE_INTERRUPTS;

            if (timer_done) begin // wait for the reset sequence of iNEMO to finish
                snd = 1'b1;
                next_state = INIT_ENABLE_GYRO;
            end
        end

        INIT_ENABLE_GYRO: begin
            cmd = CMD_SETUP_GYRO;

            if (done) begin
                snd = 1'b1;
                next_state = INIT_ENABLE_ROUNDING;
            end
        end

        INIT_ENABLE_ROUNDING: begin
            cmd = CMD_ENABLE_ROUNDING;

            if (done) begin
                snd = 1'b1;
                next_state = IDLE;
            end
        end


        ///////////////////////////////////////////////////////////////////////////////////
        // We are waiting for the iNEMO to assert an interrupt, indicating that new data //
        // is ready for us to integrate.                                                 //
        ///////////////////////////////////////////////////////////////////////////////////
        IDLE: begin
            cmd = CMD_READ_YAW_LOW;

            if (INT_q2 & done) begin // use the double-flopped signal here
                next_state = READ_YAW_LOW;
                snd = 1'b1;
                clr_vld = 1'b1;
            end
        end


        //////////////////////////////////////////////////////////////////////////////////////
        // In this state, we prepare the READ_YAW_HIGH command to be send when the SPI
        // transaction is done. When done is asserted, we store the data (low yaw)
        // and send the next command.
        //////////////////////////////////////////////////////////////////////////////////////
        READ_YAW_LOW: begin
            cmd = CMD_READ_YAW_HIGH;

            if (done) begin
                C_Y_L = 1'b1;
                snd = 1'b1;
                next_state = READ_YAW_HIGH;
            end

        end

        /////////////////////////////////////////////////////////////////////////////////////
        // Once we receive the high yaw data (done is asserted), we assert vld and go IDLE //
        /////////////////////////////////////////////////////////////////////////////////////
        READ_YAW_HIGH: begin
            if (done) begin
                C_Y_H = 1'b1;
                set_vld = 1'b1;
                next_state = IDLE;  // repeat forever. these SMs need to unionize for better
                                    // working conditions.
            end
        end

        default: next_state = INIT_ENABLE_INTERRUPTS; // how did we get here?
    endcase
end


  ////////////////////////////////////////////////////////////////////
  // Instantiate Angle Engine that takes in angular rate readings  //
  // and acceleration info and produces a heading reading         //
  /////////////////////////////////////////////////////////////////
  inertial_integrator #(FAST_SIM) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),.vld(vld),
                           .rdy(rdy),.cal_done(cal_done), .yaw_rt(yaw_rt),.moving(moving),.lftIR(lftIR),
                           .rghtIR(rghtIR),.heading(heading));
endmodule
