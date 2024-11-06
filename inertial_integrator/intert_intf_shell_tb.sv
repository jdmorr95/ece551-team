module inert_intf_shell_tb #(
    parameter FAST_SIM = 1;
    parameter CLK_PERIOD = 10;
) (
    input clk, rst_n,
    input strt_cal,						// goes high to initiate calibration
    input vld,							// goes high for 1 clock cycle when new data valid
    input signed [15:0] yaw_rt,			// raw gyro rate readings from inert_intf
    input lftIR,rghtIR,					// gaurdrail signals (fusion with gyro)
    input moving,						// Only integrate yaw when "going"
    output logic cal_done,				// asserted when calibration is completed
    output reg rdy,
    output signed [11:0] heading,
    output [7:0] LED

    
);

    // instantiate inertial interface
    inert_intf DUT( .clk(clk),.rst_n(rst_n),.strt_cal(strt_cal),.vld(vld),.yaw_rt(yaw_rt),
                    .lftIR(lftIR),.rghtIR(rghtIR),.moving(moving),.cal_done(cal_done),.rdy(rdy),
                    .heading(heading),.LED(LED) );
                    

    // instantiate iNEMO
    SPI_iNEMO2 SPI_iNEMO2(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT));

    ////////////////////
    // Testbench Code //
    ////////////////////
    initial begin
        clk = 0;
        rst_n = 0;

    end


    // clock generation
    always @(clk) #(CLK_PERIOD / 2) clk <= ~clk;


endmodule