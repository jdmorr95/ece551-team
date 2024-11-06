module inert_intf_shell_tb()


    parameter FAST_SIM = 1;
    parameter CLK_PERIOD = 10;

    logic clk;
    logic rst_n;
    logic strt_cal;
    logic cal_done;
    logic signed [11:0] heading;
    logic rdy;
    logic lftIR;
    logic rghtIR;
    logic SS_n;
    logic SCLK;
    logic MOSI;
    logic MISO;
    logic INT;
    logic moving;


    // instantiate inertial interface
    inert_intf DUT( .clk(clk),.rst_n(rst_n),.strt_cal(strt_cal),.cal_done(cal_done),
                    .heading(heading),.rdy(rdy),.lftIR(lftIR),.rghtIR(rghtIR),.SS_n(SS_n),
                    .SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),.INT(INT),.moving(moving)
                    );

    // instantiate iNEMO
    SPI_iNEMO2 SPI_iNEMO2(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT));

    ////////////////////
    // Testbench Code //
    ////////////////////
    initial begin
    end


    // clock generation
    always @(clk) #(CLK_PERIOD / 2) clk <= ~clk;


endmodule