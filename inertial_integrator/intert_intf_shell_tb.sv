module inert_intf_shell_tb #(
    parameter FAST_SIM = 1,
    parameter CLK_PERIOD = 10
);
    logic clk;
    logic rst_n;
    logic strt_cal;					// goes high to initiate calibration
    logic lftIR;
    logic rghtIR;					// gaurdrail signals (fusion with gyro)
    logic moving;						// Only integrate yaw when "going"
    logic cal_done;				// asserted when calibration is completed
    logic rdy;
    logic signed [11:0] heading;
    logic [7:0] LED;

    logic SS_n;
    logic SCLK;
    logic MOSI;
    logic MISO;
    logic INT;

    // instantiate inertial interface
    inert_intf DUT( .clk(clk),.rst_n(rst_n),.strt_cal(strt_cal), .lftIR(1'b0),.rghtIR(1'b0),
                    .moving(1'b1),.cal_done(cal_done),.rdy(rdy),.heading(heading),
                    .SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),
                    .INT(INT)
                     );


    // instantiate iNEMO
    SPI_iNEMO2 SPI_iNEMO2(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT));

    ////////////////////
    // Testbench Code //
    ////////////////////
    initial begin
        rst_n = 0;
        clk = 0;

        // lift reset
        @(negedge clk) rst_n = 1;

        $display("-----------------------");
        $display("--------TEST 1---------");

        // wait for NEMO_setup to be asserted
        fork: wait_for_NEMO_setup
            begin : timeout
                repeat (100000) @(posedge clk);
                $display("Test Failed! SPI_iNEMO2.NEMO_setup was not asserted in time!");
                disable success;
                $stop();
            end

            begin : success
                @(posedge SPI_iNEMO2.NEMO_setup) disable timeout;
            end
        join

        $display("-----------------------");
        $display("--------TEST 2---------");

    end


    // clock generation
    always @(clk) #(CLK_PERIOD / 2) clk <= ~clk;


endmodule