module reset_synch (
  input logic clk,
  input logic RST_n,
  output logic rst_n
);

    // Synchronous reset
    always_ff @(negedge clk or negedge RST_n)
        if (!RST_n)
        rst_n <= 0;
        else
        rst_n <= 1;

endmodule
