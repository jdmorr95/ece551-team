module SPI_mnrch(
    /****************************************/
    /*             serf controls            */
    /****************************************/

    output logic MOSI, // serial output to the serf.
    output logic SS_n, // active low serf select signal.
    output logic SCLK, // 1/32nd of our clock signal @ 1.5625 Mhz

    /****************************************/
    /*             serf response            */
    /****************************************/

    input logic MISO, // serial input from the serf.

    /****************************************/
    /* output for the consumers of our data */
    /****************************************/

    output logic [15:0] resp, // data from SPI serf.
    output logic done, // asserted when SPI transaction is complete.

    /****************************************/
    /* input for the consumers of our data  */
    /****************************************/

    input logic [15:0] cmd, // data sent to inertial sensor
    input logic snd, // high for one clock period when cmd is sent.

    // clock and reset @ 50MHz
    input logic clk,
    input logic rst_n
);





/**************************/
/* Internal Logic Signals */
/**************************/

typedef enum logic [1:0]
{
    IDLE,
    INITIATE_SERF_TRANSACTION,
    SERF_TRANSACTION
} state_t;

state_t state;
state_t next_state;
logic init; // asserted when an SPI transaction is initiated.
logic set_done; // sets done SR flop.
logic ld_SCLK; // asserted when an SPI transaction is initiated.
logic clear_SS_n; // asserted when we want to select a serf.
logic set_SS_n; // asserted when we want to deselect a serf.

logic [4:0] SCLK_counter; // 32-bit counter for the SCLK divider.
logic SCLK_counter_full; // asserted when 32-bit counter is all ones.
logic SCLK_counter_shift; // asserted one clock cycle after SCLK is high.

logic [4:0] bit_cnt; // number of shifted bits.
logic done16; // asserted when we have shifted 16 bits.

logic [15:0] shft_reg;






/**************************/
/* Clock Divider for SCLK */
/**************************/

assign SCLK_counter_full = &SCLK_counter;
assign SCLK_counter_shift = (SCLK_counter === 5'b10001);

assign SCLK = SCLK_counter[4];  // SCLK is the MSB of the 32-bit counter,
                                //because it will be high half of the count.

always_ff @ (posedge clk) begin
    SCLK_counter <= (ld_SCLK) ? 5'b10111 : SCLK_counter + 1;
end


/* Bit counter for shift register */
assign done16 = bit_cnt[4];

always_ff @(posedge clk) begin
    bit_cnt <= (init) ? 5'b00000 : ((SCLK_counter_shift) ? bit_cnt + 1: bit_cnt);
end


/***************************/
/* Shift Register for MOSI */
/***************************/

assign MOSI = shft_reg[15]; // MOSI is the MSB of the shift register.

always_ff @(posedge clk) begin
    unique case ({init,SCLK_counter_shift}) inside
        2'b1x: shft_reg <= cmd;
        2'b01: shft_reg <= {shft_reg[14:0],MISO};
        2'b00: shft_reg <= shft_reg;
    endcase
end

// the response is the contents of the shift register.
assign resp = shft_reg;

/*****************/
/* DONE SR FLOP  */
/*****************/
always_ff @ (posedge clk, negedge rst_n) begin
    if (!rst_n)
        done <= 0;
    else if (init)
        done <= 0;
    else if (set_done)
        done <= 1;
end


/*****************/
/* SS_N SR FLOP  */
/*****************/
always_ff @ (posedge clk, negedge rst_n) begin
    if (!rst_n)
        SS_n <= 1;
    else if (clear_SS_n)
        SS_n <= 0;
    else if (set_SS_n)
        SS_n <= 1;
end


/*****************/
/* STATE MACHINE */
/*****************/
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end


always_comb begin : next_state_logic
    // default assignments
    init = 0;
    ld_SCLK = 0;
    set_done = 0;
    set_SS_n = 0;
    clear_SS_n = 0;
    next_state = state;

    unique case (state)


        default: // IDLE
            if (snd) begin
                clear_SS_n = 1;
                ld_SCLK = 1;
                init = 1; /* TODO might not need this?*/
                next_state = INITIATE_SERF_TRANSACTION;
            end


        INITIATE_SERF_TRANSACTION: begin
            init = 1;
            if (SCLK_counter_full) begin
                next_state = SERF_TRANSACTION;
            end
        end


        SERF_TRANSACTION:
            if (done16) begin
                set_done = 1;
                set_SS_n = 1;
                next_state = IDLE;
            end
    endcase

end



endmodule