`timescale 1ns/1ps

module tb_top;
    reg clk_sys;
    reg rstn;
    reg sclk;
    reg mosi;
    reg cs_n;

    wire [15:0] sram0_data;
    wire [15:0] sram0_addr;
    wire        sram0_we_n;
    wire [15:0] sram1_data;
    wire [15:0] sram1_addr;
    wire        sram1_we_n;
    wire        dac0_clock;
    wire        dac1_clock;
    wire        opp_led;

    top_system uut (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .spi_sclk(sclk),
        .spi_mosi(mosi),
        .spi_cs_n(cs_n),
        .sram0_data(sram0_data),
        .sram0_addr(sram0_addr),
        .sram0_we_n(sram0_we_n),
        .sram1_data(sram1_data),
        .sram1_addr(sram1_addr),
        .sram1_we_n(sram1_we_n),
        .dac0_clock(dac0_clock),
        .dac1_clock(dac1_clock),
        .opp_led(opp_led)
    );

    // Clock generation
    initial clk_sys = 0;
    always #5 clk_sys = ~clk_sys; // 100 MHz system cock

    task send_spi_bit(input bit_value);
    begin
        mosi = bit_value;
        sclk = 1; #10;
        sclk = 0; #10; 
    end
    endtask

    task send_spi_frame(input [34:0] frame); 
    integer i;
    begin
        cs_n = 0;   
        for (i = 34; i >= 0; i=i-1) begin
            send_spi_bit(frame[i]);
        end
        cs_n = 1;   
        #20;        
    end
    endtask

    initial begin
        // Initialize signals
        rstn = 0; sclk = 0; mosi = 0; cs_n = 1;
        #20;
        rstn = 1;

        // {opcode, addr, data}
        send_spi_frame({3'b010, 16'h0006, 16'hABCD});  // set reset0 to 6
        send_spi_frame({3'b011, 16'h0005, 16'hABCD});  // set reset1 to 5
        send_spi_frame({3'b100, 16'h0FFF, 16'hFFFF});  // set phase0 to h0FFFFFFF
        send_spi_frame({3'b101, 16'h02FF, 16'hFFFF});  // set phase1 to h02FFFFFF
        send_spi_frame({3'b000, 16'h5678, 16'hEFAA});  // set sram0 address h5678 to hEFAA
        send_spi_frame({3'b000, 16'h1234, 16'hABEF});  // set sram0 address h1234 to hABEF
        send_spi_frame({3'b001, 16'h1234, 16'hABCD});  // set sram1 address h1234 to hABCD
        send_spi_frame({3'b010, 16'h0001, 16'hABCD});  // set reset0 to 1
        send_spi_frame({3'b001, 16'h1234, 16'hABCD});  // spacer
        send_spi_frame({3'b100, 16'h07FF, 16'hFFFF});  // set phase0 to h07FFFFFF
        send_spi_frame({3'b010, 16'h0020, 16'hABCD});  // set reset0 to 32
        send_spi_frame({3'b001, 16'h1234, 16'hABCD});  // spacer
        send_spi_frame({3'b111, 16'hAAAA, 16'hAAAA});  // run

        #5000;
        $finish;
    end

    // Dump waveforms
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top);
    end

endmodule