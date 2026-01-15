//=====================================================
// spi_peripheral.v
//=====================================================
// SPI peripheral interface that decodes incoming serial data
// Message length is always 35 bits, otherwise opcode is set to 110 (error)
// Opcodes:
//   000 = Write to SRAM0 (16-bit address, 16-bit data)
//   001 = Write to SRAM1 (16-bit address, 16-bit data)
//   010 = Set reset value of counter0 (16-bit reset value, 16 junk bits)
//   011 = Set reset value of counter1 (16-bit reset value, 16 junk bits)
//   100 = Set NCO0 phase increment (32-bit phase increment)
//   101 = Set NCO1 phase increment (32-bit phase increment)
//   110 = Error or stopped
//   111 = Running
// Frames are MSB-first, CS active low
// When CS goes high, a decoded pulse is generated

module spi_peripheral (
    input  wire       clk_sys,                 // system clock
    input  wire       rstn,                    // system reset
    input  wire       sclk,                    // SPI clock
    input  wire       mosi,                    // SPI data in
    input  wire       cs_n,                    // SPI chip select
    output reg [2:0]  opcode,                  // first three bits of message
    output reg [31:0] spi_out                  // other 32 bits of message
);

    reg [95:0] shift_reg;
    reg [6:0]  bit_cnt;

    always @(negedge sclk or negedge rstn) begin
        if (!rstn) begin
            shift_reg <= 96'd0;
            bit_cnt   <= 7'd0;
        end else if (!cs_n) begin
            shift_reg <= {shift_reg[94:0], mosi};
            bit_cnt   <= bit_cnt + 1'b1;
        end
    end

    // On CS rising, decode frame
    always @(posedge cs_n or negedge rstn) begin
        if (!rstn) begin
            opcode                 <= 3'b110;  // unused value so that nothing is active
            spi_out                <= 32'd0;
            shift_reg              <= 96'd0;
            bit_cnt                <= 7'd0;
        end else begin
            opcode                 <= 3'b110;

            if (bit_cnt == 35) begin
                opcode  <= shift_reg[bit_cnt-1 -: 3];
                spi_out <= shift_reg[bit_cnt-4 -: 32];
            end

            shift_reg <= 96'd0;
            bit_cnt   <= 7'd0;
        end
    end
endmodule

//=====================================================
// sram_controller.v
//=====================================================

module sram_controller (
    input  wire        clk_sys,             // system clock
    input  wire        rstn,                // system reset
    input  wire        write_pulse,         // if opcode is set to write to memory
    input  wire [31:0] spi_out,             // output of spi, 16 addr + 16 data
    input  wire [15:0] counter_addr,        // output of counter
    inout  wire [15:0] sram_data,           // connection to data IO pins
    output wire [15:0] sram_addr,           // connection to address IO pins
    output reg         sram_we_n            // connection to read/write IO pin
);

    reg [15:0] data_out;
    reg [15:0] addr_out;
    reg        drive_data;

    assign sram_data = drive_data ? data_out : 16'hzzzz;
    assign sram_addr = drive_data ? addr_out : counter_addr;

    always @(posedge clk_sys or negedge rstn) begin
        if (!rstn) begin
            sram_we_n  <= 1'b1;
            data_out   <= 16'b0;
            addr_out   <= 16'b0;
        end else begin
            if (write_pulse) begin
                data_out   <= spi_out[15:0];
                addr_out   <= spi_out[31:16];
                drive_data <= 1'b1;
                sram_we_n  <= 1'b0;
            end else begin
                drive_data <= 1'b0;
                sram_we_n  <= 1'b1;
            end
        end
    end

endmodule

//=====================================================
// binary_counter.v
//=====================================================
// 16-bit counter driven by VCO
// Counts from 0 to the reset value

module binary_counter (
    input  wire        clk_sys,                 // system clock
    input  wire        rstn,                    // system reset
    input  wire        step_en,                 // if high on system clock, increments counter
    input  wire        set_reset_pulse,         // if opcode is set to change reset value
    input  wire [31:0] spi_out,                 // output of spi, 16 reset + 16 junk
    input  wire        mem_write_pulse,         // if opcode is set to write to memory
    output reg  [15:0] value                    // output number
);

    reg [15:0] reset_value;

    always @(posedge clk_sys or negedge rstn) begin
        if (!rstn) begin
            reset_value <= 16'd0;
            value <= 16'd0;
        end else if (set_reset_pulse) begin
            reset_value <= spi_out[31:16];
            value <= 16'd0;
        end else if (mem_write_pulse)
            value <= 16'd0;
        else if (step_en) begin
            if (value >= reset_value)
                value <= 16'd0;
            else
                value <= value + 1;
        end
    end
endmodule

//=====================================================
// nco_square.v
//=====================================================
// Numerically controlled oscillator
// outputs a single clock pulse when counter should be stepped

module nco_square (
    input  wire        clk_sys,             // system clock
    input  wire        rstn,                // system reset
    input  wire        change_phase_pulse,  // if opcode is set to change the phase increment
    input  wire [31:0] spi_out,             // output of spi, used to set tuning word
    output reg         step_pulse,          // output pulse
    output reg         square               // square wave output
);
    reg [31:0] phase_inc;
    reg [31:0] phase_acc;
    reg        prev_msb;

    always @(posedge clk_sys or negedge rstn) begin
        if (!rstn) begin
            phase_inc  <= 32'b0;
            phase_acc  <= 32'b0;
            prev_msb   <= 1'b0;
            step_pulse <= 1'b0;
            square     <= 1'b0;
        end else if (change_phase_pulse) begin
            phase_inc  <= spi_out;
        end else begin
            phase_acc  <= phase_acc + phase_inc;
            step_pulse <= 1'b0;
            if (phase_acc[31] != prev_msb)
                step_pulse <= 1'b1;  // step pulse is only high for one clock cycle when MSB of phase_acc changes
            prev_msb <= phase_acc[31];
            square <= ~phase_acc[30];
        end
    end
endmodule

//=====================================================
// top_system.v
//=====================================================

module top_system (
    input  wire       clk_sys,
    input  wire       rstn,
    // SPI interface
    input  wire       spi_sclk,
    input  wire       spi_mosi,
    input  wire       spi_cs_n,
    // SRAM0 interface
    inout  wire [15:0] sram0_data,
    output wire [15:0] sram0_addr,
    output wire        sram0_we_n,
    // SRAM1 interface
    inout  wire [15:0] sram1_data,
    output wire [15:0] sram1_addr,
    output wire        sram1_we_n,
    // DAC Clock Outputs 
    output wire        dac0_clock,
    output wire        dac1_clock,
    // LED Outputs
    output wire        opp_led
);

    // SPI signals
    wire [2:0]  opcode;
    wire [31:0] spi_out;

    spi_peripheral spi_inst (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .cs_n(spi_cs_n),
        .opcode(opcode),
        .spi_out(spi_out)
    );

    // Memory controller signals
    wire write_mem0_pulse;
    wire write_mem1_pulse;
    assign write_mem0_pulse = (opcode == 3'b000);
    assign write_mem1_pulse = (opcode == 3'b001);

    sram_controller sc0 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .write_pulse(write_mem0_pulse),
        .spi_out(spi_out),
        .counter_addr(count0),
        .sram_data(sram0_data),
        .sram_addr(sram0_addr),
        .sram_we_n(sram0_we_n)
    );

    sram_controller sc1 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .write_pulse(write_mem1_pulse),
        .spi_out(spi_out),
        .counter_addr(count1),
        .sram_data(sram1_data),
        .sram_addr(sram1_addr),
        .sram_we_n(sram1_we_n)
    );

    // Counter signals
    wire set_reset0_pulse;
    wire set_reset1_pulse;
    assign set_reset0_pulse = (opcode == 3'b010);
    assign set_reset1_pulse = (opcode == 3'b011);
    wire [15:0] count0;
    wire [15:0] count1;

    binary_counter c0 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .step_en(nco0_step),
        .set_reset_pulse(set_reset0_pulse),
        .spi_out(spi_out),
        .mem_write_pulse(write_mem0_pulse),
        .value(count0)
    );

    binary_counter c1 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .step_en(nco1_step),
        .set_reset_pulse(set_reset1_pulse),
        .spi_out(spi_out),
        .mem_write_pulse(write_mem1_pulse),
        .value(count1)
    );

    // NCO signals
    wire change_phase0_pulse;
    wire change_phase1_pulse;
    assign change_phase0_pulse = (opcode == 3'b100);
    assign change_phase1_pulse = (opcode == 3'b101);

    nco_square nco0 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .change_phase_pulse(change_phase0_pulse),
        .spi_out(spi_out),
        .step_pulse(nco0_step),
        .square(dac0_clock)
    );

    nco_square nco1 (
        .clk_sys(clk_sys),
        .rstn(rstn),
        .change_phase_pulse(change_phase1_pulse),
        .spi_out(spi_out),
        .step_pulse(nco1_step),
        .square(dac1_clock)
    );

    // LEDs
    assign opp_led = (opcode != 3'b111);

endmodule
