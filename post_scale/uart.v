`timescale 1ns / 1ps

// ============================================================================
// 1. UART 수신 모듈 (모터용: 115200bps)
// ============================================================================
module uart_rx_motor (
    input clk, reset_p, rx_in,
    output reg [7:0] data, 
    output reg done
);
    parameter CLK_FREQ = 100_000_000;
    parameter BAUD_RATE = 115200;
    parameter TICKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg [15:0] tick_cnt; 
    reg [3:0] bit_cnt;
    reg rx_reg1, rx_reg2, active; 
    reg [7:0] sh_reg;

    always @(posedge clk) begin
        rx_reg1 <= rx_in; rx_reg2 <= rx_reg1;
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            tick_cnt <= 0; bit_cnt <= 0; active <= 0; done <= 0; data <= 0; sh_reg <= 0;
        end else begin
            done <= 0;
            if (!active) begin
                if (rx_reg2 == 0) begin 
                    active <= 1; tick_cnt <= 0; bit_cnt <= 0;
                end
            end else begin
                if (tick_cnt < TICKS_PER_BIT - 1) tick_cnt <= tick_cnt + 1;
                else begin
                    tick_cnt <= 0;
                    if (bit_cnt == 0) begin 
                        tick_cnt <= TICKS_PER_BIT / 2; bit_cnt <= 1;
                    end else if (bit_cnt <= 8) begin
                        sh_reg[bit_cnt-1] <= rx_reg2; bit_cnt <= bit_cnt + 1;
                    end else begin
                        active <= 0; bit_cnt <= 0; done <= 1; data <= sh_reg;
                    end
                end
            end
        end
    end
endmodule


// ============================================================================
// 2. UART 수신 모듈 (LCD용: 고속 460800bps)
// ============================================================================
module uart_rx_high_speed #(parameter BAUD = 460800, parameter CLK_FREQ = 100000000) (
    input wire clk,
    input wire rx,
    output reg [7:0] data,
    output reg ready
);
    localparam integer BIT_PERIOD = CLK_FREQ / BAUD;
    localparam integer HALF_PERIOD = BIT_PERIOD / 2;

    reg [1:0] state = 0;
    reg [15:0] clk_cnt = 0;
    reg [2:0] bit_idx = 0;
    reg [7:0] shft = 0;
    reg r1, r2;

    always @(posedge clk) begin r1 <= rx; r2 <= r1; end

    always @(posedge clk) begin
        ready <= 0;
        case (state)
            0: if (r2 == 0) begin clk_cnt <= 0; state <= 1; end 
            1: if (clk_cnt >= HALF_PERIOD) begin clk_cnt <= 0; state <= 2; end else clk_cnt <= clk_cnt + 1;
            2: if (clk_cnt >= BIT_PERIOD-1) begin 
                   clk_cnt <= 0; shft[bit_idx] <= r2;
                   if (bit_idx == 7) begin bit_idx <= 0; state <= 3; end else bit_idx <= bit_idx + 1;
               end else clk_cnt <= clk_cnt + 1;
            3: if (clk_cnt >= BIT_PERIOD-1) begin data <= shft; ready <= 1; state <= 0; end else clk_cnt <= clk_cnt + 1;
        endcase
    end
endmodule