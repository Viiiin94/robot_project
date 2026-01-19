`timescale 1ns / 1ps

// ============================================================================
// 서보 모터 제어 모듈
// ============================================================================
module sg90_ctrl(
    input [7:0] angle_in,     // 0~180
    output [14:0] high_dur    // PWM High Time (us)
    );
    wire [7:0] safe_angle = (angle_in > 180) ? 8'd180 : angle_in;
    wire [14:0] extended_angle = {7'd0, safe_angle};
    assign high_dur = 15'd500 + (extended_angle * 15'd11);
endmodule

// ============================================================================
// PWM 제어
// ============================================================================
module pwm_gen(
    input clk, reset_p,
    input [14:0] high_dur,
    output reg pwm
    );
    parameter CLK_FREQ = 100; // 1us Tick
    reg [8:0] cnt_1us;
    wire clk_1us = (cnt_1us == CLK_FREQ - 1);
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_1us <= 0;
        else if(clk_1us) cnt_1us <= 0;
        else cnt_1us <= cnt_1us + 1;
    end

    reg [14:0] cnt_20ms;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_20ms <= 0;
        else if(clk_1us) begin
            if(cnt_20ms >= 19999) cnt_20ms <= 0;
            else cnt_20ms <= cnt_20ms + 1;
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if(reset_p) pwm <= 0;
        else pwm <= (cnt_20ms < high_dur);
    end
endmodule