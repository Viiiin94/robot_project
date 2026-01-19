`timescale 1ns / 1ps

// ============================================================================
// DC 모터 제어 모듈
// ============================================================================
module robot_leg_motor_ctl(
    input clk, reset_p,
    input [7:0] in_left_speed,
    input [7:0] in_right_speed,
    input [3:0] in_direction,
    output [3:0] motor_in,
    output [1:0] motor_en,
    output reg [15:0] led,
    input sw
    );

    parameter PERIOD = 200000; // 500Hz
    reg [19:0] counter;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) counter <= 0;
        else begin
            if (counter < PERIOD - 1) counter <= counter + 1;
            else counter <= 0;
        end
    end

    wire [19:0] duty_left  = in_left_speed * 2000;
    wire [19:0] duty_right = in_right_speed * 2000;

    assign motor_in = (sw) ? in_direction : 4'b0000;
    assign motor_en[0] = (sw) ? (counter < duty_left) : 1'b0;
    assign motor_en[1] = (sw) ? (counter < duty_right) : 1'b0;

    always @(*) begin
        led = 0;
        if (sw) begin
            if (in_left_speed > 0 || in_right_speed > 0) led[0] = 1;
            else led[1] = 1;
            led[5:2] = in_direction; 
        end
    end
endmodule