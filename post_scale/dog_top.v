`timescale 1ns / 1ps

// ============================================================================
// 1. 최상위 모듈 (dog_top)
//    - 라즈베리파이와 연결되어 모든 장치를 총괄합니다.
// ============================================================================
module dog_top(
    input clk,              // 100MHz 시스템 클럭
    input reset_p,          // 중앙 버튼 (전체 리셋)
    input sw,               // 스위치 (모터 안전장치: 올리면 동작)
    
    // [통신 1] 모터 제어용 (115200bps, GPIO 14 -> JC1)
    input rx_motor,      
    
    // [통신 2] LCD 이미지용 (460800bps, GPIO 8 -> JA9/H2)
    input rx_lcd,        

    // [출력 1] 모터 & 서보 & LED
    output [3:0] motor_in, // L298N 방향제어
    output [1:0] motor_en, // L298N 속도제어(PWM)
    output sg90_pwm,       // 목 회전 서보모터
    output [15:0] led,     // 상태 확인용 LED

    // [출력 2] LCD 디스플레이 (Pmod JA)
    output lcd_reset,
    output lcd_cs,
    output lcd_dc,
    output lcd_sclk,
    output lcd_sdi,
    output lcd_bl
    );

    // --------------------------------------------------------
    // (1) 모터/서보 패킷 처리부
    // --------------------------------------------------------
    wire [7:0] motor_rx_data;
    wire motor_rx_done;
    wire [14:0] w_high_dur;

    reg [7:0] p_angle;
    reg [7:0] p_l_speed;
    reg [7:0] p_r_speed;
    reg [3:0] p_dir;
    
    reg [2:0] parse_state;

    // 모터용 UART 수신 (115200bps)
    uart_rx_motor motor_rx_inst (
        .clk(clk), .reset_p(reset_p), .rx_in(rx_motor), 
        .data(motor_rx_data), .done(motor_rx_done)
    );

    // 패킷 해석: [0xFF, Angle, L_Speed, R_Speed, Dir]
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            parse_state <= 0;
            p_angle <= 90; p_l_speed <= 0; p_r_speed <= 0; p_dir <= 0;
        end else if (motor_rx_done) begin
            case (parse_state)
                0: if (motor_rx_data == 8'hFF) parse_state <= 1; // 헤더 확인
                1: begin p_angle <= motor_rx_data; parse_state <= 2; end
                2: begin p_l_speed <= motor_rx_data; parse_state <= 3; end
                3: begin p_r_speed <= motor_rx_data; parse_state <= 4; end
                4: begin p_dir <= motor_rx_data; parse_state <= 0; end
            endcase
        end
    end

    // --------------------------------------------------------
    // (2) 하드웨어 구동부 연결
    // --------------------------------------------------------
    
    // 서보 모터 (각도 -> PWM 변환)
    sg90_ctrl servo_calc (.angle_in(p_angle), .high_dur(w_high_dur));
    pwm_gen servo_pwm (.clk(clk), .reset_p(reset_p), .high_dur(w_high_dur), .pwm(sg90_pwm));

    // DC 모터 (L298N 드라이버)
    robot_leg_motor_ctl motor_driver(
        .clk(clk), .reset_p(reset_p), 
        .in_left_speed(p_l_speed), 
        .in_right_speed(p_r_speed), 
        .in_direction(p_dir),
        .motor_in(motor_in), .motor_en(motor_en), 
        .led(led), .sw(sw)
    );

    // LCD 디스플레이 (ILI9341 SPI)
    ili9341_spi_robot_face lcd_inst (
        .clk(clk),
        .btnC(reset_p),     // 리셋 공유
        .rx(rx_lcd),        // LCD용 고속 UART (460800bps)
        .lcd_reset(lcd_reset),
        .lcd_cs(lcd_cs),
        .lcd_dc(lcd_dc),
        .lcd_sclk(lcd_sclk),
        .lcd_sdi(lcd_sdi),
        .lcd_bl(lcd_bl)
    );

endmodule