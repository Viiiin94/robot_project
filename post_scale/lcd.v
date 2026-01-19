`timescale 1ns / 1ps

// ============================================================================
// LCD 제어 모듈 (ILI9341)
//   - 16비트 픽셀 데이터 조립 기능이 포함되어 색상 깨짐이 없습니다.
// ============================================================================
module ili9341_spi_robot_face (
    input  wire clk,          // 100MHz
    input  wire btnC,         // Reset
    input  wire rx,           // UART RX (460,800 baud)
    output reg  lcd_reset,    
    output reg  lcd_cs,       
    output reg  lcd_dc,       
    output reg  lcd_sclk,     
    output wire lcd_sdi,      
    output wire lcd_bl        
);

    assign lcd_bl = 1'b1; // 백라이트 켜기

    localparam S_RESET      = 4'd0, S_DELAY       = 4'd1,
               S_INIT       = 4'd2, S_SET_ADDR    = 4'd3,
               S_WAIT_BYTE  = 4'd4, S_SEND_SPI    = 4'd5;

    reg [3:0]  state = S_RESET;
    reg [3:0]  after_delay_state;
    reg [31:0] delay_cnt = 0;
    reg [31:0] idle_cnt = 0; 
    reg [7:0]  send_data = 0;
    reg [7:0]  cmd_step = 0;
    reg [3:0]  bit_cnt = 0;
    
    // [핵심] 16비트 픽셀 조립용 레지스터
    reg [15:0] pixel_reg = 0;
    reg [4:0]  spi_div = 0; 
    reg [16:0] pixel_cnt = 0;
    reg        pixel_byte_toggle = 0; // 0:상위바이트, 1:하위바이트

    assign lcd_sdi = send_data[7 - bit_cnt];

    wire [7:0] rx_data;
    wire       rx_ready;
    
    // 고속 UART (LCD 전용)
    uart_rx_high_speed #(.BAUD(460800)) uart_inst (.clk(clk), .rx(rx), .data(rx_data), .ready(rx_ready));

    always @(posedge clk) begin
        if (btnC) begin
            state <= S_RESET; 
            lcd_reset <= 0; lcd_cs <= 1; lcd_sclk <= 0;
            delay_cnt <= 0; pixel_cnt <= 0; pixel_byte_toggle <= 0;
            cmd_step <= 0; idle_cnt <= 0;
        end else begin
            case (state)
                S_RESET: begin 
                    lcd_reset <= 0;
                    if (delay_cnt >= 5_000_000) begin 
                        lcd_reset <= 1; 
                        delay_cnt <= 0; 
                        state <= S_DELAY; 
                        after_delay_state <= S_INIT;
                    end else delay_cnt <= delay_cnt + 1;
                end

                S_DELAY: begin 
                    if (delay_cnt >= 20_000_000) begin 
                        delay_cnt <= 0; 
                        state <= after_delay_state; 
                    end else delay_cnt <= delay_cnt + 1;
                end

                S_INIT: begin 
                    lcd_dc <= 0;
                    case (cmd_step)
                        0: begin send_data <= 8'h01; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        1: begin send_data <= 8'h11; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        2: begin send_data <= 8'h36; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        3: begin lcd_dc <= 1; send_data <= 8'h08; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        4: begin lcd_dc <= 0; send_data <= 8'h3A; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        5: begin lcd_dc <= 1; send_data <= 8'h55; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        6: begin lcd_dc <= 0; send_data <= 8'h29; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        default: begin state <= S_SET_ADDR; cmd_step <= 0; end
                    endcase
                    cmd_step <= cmd_step + 1;
                end

                S_SET_ADDR: begin 
                    case (cmd_step)
                        0: begin lcd_dc <= 0; send_data <= 8'h2A; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        1,2,3: begin lcd_dc <= 1; send_data <= 8'h00; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end 
                        4: begin lcd_dc <= 1; send_data <= 8'hEF; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        5: begin lcd_dc <= 0; send_data <= 8'h2B; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        6,7: begin lcd_dc <= 1; send_data <= 8'h00; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        8: begin lcd_dc <= 1; send_data <= 8'h01; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        9: begin lcd_dc <= 1; send_data <= 8'h3F; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        10:begin lcd_dc <= 0; send_data <= 8'h2C; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        default: begin state <= S_WAIT_BYTE; pixel_cnt <= 0; idle_cnt <= 0; pixel_byte_toggle <= 0; end
                    endcase
                    cmd_step <= cmd_step + 1;
                end

                S_WAIT_BYTE: begin 
                    if (idle_cnt > 5_000_000) begin // 50ms 타임아웃
                         idle_cnt <= 0;
                         pixel_cnt <= 0;
                         pixel_byte_toggle <= 0;
                         state <= S_SET_ADDR; cmd_step <= 0;
                    end else if (rx_ready) begin
                        idle_cnt <= 0;
                        if (pixel_byte_toggle == 0) begin 
                            pixel_reg[15:8] <= rx_data; 
                            pixel_byte_toggle <= 1; 
                        end else begin
                            pixel_reg[7:0] <= rx_data; 
                            pixel_byte_toggle <= 0;
                            send_data <= pixel_reg[15:8]; // 상위 바이트 먼저 전송 준비
                            lcd_dc <= 1; 
                            state <= S_SEND_SPI; 
                            after_delay_state <= S_SEND_SPI; 
                        end
                    end else idle_cnt <= idle_cnt + 1;
                end

                S_SEND_SPI: begin 
                    lcd_cs <= 0;
                    if (spi_div >= 6) begin 
                        spi_div <= 0;
                        if (lcd_sclk == 0) lcd_sclk <= 1;
                        else begin
                            lcd_sclk <= 0;
                            if (bit_cnt == 7) begin
                                bit_cnt <= 0; lcd_cs <= 1;
                                // 픽셀 데이터 전송 중일 때 (상위 -> 하위 순서 처리)
                                if (after_delay_state == S_SEND_SPI) begin 
                                    send_data <= pixel_reg[7:0]; // 하위 바이트 장전
                                    after_delay_state <= S_WAIT_BYTE; // 이거 보내고 다시 수신 대기
                                    state <= S_SEND_SPI; 
                                end else begin
                                    // 일반 명령이거나 픽셀 하위바이트까지 다 보냈을 때
                                    if (after_delay_state == S_WAIT_BYTE) begin
                                        if (pixel_cnt >= 76799) begin 
                                            state <= S_SET_ADDR; cmd_step <= 0; 
                                        end else begin 
                                            pixel_cnt <= pixel_cnt + 1; 
                                            state <= S_WAIT_BYTE; 
                                        end
                                    end else state <= (after_delay_state == S_INIT) ? S_DELAY : after_delay_state;
                                end
                            end else bit_cnt <= bit_cnt + 1;
                        end
                    end else spi_div <= spi_div + 1;
                end
            endcase
        end
    end
endmodule