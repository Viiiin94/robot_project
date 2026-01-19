# 🐶 FPGA 기반 로봇 제어 시스템

본 프로젝트는 FPGA를 이용하여 **로봇의 구동부(DC 모터, 서보 모터)** 와  
**시각적 인터페이스(LCD 디스플레이)** 를 동시에 제어하는 임베디드 시스템이다.

UART 통신을 통해 외부 입력을 수신하고, 이를 기반으로  
로봇의 동작 제어 및 상태 표현(UI)을 실시간으로 수행하는 것을 목표로 한다.

### 🏄‍♂️ 팀원 소개

![Block Diagram](pic/Screenshot from 2026-01-19 10-22-34.png)

![Block Diagram](pic/Screenshot from 2026-01-19 10-22-38.png)

---

## 1. Project Goals

- FPGA 기반 로봇 제어 시스템 구조 설계
- UART 통신을 이용한 외부 제어 명령 처리
- DC 모터 및 서보 모터의 안정적인 PWM 제어
- LCD 디스플레이를 통한 로봇 상태 및 표정 표현
- **모듈화된 설계를 통한 확장성과 재사용성 확보**

---

## 2. System Overview

본 시스템은 하나의 FPGA에서 다음 기능을 동시에 수행한다.

- 모터 제어용 UART 데이터 수신
- 고속 UART 기반 LCD 데이터 수신
- DC 모터 구동 및 방향 제어
- SG90 서보 모터 PWM 제어
- SPI 기반 LCD 디스플레이 출력

각 기능은 **독립된 모듈로 분리**되어 있으며,  
최상위 모듈은 하위 모듈 간의 연결만 담당한다.

---

## 3. Key Features

### UART-Based Control
- 서로 다른 baud rate를 사용하는 UART 수신 모듈 분리
- 제어 신호와 디스플레이 데이터를 독립적으로 처리

### Motor Control
- DC 모터 방향 및 속도 제어
- PWM 기반 안정적인 구동
- LED를 활용한 디버그 출력

### Servo Motor Control
- SG90 서보 모터 제어
- 각도 값을 PWM 듀티로 변환하는 구조
- PWM 생성 로직과 제어 로직 분리

### LCD User Interface
- ILI9341 기반 SPI LCD 제어
- FSM 기반 화면 제어
- 로봇의 상태를 시각적으로 표현

---

## 4. Design Philosophy

본 프로젝트는 다음과 같은 설계 원칙을 기반으로 한다.

- **Top Module은 연결만 담당**
- 하나의 모듈은 하나의 책임만 수행
- 속도와 타이밍이 다른 시스템은 분리 설계
- FSM은 단순하게, 데이터 흐름은 명확하게

이를 통해 디버깅과 유지보수가 용이한 구조를 목표로 한다.

---

## 5. Industrial Perspective

산업용 시스템 관점에서 다음과 같은 점을 고려하여 설계되었다.

- UART 통신 분리로 데이터 충돌 최소화
- 모터·서보·디스플레이 제어의 완전한 기능 분리
- 확장 가능한 구조 (Frame Buffer, CRC, FIFO 적용 가능)

향후 산업 환경 적용을 위해  
Frame Buffer 기반 LCD 구조 및 데이터 무결성 검증 로직을 추가할 수 있다.

---

## 6. Development Environment

- FPGA Board: (사용 보드 기입)
- HDL: Verilog HDL
- Display: ILI9341 SPI LCD
- Communication: UART
- Motor: DC Motor, SG90 Servo Motor
- Toolchain: Vivado

---

## 7. Expected Extensions

- Frame Buffer 기반 LCD 출력 구조
- CRC 기반 UART 패킷 처리
- Clock Domain Crossing 보호 로직
- DMA-like 디스플레이 처리 구조

---

## 8. Conclusion

본 프로젝트는 단순한 FPGA 실습을 넘어,  
**실제 로봇 제어 시스템에서 사용 가능한 구조를 목표로 설계되었다.**

모듈화, 타이밍 분리, 확장성을 고려한 설계를 통해  
추후 기능 추가 및 산업용 시스템으로의 확장이 가능하다.

---

# 1. System Architecture

## Block Diagram
```
                          ┌────────────────────────┐
                          │        dog_top         │
                          │  (Top-Level Wiring)    │
                          └───────────┬────────────┘
                                      │
                ┌─────────────────────┼─────────────────────┐
                │                     │                     │
            ┌───▼────────┐     ┌──────▼────────┐     ┌──────▼────────┐
            │ UART RX    │     │ UART RX       │     │ LCD Controller│
            │ (Motor)    │     │ (High Speed)  │     │ ILI9341 SPI   │
            │ 115200bps  │     │ 460800bps     │     │ FSM           │
            └───┬────────┘     └──────┬────────┘     └──────┬────────┘
                │                     │                     │
            ┌───▼────────┐     ┌──────▼────────┐     ┌──────▼────────┐
            │ DC Motor   │     │ Servo Motor   │     │ LCD Panel     │
            │ Controller │     │ SG90 PWM Ctrl │     │ (Robot Face)  │
            └────────────┘     └───────────────┘     └───────────────┘
```
---

# 2. Directory Structure

```text
./
├── post_scale/
│   ├── dog_top.v
│   ├── lcd.v
│   ├── motor.v
│   ├── servo.v
│   ├── uart.v
│   └── dog.xdc
│
└── pre_scale/
    └── robot.py
