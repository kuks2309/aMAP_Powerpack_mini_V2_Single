# aMAP Powerpack Mini V2 - Pin Map

## Board: Arduino Mega 2560

---

## Digital Pins

### Motor Driver (H-Bridge PWM Control)

| Pin | Function | Description |
|-----|----------|-------------|
| 2 | MOTOR1_EN1 | Motor 1 direction control pin 1 (PWM) |
| 3 | MOTOR1_EN2 | Motor 1 direction control pin 2 (PWM) |
| 4 | MOTOR2_EN1 | Motor 2 direction control pin 1 (PWM) |
| 5 | MOTOR2_EN2 | Motor 2 direction control pin 2 (PWM) |

**Motor Control Logic:**
- Forward: EN1=PWM, EN2=0
- Reverse: EN1=0, EN2=PWM
- Brake: EN1=0, EN2=0

---

### RC Servo Control

| Pin | Function | Description |
|-----|----------|-------------|
| 6 | SERVO1_PIN | Servo 1 PWM signal |
| 7 | SERVO2_PIN | Servo 2 PWM signal |

**Servo Configuration:**
- **Servo 1:**
  - Neutral Position: 87°
  - Control Range: ±35° (52° - 122°)
  - Input Range: -35 to +35 (relative to neutral)

- **Servo 2:**
  - Neutral Position: 92°
  - Control Range: ±35° (57° - 127°)
  - Input Range: -35 to +35 (relative to neutral)

---

### LS7166 Encoder (SPI Interface)

| Pin | Function | Description |
|-----|----------|-------------|
| 8 | LS7166_CS1 | Encoder 1 (Motor 1) Chip Select |
| 9 | LS7166_CS2 | Encoder 2 (Motor 2) Chip Select |
| 50 | MISO | SPI Master In Slave Out |
| 51 | MOSI | SPI Master Out Slave In |
| 52 | SCK | SPI Clock |

**SPI Configuration:**
- Mode: SPI_MODE0
- Clock: SPI_CLOCK_DIV16 (1 MHz @ 16MHz)
- Bit Order: MSBFIRST
- Data Width: 32-bit (4 bytes)

**Encoder Features:**
- Quadrature Mode: 4X
- Resolution: 344 pulses/meter (configurable)
- Direction Reversal: Software configurable

---

## I2C Communication

| Pin | Function | Description |
|-----|----------|-------------|
| 20 | SDA | I2C Data (Jetson Nano/Orin interface) |
| 21 | SCL | I2C Clock (Jetson Nano/Orin interface) |

**I2C Configuration:**
- Slave Address: 0x08
- Role: Slave (receives commands from Jetson)
- Protocol: Custom motor/servo control protocol

---

## Configuration Settings

### Direction Reversal
```cpp
ENCODER1_REVERSE = true   // Encoder 1 direction reversed
ENCODER2_REVERSE = false  // Encoder 2 normal direction
MOTOR1_REVERSE = false    // Motor 1 normal direction
MOTOR2_REVERSE = false    // Motor 2 normal direction
```

### Encoder Calibration
```cpp
m_1_pulse = 344          // Motor 1: 344 pulses per meter
m_2_pulse = 344          // Motor 2: 344 pulses per meter
```

### Control Frequencies
- Motor Control Task: 50 Hz (20 ms period)
- Serial Debug Output: 1 Hz (1 second period)
- Watchdog Timeout: 2 seconds

---

## Pin Usage Summary

| Pin Range | Usage | Count |
|-----------|-------|-------|
| 2-5 | Motor PWM | 4 |
| 6-7 | Servo PWM | 2 |
| 8-9 | SPI CS (Encoders) | 2 |
| 20-21 | I2C | 2 |
| 50-52 | SPI Bus | 3 |

**Total Pins Used: 13 pins**

---

## Hardware Connections

### Motor Driver Interface
Connect H-Bridge motor driver:
- IN1 → MOTOR1_EN1 (Pin 2)
- IN2 → MOTOR1_EN2 (Pin 3)
- IN3 → MOTOR2_EN1 (Pin 4)
- IN4 → MOTOR2_EN2 (Pin 5)

### LS7166 Encoder Interface
Connect both LS7166 chips:
- MOSI → Pin 51 (shared)
- MISO → Pin 50 (shared)
- SCK → Pin 52 (shared)
- CS1 → Pin 8 (Encoder 1)
- CS2 → Pin 9 (Encoder 2)
- VCC → 5V
- GND → GND

### RC Servo Interface
- Servo 1 Signal → Pin 6
- Servo 2 Signal → Pin 7
- VCC → 5-6V (external power recommended)
- GND → Common ground

### Jetson Nano/Orin Interface (I2C)
- SDA → Pin 20
- SCL → Pin 21
- Common ground required

---

## Notes

1. **PWM Frequency:** Arduino default (~490 Hz on pins 2-5)
2. **Servo Power:** Use external 5-6V power supply for servos (not Arduino power)
3. **Encoder Power:** LS7166 chips use 5V from Arduino
4. **I2C Pull-ups:** External 4.7kΩ pull-ups recommended for long cables
5. **Watchdog:** 2-second hardware watchdog enabled for safety

---

**Document Version:** 1.0
**Last Updated:** 2025-01-06
**Hardware:** Arduino Mega 2560
**Firmware:** aMAP_Powerpack_mini_V2
