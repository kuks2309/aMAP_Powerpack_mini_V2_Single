# aMAP Powerpack Mini V2 I2C 통신 프로토콜

## 기본 설정
- **I2C 슬레이브 주소**: `0x08`
- **통신 방향**: Jetson (Master) ↔ Arduino (Slave)
- **데이터 형식**: Big Endian
- **제어 주기**: 50Hz (20ms)

## 제어 모드 (RC/I2C 우선순위)

시스템은 SBUS 리모컨의 채널 6에서 읽은 스위치 위치에 따라 3가지 주행 모드를 지원합니다:

| 모드 | 모터 제어 | 서보 제어 | I2C 명령 처리 |
|------|----------|----------|--------------|
| **MANUAL** (수동) | RC 조이스틱 | RC 조이스틱 | **모터/서보 명령 무시** |
| **SEMI_AUTO** (반자동) | I2C (자율주행) | RC 조이스틱 | 모터 명령만 수신, **서보 명령 무시** |
| **FULL_AUTO** (완전자동) | I2C (자율주행) | I2C (자율주행) | 모든 I2C 명령 수신 |

**중요**:
- MANUAL 모드에서는 모든 I2C 제어 명령(모터/서보)이 무시되고 조이스틱으로만 제어됩니다.
- SEMI_AUTO 모드에서는 모터는 I2C로 제어되지만, 서보 I2C 명령은 무시되고 조이스틱으로만 제어됩니다.
- FULL_AUTO 모드에서만 모든 I2C 제어가 가능합니다.
- RC가 연결되지 않은 경우, 모든 I2C 명령이 정상적으로 처리됩니다.

---

## 명령어 체계

### 1. 모터 제어 명령 (Motor Control)

#### 1.1 단일 모터 제어 (Motor1)
**명령 코드**: `0x02`

| 모드 | 설명 | 데이터 형식 | 예시 |
|------|------|------------|------|
| `0x00` PWM | 직접 PWM 제어 | `[0x02][0x00][PWM(2B)]`<br>범위: -255 ~ 255 (int16) | `[0x02][0x00][0x00][0x64]` = PWM 100 |
| `0x01` Speed | 속도 PID 제어 | `[0x02][0x01][Speed(2B)]`<br>단위: mm/s (int16) | `[0x02][0x01][0x00][0x96]` = 150 mm/s |
| `0x02` Position | 위치 PID 제어 (절대) | `[0x02][0x02][Position(4B)]`<br>단위: mm (int32) | `[0x02][0x02][0x00][0x00][0x03][0xE8]` = 1000 mm |
| `0x03` Position Relative | 위치 PID 제어 (상대) | `[0x02][0x03][Distance(4B)]`<br>단위: mm (int32) | `[0x02][0x03][0x00][0x00][0x00][0x64]` = +100 mm |

#### 제어 모드 상세

##### PWM 모드 (`0x00`)
- 피드백 없는 직접 제어
- 속도: -255(최대 역방향) ~ 255(최대 정방향)
- 실시간 응답
- 사용 예: 수동 조작, 개루프 제어

##### Speed 모드 (`0x01`)
- PID 피드백 제어 (50Hz)
- 목표: mm/s 단위 속도
- **PID 게인**:
  - Kp = 4.5
  - Ki = 0.0
  - Kd = 7.0
- 출력 제한: ±150
- 사용 예: 일정 속도 주행

##### Position 모드 (`0x02`) - 절대 위치
- PID 피드백 제어
- 목표: mm 단위 절대 위치
- **PID 게인**:
  - Kp = 0.08
  - Ki = 0.03 (Integral limit = 1000)
  - Kd = 1.0
- 출력 제한: ±255
- 사용 예: 특정 위치로 이동 (예: 원점 기준 500mm)

##### Position Relative 모드 (`0x03`) - 상대 위치
- PID 피드백 제어
- 목표: mm 단위 상대 이동 거리 (현재 위치 기준)
- **PID 게인**: Position 모드와 동일
  - Kp = 0.08
  - Ki = 0.03 (Integral limit = 1000)
  - Kd = 1.0
- 출력 제한: ±255
- 사용 예: 현재 위치에서 +100mm 이동, -50mm 이동

---

### 2. 서보 제어 명령 (Servo Control)

#### 2.1 Servo1 제어
**명령 코드**: `0x11`

**형식**: `[0x11][Angle(1B)]`

| 항목 | 값 |
|------|-----|
| 데이터 타입 | int8_t |
| 각도 범위 | -35° ~ +35° (중립 위치 기준) |
| 중립 위치 | 87° |
| 실제 범위 | 52° ~ 122° |
| 제어 핀 | Pin 6 (PWM) |

**예시**:
```
[0x11][0x14]  // +20° (중립 + 20 = 107°)
[0x11][0xEC]  // -20° (중립 - 20 = 67°)
[0x11][0x00]  // 0° (중립 = 87°)
```

---

### 3. 시스템 명령 (System Commands)

#### 3.1 엔코더 리셋
**명령 코드**: `0x20`

**형식**: `[0x20]` (데이터 없음)

- 모든 엔코더 값을 0으로 리셋
- 응답 없음

#### 3.2 상태 조회
**명령 코드**: `0x30`

**형식**: `[0x30]` (데이터 없음)

**응답**: 7 바이트

| 바이트 | 필드 | 타입 | 설명 |
|--------|------|------|------|
| 0-3 | Position | int32 | 현재 위치 (펄스, Big Endian) |
| 4-5 | Speed | int16 | 현재 속도 (펄스/20ms, Big Endian) |
| 6 | Mode | uint8 | 현재 모드 (0:PWM, 1:Speed, 2:Position) |

**참고**: 상대 위치 모드(0x03)로 명령을 보내면 Arduino는 자동으로 절대 위치 제어(Mode 2)로 전환하여 계산된 목표 위치로 이동합니다.

---

## Python 예제 코드 (Jetson)

### 기본 설정
```python
import smbus
import struct
import time

bus = smbus.SMBus(7)  # I2C bus 7 (Jetson Orin Nano)
ADDR = 0x08
```

### 1. PWM 제어
```python
# Motor1: PWM 100 (정방향)
bus.write_i2c_block_data(ADDR, 0x02, [0x00, 0x00, 0x64])

# Motor1: PWM -50 (역방향)
bus.write_i2c_block_data(ADDR, 0x02, [0x00, 0xFF, 0xCE])

# Motor1: 정지
bus.write_i2c_block_data(ADDR, 0x02, [0x00, 0x00, 0x00])
```

### 2. 속도 제어
```python
# Motor1: 150 mm/s
speed = 150
data = list(struct.pack('>h', speed))
bus.write_i2c_block_data(ADDR, 0x02, [0x01] + data)

# Motor1: -100 mm/s (역방향)
speed = -100
data = list(struct.pack('>h', speed))
bus.write_i2c_block_data(ADDR, 0x02, [0x01] + data)
```

### 3. 위치 제어 (절대)
```python
# Motor1: 1000 mm 위치로 이동
position = 1000
data = list(struct.pack('>i', position))
bus.write_i2c_block_data(ADDR, 0x02, [0x02] + data)

# Motor1: -500 mm 위치로 이동
position = -500
data = list(struct.pack('>i', position))
bus.write_i2c_block_data(ADDR, 0x02, [0x02] + data)
```

### 3-1. 위치 제어 (상대)
```python
# Motor1: 현재 위치에서 +100 mm 이동
distance = 100
data = list(struct.pack('>i', distance))
bus.write_i2c_block_data(ADDR, 0x02, [0x03] + data)

# Motor1: 현재 위치에서 -200 mm 이동 (후진)
distance = -200
data = list(struct.pack('>i', distance))
bus.write_i2c_block_data(ADDR, 0x02, [0x03] + data)
```

### 4. 서보 제어
```python
# Servo1: +20도 (중립 기준)
bus.write_i2c_block_data(ADDR, 0x11, [20])

# Servo1: -20도 (중립 기준)
angle = -20
# int8_t를 unsigned byte로 변환
angle_byte = angle & 0xFF
bus.write_i2c_block_data(ADDR, 0x11, [angle_byte])

# Servo1: 중립 위치 (0도)
bus.write_i2c_block_data(ADDR, 0x11, [0])
```

### 5. 상태 조회
```python
# 상태 요청
bus.write_byte(ADDR, 0x30)
time.sleep(0.01)  # 응답 대기

# 상태 읽기 (7 bytes)
data = bus.read_i2c_block_data(ADDR, 0, 7)

# 파싱
position = struct.unpack('>i', bytes(data[0:4]))[0]
speed = struct.unpack('>h', bytes(data[4:6]))[0]
mode = data[6]

print(f"Position: {position} pulses")
print(f"Speed: {speed} pulses/20ms")
print(f"Mode: {mode} (0:PWM, 1:Speed, 2:Position)")

# 위치를 mm로 변환
position_mm = position * 1000.0 / 16600.0
print(f"Position: {position_mm:.2f} mm")

# 속도를 mm/s로 변환
speed_mms = speed * 50.0 * 1000.0 / 16600.0
print(f"Speed: {speed_mms:.2f} mm/s")
```

### 6. 엔코더 리셋
```python
bus.write_byte(ADDR, 0x20)
print("Encoders reset to 0")
```

### 7. 통합 제어 클래스
```python
class MotorController:
    def __init__(self, i2c_bus=7, address=0x08):
        self.bus = smbus.SMBus(i2c_bus)
        self.addr = address
        self.pulse_per_m = 16600  # Calibrated: 20cm = 3320 pulses

    def set_pwm(self, pwm):
        """PWM 제어 (-255 ~ 255)"""
        pwm = max(-255, min(255, pwm))
        data = list(struct.pack('>h', pwm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x00] + data)

    def set_speed(self, speed_mms):
        """속도 제어 (mm/s)"""
        data = list(struct.pack('>h', speed_mms))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x01] + data)

    def set_position(self, position_mm):
        """위치 제어 - 절대 (mm)"""
        data = list(struct.pack('>i', position_mm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x02] + data)

    def set_position_relative(self, distance_mm):
        """위치 제어 - 상대 (mm)"""
        data = list(struct.pack('>i', distance_mm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x03] + data)

    def set_servo(self, angle):
        """서보 제어 (-35 ~ 35도)"""
        angle = max(-35, min(35, angle))
        angle_byte = angle & 0xFF
        self.bus.write_i2c_block_data(self.addr, 0x11, [angle_byte])

    def get_status(self):
        """상태 조회"""
        self.bus.write_byte(self.addr, 0x30)
        time.sleep(0.01)
        data = self.bus.read_i2c_block_data(self.addr, 0, 7)

        position = struct.unpack('>i', bytes(data[0:4]))[0]
        speed = struct.unpack('>h', bytes(data[4:6]))[0]
        mode = data[6]

        # 단위 변환
        position_mm = position * 1000.0 / self.pulse_per_m
        speed_mms = speed * 50.0 * 1000.0 / self.pulse_per_m

        return {
            'position_pulse': position,
            'position_mm': position_mm,
            'speed_pulse': speed,
            'speed_mms': speed_mms,
            'mode': mode,
            'mode_str': ['PWM', 'Speed', 'Position'][mode]
        }

    def reset_encoders(self):
        """엔코더 리셋"""
        self.bus.write_byte(self.addr, 0x20)

# 사용 예시
motor = MotorController()

# PWM 제어
motor.set_pwm(100)

# 속도 제어
motor.set_speed(150)  # 150 mm/s

# 위치 제어 (절대)
motor.set_position(1000)  # 1000 mm 위치로 이동

# 위치 제어 (상대)
motor.set_position_relative(100)  # 현재 위치에서 +100 mm 이동

# 서보 제어
motor.set_servo(20)  # +20도

# 상태 조회
status = motor.get_status()
print(status)

# 엔코더 리셋
motor.reset_encoders()
```

---

## 하드웨어 사양

### 엔코더 캘리브레이션
- **1m당 펄스**: 16,600 pulses (실측: 20cm = 3320 pulses)
- **1 펄스당 거리**: 0.060 mm (1/16600 m)
- **제어 주기**: 50Hz (20ms)
- **엔코더 타입**: LS7166 (SPI 인터페이스)

### 핀 맵
| 기능 | 핀 번호 | 설명 |
|------|---------|------|
| Motor1 EN1 | 2 | H-Bridge 방향 제어 1 |
| Motor1 EN2 | 3 | H-Bridge 방향 제어 2 |
| Servo1 PWM | 6 | 서보 PWM 신호 |
| Encoder1 CS | 8 | SPI Chip Select |
| SPI MOSI | 51 | SPI 데이터 출력 (Mega 2560) |
| SPI MISO | 50 | SPI 데이터 입력 (Mega 2560) |
| SPI SCK | 52 | SPI 클럭 (Mega 2560) |
| I2C SDA | 20 | I2C 데이터 (Mega 2560) |
| I2C SCL | 21 | I2C 클럭 (Mega 2560) |

### 방향 설정
- **Encoder1 방향 반전**: Enabled (`ENCODER1_REVERSE = true`)
- **Motor1 방향 반전**: Disabled (`MOTOR1_REVERSE = false`)

---

## 주요 특징

### 1. FreeRTOS 기반 멀티태스킹
- **TaskMotorControl**: 50Hz 모터 제어 루프 (우선순위 3 - 최고)
- **TaskSerialCommunication**: 20Hz 디버그 출력 (우선순위 2)

### 2. Watchdog Timer
- **타임아웃**: 2초
- 제어 루프가 2초 이상 멈추면 자동 리셋
- `WATCHDOG_ENABLE`로 활성화/비활성화 가능

### 3. Mutex 보호
- 멀티태스크 환경에서 데이터 안전성 보장
- `xMotor1Semaphore`: 모터 데이터 보호
- `xSerialSemaphore`: 시리얼 통신 보호

### 4. PID 제어
- 속도/위치 자동 제어 지원
- Anti-windup 기능 (적분 리미트)
- 동적 파라미터 조정 가능

### 5. 디버그 출력
- **시리얼 포트**: 115200 baud
- **출력 주기**: 50ms (20Hz)
- **출력 형식**: `M1: Pos=XXX Speed=XXX Mode=X | S1=XXX`

---

## 단위 변환 공식

### 위치 변환
```
펄스 → mm:  position_mm = position_pulse × (1000 / 16600)
mm → 펄스:  position_pulse = position_mm × (16600 / 1000)
```

### 속도 변환
```
펄스/20ms → mm/s:  speed_mms = speed_pulse × 50 × (1000 / 16600)
mm/s → 펄스/20ms:  speed_pulse = speed_mms × (16600 / 1000) / 50
```

---

## 트러블슈팅

### 1. I2C 통신 안됨
- I2C 주소 확인: `0x08`
- SDA/SCL 연결 확인 (Mega 2560: Pin 20/21)
- Pull-up 저항 확인 (일반적으로 4.7kΩ)

### 2. 엔코더 값이 -1 (0xFFFFFFFF)
- SPI 통신 실패
- 확인 사항:
  - MOSI (Pin 51) → LS7166 DIN
  - MISO (Pin 50) → LS7166 DOUT
  - SCK (Pin 52) → LS7166 CLK
  - CS (Pin 8) → LS7166 CS
  - LS7166 전원 (VCC/GND)

### 3. 모터가 반대로 움직임
- `MOTOR1_REVERSE` 플래그 토글
- 또는 PWM/속도 값에 음수 부호 적용

### 4. 엔코더 방향이 반대
- `ENCODER1_REVERSE` 플래그 토글

### 5. PID 제어가 불안정
- PID 게인 조정:
  - Kp: 응답 속도 (높을수록 빠름, 과도하면 진동)
  - Ki: 정상상태 오차 제거 (과도하면 오버슈트)
  - Kd: 댐핑 (높을수록 안정, 과도하면 느림)

---

## 예제: 기본 주행 시나리오

### 1. 초기화 및 직진 주행
```python
import time

motor = MotorController()

# 엔코더 리셋
motor.reset_encoders()
time.sleep(0.1)

# 속도 제어로 1초간 직진
motor.set_speed(200)  # 200 mm/s
time.sleep(1.0)

# 정지
motor.set_speed(0)

# 현재 위치 확인
status = motor.get_status()
print(f"이동 거리: {status['position_mm']:.2f} mm")
```

### 2. 특정 거리 이동 (절대 위치)
```python
# 위치 제어로 500mm 이동
motor.reset_encoders()
motor.set_position(500)

# 목표 도달까지 대기
while True:
    status = motor.get_status()
    error = abs(status['position_mm'] - 500)
    print(f"현재: {status['position_mm']:.2f} mm, 오차: {error:.2f} mm")

    if error < 5:  # 5mm 이내 도달
        break

    time.sleep(0.1)

print("목표 도달!")
motor.set_speed(0)  # PWM 모드로 정지
```

### 2-1. 증분 이동 (상대 위치)
```python
# 상대 위치 제어로 단계적 이동
motor.reset_encoders()

# 1단계: 앞으로 100mm 이동
motor.set_position_relative(100)
time.sleep(2.0)
status = motor.get_status()
print(f"1단계 완료: {status['position_mm']:.2f} mm")

# 2단계: 추가로 앞으로 50mm 이동
motor.set_position_relative(50)
time.sleep(2.0)
status = motor.get_status()
print(f"2단계 완료: {status['position_mm']:.2f} mm")

# 3단계: 뒤로 100mm 이동
motor.set_position_relative(-100)
time.sleep(2.0)
status = motor.get_status()
print(f"3단계 완료: {status['position_mm']:.2f} mm")
```

### 3. 서보 스캔
```python
# 서보를 좌우로 스캔
for angle in range(-35, 36, 5):
    motor.set_servo(angle)
    time.sleep(0.1)

# 중립 위치로 복귀
motor.set_servo(0)
```

---

## 참고 문서
- [aMAP_Powerpack_Mini_V2_pin_map.md](aMAP_Powerpack_Mini_V2_pin_map.md) - 핀 맵 상세
- [I2CComm.cpp](I2CComm.cpp) - I2C 프로토콜 구현
- [I2CComm.h](I2CComm.h) - I2C 인터페이스 정의
- [aMAP_Powerpack_mini_V2_Single.ino](aMAP_Powerpack_mini_V2_Single.ino) - 메인 펌웨어

---

## 버전 정보
- **프로젝트**: aMAP Powerpack Mini V2 Single
- **펌웨어**: Arduino Mega 2560 + FreeRTOS
- **프로토콜 버전**: 1.0
- **최종 수정일**: 2025-10-26
