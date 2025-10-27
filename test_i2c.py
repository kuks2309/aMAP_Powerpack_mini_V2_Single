#!/usr/bin/env python3
"""
aMAP Powerpack Mini V2 I2C Communication Test Script
Usage: python3 test_i2c.py
"""

import smbus
import struct
import time
import sys
import fcntl
import io

class MotorController:
    def __init__(self, i2c_bus=7, address=0x08):
        """
        Initialize Motor Controller
        Args:
            i2c_bus: I2C bus number (default: 7 for Jetson Orin Nano)
            address: I2C slave address (default: 0x08)
        """
        try:
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = address
            self.pulse_per_m = 16600  # Calibrated: 20cm actual movement = 3320 pulses
            print(f"✓ I2C initialized on bus {i2c_bus}, address 0x{address:02X}")
        except Exception as e:
            print(f"✗ Failed to initialize I2C: {e}")
            sys.exit(1)

    def set_pwm(self, pwm):
        """PWM 제어 (-255 ~ 255)"""
        pwm = max(-255, min(255, pwm))
        data = list(struct.pack('>h', pwm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x00] + data)
        print(f"→ PWM set to {pwm}")

    def set_speed(self, speed_mms):
        """속도 제어 (mm/s)"""
        data = list(struct.pack('>h', speed_mms))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x01] + data)
        print(f"→ Speed set to {speed_mms} mm/s")

    def set_position(self, position_mm):
        """위치 제어 - 절대 위치 (mm)"""
        data = list(struct.pack('>i', position_mm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x02] + data)
        print(f"→ Position set to {position_mm} mm (absolute)")

    def set_position_relative(self, distance_mm):
        """위치 제어 - 상대 위치 (mm)"""
        data = list(struct.pack('>i', distance_mm))
        self.bus.write_i2c_block_data(self.addr, 0x02, [0x03] + data)
        print(f"→ Position set to {distance_mm:+d} mm (relative)")

    def set_servo(self, angle):
        """서보 제어 (-35 ~ 35도)"""
        angle = max(-35, min(35, angle))
        angle_byte = angle & 0xFF
        self.bus.write_i2c_block_data(self.addr, 0x11, [angle_byte])
        print(f"→ Servo set to {angle}°")

    def get_status(self):
        """상태 조회 (Motor1 only - 7 bytes)"""
        # Simplified protocol: Arduino always returns status on read
        # No command byte needed
        data = self.bus.read_i2c_block_data(self.addr, 0x00, 7)

        position = struct.unpack('>i', bytes(data[0:4]))[0]
        speed = struct.unpack('>h', bytes(data[4:6]))[0]
        mode = data[6]

        # 단위 변환
        position_mm = position * 1000.0 / self.pulse_per_m
        speed_mms = speed * 50.0 * 1000.0 / self.pulse_per_m

        # Mode validation
        mode_names = {0: 'PWM', 1: 'Speed', 2: 'Position'}
        mode_str = mode_names.get(mode, f'Unknown({mode})')

        return {
            'position_pulse': position,
            'position_mm': position_mm,
            'speed_pulse': speed,
            'speed_mms': speed_mms,
            'mode': mode,
            'mode_str': mode_str,
            'raw_data': data  # 디버깅용 (7 bytes)
        }

    def reset_encoders(self):
        """엔코더 리셋"""
        self.bus.write_byte(self.addr, 0x20)
        print("→ Encoders reset")

    def print_status(self):
        """상태 출력"""
        status = self.get_status()
        print(f"← Position: {status['position_mm']:8.2f} mm ({status['position_pulse']:6d} pulses)")
        print(f"← Speed:    {status['speed_mms']:8.2f} mm/s ({status['speed_pulse']:4d} pulses/20ms)")
        print(f"← Mode:     {status['mode_str']}")
        print(f"← Raw data: {[f'0x{b:02X}' for b in status['raw_data']]}")


def test_basic():
    """기본 I2C 통신 테스트"""
    print("\n=== Basic I2C Communication Test ===\n")

    motor = MotorController()

    # 엔코더 리셋
    print("\n1. Resetting encoders...")
    motor.reset_encoders()
    time.sleep(0.1)
    motor.print_status()

    # PWM 테스트
    print("\n2. PWM Test (50 for 1 sec)...")
    motor.set_pwm(50)
    time.sleep(1.0)
    motor.set_pwm(0)
    motor.print_status()

    # 상태 조회
    print("\n3. Status check...")
    motor.print_status()


def test_speed_control():
    """속도 제어 테스트"""
    print("\n=== Speed Control Test ===\n")

    motor = MotorController()

    motor.reset_encoders()
    time.sleep(0.1)

    print("Moving at 200 mm/s for 2 seconds...")
    motor.set_speed(200)

    for i in range(20):
        time.sleep(0.1)
        motor.print_status()

    print("\nStopping...")
    motor.set_speed(0)
    time.sleep(0.5)
    motor.print_status()


def test_position_control():
    """위치 제어 테스트 - 절대 위치"""
    print("\n=== Position Control Test (Absolute) ===\n")

    motor = MotorController()

    motor.reset_encoders()
    time.sleep(0.1)

    target = 200  # 200mm
    print(f"Moving to {target} mm...")
    motor.set_position(target)

    while True:
        status = motor.get_status()
        error = abs(status['position_mm'] - target)

        print(f"Position: {status['position_mm']:7.2f} mm, Error: {error:6.2f} mm", end='\r')

        if error < 10:  # Allow 10mm tolerance
            print()
            print(f"\n✓ Target reached! (error < 10mm)")
            break

        time.sleep(0.1)

    motor.print_status()


def test_position_relative_control():
    """위치 제어 테스트 - 상대 위치"""
    print("\n=== Position Control Test (Relative) ===\n")

    motor = MotorController()

    motor.reset_encoders()
    time.sleep(0.1)

    # Test 1: Move forward 100mm
    print("Test 1: Moving forward 100mm...")
    initial_pos = motor.get_status()['position_mm']
    motor.set_position_relative(100)
    time.sleep(2.0)
    status1 = motor.get_status()
    print(f"  Initial: {initial_pos:7.2f} mm → Final: {status1['position_mm']:7.2f} mm")
    print(f"  Actual movement: {status1['position_mm'] - initial_pos:7.2f} mm\n")

    time.sleep(1.0)

    # Test 2: Move forward another 50mm
    print("Test 2: Moving forward 50mm...")
    initial_pos = status1['position_mm']
    motor.set_position_relative(50)
    time.sleep(2.0)
    status2 = motor.get_status()
    print(f"  Initial: {initial_pos:7.2f} mm → Final: {status2['position_mm']:7.2f} mm")
    print(f"  Actual movement: {status2['position_mm'] - initial_pos:7.2f} mm\n")

    time.sleep(1.0)

    # Test 3: Move backward 100mm
    print("Test 3: Moving backward 100mm...")
    initial_pos = status2['position_mm']
    motor.set_position_relative(-100)
    time.sleep(2.0)
    status3 = motor.get_status()
    print(f"  Initial: {initial_pos:7.2f} mm → Final: {status3['position_mm']:7.2f} mm")
    print(f"  Actual movement: {status3['position_mm'] - initial_pos:7.2f} mm\n")

    motor.print_status()


def test_servo_control():
    """서보 제어 테스트"""
    print("\n=== Servo Control Test ===\n")

    motor = MotorController()

    print("Scanning servo from -35° to +35°...")
    for angle in range(-35, 36, 5):
        motor.set_servo(angle)
        time.sleep(0.2)

    print("\nReturning to neutral (0°)...")
    motor.set_servo(0)


def interactive_mode():
    """대화형 제어 모드"""
    print("\n=== Interactive Control Mode ===\n")
    print("Commands:")
    print("  p <value>   : PWM control (-255 to 255)")
    print("  s <value>   : Speed control (mm/s)")
    print("  m <value>   : Position control - absolute (mm)")
    print("  d <value>   : Position control - relative (mm)")
    print("  v <value>   : Servo control (-35 to 35 degrees)")
    print("  r           : Reset encoders")
    print("  ?           : Get status")
    print("  q           : Quit")
    print()

    motor = MotorController()

    while True:
        try:
            cmd = input(">> ").strip().split()

            if not cmd:
                continue

            if cmd[0] == 'q':
                print("Stopping motor...")
                motor.set_pwm(0)
                break

            elif cmd[0] == '?':
                motor.print_status()

            elif cmd[0] == 'r':
                motor.reset_encoders()

            elif cmd[0] == 'p' and len(cmd) == 2:
                motor.set_pwm(int(cmd[1]))

            elif cmd[0] == 's' and len(cmd) == 2:
                motor.set_speed(int(cmd[1]))

            elif cmd[0] == 'm' and len(cmd) == 2:
                motor.set_position(int(cmd[1]))

            elif cmd[0] == 'd' and len(cmd) == 2:
                motor.set_position_relative(int(cmd[1]))

            elif cmd[0] == 'v' and len(cmd) == 2:
                motor.set_servo(int(cmd[1]))

            else:
                print("Unknown command. Type 'q' to quit.")

        except KeyboardInterrupt:
            print("\nStopping motor...")
            motor.set_pwm(0)
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    print("=" * 60)
    print("  aMAP Powerpack Mini V2 I2C Test")
    print("=" * 60)

    if len(sys.argv) > 1:
        test_mode = sys.argv[1]
    else:
        print("\nSelect test mode:")
        print("  1. Basic test")
        print("  2. Speed control test")
        print("  3. Position control test (absolute)")
        print("  4. Position control test (relative)")
        print("  5. Servo control test")
        print("  6. Interactive mode")
        choice = input("\nEnter choice (1-6): ").strip()
        test_mode = choice

    if test_mode == '1':
        test_basic()
    elif test_mode == '2':
        test_speed_control()
    elif test_mode == '3':
        test_position_control()
    elif test_mode == '4':
        test_position_relative_control()
    elif test_mode == '5':
        test_servo_control()
    elif test_mode == '6':
        interactive_mode()
    else:
        print("Invalid choice!")
        sys.exit(1)

    print("\nTest completed!")


if __name__ == '__main__':
    main()
