#!/usr/bin/env python3
"""
Simple I2C Test - Using i2ctransfer for pure I2C read
"""

import subprocess
import struct
import time

I2C_BUS = 7
I2C_ADDR = 0x08

def send_command(cmd_byte):
    """Send single byte command"""
    result = subprocess.run(
        ['i2cset', '-y', str(I2C_BUS), f'0x{I2C_ADDR:02X}', f'0x{cmd_byte:02X}'],
        capture_output=True, text=True
    )
    return result.returncode == 0

def read_status():
    """Read 7 bytes using i2ctransfer (no command needed, simplified protocol)"""
    # Simply read 7 bytes - Arduino always returns current status
    result = subprocess.run(
        ['i2ctransfer', '-y', str(I2C_BUS), f'r7@0x{I2C_ADDR:02X}'],
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print(f"Error: {result.stderr}")
        return None

    # Parse output (format: "0x00 0x01 0x02 ...")
    hex_values = result.stdout.strip().split()
    data = [int(x, 16) for x in hex_values]

    return data

def parse_status(data):
    """Parse 7-byte status response"""
    if len(data) != 7:
        print(f"Error: Expected 7 bytes, got {len(data)}")
        return None

    # Parse Motor1 status
    position = struct.unpack('>i', bytes(data[0:4]))[0]
    speed = struct.unpack('>h', bytes(data[4:6]))[0]
    mode = data[6]

    # Convert units
    pulse_per_m = 11500
    position_mm = position * 1000.0 / pulse_per_m
    speed_mms = speed * 50.0 * 1000.0 / pulse_per_m

    mode_names = {0: 'PWM', 1: 'Speed', 2: 'Position'}
    mode_str = mode_names.get(mode, f'Unknown({mode})')

    return {
        'position_pulse': position,
        'position_mm': position_mm,
        'speed_pulse': speed,
        'speed_mms': speed_mms,
        'mode': mode,
        'mode_str': mode_str,
        'raw_data': data
    }

def main():
    print("=" * 60)
    print("  Simple I2C Status Test (using i2ctransfer)")
    print("=" * 60)
    print()

    # Test 1: Read status
    print("1. Reading status...")
    data = read_status()
    if data:
        print(f"   Raw data: {[f'0x{b:02X}' for b in data]}")
        status = parse_status(data)
        if status:
            print(f"   Position: {status['position_mm']:8.2f} mm ({status['position_pulse']:6d} pulses)")
            print(f"   Speed:    {status['speed_mms']:8.2f} mm/s ({status['speed_pulse']:4d} pulses/20ms)")
            print(f"   Mode:     {status['mode_str']}")
    print()

    # Test 2: Send PWM command
    print("2. Sending PWM 100...")
    result = subprocess.run(
        ['i2ctransfer', '-y', str(I2C_BUS), f'w4@0x{I2C_ADDR:02X}', '0x02', '0x00', '0x00', '0x64'],
        capture_output=True, text=True
    )
    print(f"   Command sent: {'OK' if result.returncode == 0 else 'FAILED'}")
    time.sleep(0.5)
    print()

    # Test 3: Read status again
    print("3. Reading status after PWM command...")
    data = read_status()
    if data:
        print(f"   Raw data: {[f'0x{b:02X}' for b in data]}")
        status = parse_status(data)
        if status:
            print(f"   Position: {status['position_mm']:8.2f} mm ({status['position_pulse']:6d} pulses)")
            print(f"   Speed:    {status['speed_mms']:8.2f} mm/s ({status['speed_pulse']:4d} pulses/20ms)")
            print(f"   Mode:     {status['mode_str']}")
    print()

    # Test 4: Stop motor
    print("4. Stopping motor (PWM 0)...")
    result = subprocess.run(
        ['i2ctransfer', '-y', str(I2C_BUS), f'w4@0x{I2C_ADDR:02X}', '0x02', '0x00', '0x00', '0x00'],
        capture_output=True, text=True
    )
    print(f"   Command sent: {'OK' if result.returncode == 0 else 'FAILED'}")
    print()

    print("Test completed!")

if __name__ == '__main__':
    main()
