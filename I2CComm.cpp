#include "I2CComm.h"

/*
 * ============================================================================
 * I2C Communication Protocol for Motor & Servo Control
 * ============================================================================
 *
 * Default I2C Address: 0x08
 *
 * ============================================================================
 * COMMAND FORMAT (Jetson → Arduino)
 * ============================================================================
 *
 * 1. DUAL MOTOR CONTROL (Both motors simultaneously)
 *    Command: 0x01
 *    Format: [0x01][Mode][Motor1 Data][Motor2 Data]
 *
 *    Mode 0x00 - PWM Control:
 *      Data: 4 bytes [M1_PWM(2B)][M2_PWM(2B)]
 *      Range: -255 to 255 (int16)
 *      Example: [0x01][0x00][0x00][0x64][0xFF][0xCE]  // M1=100, M2=-50
 *
 *    Mode 0x01 - Speed Control:
 *      Data: 4 bytes [M1_Speed(2B)][M2_Speed(2B)]
 *      Range: target speed in encoder counts/100ms (int16)
 *      Example: [0x01][0x01][0x00][0x96][0x00][0xC8]  // M1=150, M2=200
 *
 *    Mode 0x02 - Position Control:
 *      Data: 8 bytes [M1_Pos(4B)][M2_Pos(4B)]
 *      Range: target position in encoder counts (int32)
 *      Example: [0x01][0x02][0x00][0x00][0x03][0xE8][0x00][0x00][0x07][0xD0]
 *               // M1=1000, M2=2000
 *
 * 2. SINGLE MOTOR CONTROL
 *    Motor1: 0x02 [Mode][Data 2B or 4B]
 *    Motor2: 0x03 [Mode][Data 2B or 4B]
 *    Mode same as dual motor (0x00: PWM, 0x01: Speed, 0x02: Position)
 *
 * 3. SERVO CONTROL
 *    Both Servos: 0x10 [Servo1_Angle(1B)][Servo2_Angle(1B)]
 *    Servo1 only: 0x11 [Angle(1B)]  // -35 to +35 degrees (int8_t)
 *    Servo2 only: 0x12 [Angle(1B)]  // -35 to +35 degrees (int8_t)
 *    Note: Input angle is relative to neutral position (SERVO1_NEUTRAL, SERVO2_NEUTRAL)
 *    Example: [0x10][0x14][0xEC]  // S1=+20°, S2=-20° (relative to neutral)
 *
 * 4. SYSTEM COMMANDS
 *    Reset Encoders: 0x20 (no data)
 *    Get Status: 0x30 (no data, returns 14 bytes)
 *
 * ============================================================================
 * RESPONSE FORMAT (Arduino → Jetson)
 * ============================================================================
 *
 * Status Response (after 0x30 command): 14 bytes total
 *   [M1_Position: 4B][M1_Speed: 2B][M1_Mode: 1B]
 *   [M2_Position: 4B][M2_Speed: 2B][M2_Mode: 1B]
 *
 *   M1_Position (int32): Motor1 encoder position
 *   M1_Speed (int16): Motor1 current speed (counts/100ms)
 *   M1_Mode (uint8): Motor1 mode (0:PWM, 1:Speed, 2:Position)
 *   M2_Position (int32): Motor2 encoder position
 *   M2_Speed (int16): Motor2 current speed (counts/100ms)
 *   M2_Mode (uint8): Motor2 mode (0:PWM, 1:Speed, 2:Position)
 *
 * ============================================================================
 * PYTHON EXAMPLE (Jetson Nano/Orin)
 * ============================================================================
 *
 * import smbus
 * import struct
 *
 * bus = smbus.SMBus(1)
 * ADDR = 0x08
 *
 * # Dual motor PWM control (M1=100, M2=-50)
 * bus.write_i2c_block_data(ADDR, 0x01, [0x00, 0x00, 0x64, 0xFF, 0xCE])
 *
 * # Dual motor speed control (M1=150, M2=200)
 * bus.write_i2c_block_data(ADDR, 0x01, [0x01, 0x00, 0x96, 0x00, 0xC8])
 *
 * # Both servos (S1=90°, S2=45°)
 * bus.write_i2c_block_data(ADDR, 0x10, [90, 45])
 *
 * # Get status
 * bus.write_byte(ADDR, 0x30)
 * data = bus.read_i2c_block_data(ADDR, 0, 14)
 * m1_pos = struct.unpack('>i', bytes(data[0:4]))[0]
 * m1_spd = struct.unpack('>h', bytes(data[4:6]))[0]
 * m1_mode = data[6]
 *
 * ============================================================================
 */

// Initialize static instance pointer
I2CComm* I2CComm::_instance = nullptr;

I2CComm::I2CComm()
{
    _dualMotorCallback = nullptr;
    _motor1Callback = nullptr;
    _motor2Callback = nullptr;
    _dualServoCallback = nullptr;
    _servo1Callback = nullptr;
    _servo2Callback = nullptr;
    _resetEncodersCallback = nullptr;

    _motor1Status = {0, 0, 0};
    _motor2Status = {0, 0, 0};
    _lastCommand = 0;

    _instance = this;
}

void I2CComm::begin(uint8_t address)
{
    Wire.begin(address);
    Wire.onReceive(onReceiveHandler);
    Wire.onRequest(onRequestHandler);
}

// Callback setters
void I2CComm::setDualMotorCallback(void (*callback)(uint8_t mode, int16_t m1_data_s, int32_t m1_data_l, int16_t m2_data_s, int32_t m2_data_l))
{
    _dualMotorCallback = callback;
}

void I2CComm::setMotor1Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l))
{
    _motor1Callback = callback;
}

void I2CComm::setMotor2Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l))
{
    _motor2Callback = callback;
}

void I2CComm::setDualServoCallback(void (*callback)(int8_t servo1_angle, int8_t servo2_angle))
{
    _dualServoCallback = callback;
}

void I2CComm::setServo1Callback(void (*callback)(int8_t angle))
{
    _servo1Callback = callback;
}

void I2CComm::setServo2Callback(void (*callback)(int8_t angle))
{
    _servo2Callback = callback;
}

void I2CComm::setResetEncodersCallback(void (*callback)())
{
    _resetEncodersCallback = callback;
}

// Status setters
void I2CComm::setMotor1Status(int32_t position, int16_t speed, uint8_t mode)
{
    _motor1Status.position = position;
    _motor1Status.speed = speed;
    _motor1Status.mode = mode;
}

void I2CComm::setMotor2Status(int32_t position, int16_t speed, uint8_t mode)
{
    _motor2Status.position = position;
    _motor2Status.speed = speed;
    _motor2Status.mode = mode;
}

// Static handlers
void I2CComm::onReceiveHandler(int numBytes)
{
    if (_instance != nullptr)
    {
        _instance->onReceive(numBytes);
    }
}

void I2CComm::onRequestHandler()
{
    if (_instance != nullptr)
    {
        _instance->onRequest();
    }
}

// Instance receive handler
void I2CComm::onReceive(int numBytes)
{
    if (numBytes < 1)
        return;

    uint8_t command = Wire.read();
    _lastCommand = command;

    switch (command)
    {
    case CMD_DUAL_MOTOR: // 0x01 - Control both motors
    {
        if (numBytes >= 2)
        {
            uint8_t mode = Wire.read();
            int16_t m1_data_s = 0;
            int32_t m1_data_l = 0;
            int16_t m2_data_s = 0;
            int32_t m2_data_l = 0;

            if (mode == I2C_MODE_PWM || mode == I2C_MODE_SPEED)
            {
                // PWM or Speed mode: 2 bytes per motor
                if (numBytes >= 6) // cmd(1) + mode(1) + m1(2) + m2(2)
                {
                    m1_data_s = (Wire.read() << 8) | Wire.read();
                    m2_data_s = (Wire.read() << 8) | Wire.read();
                }
            }
            else if (mode == I2C_MODE_POSITION)
            {
                // Position mode: 4 bytes per motor
                if (numBytes >= 10) // cmd(1) + mode(1) + m1(4) + m2(4)
                {
                    m1_data_l = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) |
                                ((int32_t)Wire.read() << 8) | Wire.read();
                    m2_data_l = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) |
                                ((int32_t)Wire.read() << 8) | Wire.read();
                }
            }

            if (_dualMotorCallback != nullptr)
            {
                _dualMotorCallback(mode, m1_data_s, m1_data_l, m2_data_s, m2_data_l);
            }
        }
        break;
    }

    case CMD_MOTOR1_ONLY: // 0x02 - Control Motor1 only
    case CMD_MOTOR2_ONLY: // 0x03 - Control Motor2 only
    {
        if (numBytes >= 2)
        {
            uint8_t mode = Wire.read();
            int16_t data_s = 0;
            int32_t data_l = 0;

            if (mode == I2C_MODE_PWM || mode == I2C_MODE_SPEED)
            {
                if (numBytes >= 4) // cmd(1) + mode(1) + data(2)
                {
                    data_s = (Wire.read() << 8) | Wire.read();
                }
            }
            else if (mode == I2C_MODE_POSITION)
            {
                if (numBytes >= 6) // cmd(1) + mode(1) + data(4)
                {
                    data_l = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) |
                            ((int32_t)Wire.read() << 8) | Wire.read();
                }
            }

            if (command == CMD_MOTOR1_ONLY && _motor1Callback != nullptr)
            {
                _motor1Callback(mode, data_s, data_l);
            }
            else if (command == CMD_MOTOR2_ONLY && _motor2Callback != nullptr)
            {
                _motor2Callback(mode, data_s, data_l);
            }
        }
        break;
    }

    case CMD_DUAL_SERVO: // 0x10 - Control both servos
    {
        if (numBytes >= 3) // cmd(1) + servo1(1) + servo2(1)
        {
            int8_t servo1_angle = (int8_t)Wire.read();
            int8_t servo2_angle = (int8_t)Wire.read();
            if (_dualServoCallback != nullptr)
            {
                _dualServoCallback(servo1_angle, servo2_angle);
            }
        }
        break;
    }

    case CMD_SERVO1_ONLY: // 0x11 - Control Servo1 only
    {
        if (numBytes >= 2) // cmd(1) + angle(1)
        {
            int8_t angle = (int8_t)Wire.read();
            if (_servo1Callback != nullptr)
            {
                _servo1Callback(angle);
            }
        }
        break;
    }

    case CMD_SERVO2_ONLY: // 0x12 - Control Servo2 only
    {
        if (numBytes >= 2) // cmd(1) + angle(1)
        {
            int8_t angle = (int8_t)Wire.read();
            if (_servo2Callback != nullptr)
            {
                _servo2Callback(angle);
            }
        }
        break;
    }

    case CMD_RESET_ENCODERS: // 0x20
    {
        if (_resetEncodersCallback != nullptr)
        {
            _resetEncodersCallback();
        }
        break;
    }

    case CMD_GET_STATUS: // 0x30
        // Command stored in _lastCommand, will be handled in onRequest
        break;

    default:
        // Unknown command, flush remaining bytes
        while (Wire.available())
        {
            Wire.read();
        }
        break;
    }
}

// Instance request handler
void I2CComm::onRequest()
{
    switch (_lastCommand)
    {
    case CMD_GET_STATUS: // 0x30
    {
        // Send both motor statuses (14 bytes total)
        // Motor 1: position(4) + speed(2) + mode(1) = 7 bytes
        // Motor 2: position(4) + speed(2) + mode(1) = 7 bytes
        uint8_t data[14];

        // Motor 1 (Big Endian)
        data[0] = (_motor1Status.position >> 24) & 0xFF;
        data[1] = (_motor1Status.position >> 16) & 0xFF;
        data[2] = (_motor1Status.position >> 8) & 0xFF;
        data[3] = _motor1Status.position & 0xFF;
        data[4] = (_motor1Status.speed >> 8) & 0xFF;
        data[5] = _motor1Status.speed & 0xFF;
        data[6] = _motor1Status.mode;

        // Motor 2 (Big Endian)
        data[7] = (_motor2Status.position >> 24) & 0xFF;
        data[8] = (_motor2Status.position >> 16) & 0xFF;
        data[9] = (_motor2Status.position >> 8) & 0xFF;
        data[10] = _motor2Status.position & 0xFF;
        data[11] = (_motor2Status.speed >> 8) & 0xFF;
        data[12] = _motor2Status.speed & 0xFF;
        data[13] = _motor2Status.mode;

        Wire.write(data, 14);
        break;
    }

    default:
        // Send error byte for unknown request
        Wire.write(0xFF);
        break;
    }
}
