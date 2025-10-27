#include "I2CComm_Single.h"

/*
 * ============================================================================
 * I2C Communication Protocol for Single Motor & Servo Control
 * ============================================================================
 *
 * Default I2C Address: 0x08
 *
 * ============================================================================
 * COMMAND FORMAT (Jetson → Arduino)
 * ============================================================================
 *
 * 1. MOTOR CONTROL
 *    Command: 0x02
 *    Format: [0x02][Mode][Data]
 *
 *    Mode 0x00 - PWM Control:
 *      Data: 2 bytes [PWM(2B)]
 *      Range: -255 to 255 (int16)
 *      Example: [0x02][0x00][0x00][0x64]  // PWM=100
 *
 *    Mode 0x01 - Speed Control:
 *      Data: 2 bytes [Speed(2B)]
 *      Range: target speed in mm/s (int16)
 *      Example: [0x02][0x01][0x00][0x96]  // 150 mm/s
 *
 *    Mode 0x02 - Position Control:
 *      Data: 4 bytes [Position(4B)]
 *      Range: target position in mm (int32)
 *      Example: [0x02][0x02][0x00][0x00][0x03][0xE8]  // 1000 mm
 *
 * 2. SERVO CONTROL
 *    Command: 0x11 [Angle(1B)]
 *    Range: -35 to +35 degrees (int8_t, relative to neutral)
 *    Example: [0x11][0x14]  // +20°
 *
 * 3. SYSTEM COMMANDS
 *    Reset Encoder: 0x20 (no data)
 *    Get Status: 0x30 (no data, returns 7 bytes)
 *
 * ============================================================================
 * RESPONSE FORMAT (Arduino → Jetson)
 * ============================================================================
 *
 * Status Response (after 0x30 command): 7 bytes total
 *   [Position: 4B][Speed: 2B][Mode: 1B]
 *
 *   Position (int32): Motor encoder position (pulses, Big Endian)
 *   Speed (int16): Motor current speed (pulses/20ms, Big Endian)
 *   Mode (uint8): Motor mode (0:PWM, 1:Speed, 2:Position)
 *
 * ============================================================================
 */

// Initialize static instance pointer
I2CComm_Single* I2CComm_Single::_instance = nullptr;

I2CComm_Single::I2CComm_Single()
{
    _motor1Callback = nullptr;
    _servo1Callback = nullptr;
    _resetEncoderCallback = nullptr;

    _motor1Status = {0, 0, 0};
    _lastCommand = 0;

    _instance = this;
}

void I2CComm_Single::begin(uint8_t address)
{
    Wire.begin(address);
    Wire.onReceive(onReceiveHandler);
    Wire.onRequest(onRequestHandler);
}

// Callback setters
void I2CComm_Single::setMotor1Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l))
{
    _motor1Callback = callback;
}

void I2CComm_Single::setServo1Callback(void (*callback)(int8_t angle))
{
    _servo1Callback = callback;
}

void I2CComm_Single::setResetEncoderCallback(void (*callback)())
{
    _resetEncoderCallback = callback;
}

// Status setter
void I2CComm_Single::setMotor1Status(int32_t position, int16_t speed, uint8_t mode)
{
    _motor1Status.position = position;
    _motor1Status.speed = speed;
    _motor1Status.mode = mode;
}

// Static handlers
void I2CComm_Single::onReceiveHandler(int numBytes)
{
    if (_instance != nullptr)
    {
        _instance->onReceive(numBytes);
    }
}

void I2CComm_Single::onRequestHandler()
{
    if (_instance != nullptr)
    {
        _instance->onRequest();
    }
}

// Instance receive handler
void I2CComm_Single::onReceive(int numBytes)
{
    if (numBytes < 1)
        return;

    uint8_t command = Wire.read();
    _lastCommand = command;

    switch (command)
    {
    case CMD_MOTOR1: // 0x02 - Control Motor1
    {
        if (numBytes >= 2)
        {
            uint8_t mode = Wire.read();
            int16_t data_s = 0;
            int32_t data_l = 0;

            if (mode == I2C_MODE_PWM || mode == I2C_MODE_SPEED)
            {
                // PWM or Speed mode: 2 bytes
                if (numBytes >= 4) // cmd(1) + mode(1) + data(2)
                {
                    data_s = (Wire.read() << 8) | Wire.read();
                }
            }
            else if (mode == I2C_MODE_POSITION || mode == I2C_MODE_POSITION_REL)
            {
                // Position mode (Absolute or Relative): 4 bytes
                if (numBytes >= 6) // cmd(1) + mode(1) + data(4)
                {
                    data_l = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) |
                            ((int32_t)Wire.read() << 8) | Wire.read();
                }
            }

            if (_motor1Callback != nullptr)
            {
                _motor1Callback(mode, data_s, data_l);
            }
        }
        break;
    }

    case CMD_SERVO1: // 0x11 - Control Servo1
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

    case CMD_RESET_ENCODER: // 0x20
    {
        if (_resetEncoderCallback != nullptr)
        {
            _resetEncoderCallback();
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
void I2CComm_Single::onRequest()
{
    // Always send current Motor1 status (7 bytes)
    // No command needed - simplifies I2C protocol and avoids timing issues
    uint8_t data[7];

    // Motor1 status: Position(4) + Speed(2) + Mode(1) in Big Endian
    data[0] = (_motor1Status.position >> 24) & 0xFF;
    data[1] = (_motor1Status.position >> 16) & 0xFF;
    data[2] = (_motor1Status.position >> 8) & 0xFF;
    data[3] = _motor1Status.position & 0xFF;
    data[4] = (_motor1Status.speed >> 8) & 0xFF;
    data[5] = _motor1Status.speed & 0xFF;
    data[6] = _motor1Status.mode;

    Wire.write(data, 7);
}
