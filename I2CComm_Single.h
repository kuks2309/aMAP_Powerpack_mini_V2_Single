#ifndef I2C_COMM_SINGLE_H
#define I2C_COMM_SINGLE_H

#include <Arduino.h>
#include <Wire.h>

// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x08

// I2C Command definitions (Single Motor Version)
#define CMD_MOTOR1 0x02              // Control Motor1 only
#define CMD_SERVO1 0x11              // Control Servo1 only
#define CMD_RESET_ENCODER 0x20       // Reset encoder to 0
#define CMD_GET_STATUS 0x30          // Get status (7 bytes: Motor1 only)

// I2C Protocol: Motor control modes
#define I2C_MODE_PWM 0x00            // Direct PWM control
#define I2C_MODE_SPEED 0x01          // Speed PID control (mm/s)
#define I2C_MODE_POSITION 0x02       // Position PID control - Absolute (mm)
#define I2C_MODE_POSITION_REL 0x03   // Position PID control - Relative (mm)

class I2CComm_Single
{
  public:
    I2CComm_Single();
    void begin(uint8_t address = I2C_SLAVE_ADDRESS);

    // Callback setters
    void setMotor1Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l));
    void setServo1Callback(void (*callback)(int8_t angle));
    void setResetEncoderCallback(void (*callback)());

    // Status setter (called by main program)
    void setMotor1Status(int32_t position, int16_t speed, uint8_t mode);

  private:
    // Static handlers for Wire library
    static void onReceiveHandler(int numBytes);
    static void onRequestHandler();

    // Instance handlers
    void onReceive(int numBytes);
    void onRequest();

    // Callbacks
    void (*_motor1Callback)(uint8_t mode, int16_t data_s, int32_t data_l);
    void (*_servo1Callback)(int8_t angle);
    void (*_resetEncoderCallback)();

    // Status data (Motor1 only)
    struct MotorStatus {
        int32_t position;
        int16_t speed;
        uint8_t mode;
    };

    MotorStatus _motor1Status;
    uint8_t _lastCommand;

    // Singleton instance for static callbacks
    static I2CComm_Single* _instance;
};

#endif // I2C_COMM_SINGLE_H
