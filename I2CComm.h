#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Arduino.h>
#include <Wire.h>

// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x08

// I2C Command definitions
#define CMD_DUAL_MOTOR 0x01             // Control both motors simultaneously
#define CMD_MOTOR1_ONLY 0x02            // Control Motor1 only
#define CMD_MOTOR2_ONLY 0x03            // Control Motor2 only
#define CMD_DUAL_SERVO 0x10             // Control both servos
#define CMD_SERVO1_ONLY 0x11            // Control Servo1 only
#define CMD_SERVO2_ONLY 0x12            // Control Servo2 only
#define CMD_RESET_ENCODERS 0x20         // Reset both encoders to 0
#define CMD_GET_STATUS 0x30             // Get all status (14 bytes)

// I2C Protocol: Motor control modes
#define I2C_MODE_PWM 0x00                   // Direct PWM control
#define I2C_MODE_SPEED 0x01                 // Speed PID control (mm/s)
#define I2C_MODE_POSITION 0x02              // Position PID control (mm)

class I2CComm
{
  public:
    I2CComm();
    void begin(uint8_t address = I2C_SLAVE_ADDRESS);

    // Callback setters
    void setDualMotorCallback(void (*callback)(uint8_t mode, int16_t m1_data_s, int32_t m1_data_l, int16_t m2_data_s, int32_t m2_data_l));
    void setMotor1Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l));
    void setMotor2Callback(void (*callback)(uint8_t mode, int16_t data_s, int32_t data_l));
    void setDualServoCallback(void (*callback)(int8_t servo1_angle, int8_t servo2_angle));
    void setServo1Callback(void (*callback)(int8_t angle));
    void setServo2Callback(void (*callback)(int8_t angle));
    void setResetEncodersCallback(void (*callback)());

    // Status setters (called by main program)
    void setMotor1Status(int32_t position, int16_t speed, uint8_t mode);
    void setMotor2Status(int32_t position, int16_t speed, uint8_t mode);

  private:
    // Static handlers for Wire library
    static void onReceiveHandler(int numBytes);
    static void onRequestHandler();

    // Instance handlers
    void onReceive(int numBytes);
    void onRequest();

    // Callbacks
    void (*_dualMotorCallback)(uint8_t mode, int16_t m1_data_s, int32_t m1_data_l, int16_t m2_data_s, int32_t m2_data_l);
    void (*_motor1Callback)(uint8_t mode, int16_t data_s, int32_t data_l);
    void (*_motor2Callback)(uint8_t mode, int16_t data_s, int32_t data_l);
    void (*_dualServoCallback)(int8_t servo1_angle, int8_t servo2_angle);
    void (*_servo1Callback)(int8_t angle);
    void (*_servo2Callback)(int8_t angle);
    void (*_resetEncodersCallback)();

    // Status data
    struct MotorStatus {
        int32_t position;
        int16_t speed;
        uint8_t mode;
    };

    MotorStatus _motor1Status;
    MotorStatus _motor2Status;
    uint8_t _lastCommand;

    // Singleton instance for static callbacks
    static I2CComm* _instance;
};

#endif // I2C_COMM_H
