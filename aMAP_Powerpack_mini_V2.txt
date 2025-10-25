
#define MOTOR1_EN1 2
#define MOTOR1_EN2 3

#define MOTOR2_EN1 4
#define MOTOR2_EN2 5

#define SERVO1_PIN 6
#define SERVO2_PIN 7

#define SERVO1_NEUTRAL 87
#define SERVO2_NEUTRAL 92

#define LS7166_CS1 8
#define LS7166_CS2 9

#include <Servo.h>
#include "LS7166.h"

Servo servo1;
Servo servo2;

// Create LS7166 encoder object
LS7166 encoder(LS7166_CS1, LS7166_CS2);

// DRV8711 motor driver-1
void motor1_speed_control(int speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR1_EN1, speed);
        analogWrite(MOTOR1_EN2, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, abs(speed));
    }
    else
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, 0);
    }
}

// DRV8711 motor driver-2
void motor2_speed_control(int speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR2_EN1, speed);
        analogWrite(MOTOR2_EN2, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR2_EN1, 0);
        analogWrite(MOTOR2_EN2, abs(speed));
    }
    else
    {
        analogWrite(MOTOR2_EN1, 0);
        analogWrite(MOTOR2_EN2, 0);
    }
}

void servo1_angle_control(int angle)
{
    angle = constrain(angle, -35, 35);
    servo1.write(SERVO1_NEUTRAL + angle);
}

void servo2_angle_control(int angle)
{
    angle = constrain(angle, -35, 35);
    servo2.write(SERVO2_NEUTRAL + angle);
}

void setup()
{
    Serial.begin(9600);

    // Initialize encoders
    encoder.begin();

    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);

    pinMode(MOTOR2_EN1, OUTPUT);
    pinMode(MOTOR2_EN2, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    servo1.write(SERVO1_NEUTRAL);
    servo2.write(SERVO2_NEUTRAL);
}

void loop()
{
    // Example usage:
    // int32_t enc1_value = encoder.read_encoder1();
    // int32_t enc2_value = encoder.read_encoder2();
    // int32_t diff = encoder.get_encoder_difference();
    // int32_t avg = encoder.get_encoder_average();
    // encoder.reset_encoders();
}