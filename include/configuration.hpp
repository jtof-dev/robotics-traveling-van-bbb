#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

// ToF sensor
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

// stepper motor
#define MOTOR_STEP_PIN 15
#define MOTOR_DIR_PIN 14
#define STEP_ANGLE 1.80f

// general
#define BALL_SETPOINT_CM 12.0f
#define MOTOR_MIN_DELAY 200  // max speed
#define MOTOR_MAX_DELAY 1200 // min speed
#define MOTOR_DEADZONE 8

#define DEFAULT_KP 30.0f
#define DEFAULT_KI 2.0f
#define DEFAULT_KD 3.0f

#define BALL_SETPOINT_CM 12.0f
#define PID_LIMIT_MIN -1000.0f
#define PID_LIMIT_MAX 1000.0f
#define PID_SAMPLE_MS 20

#define MOTOR_BASE_DELAY 1200
#define MOTOR_DEADZONE 8

enum MOTOR_DIRECTION { COUNTER_CLOCKWISE = 1, CLOCKWISE = 0 };

// #define ONBOARD_RGB_PIN 16

#endif // CONFIGURATION_HPP
