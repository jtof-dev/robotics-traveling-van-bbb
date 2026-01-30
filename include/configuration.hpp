#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

// VL53L0X
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define TOF_ADDR 0x29

// motor configuration
#define MOTOR_STEP_PIN 15
#define MOTOR_DIR_PIN 14
#define STEP_ANGLE 1.80f

enum MOTOR_DIRECTION { COUNTER_CLOCKWISE = 1, CLOCKWISE = 0 };

// PID / control
#define BALL_SETPOINT_CM 12.0f // Target position on the beam
#define PID_LOOP_MS 20         // 50Hz control loop
#define MOTOR_MIN_DELAY 200    // Max speed (min microsecond delay)
#define MOTOR_MAX_DELAY 1200   // Min speed (max microsecond delay)
#define MOTOR_DEADZONE 8       // Ignore very small PID corrections

#define BEAM_MAX_LENGTH_CM 23.0f
#define BEAM_MIN_LENGTH_CM 0.5f

#endif
