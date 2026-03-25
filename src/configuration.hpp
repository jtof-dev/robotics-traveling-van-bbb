#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

// ToF sensor
// #define I2C_PORT i2c0
// #define SDA_PIN 4
// #define SCL_PIN 5

#define I2C_PORT i2c1
#define SDA_PIN 6
#define SCL_PIN 7

#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17
#define STEP_ANGLE 1.80f

// microstepping logic (TMC2208 MS1=H, MS2=H)
// 360 / 1.8 = 200 full steps. 200 * 16 = 3200 microsteps/rev.
// #define MICROSTEPS 16
#define STEPS_PER_REV 200

// #define MOTOR_STEP_INTERVAL_US 400

#define BALL_SETPOINT_CM 17.0f
#define PID_SAMPLE_MS 20 // matches VL53L0X timing

#define DEFAULT_KP 0.50f
#define DEFAULT_KI 0.00f
#define DEFAULT_KD 0.00f

#define PID_LIMIT_MIN -3.5f
#define PID_LIMIT_MAX 3.5f

enum MOTOR_DIRECTION { COUNTER_CLOCKWISE = 1, CLOCKWISE = 0 };

#endif // CONFIGURATION_HPP
