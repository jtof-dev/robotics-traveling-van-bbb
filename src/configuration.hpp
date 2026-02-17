#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

// ToF sensor
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17
#define STEP_ANGLE 1.80f

// microstepping logic (TMC2208 MS1=H, MS2=H)
// 360 / 1.8 = 200 full steps. 200 * 16 = 3200 microsteps/rev.
#define MICROSTEPS 16
#define STEPS_PER_REV 3200

// time in microseconds between steps in the timer.
// lower = faster beam movement.
#define MOTOR_STEP_INTERVAL_US 400

#define BALL_SETPOINT_CM 12.0f
#define PID_SAMPLE_MS 20 // matches VL53L0X timing

#define DEFAULT_KP 15.0f
#define DEFAULT_KI 0.0f
#define DEFAULT_KD 10.0f

// 1 step = 0.1125 degrees.
// 178 steps is ~20 degree tilt.
#define PID_LIMIT_MIN -178.0f
#define PID_LIMIT_MAX 178.0f

enum MOTOR_DIRECTION { COUNTER_CLOCKWISE = 1, CLOCKWISE = 0 };

#endif // CONFIGURATION_HPP
