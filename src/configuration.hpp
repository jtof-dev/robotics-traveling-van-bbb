#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

// ToF sensor
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define TOUCH_I2C_PORT 1 // set to 0 for I2C0, or 1 for I2C1

// touch screen pins I2C1
// NOTE: the ToF and the touchscreen can't share the same I2C bus, one must use
// I2C0 and the other I2C1
#define TOUCH_SDA 2
#define TOUCH_SCL 3
#define TOUCH_RST 6
#define TOUCH_INT 7

#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17
#define STEP_ANGLE 1.80f

// microstepping logic (TMC2208 MS1=H, MS2=H)
// 360 / 1.8 = 200 full steps. 200 * 16 = 3200 microsteps/rev.
// #define MICROSTEPS 16
#define STEPS_PER_REV 1800

// #define MOTOR_STEP_INTERVAL_US 400

#define PID_SAMPLE_MS 20 // matches VL53L0X timing

#define DEFAULT_KP 0.75f
#define DEFAULT_KI 0.30f
#define DEFAULT_KD 0.05f

#define PID_LIMIT_MIN -5.0f
#define PID_LIMIT_MAX 5.0f

#define BALL_SETPOINT_CM 17.0f
#define BALL_MIN_DIST_CM 5.0f
#define BALL_MAX_DIST_CM 27.0f
#define BALL_ABSENT_DIST_CM 28.5f
#define BALL_RESUME_MIN_CM 13.0f
#define BALL_RESUME_MAX_CM 19.0f
#define FT6336U_ADDR 0x38

enum MOTOR_DIRECTION { COUNTER_CLOCKWISE = 1, CLOCKWISE = 0 };

#endif // CONFIGURATION_HPP
