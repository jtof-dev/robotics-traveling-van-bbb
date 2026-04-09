#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#include "VL53L0X.h"
#include "configuration.hpp"
#include "pid.h"
#include "motor.hpp"

void core1_entry() {
  while (1) {
    tight_loop_contents();
  }
}

int main() {
  stdio_init_all();
  // allow sensors and power rails to stabilize
  sleep_ms(2000);

  // TOF Sensor Init
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);


  // ToF sensor
  VL53L0X sensor(I2C_PORT, 0x29);
  if (!sensor.init()) {
    printf("failed to detect VL53L0X sensor!\n");

    while (1)
      tight_loop_contents();

  }

  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(50);

  // PID setup
  float distance = BALL_SETPOINT_CM;
  float set_point = BALL_SETPOINT_CM;
  float control_output = 0.0f;

  PID myPID(&distance, &control_output, &set_point, DEFAULT_KP, DEFAULT_KI,
            DEFAULT_KD, DIRECT);

  myPID.SetMode(AUTOMATIC);
  // limits are now in steps/sec
  myPID.SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX);
  myPID.SetSampleTime(PID_SAMPLE_MS);

  multicore_launch_core1(core1_entry);
  // Motor Setup
  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  MOTOR motor(MOTOR_STEP_PIN, MOTOR_DIR_PIN);
  motor.reset_beam_angle();


  //multicore_launch_core1(core1_entry);

  while (true) {
    if (multicore_fifo_rvalid()) {
      uint32_t cmd = multicore_fifo_pop_blocking();
      if (cmd == 0x01) { // CMD_UPDATE_KP
        uint32_t raw_val = multicore_fifo_pop_blocking();
        float new_kp = (float)raw_val / 100.0f;
        myPID.SetTunings(new_kp, myPID.GetKi(), myPID.GetKd());
      }
    }

    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred() && mm < 1200) {
	    distance = (float)mm / 10.0f; // mm to cm

	    myPID.Compute();

	    motor.set_angle(control_output);

	    printf("Dist: %.1f cm | Speed: %.1f steps/s\n", distance, control_output);
    }
    


    sleep_ms(50);
  }

  return 0;
}
