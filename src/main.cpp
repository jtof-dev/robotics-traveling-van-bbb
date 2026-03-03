#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "VL53L0X.h"
#include "configuration.hpp"
#include "pid.h"

// thread-safe motor variables
volatile int32_t current_step_pos = 0;
volatile int32_t target_step_pos = 0;
volatile bool motor_enabled = false;

bool motor_step_callback(struct repeating_timer *t) {
  if (motor_enabled && current_step_pos != target_step_pos) {
    // Set Direction based on position error
    if (target_step_pos > current_step_pos) {
      gpio_put(MOTOR_DIR_PIN, CLOCKWISE);
      current_step_pos++;
    } else {
      gpio_put(MOTOR_DIR_PIN, COUNTER_CLOCKWISE);
      current_step_pos--;
    }

    gpio_put(MOTOR_STEP_PIN, 1);
    sleep_us(2); // minimum pulse width
    gpio_put(MOTOR_STEP_PIN, 0);
  }
  return true;
}

// future core 1 touchscreen function
void core1_entry() {
  while (1) {
    tight_loop_contents();
  }
}

int main() {
  stdio_init_all();
  // allow sensors and power rails to stabilize
  sleep_ms(2000);

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
  sensor.startContinuous();

  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  float distance = BALL_SETPOINT_CM;
  float set_point = BALL_SETPOINT_CM;
  float control_output = 0.0f;

  PID myPID(&distance, &control_output, &set_point, DEFAULT_KP, DEFAULT_KI,
            DEFAULT_KD, DIRECT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX); // limits in microsteps
  myPID.SetSampleTime(PID_SAMPLE_MS);

  multicore_launch_core1(core1_entry);

  struct repeating_timer timer;
  add_repeating_timer_us(-MOTOR_STEP_INTERVAL_US, motor_step_callback, NULL,
                         &timer);

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

      // the PID output is now the absolute step position (angle)
      target_step_pos = (int32_t)control_output;
      motor_enabled = true;

      printf("Dist: %.1f cm | Target Step: %ld | Current: %ld\n", distance,
             target_step_pos, current_step_pos);
    } else {
      target_step_pos = 0;
    }

    sleep_ms(500);
  }

  return 0;
}
