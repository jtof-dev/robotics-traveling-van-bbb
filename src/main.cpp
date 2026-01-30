#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "PID_v1.h"
#include "VL53L0X.h"
#include "configuration.hpp"

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

// future touchscreen commands
#define CMD_UPDATE_KP 0x01
#define CMD_UPDATE_KI 0x02
#define CMD_UPDATE_KD 0x03

volatile int motor_pwm_delay = 1000;
volatile bool motor_enabled = false;
volatile int motor_direction = CLOCKWISE;

bool motor_step_callback(struct repeating_timer *t) {
  static bool step_state = false;
  if (motor_enabled) {
    gpio_put(MOTOR_DIR_PIN, motor_direction);
    step_state = !step_state;
    gpio_put(MOTOR_STEP_PIN, step_state);

    t->delay_us = -motor_pwm_delay;
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
  // give the system and ToF sensor time to stabilize
  sleep_ms(2000);

  i2c_init(I2C_PORT, 400 * 1000); // fast mode 400kHz
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  VL53L0X sensor(I2C_PORT, 0x29);
  if (!sensor.init()) {
    printf("failed to detect VL53L0X sensor!\n");
    while (1)
      tight_loop_contents();
  }

  // set timing budget to 20ms for high-speed balancing
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  float distance = 12.0f, set_point = 12.0f, control_output = 0.0f;
  PID myPID(&distance, &control_output, &set_point, 30, 2, 3, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1000, 1000);

  multicore_launch_core1(core1_entry);

  struct repeating_timer timer;
  add_repeating_timer_us(-1000, motor_step_callback, NULL, &timer);

  while (true) {
    if (multicore_fifo_rvalid()) {
      uint32_t cmd = multicore_fifo_pop_blocking();
      if (cmd == CMD_UPDATE_KP) {
        uint32_t raw_val = multicore_fifo_pop_blocking();
        float new_kp = (float)raw_val / 100.0f;
        myPID.SetTunings(new_kp, myPID.GetKi(), myPID.GetKd());
      }
    }

    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
      printf("Sensor Timeout!\n");
      motor_enabled = false;
    } else if (mm < 1200) {         // standard range check
      distance = (float)mm / 10.0f; // convert mm to cm

      myPID.Compute();

      motor_direction = (control_output > 0) ? CLOCKWISE : COUNTER_CLOCKWISE;

      motor_pwm_delay = 1200 - abs((int)control_output);

      motor_enabled = (abs(control_output) > 8);

      printf("Dist: %.1f cm | Out: %.1f | Speed: %d\n", distance,
             control_output, motor_pwm_delay);
    } else {
      motor_enabled = false;
    }

    sleep_ms(20);
  }

  return 0;
}
