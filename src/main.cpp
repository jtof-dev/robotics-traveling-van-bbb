#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "VL53L0X.h"
#include "configuration.hpp"
#include "pid.h"

// Optimized set_speed with state management
void set_speed(float steps_per_sec) {
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);
    
    if (steps_per_sec < 10.0f) { 
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_STEP_PIN), 0);
        pwm_set_enabled(slice_num, false);
        return;
    }

    // Use a higher divider for low speeds to avoid 16-bit wrap overflow
    // 125 divider = 1MHz clock. Wrap of 65535 = ~15Hz minimum speed.
    float divider = 125.0f;
    uint32_t wrap = 1000000 / steps_per_sec;

    // Safety: PWM wrap is a 16-bit register (max 65535)
    if (wrap > 65535) wrap = 65535;

    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_STEP_PIN), wrap / 2);
    pwm_set_enabled(slice_num, true);
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
  sensor.startContinuous(50);

  gpio_set_function(MOTOR_STEP_PIN, GPIO_FUNC_PWM);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  float distance = BALL_SETPOINT_CM;
  float set_point = BALL_SETPOINT_CM;
  float control_output = 0.0f;

  PID myPID(&distance, &control_output, &set_point, DEFAULT_KP, DEFAULT_KI,
            DEFAULT_KD, DIRECT);

  myPID.SetMode(AUTOMATIC);
  // limits are now in steps/sec
  myPID.SetOutputLimits(-1000, 1000); 
  myPID.SetSampleTime(PID_SAMPLE_MS);

  multicore_launch_core1(core1_entry);

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

      gpio_put(MOTOR_DIR_PIN, (control_output > 0) ? CLOCKWISE : COUNTER_CLOCKWISE);
      set_speed(fabs(control_output));

      printf("Dist: %.1f cm | Speed: %.1f steps/s\n", distance,
             control_output);
    } else {
      set_speed(0);
    }

    sleep_ms(50);
  }

  return 0;
}
