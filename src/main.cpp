#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define MOTOR_STEP_PIN 19
#define MOTOR_DIR_PIN 18

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

// Set the PWM frequency in Hz. This determines the motor speed.
const float PWM_FREQUENCY = 1667;

void move_steps(int steps, bool direction, uint slice_num,
                float pwm_frequency) {
  gpio_put(MOTOR_DIR_PIN, direction);

  // Calculate time to run PWM for.
  // The PWM frequency determines steps per second.
  if (pwm_frequency > 0) {
    float duration_s = (float)steps / pwm_frequency;
    pwm_set_enabled(slice_num, true);
    sleep_ms(duration_s * 1000);
    pwm_set_enabled(slice_num, false);
  }
}

int main() {
  stdio_init_all();

  // --- GPIO Initialization for Direction Pin ---
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // --- PWM Initialization for Step Pin ---
  // Tell GPIO 16 it is allocated to the PWM
  gpio_set_function(MOTOR_STEP_PIN, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO 16
  uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);
  uint chan = pwm_gpio_to_channel(MOTOR_STEP_PIN);

  // Get default PWM config
  pwm_config config = pwm_get_default_config();

  // Set clock divider. The system clock is 125MHz.
  // A divider of 250 gives a 500kHz clock for the PWM counter.
  pwm_config_set_clkdiv(&config, 250.0f);

  // Set the wrap value to set the PWM frequency.
  // wrap = (counter_clock / PWM_FREQUENCY) - 1
  uint16_t wrap = (500000 / PWM_FREQUENCY) - 1;
  pwm_config_set_wrap(&config, wrap);

  // Set the duty cycle to 50%
  pwm_set_chan_level(slice_num, chan, wrap / 2);

  // Load the configuration and disable the PWM. It will be enabled in
  // move_steps.
  pwm_init(slice_num, &config, false);

  // The motor has 200 steps per revolution (1.8 degrees per step).
  // The original code used 150 steps, which is 150 * 1.8 = 270 degrees.
  int steps = 10;

  sleep_ms(1000);
  printf("Starting motor control...\n");

  while (true) {
    // Move 270 degrees
    printf("Moving +270 degrees\n");
    move_steps(steps, CLOCKWISE, slice_num, PWM_FREQUENCY);
    sleep_ms(500);

    // Move 270 degrees in the other direction
    printf("Moving -270 degrees\n");
    move_steps(steps, COUNTER_CLOCKWISE, slice_num, PWM_FREQUENCY);
    sleep_ms(500);
  }

  return 0;
}
