#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Pins
#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17

// --- Motor & Speed Configuration ---
#define STEPS_PER_REV 200 // Standard 1.8 degree stepper motor
#define MICROSTEPS 8      // TMC2209 configured for 1/8 microstepping
#define TARGET_RPM 120    // Desired speed in Revolutions Per Minute

// Calculate required PWM frequency (Hz) to achieve target RPM
// (RPM / 60 seconds) * Steps_Per_Rev * Microsteps
uint32_t MOTOR_SPEED_HZ = (TARGET_RPM * STEPS_PER_REV * MICROSTEPS) / 60;

// Helper to update speed (PWM frequency)
void set_motor_speed(uint slice_num, uint chan, uint32_t freq) {
  if (freq == 0)
    return;

  // wrap = (SystemClock / Divider / Frequency) - 1
  // With 125.0 divider, clock is 1MHz
  uint16_t wrap = (1000000 / freq) - 1;

  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, chan, (wrap + 1) / 2);
}

int main() {
  stdio_init_all();

  // Setup Direction Pin
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // Setup PWM Step Pin
  gpio_set_function(MOTOR_STEP_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);
  uint chan = pwm_gpio_to_channel(MOTOR_STEP_PIN);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 125.0f); // 1MHz base

  // Initialize PWM (starts disabled)
  pwm_init(slice_num, &config, false);

  // Set initial speed and enable
  set_motor_speed(slice_num, chan, MOTOR_SPEED_HZ);
  pwm_set_enabled(slice_num, true);

  printf("Target RPM: %d | Calculated Hz: %d\n", TARGET_RPM, MOTOR_SPEED_HZ);
  printf("Starting direction toggle test...\n");

  while (true) {
    // Spin Clockwise
    printf("Direction: Clockwise\n");
    gpio_put(MOTOR_DIR_PIN, 1);
    sleep_ms(2000); // 2 Seconds

    // Spin Counter-Clockwise
    printf("Direction: Counter-Clockwise\n");
    gpio_put(MOTOR_DIR_PIN, 0);
    sleep_ms(2000); // 2 Seconds
  }

  return 0;
}
