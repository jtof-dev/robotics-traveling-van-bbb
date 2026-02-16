#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define MOTOR_STEP_PIN 15
#define MOTOR_DIR_PIN 14

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

// Motor characteristics
const float STEP_ANGLE = 1.8f;
const int MICROSTEPS = 16;
const int STEPS_PER_REV_FULL = 360 / STEP_ANGLE;
const int STEPS_PER_REV_MICRO = STEPS_PER_REV_FULL * MICROSTEPS;

// Speed control
const int DELAY_US = 500; // smaller is faster

void move_steps(int steps, bool direction) {
  gpio_put(MOTOR_DIR_PIN, direction);
  for (int i = 0; i < steps; ++i) {
    gpio_put(MOTOR_STEP_PIN, 1);
    sleep_us(1);
    gpio_put(MOTOR_STEP_PIN, 0);
    sleep_us(DELAY_US);
  }
}

int main() {
  stdio_init_all();

  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);

  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // Calculate steps for 15 degrees
  int steps_for_15_deg = (int)((15.0f / 360.0f) * STEPS_PER_REV_MICRO);

  // Steps for 30 degrees (to move from +15 to -15 and back)
  int steps_for_30_deg = steps_for_15_deg * 2;

  sleep_ms(1000);
  printf("Starting motor control...\n");

  // Move to +15 degrees to start
  move_steps(steps_for_15_deg, CLOCKWISE);
  sleep_ms(2000);

  while (true) {
    // Move to -15 degrees
    printf("Moving to -15 degrees\n");
    move_steps(steps_for_30_deg, COUNTER_CLOCKWISE);
    sleep_ms(3000);

    // Move to +15 degrees
    printf("Moving to +15 degrees\n");
    move_steps(steps_for_30_deg, CLOCKWISE);
    sleep_ms(3000);
  }

  return 0;
}
