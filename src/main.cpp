#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

// Speed control
const int DELAY_US = 500; // smaller is faster

void move_steps(int steps, bool direction) {
  gpio_put(MOTOR_DIR_PIN, direction);
  for (int i = 0; i < steps; ++i) {
    gpio_put(MOTOR_STEP_PIN, 1);
    sleep_us(100);
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

  int steps = 150;

  sleep_ms(1000);
  printf("Starting motor control...\n");

  while (true) {
    // Move to +15 degrees from horizontal
    printf("Moving to +15 degrees\n");
    move_steps(steps, CLOCKWISE);
    sleep_ms(500);

    // // Move back to horizontal from +15
    // printf("Returning to horizontal\n");
    // move_steps(steps, COUNTER_CLOCKWISE);
    // sleep_ms(500);

    // Move to -15 degrees from horizontal
    printf("Moving to -15 degrees\n");
    move_steps(steps, COUNTER_CLOCKWISE);
    sleep_ms(500);

    // // Move back to horizontal from -15
    // printf("Returning to horizontal\n");
    // move_steps(steps, CLOCKWISE);
    // sleep_ms(500);
  }

  return 0;
}
