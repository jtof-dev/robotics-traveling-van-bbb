#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

// ==========================================
// PIN CONFIGURATION
// ==========================================
#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 18

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
  stdio_init_all();

  // Give yourself 3 seconds after plugging it in before it starts moving
  sleep_ms(3000);
  printf("Starting Barebones Motor Test...\n");

  // Initialize pins as standard outputs
  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);

  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  while (true) {
    // --- 1. SPIN FORWARD ---
    printf("Spinning Forward...\n");
    gpio_put(MOTOR_DIR_PIN, 1);

    // Take 1600 steps (1 full revolution if using 1/8th microstepping)
    for (int i = 0; i < 1600; i++) {
      gpio_put(MOTOR_STEP_PIN, 1);
      sleep_us(1500); // 1.5ms high
      gpio_put(MOTOR_STEP_PIN, 0);
      sleep_us(1500); // 1.5ms low
    }

    // --- 2. PAUSE ---
    // A full 2-second stop prevents back-EMF spikes when reversing
    printf("Stopped.\n");
    sleep_ms(2000);

    // --- 3. SPIN BACKWARD ---
    printf("Spinning Backward...\n");
    gpio_put(MOTOR_DIR_PIN, 0);

    for (int i = 0; i < 1600; i++) {
      gpio_put(MOTOR_STEP_PIN, 1);
      sleep_us(1500);
      gpio_put(MOTOR_STEP_PIN, 0);
      sleep_us(1500);
    }

    // --- 4. PAUSE ---
    printf("Stopped.\n");
    sleep_ms(2000);
  }

  return 0;
}
