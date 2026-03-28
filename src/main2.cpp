#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#include "VL53L0X.h"

// ==========================================
// PIN CONFIGURATION
// ==========================================

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define MOTOR_STEP_PIN 16
#define MOTOR_DIR_PIN 17
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define ENCODER_PIN_A 6
#define ENCODER_PIN_B 7
#define ENCODER_PIN_Z 8

// ==========================================
// GLOBALS & INTERRUPTS
// ==========================================

volatile int position = 0;
volatile int revolutions = 0;
volatile bool last_state_a = false;
bool tof_connected = false;

// ISR for the magnetic encoder
void encoder_callback(uint gpio, uint32_t events) {
  if (gpio == ENCODER_PIN_A) {
    bool state_a = gpio_get(ENCODER_PIN_A);
    bool state_b = gpio_get(ENCODER_PIN_B);

    if (state_a != last_state_a) {
      if (state_a != state_b) {
        position++; // forward
      } else {
        position--; // reverse
      }
      last_state_a = state_a;
    }
  } else if (gpio == ENCODER_PIN_Z) {
    revolutions++;
  }
}

// ==========================================
// BLOCKING MOTOR CONTROL
// ==========================================

// Borrowed directly from your script, modified slightly to accept a delay
// so we can change speeds without hardcoding the sleep_us value.
void change_angle_blocking(int steps, uint motor_pin, uint dir_pin, int dir,
                           int delay_us) {
  int counter = 0;

  gpio_put(dir_pin, dir);

  while (counter < steps) {
    gpio_put(motor_pin, 1);
    sleep_us(delay_us);
    gpio_put(motor_pin, 0);
    sleep_us(delay_us);
    counter++;
  }
}

// ==========================================
// MAIN LOOP
// ==========================================

int main() {
  stdio_init_all();
  sleep_ms(2000);

  // --- I2C & ToF Setup ---
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  VL53L0X sensor(I2C_PORT, 0x29);
  if (!sensor.init()) {
    printf("⚠️ Failed to detect VL53L0X sensor! Bypassing ToF readings.\n");
    tof_connected = false;
  } else {
    printf("✅ VL53L0X sensor detected.\n");
    tof_connected = true;
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous(50);
  }

  // --- Motor Setup (Standard GPIO now, no PWM!) ---
  gpio_init(MOTOR_STEP_PIN);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // --- Encoder Setup ---
  gpio_init(ENCODER_PIN_A);
  gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
  gpio_pull_up(ENCODER_PIN_A);
  gpio_init(ENCODER_PIN_B);
  gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
  gpio_pull_up(ENCODER_PIN_B);
  gpio_init(ENCODER_PIN_Z);
  gpio_set_dir(ENCODER_PIN_Z, GPIO_IN);
  gpio_pull_up(ENCODER_PIN_Z);

  last_state_a = gpio_get(ENCODER_PIN_A);
  gpio_set_irq_enabled_with_callback(ENCODER_PIN_A,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &encoder_callback);
  gpio_set_irq_enabled(ENCODER_PIN_Z, GPIO_IRQ_EDGE_RISE, true);

  printf("Starting Blocking GPIO Motor Test Sequence...\n");

  while (true) {
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time()) % 12000;

    const char *current_phase = "";
    int steps_to_take = 0;
    int current_delay_us = 0;
    int current_dir = CLOCKWISE;

    // 1. Setup the movement chunk based on the time phase
    // We move in ~100ms chunks so we have time to read sensors between
    // movements
    if (current_time_ms < 3000) {
      current_phase = "Slow FWD";
      current_dir = COUNTER_CLOCKWISE;
      current_delay_us = 1250; // 400 steps/sec = 2.5ms per step
      steps_to_take = 40;      // 40 steps * 2.5ms = 100ms blocking chunk
    } else if (current_time_ms < 6000) {
      current_phase = "Fast FWD";
      current_dir = COUNTER_CLOCKWISE;
      current_delay_us = 250; // 2000 steps/sec = 0.5ms per step
      steps_to_take = 200;    // 200 steps * 0.5ms = 100ms blocking chunk
    } else if (current_time_ms < 9000) {
      current_phase = "Slow BWD";
      current_dir = CLOCKWISE;
      current_delay_us = 1250;
      steps_to_take = 40;
    } else {
      current_phase = "Fast BWD";
      current_dir = CLOCKWISE;
      current_delay_us = 250;
      steps_to_take = 200;
    }

    // 2. Lock the processor and physically step the motor
    change_angle_blocking(steps_to_take, MOTOR_STEP_PIN, MOTOR_DIR_PIN,
                          current_dir, current_delay_us);

    // 3. Read the ToF sensor (This takes ~20ms)
    float distance = 0.0f;
    if (tof_connected) {
      uint16_t mm = sensor.readRangeContinuousMillimeters();
      if (!sensor.timeoutOccurred() && mm < 1200) {
        distance = (float)mm / 10.0f;
      }
    }

    // 4. Print Telemetry
    if (tof_connected) {
      printf("[%s] Dist: %.1f cm | Enc Pos: %d | Revs: %d\n", current_phase,
             distance, position, revolutions);
    } else {
      printf("[%s] Enc Pos: %d | Revs: %d\n", current_phase, position,
             revolutions);
    }

    // Note: No extra sleep_ms() here because the change_angle_blocking function
    // acts as our delay. The loop naturally cycles about 8 to 10 times a
    // second.
  }

  return 0;
}
