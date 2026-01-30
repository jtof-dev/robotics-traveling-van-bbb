#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Pico-specific standalone driver
#include "VL53L0X.h"

// Your project headers
#include "PID_v1.h"
#include "configuration.hpp"

// Hardware I2C Configuration for RP2040-Zero
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

// FIFO Command IDs for Core 1 (Touchscreen) communication
#define CMD_UPDATE_KP 0x01
#define CMD_UPDATE_KI 0x02
#define CMD_UPDATE_KD 0x03

// Shared State (Accessed by Timer Interrupt)
volatile int motor_pwm_delay = 1000;
volatile bool motor_enabled = false;
volatile int motor_direction = CLOCKWISE;

// --- Motor Timer Interrupt (Core 0 Background) ---
// This handles the physical pulses for the stepper motor
bool motor_step_callback(struct repeating_timer *t) {
  static bool step_state = false;
  if (motor_enabled) {
    gpio_put(MOTOR_DIR_PIN, motor_direction);
    step_state = !step_state;
    gpio_put(MOTOR_STEP_PIN, step_state);

    // Update the timer delay based on the latest PID calculation
    t->delay_us = -motor_pwm_delay;
  }
  return true;
}

// --- Core 1: Touchscreen UI ---
void core1_entry() {
  // SKELETON: Logic for your touchscreen SPI interface goes here.
  // Example: To update Kp to 35.5, push:
  // multicore_fifo_push_blocking(CMD_UPDATE_KP);
  // multicore_fifo_push_blocking(3550); // Value * 100
  while (1) {
    tight_loop_contents();
  }
}

int main() {
  // 1. Basic Initialization
  stdio_init_all();
  // Give the system and ToF sensor time to stabilize
  sleep_ms(2000);

  // 2. I2C Setup for VL53L0X
  i2c_init(I2C_PORT, 400 * 1000); // Fast mode 400kHz
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  // 3. Sensor Initialization using Pico SDK Driver
  VL53L0X sensor;
  sensor.setAddress(0x29);
  if (!sensor.init()) {
    printf("Failed to detect VL53L0X sensor!\n");
    while (1)
      tight_loop_contents();
  }

  // Set timing budget to 20ms for high-speed balancing
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(); //

  // 4. Motor Pin Initialization
  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // 5. PID Setup
  double distance = 12.0, set_point = 12.0, control_output = 0;
  // Note: We use the same gains from your friend's original code
  PID myPID(&distance, &control_output, &set_point, 30, 2, 3, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1000, 1000);

  // 6. Launch Secondary Core for UI
  multicore_launch_core1(core1_entry);

  // 7. Start Background Motor Timer on Core 0
  struct repeating_timer timer;
  add_repeating_timer_us(-1000, motor_step_callback, NULL, &timer);

  // 8. Main Control Loop (Core 0)
  while (true) {
    // A. Handle FIFO Updates from Touchscreen
    if (multicore_fifo_rvalid()) {
      uint32_t cmd = multicore_fifo_pop_blocking();
      if (cmd == CMD_UPDATE_KP) {
        uint32_t raw_val = multicore_fifo_pop_blocking();
        float new_kp = (float)raw_val / 100.0f;
        myPID.SetTunings(new_kp, myPID.GetKi(), myPID.GetKd());
      }
      // Add KI and KD cases here similarly
    }

    // B. Read Sensor Data
    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
      printf("Sensor Timeout!\n");
      motor_enabled = false;
    } else if (mm < 1200) {         // Standard range check
      distance = (double)mm / 10.0; // Convert mm to cm

      // C. Compute PID
      myPID.Compute();

      // D. Update Motor State
      motor_direction = (control_output > 0) ? CLOCKWISE : COUNTER_CLOCKWISE;

      // Map PID effort to PWM delay (smaller delay = faster motor)
      motor_pwm_delay = 1200 - abs((int)control_output);

      // Small deadzone to prevent motor vibrating at center
      motor_enabled = (abs(control_output) > 8);

      printf("Dist: %.1f cm | Out: %.1f | Speed: %d\n", distance,
             control_output, motor_pwm_delay);
    } else {
      // Ball is likely off the beam
      motor_enabled = false;
    }

    // Maintain a steady 50Hz loop for the PID logic
    sleep_ms(20);
  }

  return 0;
}
