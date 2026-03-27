#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#include "VL53L0X.h"

// ==========================================
// PIN CONFIGURATION
// ==========================================

// I2C for ToF sensor
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

// Stepper motor
#define MOTOR_STEP_PIN 16 // Configured as Hardware PWM
#define MOTOR_DIR_PIN 17
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

// Magnetic Encoder (Positive lines only)
#define ENCODER_PIN_A 6
#define ENCODER_PIN_B 7
#define ENCODER_PIN_Z 8

// ==========================================
// GLOBALS & INTERRUPTS
// ==========================================

volatile int position = 0;
volatile int revolutions = 0;
volatile bool last_state_a = false;
bool tof_connected = false; // Flag to gracefully handle missing ToF sensor

// PWM State Trackers (Prevents resetting hardware timers unnecessarily)
uint32_t current_speed = 0;
int current_dir = -1;

// ISR for the magnetic encoder
void encoder_callback(uint gpio, uint32_t events) {
  if (gpio == ENCODER_PIN_A) {
    bool state_a = gpio_get(ENCODER_PIN_A);
    bool state_b = gpio_get(ENCODER_PIN_B);

    if (state_a != last_state_a) {
      if (state_a != state_b) {
        position++; // Forward
      } else {
        position--; // Reverse
      }
      last_state_a = state_a;
    }
  } else if (gpio == ENCODER_PIN_Z) {
    revolutions++; // Tracks full rotations
  }
}

// ==========================================
// HARDWARE PWM MOTOR CONTROL
// ==========================================

// Sets the motor speed using hardware PWM in the background
void set_motor_speed(uint32_t steps_per_sec, int dir) {
  // CRITICAL FIX: If the motor is already doing exactly this, do nothing!
  // This prevents us from resetting the hardware timers while they are running.
  if (steps_per_sec == current_speed && dir == current_dir) {
    return;
  }

  // Update our trackers
  current_speed = steps_per_sec;
  current_dir = dir;

  uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);
  uint channel = pwm_gpio_to_channel(MOTOR_STEP_PIN);

  // Stop the motor if speed is 0
  if (steps_per_sec == 0) {
    pwm_set_enabled(slice_num, false);
    return;
  }

  // Set direction
  gpio_put(MOTOR_DIR_PIN, dir);

  // Calculate hardware PWM parameters
  // With a 125.0 divider, the base clock is 1MHz
  // wrap = (1MHz / Frequency) - 1
  uint16_t wrap = (1000000 / steps_per_sec) - 1;

  // Apply settings
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel, (wrap + 1) / 2); // 50% duty cycle
  pwm_set_enabled(slice_num, true);
}

// ==========================================
// MAIN LOOP
// ==========================================

int main() {
  stdio_init_all();
  sleep_ms(2000); // Allow sensors and power rails to stabilize

  // --- I2C & ToF Setup ---
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  VL53L0X sensor(I2C_PORT, 0x29);

  // Graceful fallback if ToF is unplugged
  if (!sensor.init()) {
    printf("⚠️ Failed to detect VL53L0X sensor! Bypassing ToF readings.\n");
    tof_connected = false;
  } else {
    printf("✅ VL53L0X sensor detected.\n");
    tof_connected = true;
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous(50); // Read every 50ms
  }

  // --- Motor Setup ---
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // Initialize STEP pin for Hardware PWM
  gpio_set_function(MOTOR_STEP_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);

  // Explicitly configure the PWM slice base clock to 1MHz
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 125.0f);
  pwm_init(slice_num, &config, false); // Initialize but don't enable yet

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

  printf("Starting Motor PWM Test Sequence...\n");

  // --- Test Sequence Loop ---
  while (true) {
    // 1. Determine which phase of the test we are in based on elapsed time
    // The % 12000 loops the sequence every 12 seconds
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time()) % 12000;
    const char *current_phase = "";

    if (current_time_ms < 3000) {
      set_motor_speed(400, COUNTER_CLOCKWISE); // Slow Forward (400 steps/sec)
      current_phase = "Slow FWD";
    } else if (current_time_ms < 6000) {
      set_motor_speed(2000, COUNTER_CLOCKWISE); // Fast Forward (2000 steps/sec)
      current_phase = "Fast FWD";
    } else if (current_time_ms < 9000) {
      set_motor_speed(400, CLOCKWISE); // Slow Backward (400 steps/sec)
      current_phase = "Slow BWD";
    } else {
      set_motor_speed(2000, CLOCKWISE); // Fast Backward (2000 steps/sec)
      current_phase = "Fast BWD";
    }

    // 2. Read Time-of-Flight Sensor (ONLY if connected)
    float distance = 0.0f;
    if (tof_connected) {
      uint16_t mm = sensor.readRangeContinuousMillimeters();
      if (!sensor.timeoutOccurred() && mm < 1200) {
        distance = (float)mm / 10.0f; // Convert to cm
      }
    }

    // 3. Print Telemetry
    if (tof_connected) {
      printf("[%s] Dist: %.1f cm | Enc Pos: %d | Revs: %d\n", current_phase,
             distance, position, revolutions);
    } else {
      printf("[%s] Enc Pos: %d | Revs: %d\n", current_phase, position,
             revolutions);
    }

    sleep_ms(50);
  }

  return 0;
}
