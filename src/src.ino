#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include <math.h>

#include "VL53L0X.h"
#include "configuration.hpp"
#include "pid.h"
#include "motor.hpp"
#include "screen.hpp"

// --- SHARED VOLATILE VARIABLES ---
// These allow Core 1 to send data to Core 0 safely
volatile float current_distance = 0.0f;
volatile float current_speed = 0.0f;

// --- GLOBALS FOR CORE 1 ---
float distance_val = BALL_SETPOINT_CM;
float set_point = BALL_SETPOINT_CM;
float control_output = 0.0f;

// Objects are declared as pointers so we can instantiate them 
// safely inside setup1() after the hardware rails stabilize.
PID* myPID;
VL53L0X* sensor;
MOTOR* motor;


// ==========================================
// CORE 0: TOUCHSCREEN UI (Slow, heavy)
// ==========================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  initScreen();
}

void loop() {
  uint32_t loopStart = millis();

  // 1. Safely grab the latest values from Core 1
  float display_dist = current_distance;
  float display_speed = current_speed;

  // 2. Gather system metrics
  float tempC = analogReadTemp();
  uint32_t freeRam = rp2040.getFreeHeap();

  // 3. Push to screen
  updateScreen(display_dist, display_speed, tempC, freeRam, millis() - loopStart);

  // Blink LED and throttle UI to ~10 FPS (plenty for a screen)
  digitalWrite(LED_BUILTIN, millis() % 1000 < 500 ? HIGH : LOW);
  delay(100); 
}


// ==========================================
// CORE 1: BALL BALANCING (Fast, real-time)
// ==========================================
void setup1() {
  // Allow sensors and power rails to stabilize
  delay(2000); 

  // TOF Sensor Hardware Init
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  sensor = new VL53L0X(I2C_PORT, 0x29);
  if (!sensor->init()) {
    Serial.println("failed to detect VL53L0X sensor!");
    while (1) { delay(10); } // Yield to prevent watchdog crash
  }

  sensor->setMeasurementTimingBudget(20000);
  sensor->startContinuous(50);

  // PID Setup
  myPID = new PID(&distance_val, &control_output, &set_point, 
                  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
  myPID->SetMode(AUTOMATIC);
  myPID->SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX);
  myPID->SetSampleTime(PID_SAMPLE_MS);

  // Motor Setup
  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  motor = new MOTOR(MOTOR_STEP_PIN, MOTOR_DIR_PIN);
  motor->reset_beam_angle();
}

void loop1() {
  // Check for configuration updates via FIFO
  if (multicore_fifo_rvalid()) {
    uint32_t cmd = multicore_fifo_pop_blocking();
    if (cmd == 0x01) { // CMD_UPDATE_KP
      uint32_t raw_val = multicore_fifo_pop_blocking();
      float new_kp = (float)raw_val / 100.0f;
      myPID->SetTunings(new_kp, myPID->GetKi(), myPID->GetKd());
    }
  }

  // Read TOF and run PID loop
  uint16_t mm = sensor->readRangeContinuousMillimeters();
  if (!sensor->timeoutOccurred() && mm < 1200) {
    distance_val = (float)mm / 10.0f; // mm to cm
    
    myPID->Compute();
    motor->set_angle(control_output);

    // Push the newest calculated values out to the shared variables for Core 0
    current_distance = distance_val;
    current_speed = control_output;
    
    Serial.printf("Dist: %.1f cm | Speed: %.1f steps/s\n", distance_val, control_output);
  }

  delay(50);
}
