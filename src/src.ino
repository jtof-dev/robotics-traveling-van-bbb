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

// shared volatile variables to communicate between loops
volatile float current_distance = 0.0f;
volatile float current_speed = 0.0f;

// globals for core 1
float distance_val = BALL_SETPOINT_CM;
float set_point = BALL_SETPOINT_CM;
float control_output = 0.0f;

PID* myPID;
VL53L0X* sensor;
MOTOR* motor;

// core 0: touchscreen UI
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  initScreen();
}

void loop() {
  // this is mostly placeholder UI to show that the screen is capable of working in real-time
  uint32_t loopStart = millis();

  float display_dist = current_distance;
  float display_speed = current_speed;

  float tempC = analogReadTemp();
  uint32_t freeRam = rp2040.getFreeHeap();

  updateScreen(display_dist, display_speed, tempC, freeRam, millis() - loopStart);

  digitalWrite(LED_BUILTIN, millis() % 1000 < 500 ? HIGH : LOW);
  delay(100); 
}

// core 1: ball balancing
void setup1() {
  // allow sensors and power rails to stabilize
  delay(2000); 

  // TOF sensor hardware init
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  sensor = new VL53L0X(I2C_PORT, 0x29);
  if (!sensor->init()) {
    Serial.println("failed to detect VL53L0X sensor!");
    while (1) { delay(10); } // yield to prevent watchdog crash
  }

  sensor->setMeasurementTimingBudget(20000);
  sensor->startContinuous(50);

  // PID setup
  myPID = new PID(&distance_val, &control_output, &set_point, 
                  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
  myPID->SetMode(AUTOMATIC);
  myPID->SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX);
  myPID->SetSampleTime(PID_SAMPLE_MS);

  // motor setup
  gpio_init(MOTOR_STEP_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  motor = new MOTOR(MOTOR_STEP_PIN, MOTOR_DIR_PIN);
  motor->reset_beam_angle();
}

void loop1() {
  // check for configuration updates via FIFO
  if (multicore_fifo_rvalid()) {
    uint32_t cmd = multicore_fifo_pop_blocking();
    if (cmd == 0x01) { // CMD_UPDATE_KP
      uint32_t raw_val = multicore_fifo_pop_blocking();
      float new_kp = (float)raw_val / 100.0f;
      myPID->SetTunings(new_kp, myPID->GetKi(), myPID->GetKd());
    }
  }

  // read TOF and run PID loop
  uint16_t mm = sensor->readRangeContinuousMillimeters();
  if (!sensor->timeoutOccurred() && mm < 1200) {
    distance_val = (float)mm / 10.0f; // mm to cm
    
    // ball checks
    bool ball_absent = (distance_val >= BALL_ABSENT_DIST_CM);
    bool ball_stuck = (distance_val <= BALL_MIN_DIST_CM || distance_val >= BALL_MAX_DIST_CM);

    if (ball_absent || ball_stuck) {
      // STOP WORKING: ball is missing or stuck at the ends
      if (myPID->GetMode() == AUTOMATIC) {
        myPID->SetMode(MANUAL);   // turn off PID to prevent I-term windup
        control_output = 0.0f;    // set target angle to 0 (flat/neutral)
        motor->set_angle(control_output);
      }
      
      if (ball_absent) {
        Serial.printf("ERR: No Ball Detected (%.1f cm)\n", distance_val);
      } else {
        Serial.printf("ERR: Ball Stuck/Out of bounds (%.1f cm)\n", distance_val);
      }

    } else {
      // NORMAL OPERATION: ball is within valid bounds
      if (myPID->GetMode() != AUTOMATIC) {
        myPID->SetMode(AUTOMATIC); // turn PID back on if it was disabled
      }

      myPID->Compute();
      motor->set_angle(control_output);
      
      Serial.printf("Dist: %.1f cm | Speed: %.1f steps/s\n", distance_val, control_output);
    }

    // push the newest calculated values out to the shared variables for core 0
    current_distance = distance_val;
    current_speed = control_output;
  }

  delay(50);
}
