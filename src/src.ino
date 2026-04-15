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
#include "touch.hpp"

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

// core 0: touchscreen ui
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  initScreen();
  initTouch(); // <-- initialize the FT6336U
}

void loop() {
  uint32_t loopStart = millis();

  // check for touch input
  uint16_t touchX, touchY;
  if (readTouch(touchX, touchY)) {
    Serial.printf("Touched! X: %d, Y: %d\n", touchX, touchY);
  }

  // update the screen
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

// define our system states
enum SystemState {
  STATE_BALANCING,
  STATE_NEEDS_RESET,
  STATE_WAITING
};

// global state variable for core 1
SystemState current_state = STATE_BALANCING;

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
    
    // evaluate if the ball is out of bounds
    bool ball_absent = (distance_val >= BALL_ABSENT_DIST_CM);
    bool ball_stuck = (distance_val <= BALL_MIN_DIST_CM || distance_val >= BALL_MAX_DIST_CM);

    switch (current_state) {
      case STATE_BALANCING:
        if (ball_absent || ball_stuck) {
          // ball messed up: transition to reset state
          Serial.printf("ERR: Ball fault (%.1f cm). Initiating reset...\n", distance_val);
          current_state = STATE_NEEDS_RESET;
        } else {
          // normal balancing routine
          myPID->Compute();
          motor->set_angle(control_output);
          Serial.printf("Dist: %.1f cm | Speed: %.1f steps/s\n", distance_val, control_output);
        }
        break;

      case STATE_NEEDS_RESET:
        // turn off PID so the I-term doesn't wind up during the 2-second sleep
        myPID->SetMode(MANUAL); 
        control_output = 0.0f; 
        
        // run the blocking reset routine
        motor->reset_beam_angle(); 
        
        Serial.println("Reset complete. Waiting for ball at center...");
        current_state = STATE_WAITING;
        break;

      case STATE_WAITING:
        // continuously read the sensor without moving the motor.
        // if the ball is placed within the setpoint window, resume!
        if (distance_val >= BALL_RESUME_MIN_CM && distance_val <= BALL_RESUME_MAX_CM) {
          Serial.printf("Ball detected at %.1f cm. Resuming balance!\n", distance_val);
          myPID->SetMode(AUTOMATIC);
          current_state = STATE_BALANCING;
        }
        break;
    }

    // push the newest calculated values out to the shared variables for core 0
    current_distance = distance_val;
    current_speed = control_output;
  }

  delay(50);
}
