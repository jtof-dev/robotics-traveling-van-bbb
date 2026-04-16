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
volatile float current_setpoint = BALL_SETPOINT_CM;
volatile bool system_running = true;

bool hardware_present = true;

// globals for core 1
float distance_val = BALL_SETPOINT_CM;
float set_point = BALL_SETPOINT_CM;
float control_output = 0.0f;

// define our system states
enum SystemState {
  STATE_BALANCING,
  STATE_NEEDS_RESET,
  STATE_WAITING
};

SystemState current_state = STATE_BALANCING;

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
  static uint32_t lastTouchTime = 0; // for debouncing

  uint16_t rawX, rawY;
  
  if (millis() - lastTouchTime > 200) {
    if (readTouch(rawX, rawY)) {
      lastTouchTime = millis();
      
      // makes the touch and screen think they are both horizontal
      uint16_t mappedX = rawY; 
      // uint16_t mappedX = 480 - rawY; 
      // uint16_t mappedY = rawX;       
      uint16_t mappedY = 320 - rawX;

      // Serial.printf("Raw -> X: %d, Y: %d | Mapped -> X: %d, Y: %d\n", rawX, rawY, mappedX, mappedY);
      Serial.printf("Raw -> X: %d, Y: %d | Mapped -> X: %d, Y: %d\r\n", rawX, rawY, mappedX, mappedY);
      
      ButtonID btn = checkButtons(mappedX, mappedY);
      
      switch(btn) {
        case BTN_TOGGLE_BALANCE:
          system_running = !system_running;
          Serial.println(system_running ? "System ON" : "System OFF");
          break;
          
        case BTN_RESET:
          multicore_fifo_push_blocking(0x04); 
          Serial.println("Force Reset Triggered");
          break;
          
        case BTN_SETPOINT_UP:
          current_setpoint += 1.0f;
          if (current_setpoint > 25.0f) current_setpoint = 25.0f; 
          break;
          
        case BTN_SETPOINT_DOWN:
          current_setpoint -= 1.0f;
          if (current_setpoint < 5.0f) current_setpoint = 5.0f;   
          break;
          
        case BTN_NONE:
        default:
          break;
      }
    }
  }

  updateScreen(current_distance, current_speed, analogReadTemp(), rp2040.getFreeHeap(), 
               millis() - loopStart, current_setpoint, system_running);

  delay(50); 
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
  
  // automatic hardware detection
  if (!sensor->init()) {
    Serial.println("WARNING: VL53L0X missing! Entering UI Simulation Mode...");
    hardware_present = false; 
  } else {
    Serial.println("Hardware detected. Normal mode active.");
    sensor->setMeasurementTimingBudget(20000);
    sensor->startContinuous(50);
  }

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
  
  // only run the 2-second blocking reset if hardware actually exists
  if (hardware_present) {
    motor->reset_beam_angle();
  }
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
    // pop any other commands (like BTN_RESET) so the FIFO doesn't jam up in simulation mode
    else if (!hardware_present) {
       Serial.printf("Sim Mode: Ignored Command 0x%02X\r\n", cmd);
    }
  }

  // ui simulation mode
  if (!hardware_present) {
    // generate a smooth fake sine wave between 5cm and 26cm
    // this makes the ball roll back and forth on the screen visualizer!
    distance_val = 15.5f + (sin(millis() / 1000.0f) * 8.0f);
    control_output = cos(millis() / 1000.0f) * 50.0f; // fake motor speed

    current_distance = distance_val;
    current_speed = control_output;

    delay(50);
    return; // skip the rest of the real hardware loop
  }

  // normal hardware mode
  uint16_t mm = sensor->readRangeContinuousMillimeters();
  if (!sensor->timeoutOccurred() && mm < 1200) {
    float raw_distance = (float)mm / 10.0f; // mm to cm
    
    // 3 point rolling average
    static float history[3] = {0.0f, 0.0f, 0.0f};
    static uint8_t h_idx = 0;
    static bool first_read = true;

    if (first_read) {
      history[0] = history[1] = history[2] = raw_distance;
      first_read = false;
    } else {
      history[h_idx] = raw_distance;
      h_idx = (h_idx + 1) % 3; 
    }

    distance_val = (history[0] + history[1] + history[2]) / 3.0f;
    
    // evaluate if the ball is out of bounds
    bool ball_absent = (distance_val >= BALL_ABSENT_DIST_CM);
    bool ball_stuck = (distance_val <= BALL_MIN_DIST_CM || distance_val >= BALL_MAX_DIST_CM);

    switch (current_state) {
      case STATE_BALANCING:
        if (ball_absent || ball_stuck) {
          Serial.printf("ERR: Ball fault (%.1f cm). Initiating reset...\r\n", distance_val);
          current_state = STATE_NEEDS_RESET;
        } else {
          myPID->Compute();
          motor->set_angle(control_output);
        }
        break;

      case STATE_NEEDS_RESET:
        myPID->SetMode(MANUAL); 
        control_output = 0.0f; 
        motor->reset_beam_angle(); 
        
        Serial.println("Reset complete. Waiting for ball at center...");
        current_state = STATE_WAITING;
        first_read = true; 
        break;

      case STATE_WAITING:
        if (distance_val >= BALL_RESUME_MIN_CM && distance_val <= BALL_RESUME_MAX_CM) {
          Serial.printf("Ball detected at %.1f cm. Resuming balance!\r\n", distance_val);
          myPID->SetMode(AUTOMATIC);
          current_state = STATE_BALANCING;
        }
        break;
    }

    current_distance = distance_val;
    current_speed = control_output;
  }

  delay(50);
}
