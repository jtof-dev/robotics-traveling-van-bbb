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

// ==============================================================================
// CORE 0: TOUCHSCREEN UI
// ==============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  initScreen();
  initTouch(); 
}

void loop() {
  uint32_t loopStart = millis();
  
  // State tracker for our button lockout
  static bool was_touched = false; 

  uint16_t rawX, rawY;
  bool is_touching = readTouch(rawX, rawY);
  
  if (is_touching) {
    // Only fire the button logic if this is a NEW touch event
    if (!was_touched) {
      was_touched = true; // Lockout engaged until finger is lifted
      
      uint16_t mappedX = rawY; 
      uint16_t mappedY = 320 - rawX;

      Serial.printf("Raw -> X: %d, Y: %d | Mapped -> X: %d, Y: %d\r\n", rawX, rawY, mappedX, mappedY);
      
      ButtonID btn = checkButtons(mappedX, mappedY);
      
      switch(btn) {
        case BTN_TOGGLE_BALANCE:
          system_running = !system_running;
          Serial.println(system_running ? "System ON" : "System OFF");
          break;
          
        case BTN_RESET:
          multicore_fifo_push_blocking(0x04); 
          Serial.println("Force Reset Triggered by Core 0");
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
  } else {
    // Finger was lifted, unlock the buttons
    was_touched = false;
  }

  updateScreen(current_distance, current_speed, analogReadTemp(), rp2040.getFreeHeap(), 
               millis() - loopStart, current_setpoint, system_running);

  delay(50); 
}

// ==============================================================================
// CORE 1: BALL BALANCING & MOTOR CONTROL
// ==============================================================================
void setup1() {
  delay(2000); 

  // FLUSH THE FIFO: Clear out any garbage from soft-reboots!
  while (multicore_fifo_rvalid()) multicore_fifo_pop_blocking();

  // TOF sensor hardware init
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  sensor = new VL53L0X(I2C_PORT, 0x29);
  
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
  
  if (hardware_present) {
    motor->reset_beam_angle();
  }
}

void loop1() {
  set_point = current_setpoint;

  // 1. DRAIN THE FIFO (Prevents Core 0 freeze & handles UI commands)
  while (multicore_fifo_rvalid()) {
    uint32_t cmd = multicore_fifo_pop_blocking();
    
    if (cmd == 0x01) { 
      // ONLY block for a second value if one actually exists!
      if (multicore_fifo_rvalid()) {
        uint32_t raw_val = multicore_fifo_pop_blocking();
        float new_kp = (float)raw_val / 100.0f;
        myPID->SetTunings(new_kp, myPID->GetKi(), myPID->GetKd());
      } else {
        Serial.println("ERR: Received 0x01 but missing payload!");
      }
    } 
    else if (cmd == 0x04) {
      current_state = STATE_NEEDS_RESET;
      Serial.println("Core 1: Executing Hardware Reset...");
    }
    else if (!hardware_present) {
       Serial.printf("Sim Mode: Ignored Command 0x%02X\r\n", cmd);
    }
  }

  // ui simulation mode
  if (!hardware_present) {
    distance_val = 15.5f + (sin(millis() / 1000.0f) * 8.0f);
    control_output = cos(millis() / 1000.0f) * 50.0f; 

    current_distance = distance_val;
    current_speed = control_output;

    delay(50);
    return; 
  }

  // normal hardware mode
  static float history[3] = {0.0f, 0.0f, 0.0f};
  static uint8_t h_idx = 0;
  static bool first_read = true;

  uint16_t mm = sensor->readRangeContinuousMillimeters();
  
  // 2. SENSOR CHECK
  if (!sensor->timeoutOccurred() && mm < 1200) {
    float raw_distance = (float)mm / 10.0f; 

    if (first_read) {
      history[0] = history[1] = history[2] = raw_distance;
      first_read = false;
    } else {
      history[h_idx] = raw_distance;
      h_idx = (h_idx + 1) % 3; 
    }
    distance_val = (history[0] + history[1] + history[2]) / 3.0f;
  } else {
    // If sensor loses ball, force a fault distance
    distance_val = BALL_ABSENT_DIST_CM; 
  }
    
  bool ball_absent = (distance_val >= BALL_ABSENT_DIST_CM);
  bool ball_stuck = (distance_val <= BALL_MIN_DIST_CM || distance_val >= BALL_MAX_DIST_CM);

  // 3. THE STATE MACHINE
  if (!system_running && current_state != STATE_NEEDS_RESET) {
    myPID->SetMode(MANUAL);
    control_output = 0.0f;
    motor->set_angle(0.0f); 
    current_state = STATE_BALANCING; 
  } 
  else {
    switch (current_state) {
      case STATE_BALANCING:
        if (ball_absent || ball_stuck) {
          Serial.printf("ERR: Ball fault (%.1f cm). Initiating reset...\r\n", distance_val);
          current_state = STATE_NEEDS_RESET;
        } else if (system_running) {
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
        } else if (!system_running) {
          motor->set_angle(0.0f); 
        }
        break;
    }
  }

  current_distance = distance_val;
  current_speed = control_output;
  
  delay(50);
}
