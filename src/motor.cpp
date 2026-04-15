#include "motor.hpp"
#include "configuration.hpp"
#include "pico/stdlib.h"
#include <math.h>

MOTOR::MOTOR(int step_pin, int dir_pin) { set_motor_pins(step_pin, dir_pin); }

void MOTOR::set_angle(float angle) {

  int new_angle = angle / 0.225;

  if (abs(new_angle - current_angle) > 1) {
    change_angle(new_angle - current_angle);
    current_angle = current_angle + (new_angle - current_angle);
  }
}

void MOTOR::set_motor_pins(int step_pin, int dir_pin) {
  STEP_PIN = step_pin;
  DIR_PIN = dir_pin;
}

float MOTOR::get_current_angle() { return current_angle * angle_conversion; }

// void MOTOR::set_pulsewidth_us(uint us) { pulsewidth = 1000; }
void MOTOR::set_pulsewidth_us(uint us) { pulsewidth = us; }

void MOTOR::reset_beam_angle() {
  // save the normal balancing speed
  uint original_pulsewidth = pulsewidth;

  // set a slower speed for the reset routine
  pulsewidth = 3000; // larger is slower

  // run the reset movements
  // changing the angle by 120 guarantees that the beam will be on the edge
  change_angle(60 / angle_conversion);
  sleep_ms(1000);
  change_angle(-30 / angle_conversion);
  sleep_ms(1000);
  current_angle = 0;

  // restore the fast speed so the PID loop works properly afterwards
  pulsewidth = original_pulsewidth;
}

void MOTOR::change_angle(int angle) {
  int new_angle = abs(angle);
  int counter = 0;

  if (angle > 0) {
    gpio_put(DIR_PIN, CLOCKWISE);
  } else {
    gpio_put(DIR_PIN, COUNTER_CLOCKWISE);
  }
  sleep_us(10);

  while (counter < new_angle) {
    gpio_put(STEP_PIN, 1);
    sleep_us(pulsewidth / 2);
    gpio_put(STEP_PIN, 0);
    sleep_us(pulsewidth / 2);

    counter++;
  }
}
