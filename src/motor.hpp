#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "pico/stdlib.h"

class MOTOR {
public:
  MOTOR(int step_pin, int dir_pin);

  void set_angle(float angle);
  void set_motor_pins(int step_pin, int dir_pin);
  float get_current_angle();
  void set_pulsewidth_us(uint us);
  void reset_beam_angle();

private:
  int STEP_PIN;
  int DIR_PIN;
  int pulsewidth = 1000;
  int current_angle = 0;
  const float angle_conversion =
      1.8 / 8; // 1.8 steps with 8 microsteps in between

  void change_angle(int angle);
};

#endif // MOTOR_HPP
