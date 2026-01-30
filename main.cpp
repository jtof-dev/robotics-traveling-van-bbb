#include <math.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "PID_v1.h"

#include "configuration.hpp"

class BBB {
public:
  BBB() {
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_init(MOTOR_STEP_PIN);
    gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);

    gpio_init(MOTOR_DIR_PIN);
    gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);
  }

  /*
   * Reads the sensor distance in cm
   */

  double readSensor(void) {

    absolute_time_t start_time, end_time;
    const uint64_t sensor_timeout_time = 2 * 1e6; // us
    const double speed_of_sound = 0.0343;         // cm/us
    double distance;

    gpio_put(TRIGGER_PIN, 0);
    sleep_us(2); // Sleep just a little bit

    // Send a request to read the sensor
    gpio_put(TRIGGER_PIN, 1);
    sleep_us(SENSOR_TRIGGER_TIME);
    gpio_put(TRIGGER_PIN, 0);

    //
    //
    while (gpio_get(ECHO_PIN) == 0)
      ;
    start_time = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1)
      ;
    end_time = get_absolute_time();

    int64_t duration_us = absolute_time_diff_us(start_time, end_time);

    distance = duration_us * speed_of_sound / 2.0f;
    return distance;
  }

  double findMotorPeriod(double rpm) {
    double frequency;

    frequency = rpm / ((STEP_ANGLE / 360.f) * 60.f);

    return 1.0 / frequency;
  }

private:
};

int MOTOR_PWM = 0;
int MOTOR_ENABLED = 1;
int MOTOR_DIRECTION = 0;

int clamp(int x, int min, int max) {

  if (x >= max) {
    return max;
  } else if (x <= min) {
    return min;
  }

  return x;
}

void runPWM() {

  int step_signal = 0;
  // Max has to be 740
  double upper_bound = 400;
  double lower_bound = 100;
  double bound = 0;

  while (true) {

    if (MOTOR_ENABLED == 1) {
      /*
         if (bound <= 30 && bound >= 0) {
         step_signal = ~step_signal;
         gpio_put(MOTOR_STEP_PIN, step_signal);
         sleep_us((MOTOR_PWM));



         }
         */

      if (MOTOR_DIRECTION == COUNTER_CLOCKWISE && bound <= upper_bound) {
        bound += STEP_ANGLE / 2;
        step_signal = ~step_signal;
        gpio_put(MOTOR_STEP_PIN, step_signal);
        sleep_us((MOTOR_PWM));

      } else if (MOTOR_DIRECTION == CLOCKWISE && bound >= lower_bound) {
        bound -= STEP_ANGLE / 2;
        step_signal = ~step_signal;
        gpio_put(MOTOR_STEP_PIN, step_signal);
        sleep_us((MOTOR_PWM));
      }
    }
  }
}

int main(void) {

  BBB brain;
  float distance;
  float set_point = 12;
  float error = 0;
  double period;
  int step_signal = 0;
  int kl;
  int PWM_MAX = 1200;

  double input, output;

  PID mypid(&distance, &error, &set_point, 30, 2, 3, DIRECT);
  mypid.SetMode(AUTOMATIC);
  mypid.SetOutputLimits(-PWM_MAX, PWM_MAX);
  stdio_init_all();

  multicore_launch_core1(runPWM);

  gpio_put(MOTOR_DIR_PIN, 1);

  absolute_time_t last_time = get_absolute_time();
  absolute_time_t ultra_sonic_timer = get_absolute_time();
  while (true) {

    distance = brain.readSensor();
    if (distance < 23 && distance > 0.15) {
      // error = -(distance - set_point);
      // error = pid.calculate_error(error, 0.1);
      mypid.Compute();

      MOTOR_PWM = PWM_MAX - abs(error);
      MOTOR_ENABLED = 1;

      if ((set_point) - (distance) <= 0) {
        MOTOR_DIRECTION = COUNTER_CLOCKWISE;
        gpio_put(MOTOR_DIR_PIN, COUNTER_CLOCKWISE);
      }

      else {
        MOTOR_DIRECTION = CLOCKWISE;
        gpio_put(MOTOR_DIR_PIN, CLOCKWISE);
      }
      printf(
          "Distance: %.2f cm | Error: %.6f | Period = %d us | Direction: %s\n",
          distance, error, MOTOR_PWM, MOTOR_DIRECTION ? "CW" : "CCW");
    }
  }
  return 0;
}
