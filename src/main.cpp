#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#include "VL53L0X.h"
#include "configuration.hpp"
#include "pid.h"

// added code for magnetic encoder
volatile int position = 0;
volatile bool last_state_a = false;

// ISR for encoder
void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_PIN_A) {
        bool state_a = gpio_get(ENCODER_PIN_A);
        bool state_b = gpio_get(ENCODER_PIN_B);

        if (state_a != last_state_a) {
            if (state_a != state_b) {
                position++; // forward
            } else {
                position--; // reverse
            }
            last_state_a = state_a;
        }
    }
}

/*

// Optimized set_speed with state management
void set_speed(float steps_per_sec) {
  uint slice_num = pwm_gpio_to_slice_num(MOTOR_STEP_PIN);

  // if (steps_per_sec < 10.0f) {
  //   pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_STEP_PIN), 0);
  //   pwm_set_enabled(slice_num, false);
  //   return;
  // }

  // Use a higher divider for low speeds to avoid 16-bit wrap overflow
  // 125 divider = 1MHz clock. Wrap of 65535 = ~15Hz minimum speed.
  float divider = 125.0f;
  uint32_t wrap = 1000000 / steps_per_sec;

  // Safety: PWM wrap is a 16-bit register (max 65535)
  if (wrap > 65535)
    wrap = 65535;

  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MOTOR_STEP_PIN), wrap / 2);
  pwm_set_enabled(slice_num, true);
}

*/

// future core 1 touchscreen function
void core1_entry() {
  while (1) {
    tight_loop_contents();
  }
}

void change_angle(int angle, uint motor_pin, uint dir_pin) {

	//const float conversion = 1.8f/8.f;

	int new_angle = abs(angle);
	int counter = 0;


	if (angle < 0) {
		gpio_put(dir_pin, CLOCKWISE);
	} else {
		gpio_put(dir_pin, COUNTER_CLOCKWISE);
	}

	while (counter < new_angle) {
		gpio_put(motor_pin, 1);
		sleep_us(500);
		gpio_put(motor_pin, 0);
		sleep_us(500);

		counter++;
	}
}


int set_angle(float angle, uint motor_pin, uint dir_pin) {

	static int current_angle = 0;
	int new_angle = angle / 0.225;

	//if (abs(new_angle - current_angle) > 1) {
	change_angle(angle - current_angle, motor_pin, dir_pin); 
	current_angle = current_angle + (new_angle - current_angle);

	//}
	return current_angle;


}


int main() {
  stdio_init_all();
  // allow sensors and power rails to stabilize
  sleep_ms(2000);

  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  
  // ToF sensor
  VL53L0X sensor(I2C_PORT, 0x29);
  if (!sensor.init()) {
    printf("failed to detect VL53L0X sensor!\n");
    
    while (1)
      tight_loop_contents();
      
  }

  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(50);

  //gpio_set_function(MOTOR_STEP_PIN, GPIO_FUNC_PWM);
  gpio_init(MOTOR_STEP_PIN);
  gpio_init(MOTOR_DIR_PIN);
  gpio_set_dir(MOTOR_STEP_PIN, GPIO_OUT);
  gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);

  // initialize encoder pins and enable pull-ups
  gpio_init(ENCODER_PIN_A);
  gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
  gpio_pull_up(ENCODER_PIN_A);

  gpio_init(ENCODER_PIN_B);
  gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
  gpio_pull_up(ENCODER_PIN_B);

  last_state_a = gpio_get(ENCODER_PIN_A);

  // attach interrupt to PIN_A for both rising and falling edges
  gpio_set_irq_enabled_with_callback(ENCODER_PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

  float distance = BALL_SETPOINT_CM;
  float old_distance = 0;
  float speed = 0;
  float set_point = BALL_SETPOINT_CM;
  float control_output = 0.0f;
  int current_angle = 0;

  PID myPID(&distance, &control_output, &set_point, DEFAULT_KP, DEFAULT_KI,
            DEFAULT_KD, DIRECT);

  myPID.SetMode(AUTOMATIC);
  // limits are now in steps/sec
  myPID.SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX);
  myPID.SetSampleTime(PID_SAMPLE_MS);

  multicore_launch_core1(core1_entry);

  int angle = -20;
  int angle_dir = 1;

  //change_angle(50, MOTOR_STEP_PIN, MOTOR_DIR_PIN);

  while (true) {
    if (multicore_fifo_rvalid()) {
      uint32_t cmd = multicore_fifo_pop_blocking();
      if (cmd == 0x01) { // CMD_UPDATE_KP
        uint32_t raw_val = multicore_fifo_pop_blocking();
        float new_kp = (float)raw_val / 100.0f;
        myPID.SetTunings(new_kp, myPID.GetKi(), myPID.GetKd());
      }
    }

    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred() && mm < 1200) {
      old_distance = distance;
      distance = (float)mm / 10.0f; // mm to cm

      speed = (distance - old_distance) / 10; // mm/ms
      myPID.Compute();

	current_angle = set_angle( 0.3*(set_point - distance), MOTOR_STEP_PIN, MOTOR_DIR_PIN);
	sleep_ms(20);

     
      printf("Dist: %.1f cm | Desired Angle: %.1f degrees | Current angle: %f | Encoder: %d\n", distance, control_output, current_angle * 0.225, position);
    } else {
      //set_speed(0);
    }

    sleep_ms(50);
  }

  return 0;
}
