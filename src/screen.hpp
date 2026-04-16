#pragma once
#include <Arduino.h>

enum ButtonID {
  BTN_NONE = 0,
  BTN_TOGGLE_BALANCE,
  BTN_RESET,
  BTN_SETPOINT_DOWN,
  BTN_SETPOINT_UP
};

void initScreen();

void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime, float current_setpoint, bool is_balancing);

ButtonID checkButtons(uint16_t touchX, uint16_t touchY);
