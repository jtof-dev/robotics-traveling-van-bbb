#pragma once
#include <Arduino.h>

// Define our button outputs
enum ButtonID {
  BTN_NONE = 0,
  BTN_TOGGLE_BALANCE,
  BTN_RESET,
  BTN_SETPOINT_DOWN,
  BTN_SETPOINT_UP
};

void initScreen();

// Added setpoint and is_balancing parameters to update the UI state
void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime, float current_setpoint, bool is_balancing);

// Pass your touch coordinates here to see what was pressed
ButtonID checkButtons(uint16_t touchX, uint16_t touchY);
