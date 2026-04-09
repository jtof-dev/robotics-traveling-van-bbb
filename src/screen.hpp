#pragma once
#include <stdint.h>

// Initialize the TFT screen
void initScreen();

// Update the screen with new metrics
void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime);
