#pragma once
#include <stdint.h>

void initScreen();

void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime);
