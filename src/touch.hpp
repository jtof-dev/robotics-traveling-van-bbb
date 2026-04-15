#pragma once
#include "configuration.hpp"
#include <Arduino.h>

// Initializes the I2C bus and resets the touch controller
void initTouch();

// Returns true if touched, and populates the x and y variables
bool readTouch(uint16_t &x, uint16_t &y);
