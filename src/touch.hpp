#pragma once
#include "configuration.hpp"
#include <Arduino.h>

void initTouch();

bool readTouch(uint16_t &x, uint16_t &y);
