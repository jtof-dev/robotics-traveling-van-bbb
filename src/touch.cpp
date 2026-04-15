#include "touch.hpp"
#include <Wire.h>

void initTouch() {
  // 1. Hardware Reset the Touch Controller
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(50); // Give it a moment to boot up

  // 2. Initialize I2C1 for the Touch Controller
  Wire1.setSDA(TOUCH_SDA);
  Wire1.setSCL(TOUCH_SCL);
  Wire1.begin();
}

bool readTouch(uint16_t &x, uint16_t &y) {
  // Read the "TD_STATUS" register (0x02) to see if a finger is touching
  Wire1.beginTransmission(FT6336U_ADDR);
  Wire1.write(0x02);
  if (Wire1.endTransmission() != 0)
    return false; // Sensor not responding

  // Request 5 bytes of data starting from 0x02
  Wire1.requestFrom(FT6336U_ADDR, 5);
  if (Wire1.available() == 5) {
    uint8_t touches = Wire1.read() & 0x0F;
    if (touches == 0)
      return false; // No fingers detected

    uint8_t xh = Wire1.read();
    uint8_t xl = Wire1.read();
    uint8_t yh = Wire1.read();
    uint8_t yl = Wire1.read();

    // Mask the high byte to 4 bits and combine it with the low byte
    x = ((xh & 0x0F) << 8) | xl;
    y = ((yh & 0x0F) << 8) | yl;

    return true;
  }
  return false;
}
