#include "touch.hpp"
#include <Wire.h>

void initTouch() {
  // 1. Stabilize the interrupt pin during boot (forces standard 0x38 address)
  pinMode(TOUCH_INT, INPUT_PULLUP);

  // 2. Hardware Reset the Touch Controller
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(20);
  digitalWrite(TOUCH_RST, HIGH);
  delay(100); // Give it plenty of time to boot up

  // 3. Initialize I2C1 for the Touch Controller
  Wire1.setSDA(TOUCH_SDA);
  Wire1.setSCL(TOUCH_SCL);
  Wire1.begin();

  // --- DIAGNOSTIC: Check if device is alive ---
  Wire1.beginTransmission(FT6336U_ADDR);
  if (Wire1.endTransmission() == 0) {
    Serial.println("\nSUCCESS: FT6336U Touch Controller Found!");
  } else {
    Serial.printf("\nERROR: Touch Controller NOT found at 0x%02X.\n",
                  FT6336U_ADDR);
    Serial.println(
        "Check: SDA/SCL flipped? Ground connected? Bad jumper wire?");
  }
}

bool readTouch(uint16_t &x, uint16_t &y) {
  Wire1.beginTransmission(FT6336U_ADDR);
  Wire1.write(0x02);
  if (Wire1.endTransmission() != 0)
    return false;

  Wire1.requestFrom((uint8_t)FT6336U_ADDR, (uint8_t)5);
  if (Wire1.available() >= 5) {
    uint8_t touches = Wire1.read() & 0x0F;
    if (touches == 0)
      return false;

    uint8_t xh = Wire1.read();
    uint8_t xl = Wire1.read();
    uint8_t yh = Wire1.read();
    uint8_t yl = Wire1.read();

    x = ((xh & 0x0F) << 8) | xl;
    y = ((yh & 0x0F) << 8) | yl;
    return true;
  }
  return false;
}
