#include "touch.hpp"
#include "configuration.hpp"
#include <Wire.h>

// select the correct i2c bus based on configuration.hpp
#if TOUCH_I2C_PORT == 1
#define TOUCH_WIRE Wire1
#else
#define TOUCH_WIRE Wire
#endif

void initTouch() {
  // stabilize the interrupt pin during boot
  // forces standard 0x38 address
  pinMode(TOUCH_INT, INPUT_PULLUP);

  // hardware reset the touch controller
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(20);
  digitalWrite(TOUCH_RST, HIGH);
  delay(100); // give it plenty of time to boot up

  // initialize the selected i2c bus for the touch controller
  TOUCH_WIRE.setSDA(TOUCH_SDA);
  TOUCH_WIRE.setSCL(TOUCH_SCL);
  TOUCH_WIRE.begin();

  // check if device is alive
  TOUCH_WIRE.beginTransmission(FT6336U_ADDR);
  if (TOUCH_WIRE.endTransmission() == 0) {
    Serial.println("\nSUCCESS: FT6336U Touch Controller Found!");
  } else {
    Serial.printf("\nERROR: Touch Controller NOT found at 0x%02X.\n",
                  FT6336U_ADDR);
    Serial.println(
        "Check: SDA/SCL flipped? Ground connected? Bad jumper wire?");
  }
}

bool readTouch(uint16_t &x, uint16_t &y) {
  TOUCH_WIRE.beginTransmission(FT6336U_ADDR);
  TOUCH_WIRE.write(0x02);
  if (TOUCH_WIRE.endTransmission() != 0)
    return false;

  TOUCH_WIRE.requestFrom((uint8_t)FT6336U_ADDR, (uint8_t)5);
  if (TOUCH_WIRE.available() >= 5) {
    uint8_t touches = TOUCH_WIRE.read() & 0x0F;
    if (touches == 0)
      return false;

    uint8_t xh = TOUCH_WIRE.read();
    uint8_t xl = TOUCH_WIRE.read();
    uint8_t yh = TOUCH_WIRE.read();
    uint8_t yl = TOUCH_WIRE.read();

    x = ((xh & 0x0F) << 8) | xl;
    y = ((yh & 0x0F) << 8) | yl;
    return true;
  }
  return false;
}
