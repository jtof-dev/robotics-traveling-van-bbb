#include "screen.hpp"
#include <Arduino.h>
#include <TFT_eSPI.h>

// instantiate the display object locally
TFT_eSPI tft = TFT_eSPI();

void initScreen() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // draw static UI framework
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Ball Balancer Monitor");
  tft.drawLine(10, 35, 310, 35, TFT_BLUE);
}

void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime) {
  static int counter = 0;
  tft.setTextSize(2);

  // print PID and balancing metrics
  // some placeholder UI to draw to show that the touchscreen works
  tft.setCursor(10, 50);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.printf("Dist:  %5.1f cm   \n", distance);

  tft.setCursor(10, 80);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.printf("Speed: %5.1f s/s \n", speed);

  // print system metrics
  tft.setCursor(10, 110);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.printf("RAM:  %5lu KB   \n", freeRam / 1024);

  tft.setCursor(10, 140);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.printf("Time: %5lu ms   \n", loopTime);

  // print loop count to ensure UI is not frozen
  tft.setCursor(10, 170);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Ticks: %d        \n", counter);

  counter++;
}
