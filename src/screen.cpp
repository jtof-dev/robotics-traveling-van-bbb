#include "screen.hpp"
#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// layout ui variables for easy editing
const int COL_LEFT = 10;
const int COL_RIGHT = 250;
const int BTN_WIDTH = 210;
const int BTN_HEIGHT = 50;

const int ROW_1 = 20;
const int ROW_2 = 100;
const int ROW_3 = 190;

// beam visualizer math
const int BEAM_Y = 290;
const int BEAM_X_START = 30;
const int BEAM_X_END = 450;

int mapDistanceToPixels(float dist) {
  if (dist < 5.0f)
    dist = 5.0f;
  if (dist > 26.0f)
    dist = 26.0f;
  return (int)((dist - 5.0f) * (BEAM_X_END - BEAM_X_START) / (26.0f - 5.0f) +
               BEAM_X_START);
}

void initScreen() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // draw static buttons
  tft.drawRect(COL_RIGHT, ROW_1, BTN_WIDTH, BTN_HEIGHT, TFT_WHITE);

  tft.fillRect(COL_RIGHT, ROW_2, BTN_WIDTH, BTN_HEIGHT, TFT_DARKGREY);
  tft.drawRect(COL_RIGHT, ROW_2, BTN_WIDTH, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.drawCentreString("FORCE RESET", COL_RIGHT + (BTN_WIDTH / 2), ROW_2 + 12,
                       4);

  int half_btn = (BTN_WIDTH / 2) - 5;
  int plus_x = COL_RIGHT + half_btn + 10;

  // [-] button
  tft.fillRect(COL_RIGHT, ROW_3, half_btn, BTN_HEIGHT, TFT_MAROON);
  tft.drawRect(COL_RIGHT, ROW_3, half_btn, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.drawCentreString("-", COL_RIGHT + (half_btn / 2), ROW_3 + 10, 4);

  // [+] button
  tft.fillRect(plus_x, ROW_3, half_btn, BTN_HEIGHT, TFT_DARKGREEN);
  tft.drawRect(plus_x, ROW_3, half_btn, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.drawCentreString("+", plus_x + (half_btn / 2), ROW_3 + 10, 4);

  // draw static beam visualizer elements
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);

  tft.drawString("5cm (Motor)", 15, BEAM_Y - 30, 2);
  tft.drawRightString("26cm", 465, BEAM_Y - 30, 2);
}

void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime, float current_setpoint,
                  bool is_balancing) {
  static int counter = 0;
  static int prev_ball_x = -1;
  static int prev_sp_x = -1;

  static bool prev_balancing = !is_balancing;
  static float prev_setpoint = -1.0f;

  tft.setTextSize(2);

  // update left column (metrics)
  tft.setCursor(COL_LEFT, ROW_1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.printf("Dist:  %5.1f cm   \n", distance);

  tft.setCursor(COL_LEFT, ROW_1 + 40);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.printf("Speed: %5.1f s/s \n", speed);

  tft.setCursor(COL_LEFT, ROW_1 + 80);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.printf("RAM:   %5lu KB   \n", freeRam / 1024);

  tft.setCursor(COL_LEFT, ROW_1 + 120);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.printf("Time:  %5lu ms   \n", loopTime);

  tft.setCursor(COL_LEFT, ROW_1 + 160);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Ticks: %d        \n", counter);

  // update right column
  if (is_balancing != prev_balancing) {
    if (is_balancing) {
      tft.fillRect(COL_RIGHT + 1, ROW_1 + 1, BTN_WIDTH - 2, BTN_HEIGHT - 2,
                   TFT_DARKGREEN);
      tft.setTextColor(TFT_WHITE);
      tft.drawCentreString("SYSTEM: ON", COL_RIGHT + (BTN_WIDTH / 2),
                           ROW_1 + 13, 2);
    } else {
      tft.fillRect(COL_RIGHT + 1, ROW_1 + 1, BTN_WIDTH - 2, BTN_HEIGHT - 2,
                   TFT_MAROON);
      tft.setTextColor(TFT_WHITE);
      tft.drawCentreString("SYSTEM: OFF", COL_RIGHT + (BTN_WIDTH / 2),
                           ROW_1 + 13, 2);
    }
    prev_balancing = is_balancing;
  }

  if (current_setpoint != prev_setpoint) {
    tft.fillRect(COL_RIGHT, ROW_3 - 25, BTN_WIDTH, 25, TFT_BLACK);
    tft.setTextColor(TFT_CYAN);

    char targetBuf[32];
    sprintf(targetBuf, "Target: %.1f cm", current_setpoint);

    tft.drawCentreString(targetBuf, COL_RIGHT + (BTN_WIDTH / 2), ROW_3 - 20, 1);

    prev_setpoint = current_setpoint;
  }

  // live beam visualizer
  int current_ball_x = mapDistanceToPixels(distance);
  int current_sp_x = mapDistanceToPixels(current_setpoint);

  if (prev_ball_x != -1) {
    tft.fillCircle(prev_ball_x, BEAM_Y, 12, TFT_BLACK);
    tft.drawFastVLine(prev_sp_x, BEAM_Y - 10, 20, TFT_BLACK);
  }

  tft.drawLine(BEAM_X_START, BEAM_Y, BEAM_X_END, BEAM_Y, TFT_DARKGREY);
  tft.drawFastVLine(current_sp_x, BEAM_Y - 10, 20, TFT_BLUE);

  if (distance >= 28.0f || distance <= 5.0f || distance >= 26.0f) {
    tft.fillCircle(current_ball_x, BEAM_Y, 10, TFT_RED);
  } else {
    tft.fillCircle(current_ball_x, BEAM_Y, 10, TFT_ORANGE);
  }

  prev_ball_x = current_ball_x;
  prev_sp_x = current_sp_x;
  counter++;
}

ButtonID checkButtons(uint16_t touchX, uint16_t touchY) {
  if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + BTN_WIDTH &&
      touchY >= ROW_1 && touchY <= ROW_1 + BTN_HEIGHT)
    return BTN_TOGGLE_BALANCE;

  if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + BTN_WIDTH &&
      touchY >= ROW_2 && touchY <= ROW_2 + BTN_HEIGHT)
    return BTN_RESET;

  int half_btn = (BTN_WIDTH / 2) - 5;
  int plus_x = COL_RIGHT + half_btn + 10;

  if (touchY >= ROW_3 && touchY <= ROW_3 + BTN_HEIGHT) {
    if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + half_btn)
      return BTN_SETPOINT_DOWN;
    if (touchX >= plus_x && touchX <= plus_x + half_btn)
      return BTN_SETPOINT_UP;
  }
  return BTN_NONE;
}
