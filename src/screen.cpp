#include "screen.hpp"
#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// --- Layout Math (480x320 Screen) ---
const int COL_LEFT = 10;
const int COL_RIGHT = 250;
const int BTN_WIDTH = 210;
const int BTN_HEIGHT = 60;

const int ROW_1 = 50;
const int ROW_2 = 130;
const int ROW_3 = 210;

// --- Beam Visualizer Math ---
const int BEAM_Y = 295;      // Y coordinate for the visual beam
const int BEAM_X_START = 30; // Left edge of the beam (pixels)
const int BEAM_X_END = 450;  // Right edge of the beam (pixels)

// Helper function to map float distance to pixel coordinates
int mapDistanceToPixels(float dist) {
  // Constrain the distance to your physical limits so the ball doesn't draw
  // off-screen
  if (dist < 5.0f)
    dist = 5.0f;
  if (dist > 26.0f)
    dist = 26.0f;

  // Map 5cm-26cm to X-coordinates 30-450
  return (int)((dist - 5.0f) * (BEAM_X_END - BEAM_X_START) / (26.0f - 5.0f) +
               BEAM_X_START);
}

void initScreen() {
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);

  // --- Static Text ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Ball Balancer Terminal");
  tft.drawLine(10, 35, 470, 35, TFT_BLUE);

  // --- Draw Static Buttons ---
  // 1. Toggle Button Background
  tft.drawRect(COL_RIGHT, ROW_1, BTN_WIDTH, BTN_HEIGHT, TFT_WHITE);

  // 2. Reset Button Background
  tft.fillRect(COL_RIGHT, ROW_2, BTN_WIDTH, BTN_HEIGHT, TFT_DARKGREY);
  tft.drawRect(COL_RIGHT, ROW_2, BTN_WIDTH, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.drawCentreString("FORCE RESET", COL_RIGHT + (BTN_WIDTH / 2), ROW_2 + 20,
                       2);

  // 3. Setpoint Adjust Buttons
  int half_btn = (BTN_WIDTH / 2) - 5;
  tft.fillRect(COL_RIGHT, ROW_3, half_btn, BTN_HEIGHT, TFT_MAROON);
  tft.drawRect(COL_RIGHT, ROW_3, half_btn, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_MAROON);
  tft.drawCentreString("-", COL_RIGHT + (half_btn / 2), ROW_3 + 20, 4);

  int plus_x = COL_RIGHT + half_btn + 10;
  tft.fillRect(plus_x, ROW_3, half_btn, BTN_HEIGHT, TFT_DARKGREEN);
  tft.drawRect(plus_x, ROW_3, half_btn, BTN_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.drawCentreString("+", plus_x + (half_btn / 2), ROW_3 + 20, 4);

  // --- Draw Static Beam Visualizer Elements ---
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.drawCentreString("5cm (Motor)", BEAM_X_START, BEAM_Y + 15, 2);
  tft.drawCentreString("26cm", BEAM_X_END, BEAM_Y + 15, 2);
}

void updateScreen(float distance, float speed, float tempC, uint32_t freeRam,
                  uint32_t loopTime, float current_setpoint,
                  bool is_balancing) {
  static int counter = 0;

  // Variables to remember where the ball was last frame so we can erase it
  static int prev_ball_x = -1;
  static int prev_sp_x = -1;

  tft.setTextSize(2);

  // --- Update Left Column (Metrics) ---
  tft.setCursor(COL_LEFT, ROW_1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.printf("Dist:  %5.1f cm   \n", distance);

  tft.setCursor(COL_LEFT, ROW_1 + 30);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.printf("Speed: %5.1f s/s \n", speed);

  tft.setCursor(COL_LEFT, ROW_1 + 70);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.printf("RAM:   %5lu KB   \n", freeRam / 1024);

  tft.setCursor(COL_LEFT, ROW_1 + 100);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.printf("Time:  %5lu ms   \n", loopTime);

  tft.setCursor(COL_LEFT, ROW_1 + 130);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Ticks: %d        \n", counter);

  // --- Update Right Column (Dynamic Button Data) ---
  if (is_balancing) {
    tft.fillRect(COL_RIGHT + 1, ROW_1 + 1, BTN_WIDTH - 2, BTN_HEIGHT - 2,
                 TFT_DARKGREEN);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    tft.drawCentreString("SYSTEM: ON", COL_RIGHT + (BTN_WIDTH / 2), ROW_1 + 20,
                         2);
  } else {
    tft.fillRect(COL_RIGHT + 1, ROW_1 + 1, BTN_WIDTH - 2, BTN_HEIGHT - 2,
                 TFT_MAROON);
    tft.setTextColor(TFT_WHITE, TFT_MAROON);
    tft.drawCentreString("SYSTEM: OFF", COL_RIGHT + (BTN_WIDTH / 2), ROW_1 + 20,
                         2);
  }

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(COL_RIGHT, ROW_3 - 25);
  tft.printf("Target: %4.1f cm  ", current_setpoint);

  // --- LIVE BEAM VISUALIZER ---

  int current_ball_x = mapDistanceToPixels(distance);
  int current_sp_x = mapDistanceToPixels(current_setpoint);

  // 1. Erase the old ball and setpoint to prevent smearing
  if (prev_ball_x != -1) {
    tft.fillCircle(prev_ball_x, BEAM_Y, 12, TFT_BLACK);       // Erase Ball
    tft.drawFastVLine(prev_sp_x, BEAM_Y - 10, 20, TFT_BLACK); // Erase Setpoint
  }

  // 2. Redraw the underlying beam line (in case the eraser erased a chunk of
  // it)
  tft.drawLine(BEAM_X_START, BEAM_Y, BEAM_X_END, BEAM_Y, TFT_DARKGREY);

  // 3. Draw the target setpoint as a blue vertical marker
  tft.drawFastVLine(current_sp_x, BEAM_Y - 10, 20, TFT_BLUE);

  // 4. Draw the live ToF sensor ball
  // Make it red if it goes out of bounds, orange if it's balancing normally!
  if (distance >= 28.0f || distance <= 5.0f || distance >= 26.0f) {
    tft.fillCircle(current_ball_x, BEAM_Y, 10, TFT_RED);
  } else {
    tft.fillCircle(current_ball_x, BEAM_Y, 10, TFT_ORANGE);
  }

  // Save positions for the next frame
  prev_ball_x = current_ball_x;
  prev_sp_x = current_sp_x;

  counter++;

} // --- Touch Detection Logic ---
ButtonID checkButtons(uint16_t touchX, uint16_t touchY) {
  // Check Toggle Button
  if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + BTN_WIDTH &&
      touchY >= ROW_1 && touchY <= ROW_1 + BTN_HEIGHT) {
    return BTN_TOGGLE_BALANCE;
  }

  // Check Reset Button
  if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + BTN_WIDTH &&
      touchY >= ROW_2 && touchY <= ROW_2 + BTN_HEIGHT) {
    return BTN_RESET;
  }

  // Check Setpoint Buttons
  int half_btn = (BTN_WIDTH / 2) - 5;
  int plus_x = COL_RIGHT + half_btn + 10;

  if (touchY >= ROW_3 && touchY <= ROW_3 + BTN_HEIGHT) {
    if (touchX >= COL_RIGHT && touchX <= COL_RIGHT + half_btn) {
      return BTN_SETPOINT_DOWN;
    }
    if (touchX >= plus_x && touchX <= plus_x + half_btn) {
      return BTN_SETPOINT_UP;
    }
  }

  return BTN_NONE;
}
