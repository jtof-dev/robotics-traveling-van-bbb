#include <LovyanGFX.hpp>
#include <lvgl.h>

/* ---------------------------------------------------------------------
 * LovyanGFX Custom Configuration for Raspberry Pi Pico
 * --------------------------------------------------------------------- */
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Touch_FT5x06 _touch_instance; // LovyanGFX handles FT6336 here

public:
  LGFX(void) {
    { // SPI Bus Configuration (Using SPI0 on Pico)
      auto cfg = _bus_instance.config();
      cfg.spi_host = 0; // Hardware SPI0
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000; // 40 MHz
      cfg.freq_read = 16000000;  // 16 MHz
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = 1;

      // Custom Pins avoiding 4, 5, 14, 15
      cfg.pin_miso = 16;
      cfg.pin_cs = 17;
      cfg.pin_sclk = 18;
      cfg.pin_mosi = 19;
      cfg.pin_dc = 20;

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // Display Panel Configuration
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 17;
      cfg.pin_rst = 21;
      cfg.pin_busy = -1;
      cfg.memory_width = 320;
      cfg.memory_height = 480;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;

      _panel_instance.config(cfg);
    }

    { // Touch Screen Configuration
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = 319;
      cfg.y_min = 0;
      cfg.y_max = 479;
      cfg.pin_int = -1;
      cfg.pin_rst = -1;
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      // Custom I2C Pins avoiding 4, 5, 14, 15
      cfg.i2c_port = 0;    // Hardware I2C0
      cfg.i2c_addr = 0x38; // Default FT6336 address
      cfg.pin_sda = 12;
      cfg.pin_scl = 13;
      cfg.freq = 400000;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX tft;

/* ---------------------------------------------------------------------
 * LVGL v9 Setup & Variables
 * --------------------------------------------------------------------- */
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 480

// Buffer in v9 must be sized in bytes. 2 bytes per pixel for RGB565.
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * 2)
uint8_t draw_buf[DRAW_BUF_SIZE];

/* ---------------------------------------------------------------------
 * LVGL v9 Display Flushing Callback
 * --------------------------------------------------------------------- */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  // LovyanGFX handles the color format pushing efficiently
  tft.pushImage(area->x1, area->y1, w, h, (lgfx::rgb565_t *)px_map);

  lv_display_flush_ready(disp);
}

/* ---------------------------------------------------------------------
 * LVGL v9 Touchpad Reading Callback
 * --------------------------------------------------------------------- */
void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t touchX, touchY;

  // LovyanGFX seamlessly polls the I2C FT6336 internally
  if (tft.getTouch(&touchX, &touchY)) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

/* ---------------------------------------------------------------------
 * Setup & Main Loop
 * --------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);

  // 1. Initialize LovyanGFX (Initializes both ST7796 and FT6336)
  tft.init();
  tft.setRotation(0);

  // 2. Initialize LVGL v9 Core
  lv_init();

  // 3. Register Display in v9
  lv_display_t *disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(disp, my_disp_flush);

  // 4. Register Input Device (Touch) in v9
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read);

  // 5. Create a Simple LVGL v9 UI
  lv_obj_t *btn = lv_button_create(lv_screen_active());
  lv_obj_set_size(btn, 180, 60);
  lv_obj_center(btn);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, "LVGL 9 + Lovyan!");
  lv_obj_center(label);
}

void loop() {
  // Let LVGL process UI updates and inputs
  lv_timer_handler();
  delay(5);
}
