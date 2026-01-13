#include "LCD.h"
#include <LiquidCrystal_PCF8574.h>

// Stored config
static uint8_t   s_addr = 0x27;
static uint8_t   s_cols = 16;
static uint8_t   s_rows = 2;
static TwoWire*  s_bus  = &Wire;

// Single global instance
static LiquidCrystal_PCF8574* s_lcd = nullptr;

static void ensure_instance() {
  if (!s_lcd) {
    s_lcd = new LiquidCrystal_PCF8574(s_addr);
  }
}

void lcd_begin(TwoWire& bus, uint8_t addr, uint8_t cols, uint8_t rows) {
  s_bus  = &bus;
  s_addr = addr;
  s_cols = cols;
  s_rows = rows;

  ensure_instance();

  // NOTE: library expects a TwoWire& — pass *s_bus
  s_lcd->begin(s_cols, s_rows, *s_bus);
  s_lcd->setBacklight(255);
  s_lcd->clear();
}

void lcd_begin(uint8_t addr, uint8_t cols, uint8_t rows) {
  s_bus  = &Wire;
  s_addr = addr;
  s_cols = cols;
  s_rows = rows;

  ensure_instance();
  s_lcd->begin(s_cols, s_rows, *s_bus);  // <-- dereference
  s_lcd->setBacklight(255);
  s_lcd->clear();
}

void lcd_clear() {
  if (!s_lcd) return;
  s_lcd->clear();
}

void lcd_set_backlight(uint8_t level) {
  if (!s_lcd) return;
  s_lcd->setBacklight(level);  // 0..255
}

void lcd_show_text(const char* text, uint8_t len, bool force) {
  if (!s_lcd || !text || len == 0) return;

  // static unsigned long lastUpdate = 0;
  // const unsigned long interval = 1000;  // ms between updates (tune this)

  // unsigned long now = millis();
  // if (now - lastUpdate < interval) {
  //   return;  // too soon, skip
  // }
  // lastUpdate = now;

  if (!force) {
    static unsigned long lastUpdate = 0;
    const unsigned long interval = 1000;

    unsigned long now = millis();
    if (now - lastUpdate < interval) {
        return;
    }
    lastUpdate = now;
  }

  s_lcd->clear();

  uint8_t row = 0, col = 0;
  for (uint8_t i = 0; i < len && row < s_rows; i++) {
    char c = text[i];

    if (c == '\n') { row++; col = 0; continue; }
    if (col >= s_cols) { row++; col = 0; if (row >= s_rows) break; }

    s_lcd->setCursor(col, row);
    s_lcd->print(c);
    col++;
  }
}

