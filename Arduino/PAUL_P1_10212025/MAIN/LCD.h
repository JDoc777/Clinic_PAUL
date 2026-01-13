#pragma once
#include <Arduino.h>
#include <Wire.h>

// Initialize on a specific I2C bus (e.g., Wire2) and address.
// Call after you have called WireX.begin().
void lcd_begin(TwoWire& bus, uint8_t addr = 0x27, uint8_t cols = 16, uint8_t rows = 2);

// Convenience: uses default Wire
void lcd_begin(uint8_t addr = 0x27, uint8_t cols = 16, uint8_t rows = 2);

// Clear display
void lcd_clear();

// Set backlight (0..255)
void lcd_set_backlight(uint8_t level);

// Print up to len chars with wrapping and '\n' support
void lcd_show_text(const char* text, uint8_t len);
