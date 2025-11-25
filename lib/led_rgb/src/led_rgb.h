#pragma once

#include <Adafruit_NeoPixel.h>

class RGBLedControl {
private:
  Adafruit_NeoPixel pixel_rgb;
  uint8_t gpio_pin;
  
public:
  RGBLedControl(uint8_t pin = 48);
  void begin();
  void setColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness, bool enabled);
  void turnOff();
};