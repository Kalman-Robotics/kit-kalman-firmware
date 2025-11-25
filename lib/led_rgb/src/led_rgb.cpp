#include "led_rgb.h"

RGBLedControl::RGBLedControl(uint8_t pin) 
  : pixel_rgb(1, pin, NEO_GRBW + NEO_KHZ800), gpio_pin(pin) {
}

void RGBLedControl::begin() {
  pixel_rgb.begin();
  RGBLedControl::turnOff();
  pixel_rgb.show();
}

void RGBLedControl::setColor(uint8_t red, uint8_t green, uint8_t blue, 
                             uint8_t brightness, bool enabled) {
  if (enabled) {
    pixel_rgb.setBrightness(brightness);
    pixel_rgb.setPixelColor(0, pixel_rgb.Color(red, green, blue));
  } else {
    RGBLedControl::turnOff();
  }
  pixel_rgb.show();
}

void RGBLedControl::turnOff() {
  pixel_rgb.setPixelColor(0, pixel_rgb.Color(0, 0, 0));
}