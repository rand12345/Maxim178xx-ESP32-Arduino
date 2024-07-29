#include "M5StampC3LED.h"

#define LED_PIN ((uint16_t)2)

M5StampC3LED::M5StampC3LED() {
  this->isOn = true;
  this->pixels = new Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);
  this->pixels->begin();
}

M5StampC3LED::~M5StampC3LED() {
  this->clear();
  delete this->pixels;
}

M5StampC3LED& M5StampC3LED::red(uint8_t r) {
  redValue = r;
  return *this;
}

M5StampC3LED& M5StampC3LED::green(uint8_t g) {
  greenValue = g;
  return *this;
}

M5StampC3LED& M5StampC3LED::blue(uint8_t b) {
  blueValue = b;
  return *this;
}

void M5StampC3LED::update() {
  if (isOn) {
    for (uint16_t i = 0; i < pixels->numPixels(); i++) {
      pixels->setPixelColor(i, pixels->Color(redValue, greenValue, blueValue));
    }
  } else {
    clear();
  }
  pixels->show();
}

void M5StampC3LED::clear() {
  for (uint16_t i = 0; i < pixels->numPixels(); i++) {
    pixels->setPixelColor(i, 0);
  }
  pixels->show();
}

bool M5StampC3LED::toggle() {
  isOn = !isOn;
  update();
  return isOn;
}

void M5StampC3LED::show(uint8_t r, uint8_t g, uint8_t b) {
  redValue = r;
  greenValue = g;
  blueValue = b;
  update();
}
