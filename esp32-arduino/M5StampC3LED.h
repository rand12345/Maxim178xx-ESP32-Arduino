#ifndef M5StampC3LED_h
#define M5StampC3LED_h

//Extended from this library https://github.com/yuuu/M5StampC3LED by Yuuu and added state

#include <Adafruit_NeoPixel.h>

class M5StampC3LED {

public:
  M5StampC3LED();
  ~M5StampC3LED();
  M5StampC3LED& red(uint8_t r);
  M5StampC3LED& green(uint8_t g);
  M5StampC3LED& blue(uint8_t b);
  void update();
  void clear();
  bool toggle();
  void invert();

  void show(uint8_t r, uint8_t g, uint8_t b);
  // void clear();

private:
  Adafruit_NeoPixel* pixels;
  uint8_t redValue;
  uint8_t greenValue;
  uint8_t blueValue;
  bool isOn;
};

#endif