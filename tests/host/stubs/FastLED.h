#pragma once
#include <Arduino.h>
struct CRGB {
  union { struct { uint8_t r, g, b; }; uint8_t raw[3]; };
  CRGB() : r(0), g(0), b(0) {}
  CRGB(int R, int G, int B) : r((uint8_t)R), g((uint8_t)G), b((uint8_t)B) {}
  uint8_t& operator[](int i) { return raw[i]; }
  const uint8_t& operator[](int i) const { return raw[i]; }
  CRGB& operator+=(const CRGB& o) {
    r = std::min(255, r + o.r); g = std::min(255, g + o.g); b = std::min(255, b + o.b); return *this;
  }
};
template <int N> struct CRGBArray {
  CRGB leds[N];
  CRGB& operator[](int i) { return leds[i]; }
  operator CRGB*() { return leds; }
};
inline uint8_t scale8(uint8_t i, uint8_t s) { return ((uint16_t)i * (1 + (uint16_t)s)) >> 8; }
inline void fadeToBlackBy(CRGB* l, int n, uint8_t amt) {
  for (int i = 0; i < n; i++) for (int c = 0; c < 3; c++) l[i][c] = scale8(l[i][c], 255 - amt);
}
inline uint8_t blend8(uint8_t a, uint8_t b, uint8_t amt) {
  uint16_t partial = (a << 8) | b;
  partial += (b * amt);
  partial -= (a * amt);
  return partial >> 8;
}
inline CRGB blend(const CRGB& a, const CRGB& b, uint8_t amt) {
  return CRGB(blend8(a.r, b.r, amt), blend8(a.g, b.g, amt), blend8(a.b, b.b, amt));
}
#define WS2812B 0
enum { GRB = 0 };
void simOnShow();
struct FastLEDStub {
  CRGB* leds = nullptr; int n = 0; uint8_t brightness = 255;
  template <int CHIP, int PIN, int ORDER> void addLeds(CRGB* l, int count) { leds = l; n = count; }
  void setBrightness(uint8_t b) { brightness = b; }
  uint8_t getBrightness() { return brightness; }
  void setMaxPowerInVoltsAndMilliamps(int, int) {}
  void setDither(int) {}
  void clear(bool = false) { for (int i = 0; i < n; i++) leds[i] = CRGB(); }
  void show() { simOnShow(); }
};
static FastLEDStub FastLED;
