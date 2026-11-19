// Brightness override above 100 %: shown brightness is look x dimmer, capped at 255, with the same glide.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static void settleLook(int look) {
  resetOverrides();
  startTestHold(-1, look, 0);
  uint32_t until = millis() + 10000;
  while (millis() < until) loop();
}

// Sets the override, runs 600 frames and returns the ms until the shown brightness last changed.
static uint32_t settle(int pct, bool checkMonotonic, int* falls) {
  setBrightnessPct(pct);
  uint32_t t0 = millis(), reached = t0;
  int prev = shownBrightness();
  for (int f = 0; f < 600; f++) {
    loop();
    int now = shownBrightness();
    if (checkMonotonic && now < prev) (*falls)++;
    if (now != prev) reached = millis();
    prev = now;
  }
  return reached - t0;
}

static void ceiling() {
  settleLook(0);
  int falls = 0;
  settle(400, true, &falls);
  printf("day at 400%%: shown %d, %d falling frames\n", shownBrightness(), falls);
  CHECK(falls == 0, "day at 400%%: brightness fell %d times during the glide (wrapped)", falls);
  CHECK(shownBrightness() == 255, "day at 400%%: shown %d, want 255", shownBrightness());
}

static void scaled() {
  int falls = 0;
  settleLook(3);
  settle(400, false, &falls);
  printf("embers at 400%%: %d\n", shownBrightness());
  CHECK(shownBrightness() == 40, "embers at 400%%: %d, want 40", shownBrightness());
  settleLook(1);
  settle(200, false, &falls);
  printf("golden at 200%%: %d\n", shownBrightness());
  CHECK(shownBrightness() == 140, "golden at 200%%: %d, want 140", shownBrightness());
  settleLook(0);
  settle(0, false, &falls);
  CHECK(shownBrightness() == 0, "0%%: %d, want 0", shownBrightness());
  settle(100, false, &falls);
  CHECK(shownBrightness() == (uint8_t)lroundf(activeLook.brightness), "100%%: %d, want look brightness %.0f", shownBrightness(), activeLook.brightness);
}

static void timing() {
  int falls = 0;
  settleLook(0);
  setBrightnessPct(400);
  uint32_t t0 = millis();
  while (millis() - t0 < 1100) loop();
  printf("100->400 after 1.1 s: dimmer %.3f\n", dimmer);
  CHECK(fabsf(dimmer - 4.0f) < 0.01f, "dimmer %.3f after 1.1 s, want 4", dimmer);
  settleLook(0);
  setBrightnessPct(400);
  t0 = millis();
  while (millis() - t0 < 500) loop();
  printf("100->400 after 0.5 s: dimmer %.3f\n", dimmer);
  CHECK(dimmer > 1.5f && dimmer < 3.0f, "dimmer %.3f after 0.5 s, want about halfway (2.5)", dimmer);
  settleLook(0);
  uint32_t down = settle(0, false, &falls);
  printf("100->0 reached 0 after %u ms\n", down);
  CHECK(down >= 900 && down <= 1100, "100%% to 0%% took %u ms, want about 1000", down);
}

static void sleepRamp() {
  int falls = 0;
  settleLook(0);
  settle(300, false, &falls);
  CHECK(shownBrightness() == 255, "300%% on day: %d, want 255", shownBrightness());
  startSleep(1);
  uint32_t t0 = millis();
  while (millis() - t0 < 30000) loop();
  printf("sleep from 300%%: dimmer %.2f after 30 s\n", dimmer);
  CHECK(fabsf(dimmer - 1.5f) < 0.05f, "dimmer %.2f after 30 s of a 1 minute sleep from 300%%, want 1.5", dimmer);
  while (millis() - t0 < 61000) loop();
  CHECK(shownBrightness() == 0, "after the sleep: %d, want 0", shownBrightness());
}

static void reset() {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  resetOverrides();
  uint32_t t0 = millis();
  while (millis() - t0 < 3100) loop();
  printf("reset from 400%%: dimmer %.2f after 3.1 s\n", dimmer);
  CHECK(fabsf(dimmer - 1.0f) < 0.01f, "dimmer %.2f after reset, want 1", dimmer);
  CHECK(brightnessPct == 100, "brightnessPct %d after reset, want 100", brightnessPct);
}

int main() {
  simRng.seed(1);
  setup();
  ceiling();
  scaled();
  timing();
  sleepRamp();
  reset();
  printf(failures ? "brightness: %d failure(s)\n" : "brightness: ok\n", failures);
  return failures != 0;
}
