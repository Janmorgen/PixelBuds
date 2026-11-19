// Brightness override above 100 %: the dimmer stops at the headroom where the look reaches 255, so glides and fades
// show over their whole length; the sleep ramp and power fades keep their timing at any setting.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static const float frameMs = FRAME_US / 1000.0f;

static void settleLook(int look) {
  resetOverrides();
  setPower(true, false, 0);
  startTestHold(-1, look, 0);
  uint32_t until = millis() + 10000;
  while (millis() < until) loop();
}

// Runs frames for `ms`.
static void runFor(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) loop();
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

// When the shown brightness first left `from` and first reached `to`, in ms from now, over `ms` of frames; -1 if never.
struct Track { int32_t left, reached; };
static Track track(uint32_t ms, int from, int to) {
  Track r = {-1, -1};
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    loop();
    int s = shownBrightness();
    if (r.left < 0 && s != from) r.left = millis() - t0;
    if (r.reached < 0 && s == to) r.reached = millis() - t0;
  }
  return r;
}

static void ceiling() {
  settleLook(0);
  int falls = 0;
  settle(400, true, &falls);
  printf("day at 400%%: shown %d, dimmer %.3f, brightnessPct %d, %d falling frames\n", shownBrightness(), dimmer,
         brightnessPct, falls);
  CHECK(falls == 0, "day at 400%%: brightness fell %d times during the glide (wrapped)", falls);
  CHECK(shownBrightness() == 255, "day at 400%%: shown %d, want 255", shownBrightness());
  CHECK(fabsf(dimmer - 2.55f) < 0.001f, "day at 400%%: dimmer %.3f, want the headroom 2.55", dimmer);
  CHECK(brightnessPct == 400, "day at 400%%: brightnessPct %d, want 400 as requested", brightnessPct);
}

static void scaled() {
  int falls = 0;
  settleLook(3);
  settle(400, false, &falls);
  printf("embers at 400%%: %d, dimmer %.3f\n", shownBrightness(), dimmer);
  CHECK(shownBrightness() == 40, "embers at 400%%: %d, want 40", shownBrightness());
  CHECK(fabsf(dimmer - 4.0f) < 0.001f, "embers at 400%%: dimmer %.3f, want 4 (under the headroom 25.5)", dimmer);
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
  settleLook(3);
  setBrightnessPct(400);
  runFor(1100);
  printf("embers 100->400 after 1.1 s: dimmer %.3f\n", dimmer);
  CHECK(fabsf(dimmer - 4.0f) < 0.01f, "embers: dimmer %.3f after 1.1 s, want 4", dimmer);
  settleLook(3);
  setBrightnessPct(400);
  runFor(500);
  printf("embers 100->400 after 0.5 s: dimmer %.3f\n", dimmer);
  CHECK(dimmer > 2.2f && dimmer < 2.8f, "embers: dimmer %.3f after 0.5 s, want about halfway (2.5)", dimmer);
  settleLook(0);
  setBrightnessPct(400);
  Track up = track(1500, 100, 255);
  printf("day 100->400: shown reached 255 after %d ms\n", up.reached);
  CHECK(up.reached >= 1000 - frameMs && up.reached <= 1000 + frameMs, "day 100->400 reached 255 after %d ms, want 1000", up.reached);
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
  runFor(30000);
  printf("sleep from 300%%: shown %d after 30 s\n", shownBrightness());
  CHECK(abs(shownBrightness() - 128) <= 1, "shown %d after 30 s of a 1 minute sleep from 300%% on day, want 128", shownBrightness());
  runFor(31000);
  CHECK(shownBrightness() == 0, "after the sleep: %d, want 0", shownBrightness());
}

static void reset() {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  resetOverrides();
  Track t = track(3500, 255, 100);
  printf("reset from 400%% on day: shown left 255 after %d ms, reached 100 after %d ms, dimmer %.2f\n", t.left, t.reached, dimmer);
  CHECK(t.left >= 0 && t.left <= 2 * frameMs, "reset from 400%%: shown held 255 for %d ms, want it to start falling at once", t.left);
  CHECK(t.reached >= 3000 - frameMs && t.reached <= 3000 + frameMs, "reset from 400%%: reached 100 after %d ms, want 3000", t.reached);
  CHECK(fabsf(dimmer - 1.0f) < 0.01f, "dimmer %.2f after reset, want 1", dimmer);
  CHECK(brightnessPct == 100, "brightnessPct %d after reset, want 100", brightnessPct);
}

// Power fades from 400 % on the day look last the requested time and start at once.
static void powerFade() {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  setPower(false, false, 2000);
  Track off = track(2500, 255, 0);
  printf("power off over 2 s from 400%%: left 255 after %d ms, reached 0 after %d ms\n", off.left, off.reached);
  CHECK(off.left >= 0 && off.left <= 2 * frameMs, "power off from 400%%: shown held 255 for %d ms, want it to start falling at once", off.left);
  CHECK(off.reached >= 2000 - frameMs && off.reached <= 2000 + frameMs, "power off from 400%%: reached 0 after %d ms, want 2000", off.reached);
  setPower(true, false, 2000);
  Track on = track(2500, 0, 255);
  printf("power on over 2 s at 400%%: left 0 after %d ms, reached 255 after %d ms\n", on.left, on.reached);
  CHECK(on.left >= 0 && on.left <= 2 * frameMs, "power on at 400%%: shown held 0 for %d ms", on.left);
  CHECK(on.reached >= 2000 - frameMs && on.reached <= 2000 + frameMs, "power on at 400%%: reached 255 after %d ms, want 2000", on.reached);
}

// A 1 minute sleep from 400 % on day starts dimming at once and ends at 0 at 60 s.
static void sleepFrom400() {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  startSleep(1);
  Track t = track(62000, 255, 0);
  printf("sleep 1 min from 400%%: left 255 after %d ms, reached 0 after %d ms\n", t.left, t.reached);
  CHECK(t.left >= 0 && t.left <= 235, "sleep from 400%%: shown held 255 for %d ms, want under one brightness step (235 ms)", t.left);
  CHECK(t.reached >= 59800 && t.reached <= 60000 + frameMs, "sleep from 400%%: reached 0 after %d ms, want 60000", t.reached);
}

// Interrupts a sleep at 400 % on day 1 s in: mode 1 re-sends 400 %, mode 2 switches the hold to golden (a phase change).
static void sleepInterrupted(int minutes, int mode) {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  startSleep(minutes);
  runFor(1000);
  if (mode == 1) setBrightnessPct(400);
  else startTestHold(-1, 1, 0);
  runFor(minutes * 60000UL - 1000);
  int atEnd = shownBrightness();
  Track t = track(5000, -1, 0);
  const char* what = mode == 1 ? "re-sent 400%" : "phase change";
  printf("sleep %d min from 400%%, %s at 1 s: shown %d at the end, 0 after %d ms more (dimmer %.3f)\n", minutes, what,
         atEnd, t.reached, dimmer);
  CHECK(t.reached >= 0 && t.reached <= 1000, "sleep %d min, %s: shown %d at the end, reached 0 %d ms later, want within 1 s",
        minutes, what, atEnd, t.reached);
}

// From off, a sleep at 400 % on day comes back to the ramp's top within about the 1 s change glide.
static void sleepFromOff() {
  int falls = 0;
  settleLook(0);
  settle(400, false, &falls);
  setPower(false, false, 0);
  runFor(100);
  startSleep(60);
  Track t = track(5000, 0, 255);
  printf("off then 60 min sleep at 400%%: reached 255 after %d ms\n", t.reached);
  CHECK(t.reached >= 0 && t.reached <= 1000 + frameMs, "off then sleep at 400%%: reached 255 after %d ms, want about 1000", t.reached);
}

// Within golden hour the look dims from day to golden, so the headroom rises; at 400 % the shown brightness stays 255.
static void risingHeadroom() {
  int falls = 0;
  float from = schedule[3].minute + 0.01f, to = schedule[4].minute;
  settleLook(0);
  startTestHold(from, -1, 0);
  runFor(10000);
  settle(400, false, &falls);
  printf("golden hour start at 400%%: look %.1f, dimmer %.3f, shown %d, phase %s\n", activeLook.brightness, dimmer,
         shownBrightness(), activePhase);
  int under = 0;
  for (int f = 0; f <= 500; f++) {
    startTestHold(from + (to - from) * f / 500.0f, -1, 0);
    loop();
    if (shownBrightness() < 255) under++;
  }
  printf("ramp to golden at 400%%: look %.1f, dimmer %.3f, %d frames under 255\n", activeLook.brightness, dimmer, under);
  CHECK(under == 0, "ramp to golden at 400%%: %d frames under 255, want none", under);
  CHECK(fabsf(dimmer - 255.0f / 70.0f) < 0.01f, "ramp to golden at 400%%: dimmer %.3f, want the headroom 3.64", dimmer);
  startTestHold(from, -1, 0);
  loop();
  startTestHold(to, -1, 0);
  under = 0;
  for (int f = 0; f < 10; f++) {
    loop();
    if (shownBrightness() < 255) under++;
  }
  printf("jump from day to golden within golden hour at 400%%: %d frames under 255, brightnessPct %d\n", under, brightnessPct);
  CHECK(under == 0, "jump to golden within golden hour at 400%%: %d frames under 255, want none", under);
  CHECK(brightnessPct == 400, "brightnessPct %d after the ramp, want 400 (no phase change)", brightnessPct);

  // A slow glide (a 600 s power fade while on) still follows the rising headroom.
  startTestHold(from, -1, 0);
  settle(400, false, &falls);
  setPower(true, false, 600000);
  under = 0;
  for (int f = 0; f <= 2000; f++) {
    startTestHold(from + (to - from) * f / 2000.0f, -1, 0);
    loop();
    if (shownBrightness() < 255) under++;
  }
  printf("ramp to golden at 400%% during a 600 s fade: %d frames under 255\n", under);
  CHECK(under == 0, "ramp to golden during a slow glide: %d frames under 255, want none", under);
}

// A hold switch from golden at 400 % to embers clears the override and glides the dimmer down over 3 s.
static void fallToEmbers() {
  int falls = 0;
  settleLook(1);
  settle(400, false, &falls);
  startTestHold(-1, 3, 0);
  runFor(1500);
  float want = 255.0f / 70.0f - (255.0f / 70.0f - 1.0f) / 2;
  printf("golden 400%% to embers: dimmer %.3f after 1.5 s\n", dimmer);
  CHECK(fabsf(dimmer - want) < 0.05f, "golden 400%% to embers: dimmer %.3f after 1.5 s, want about %.2f", dimmer, want);
}

// Holds the light clock at `minute` and lets the look settle there.
static void holdMinute(float minute) {
  resetOverrides();
  setPower(true, false, 0);
  startTestHold(minute, -1, 0);
  runFor(10000);
}

// A 1 minute sleep at 400 % under the headroom (level 4) is interrupted 1 s in: on embers by re-sending 400 %, on
// evening by the phase change to wind-down (same look). The dimmer keeps up with the ramp and is 0 at the end.
static void sleepUnderHeadroom() {
  int falls = 0;
  for (int mode = 1; mode <= 2; mode++) {
    if (mode == 1) settleLook(3);
    else holdMinute(schedule[6].minute - 0.5f);
    settle(400, false, &falls);
    float look = activeLook.brightness;
    startSleep(1);
    runFor(1000);
    if (mode == 1) setBrightnessPct(400);
    else startTestHold(schedule[6].minute + 0.01f, -1, 0);
    runFor(59000);
    int atEnd = shownBrightness();
    Track t = track(5000, -1, 0);
    const char* what = mode == 1 ? "embers, re-sent 400%" : "evening to wind-down";
    printf("sleep 1 min at 400%% (%s, look %.0f, now %s): shown %d at the end, 0 after %d ms more\n", what, look,
           activePhase, atEnd, t.reached);
    CHECK(t.reached >= 0 && t.reached <= 1000, "sleep at 400%%, %s: shown %d at the end, reached 0 %d ms later, want within 1 s",
          what, atEnd, t.reached);
  }
}

// The shown brightness the sleep ramp asks for right now.
static float rampShown() { return activeLook.brightness * dimmerTarget(); }

// A 1 minute sleep at 400 % held within golden hour; at 18 s the hold moves within the same phase. `brightens` moves
// it from near golden (look 70) to near day (look 100), so the headroom falls below the dimmer's ramp; otherwise the
// opposite. The shown brightness is 0 within 1 s of the end; when the look brightens it never falls below the ramp,
// and when the look dims it never rises above it.
static void sleepLookMoves(bool brightens, bool wrap = false) {
  int falls = 0;
  float dayEnd = schedule[3].minute + 0.01f, goldenEnd = schedule[4].minute - 0.01f;
  holdMinute(brightens ? goldenEnd : dayEnd);
  settle(400, false, &falls);
  if (wrap) {   // the sleep starts 10 s before millis() wraps at 2^32 ms
    simMicros = (4294967296ULL - 10000) * 1000;
    frameClockStarted = false; showStarted = false; dimmerLastMs = millis();
  }
  float before = activeLook.brightness;
  startSleep(1);
  bool moved = false;
  int above = 0, below = 0;
  for (uint32_t t0 = millis(); millis() - t0 < 60000;) {
    if (!moved && millis() - t0 >= 18000) {
      startTestHold(brightens ? dayEnd : goldenEnd, -1, 0);
      moved = true;
    }
    loop();
    if (shownBrightness() > lroundf(rampShown()) + 1) above++;
    if (shownBrightness() < lroundf(rampShown()) - 1) below++;
  }
  int atEnd = shownBrightness();
  Track t = track(5000, -1, 0);
  const char* what = brightens ? "brightens" : "dims";
  printf("sleep 1 min at 400%%, look %.0f -> %.0f at 18 s (%s): shown %d at the end, 0 after %d ms more, %d frames above "
         "the ramp, %d below\n", before, activeLook.brightness, activePhase, atEnd, t.reached, above, below);
  CHECK(atEnd <= 1, "sleep at 400%%, look %s mid-sleep%s: shown %d on the last frame, want 0", what, wrap ? " across the millis wrap" : "", atEnd);
  if (!brightens && !wrap) CHECK(below <= 335, "sleep at 400%%, look dims mid-sleep: %d frames below the ramp, want at most 335", below);
  CHECK(t.reached >= 0 && t.reached <= 1000, "sleep at 400%%, look %s mid-sleep: shown %d at the end, reached 0 %d ms later, "
        "want within 1 s", what, atEnd, t.reached);
  if (brightens) CHECK(below == 0, "sleep at 400%%, look brightens mid-sleep: %d frames below the ramp, want none", below);
  else CHECK(above == 0, "sleep at 400%%, look dims mid-sleep: %d frames above the ramp, want none", above);
}

// An undisturbed 1 minute sleep at 400 % on day (or on embers, where the ramp's slope is above the glide floor) follows the ramp: halfway at 30 s and never below it.
static void sleepOnRamp(bool embers = false) {
  int falls = 0;
  settleLook(embers ? 3 : 0);
  settle(400, false, &falls);
  float half = activeLook.brightness * dimmerLevel() / 2;
  startSleep(1);
  int below = 0;
  for (uint32_t t0 = millis(); millis() - t0 < 30000;) {
    loop();
    if (shownBrightness() < lroundf(rampShown()) - 1) below++;
  }
  printf("sleep 1 min at 400%% on %s: shown %d at 30 s, ramp halfway %.1f, %d frames below the ramp\n", embers ? "embers" : "day",
         shownBrightness(), half, below);
  CHECK(fabsf(shownBrightness() - half) <= 1.0f, "sleep at 400%% on %s: shown %d at 30 s, want %.1f (the ramp's halfway)",
        embers ? "embers" : "day", shownBrightness(), half);
  CHECK(below == 0, "sleep at 400%% on %s: %d frames below the ramp in the first 30 s, want none", embers ? "embers" : "day", below);
}

// A status request served after a sleep ends but before the next frame shows 0 seconds left.
static void statusAfterSleepEnd() {
  resetOverrides();
  setPower(true, false, 0);
  startSleep(1);
  simMicros += 160000ULL * 1000;
  String json = overridesJson();
  printf("status after the sleep's end: %s\n", json.c_str());
  CHECK(strstr(json.c_str(), "\"sleepSecondsLeft\":0,"), "status after the sleep's end: sleepSecondsLeft not 0: %s", json.c_str());
  String text = overridesStatus();
  CHECK(strstr(text.c_str(), "off in 0 min"), "status text after the sleep's end: want 'off in 0 min': %s", text.c_str());
}

// In wake the look brightens from embers to day, so the headroom falls below a 400 % dimmer; a power-off fade then
// starts at once.
static void fallingHeadroom() {
  int falls = 0;
  holdMinute(schedule[1].minute + 0.01f);
  settle(400, false, &falls);
  printf("wake start at 400%%: look %.1f, dimmer %.3f, phase %s\n", activeLook.brightness, dimmer, activePhase);
  startTestHold(schedule[2].minute - 0.01f, -1, 0);
  loop();
  printf("wake end at 400%%: look %.1f, dimmer %.3f, headroom %.3f, phase %s\n", activeLook.brightness, dimmer,
         255.0f / activeLook.brightness, activePhase);
  CHECK(dimmer <= 255.0f / activeLook.brightness + 1e-4f, "wake end at 400%%: dimmer %.3f above the headroom %.3f", dimmer,
        255.0f / activeLook.brightness);
  setPower(false, false, 2000);
  Track off = track(2500, 255, 0);
  printf("power off over 2 s at wake end: left 255 after %d ms, reached 0 after %d ms\n", off.left, off.reached);
  CHECK(off.left >= 0 && off.left <= 2 * frameMs, "power off at wake end: shown held 255 for %d ms", off.left);
  CHECK(off.reached >= 2000 - frameMs && off.reached <= 2000 + frameMs, "power off at wake end: reached 0 after %d ms, want 2000", off.reached);
}

int main() {
  simRng.seed(1);
  setup();
  ceiling();
  scaled();
  timing();
  sleepRamp();
  reset();
  powerFade();
  sleepFrom400();
  sleepInterrupted(1, 1);
  sleepInterrupted(1, 2);
  sleepInterrupted(3, 2);
  sleepFromOff();
  risingHeadroom();
  fallToEmbers();
  sleepUnderHeadroom();
  fallingHeadroom();
  sleepLookMoves(true);
  sleepLookMoves(false);
  sleepLookMoves(true, true);
  sleepOnRamp();
  sleepOnRamp(true);
  statusAfterSleepEnd();
  printf(failures ? "brightness: %d failure(s)\n" : "brightness: ok\n", failures);
  return failures != 0;
}
