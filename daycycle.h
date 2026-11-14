#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "secrets.h"

// Time-of-day looks for the pixel simulation, anchored to local sunrise and sunset.

#define WAKE_RAMP_MINUTES  60
#define GOLDEN_MINUTES     60
#define WIND_DOWN_MINUTE   (22 * 60)
#define EMBERS_MINUTE      (23 * 60)
#define TEST_START_MINUTE  (5 * 60)
#define TEST_MS_PER_MINUTE 1000UL      // one real minute plays one hour
#define MINUTES_PER_DAY    1440

#define PALETTE_SIZE 4
static const char* const paletteNames[PALETTE_SIZE] = {"red", "green", "blue", "amber"};
static const CRGB paletteColors[PALETTE_SIZE] = {
  CRGB(255, 0, 0), CRGB(0, 255, 0), CRGB(0, 0, 255), CRGB(255, 72, 0)
};

// Simulation settings for one moment of the day.
struct Look {
  float brightness;              // FastLED brightness, 0-255
  float weights[PALETTE_SIZE];   // relative chance of each spawn colour
  float pixels;                  // target pixel count
  float moveEvery;               // frames between moves
  float colorEvery;              // frames between colour mixes
  float fade;                    // per-frame trail fade, 0-255
};

static const Look LOOK_DAY     = {100, {1.0f, 1.0f, 1.0f, 0.0f}, 30, 1, 1, 56};
static const Look LOOK_GOLDEN  = { 70, {1.0f, 0.6f, 0.0f, 0.8f}, 24, 2, 1, 40};
static const Look LOOK_EVENING = { 40, {1.0f, 0.3f, 0.0f, 1.0f}, 18, 3, 2, 28};
static const Look LOOK_EMBERS  = { 10, {1.0f, 0.0f, 0.0f, 0.4f},  6, 6, 3, 16};

// A look pinned to a minute of the day; name labels the stretch that starts there.
struct Keyframe {
  float minute;
  const Look* look;
  const char* name;
};

#define KEYFRAME_COUNT 9
static Keyframe schedule[KEYFRAME_COUNT];
static float sunriseMinute = 6 * 60;
static float sunsetMinute = 18 * 60;
static bool sunFromClock = false;
static int scheduleYearDay = -1;

static bool testActive = false;
static uint32_t testStartMs = 0;

static Look activeLook = LOOK_DAY;
static const char* activePhase = "day";
static float activeMinute = 12 * 60;
static bool activeFromClock = false;
static int reportedPixels = 0;

// NOAA sunrise/sunset for a day of the year, as minutes after UTC midnight (may fall outside 0-1440).
static bool sunTimesUtc(int dayOfYear, double lat, double lon, double& riseUtc, double& setUtc) {
  double g = 2.0 * M_PI / 365.0 * (dayOfYear - 1);
  double eqTime = 229.18 * (0.000075 + 0.001868 * cos(g) - 0.032077 * sin(g)
                            - 0.014615 * cos(2 * g) - 0.040849 * sin(2 * g));
  double decl = 0.006918 - 0.399912 * cos(g) + 0.070257 * sin(g) - 0.006758 * cos(2 * g)
              + 0.000907 * sin(2 * g) - 0.002697 * cos(3 * g) + 0.00148 * sin(3 * g);
  double latR = lat * M_PI / 180.0;
  double cosHa = cos(90.833 * M_PI / 180.0) / (cos(latR) * cos(decl)) - tan(latR) * tan(decl);
  if (cosHa < -1.0 || cosHa > 1.0) return false;
  double ha = acos(cosHa) * 180.0 / M_PI;
  riseUtc = 720.0 - 4.0 * (lon + ha) - eqTime;
  setUtc = 720.0 - 4.0 * (lon - ha) - eqTime;
  return true;
}

// Rebuilds the keyframes around the given sunrise and sunset; sun-anchored frames never pass the wind-down.
static void buildSchedule(float rise, float set) {
  float dayStart = rise + WAKE_RAMP_MINUTES;
  float goldenStart = min(set - GOLDEN_MINUTES, (float)WIND_DOWN_MINUTE);
  float sunset = min(set, (float)WIND_DOWN_MINUTE);
  float evening = min(set + GOLDEN_MINUTES, (float)WIND_DOWN_MINUTE);

  Keyframe frames[KEYFRAME_COUNT] = {
    {0,                        &LOOK_EMBERS,  "embers"},
    {rise,                     &LOOK_EMBERS,  "wake"},
    {dayStart,                 &LOOK_DAY,     "day"},
    {goldenStart,              &LOOK_DAY,     "golden hour"},
    {sunset,                   &LOOK_GOLDEN,  "golden hour"},
    {evening,                  &LOOK_EVENING, "evening"},
    {WIND_DOWN_MINUTE,         &LOOK_EVENING, "wind-down"},
    {EMBERS_MINUTE,            &LOOK_EMBERS,  "embers"},
    {MINUTES_PER_DAY,          &LOOK_EMBERS,  "embers"},
  };
  for (int i = 1; i < KEYFRAME_COUNT; i++) {
    if (frames[i].minute < frames[i - 1].minute) frames[i].minute = frames[i - 1].minute;
  }
  memcpy(schedule, frames, sizeof(frames));
}

// Recomputes today's sun times from the local date and UTC offset at `now`.
static void refreshSun(time_t now) {
  struct tm local, utc;
  localtime_r(&now, &local);
  gmtime_r(&now, &utc);
  int offset = (local.tm_hour * 60 + local.tm_min) - (utc.tm_hour * 60 + utc.tm_min);
  if (offset > 720) offset -= MINUTES_PER_DAY;
  if (offset <= -720) offset += MINUTES_PER_DAY;

  double riseUtc, setUtc;
  if (sunTimesUtc(local.tm_yday + 1, LATITUDE, LONGITUDE, riseUtc, setUtc)) {
    sunriseMinute = fmod(riseUtc + offset + 2 * MINUTES_PER_DAY, MINUTES_PER_DAY);
    sunsetMinute = fmod(setUtc + offset + 2 * MINUTES_PER_DAY, MINUTES_PER_DAY);
    sunFromClock = true;
  }
  buildSchedule(sunriseMinute, sunsetMinute);
  scheduleYearDay = local.tm_yday;
}

// Blends the looks either side of `minute`, eased so each stretch starts and ends gently.
static Look lookAt(float minute, const char** phase) {
  if (minute < 0) minute = 0;
  if (minute >= MINUTES_PER_DAY) minute = MINUTES_PER_DAY - 0.001f;
  int i = 0;
  while (i < KEYFRAME_COUNT - 2 && minute >= schedule[i + 1].minute) i++;
  const Keyframe& a = schedule[i];
  const Keyframe& b = schedule[i + 1];
  if (phase) *phase = a.name;

  float span = b.minute - a.minute;
  float t = span > 0 ? (minute - a.minute) / span : 1.0f;
  t = t * t * (3.0f - 2.0f * t);

  Look out;
  out.brightness = a.look->brightness + (b.look->brightness - a.look->brightness) * t;
  for (int c = 0; c < PALETTE_SIZE; c++) {
    out.weights[c] = a.look->weights[c] + (b.look->weights[c] - a.look->weights[c]) * t;
  }
  out.pixels = a.look->pixels + (b.look->pixels - a.look->pixels) * t;
  out.moveEvery = a.look->moveEvery + (b.look->moveEvery - a.look->moveEvery) * t;
  out.colorEvery = a.look->colorEvery + (b.look->colorEvery - a.look->colorEvery) * t;
  out.fade = a.look->fade + (b.look->fade - a.look->fade) * t;
  return out;
}

// True once SNTP has set the wall clock.
static bool clockIsSet(time_t now) {
  return now > 1700000000;
}

// Starts a 24-minute test day at 05:00 using today's sun times.
void startTestDay() {
  testActive = true;
  testStartMs = millis();
}

void stopTestDay() {
  testActive = false;
}

// Updates and returns the look for this frame: test clock, wall clock, or the day look until the clock is set.
const Look& dayCycleUpdate() {
  time_t now = time(nullptr);
  bool clockSet = clockIsSet(now);
  if (clockSet) {
    struct tm local;
    localtime_r(&now, &local);
    if (local.tm_yday != scheduleYearDay) refreshSun(now);
  } else if (scheduleYearDay == -1 && schedule[KEYFRAME_COUNT - 1].minute == 0) {
    buildSchedule(sunriseMinute, sunsetMinute);
  }

  if (testActive && millis() - testStartMs >= MINUTES_PER_DAY * TEST_MS_PER_MINUTE) testActive = false;

  if (testActive) {
    float elapsed = (millis() - testStartMs) / (float)TEST_MS_PER_MINUTE;
    activeMinute = fmod(TEST_START_MINUTE + elapsed, (float)MINUTES_PER_DAY);
    activeLook = lookAt(activeMinute, &activePhase);
    activeFromClock = clockSet;
  } else if (clockSet) {
    struct tm local;
    localtime_r(&now, &local);
    activeMinute = local.tm_hour * 60 + local.tm_min + local.tm_sec / 60.0f;
    activeLook = lookAt(activeMinute, &activePhase);
    activeFromClock = true;
  } else {
    activeLook = LOOK_DAY;
    activePhase = "day (waiting for clock)";
    activeFromClock = false;
  }
  return activeLook;
}

// Picks a palette index at random, weighted by the active look.
int pickSpawnIndex() {
  float total = 0;
  for (int c = 0; c < PALETTE_SIZE; c++) total += max(activeLook.weights[c], 0.0f);
  if (total <= 0) return 0;
  float r = random(100000) / 100000.0f * total;
  for (int c = 0; c < PALETTE_SIZE; c++) {
    r -= max(activeLook.weights[c], 0.0f);
    if (r < 0) return c;
  }
  return PALETTE_SIZE - 1;
}

// True when the active look has (almost) dropped this palette colour.
bool spawnColorRetired(int index) {
  return activeLook.weights[index] < 0.05f;
}

void dayCycleReportPixels(int count) {
  reportedPixels = count;
}

static void formatMinute(char* out, size_t len, float minute) {
  int m = ((int)minute) % MINUTES_PER_DAY;
  snprintf(out, len, "%02d:%02d", m / 60, m % 60);
}

// Plain-text summary of the clock, schedule and current look.
String dayCycleStatus() {
  char buf[1024];
  char t[8], rise[8], set[8];
  formatMinute(t, sizeof(t), activeMinute);
  formatMinute(rise, sizeof(rise), sunriseMinute);
  formatMinute(set, sizeof(set), sunsetMinute);

  char clock[40] = "not set yet";
  time_t now = time(nullptr);
  if (clockIsSet(now)) {
    struct tm local;
    localtime_r(&now, &local);
    strftime(clock, sizeof(clock), "%Y-%m-%d %H:%M:%S %Z", &local);
  }

  char mode[64];
  if (testActive) {
    uint32_t elapsed = (millis() - testStartMs) / 1000;
    uint32_t left = MINUTES_PER_DAY * TEST_MS_PER_MINUTE / 1000 - elapsed;
    snprintf(mode, sizeof(mode), "test day (%lu:%02lu left)", (unsigned long)(left / 60), (unsigned long)(left % 60));
  } else {
    snprintf(mode, sizeof(mode), activeFromClock ? "schedule" : "fallback (no clock)");
  }

  int n = snprintf(buf, sizeof(buf),
    "PixelBuds\n"
    "clock:      %s\n"
    "mode:       %s\n"
    "light time: %s  phase: %s\n"
    "sun:        rise %s  set %s%s\n"
    "look:       brightness %.0f  pixels %d/%.0f  move every %.1f  mix every %.1f  fade %.0f\n"
    "palette:   ",
    clock, mode, t, activePhase, rise, set, sunFromClock ? "" : " (defaults, no clock yet)",
    activeLook.brightness, reportedPixels, activeLook.pixels, activeLook.moveEvery,
    activeLook.colorEvery, activeLook.fade);
  for (int c = 0; c < PALETTE_SIZE && n < (int)sizeof(buf); c++) {
    n += snprintf(buf + n, sizeof(buf) - n, " %s %.2f", paletteNames[c], activeLook.weights[c]);
  }
  n += snprintf(buf + n, sizeof(buf) - n, "\nschedule:  ");
  for (int i = 0; i < KEYFRAME_COUNT - 1 && n < (int)sizeof(buf); i++) {
    char k[8];
    formatMinute(k, sizeof(k), schedule[i].minute);
    n += snprintf(buf + n, sizeof(buf) - n, " %s %s%s", k, schedule[i].name, i < KEYFRAME_COUNT - 2 ? " |" : "\n");
  }
  return String(buf);
}
