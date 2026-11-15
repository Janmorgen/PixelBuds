#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "daycycle.h"
#include "islands.h"

// Temporary changes made over HTTP. Most clear when the phase name changes; power-off "until morning"
// and the sleep timer last until the wake or day phase. Nothing here survives a reboot.

#define GLIDE_MS_CHANGE  1000   // a brightness change glides over this long
#define GLIDE_MS_RELEASE 3000   // overrides ending at a phase change glide back over this long

static bool powerOff = false;
static bool powerUntilMorning = false;   // power-off survives phase changes until wake
static int brightnessPct = 100;
static bool sleepActive = false;
static uint32_t sleepStartMs = 0;
static uint32_t sleepLengthMs = 0;
static int adjustSpeed = 100;    // percent of the look's movement rate
static int adjustPixels = 100;   // percent of the look's pixel count
static int adjustTrail = 100;    // percent of the look's trail length
static int adjustMix = 100;      // percent of the look's mixing rate
static bool paused = false;
static bool shuffleRequested = false;

static float dimmer = 1.0f;          // shown share of the look's brightness, 0-1
static float dimmerRate = 0.001f;    // largest dimmer change per millisecond
static uint32_t dimmerLastMs = 0;
static char overridePhase[24] = "";  // phase the overrides were last checked against
static float scheduledIslandHold = 0;  // the look's island hold before the /islands setting

// The dimmer the overrides ask for right now.
static float dimmerTarget() {
  if (powerOff) return 0.0f;
  float level = brightnessPct / 100.0f;
  if (sleepActive) level *= max(0.0f, 1.0f - (millis() - sleepStartMs) / (float)sleepLengthMs);
  return level;
}

// Makes the dimmer reach its new target in about `ms` milliseconds; 0 jumps straight there.
static void glideOver(uint32_t ms) {
  float distance = fabsf(dimmerTarget() - dimmer);
  dimmerRate = ms == 0 ? 1e9f : max(distance, 0.05f) / ms;
}

void setPower(bool on, bool untilMorning, uint32_t fadeMs) {
  powerOff = !on;
  powerUntilMorning = !on && untilMorning;
  sleepActive = false;
  glideOver(fadeMs);
}

void setBrightnessPct(int pct) {
  brightnessPct = pct;
  glideOver(GLIDE_MS_CHANGE);
}

// Dims gradually to off over `minutes`, then stays off until morning.
void startSleep(int minutes) {
  sleepActive = true;
  sleepStartMs = millis();
  sleepLengthMs = minutes * 60000UL;
  powerOff = false;
  powerUntilMorning = false;
  dimmerRate = 1.0f / GLIDE_MS_CHANGE;
}

// Clears the overrides that last only for one phase.
static void clearPhaseOverrides() {
  if (!powerUntilMorning) powerOff = false;
  if (!sleepActive) brightnessPct = 100;   // a sleep keeps dimming from where it started
  adjustSpeed = adjustPixels = adjustTrail = adjustMix = 100;
  for (int c = 0; c < PALETTE_SIZE; c++) colourOverride[c] = -1;
  paused = false;
  glideOver(GLIDE_MS_RELEASE);
}

// Clears every override, including power-off, sleep and a skip ahead.
void resetOverrides() {
  powerUntilMorning = false;
  sleepActive = false;
  cancelSkip();
  clearPhaseOverrides();
}

// Trail fade per frame that makes trails last `pct` percent as long as with `fade`.
static float scaleTrail(float fade, int pct) {
  if (fade <= 0.0f || pct == 100) return fade;
  return 256.0f * (1.0f - powf(1.0f - fade / 256.0f, 100.0f / pct));
}

// Call once per frame after dayCycleUpdate(): ends overrides on a phase change and applies the rest to activeLook.
void overridesUpdate() {
  if (strcmp(overridePhase, activePhase) != 0) {
    bool morning = strcmp(activePhase, "wake") == 0 || strcmp(activePhase, "day") == 0;
    if (overridePhase[0]) {
      if (morning) { powerUntilMorning = false; sleepActive = false; }
      clearPhaseOverrides();
    }
    strncpy(overridePhase, activePhase, sizeof(overridePhase) - 1);
  }
  if (sleepActive && millis() - sleepStartMs >= sleepLengthMs) {
    sleepActive = false;
    powerOff = true;
    powerUntilMorning = true;
  }

  uint32_t now = millis();
  float step = dimmerRate * (now - dimmerLastMs);
  dimmerLastMs = now;
  float target = dimmerTarget();
  dimmer = dimmer < target ? min(dimmer + step, target) : max(dimmer - step, target);

  activeLook.moveEvery *= 100.0f / adjustSpeed;
  activeLook.colorEvery *= 100.0f / adjustMix;
  activeLook.fade = scaleTrail(activeLook.fade, adjustTrail);
  activeLook.pixels *= adjustPixels / 100.0f;
  for (int c = 0; c < PALETTE_SIZE; c++) {
    if (colourOverride[c] >= 0) activeLook.weights[c] = colourOverride[c];
  }
  scheduledIslandHold = activeLook.islandHold;
  if (islandHoldSetting >= 0) activeLook.islandHold = islandHoldSetting;
}

// Island hold in force: the fixed setting, or the schedule's.
float currentIslandHold() {
  return islandHoldSetting >= 0 ? islandHoldSetting : scheduledIslandHold;
}

// FastLED brightness for this frame.
uint8_t shownBrightness() {
  return (uint8_t)lroundf(activeLook.brightness * dimmer);
}

// "until wind-down (22:00)", or how a hold ends.
static String untilText() {
  const char* name;
  float at;
  if (holdActive()) return "until the hold ends";
  if (!nextPhaseChange(&name, &at)) return "until the phase changes";
  char t[8];
  formatMinute(t, sizeof(t), at);
  return String("until ") + name + " (" + t + ")";
}

// Wake time of the schedule, for "until morning".
static String morningText() {
  char t[8];
  formatMinute(t, sizeof(t), schedule[1].minute);
  return String("until wake (") + t + ")";
}

static bool anyPhaseOverride() {
  bool colours = false;
  for (int c = 0; c < PALETTE_SIZE; c++) colours |= colourOverride[c] >= 0;
  return (powerOff && !powerUntilMorning) || brightnessPct != 100 || adjustSpeed != 100 || adjustPixels != 100 ||
         adjustTrail != 100 || adjustMix != 100 || colours || paused;
}

// Status lines for the overrides.
String overridesStatus() {
  String s = "power:       ";
  if (sleepActive) {
    uint32_t left = sleepLengthMs - (millis() - sleepStartMs);
    s += "sleeping, off in " + String((left + 59999) / 60000) + " min, then off " + morningText();
  } else if (powerOff) {
    s += "off " + (powerUntilMorning ? morningText() : untilText());
  } else {
    s += "on";
  }
  s += "  dimmer " + String(lroundf(dimmer * 100)) + "%\n";

  s += "overrides:   ";
  if (!anyPhaseOverride()) return s + "none\n";
  if (brightnessPct != 100) s += "brightness " + String(brightnessPct) + "%  ";
  if (adjustSpeed != 100) s += "speed " + String(adjustSpeed) + "%  ";
  if (adjustPixels != 100) s += "pixels " + String(adjustPixels) + "%  ";
  if (adjustTrail != 100) s += "trail " + String(adjustTrail) + "%  ";
  if (adjustMix != 100) s += "mix " + String(adjustMix) + "%  ";
  for (int c = 0; c < PALETTE_SIZE; c++) {
    if (colourOverride[c] >= 0) s += String(paletteNames[c]) + " " + String(lroundf(colourOverride[c] * 100)) + "  ";
  }
  if (paused) s += "paused  ";
  return s + untilText() + "\n";
}

String overridesJson() {
  const char* name;
  float at;
  String until = "null";
  if (nextPhaseChange(&name, &at) && !holdActive()) {
    char t[8];
    formatMinute(t, sizeof(t), at);
    until = String("{\"phase\":\"") + name + "\",\"at\":\"" + t + "\"}";
  }
  String colours;
  for (int c = 0; c < PALETTE_SIZE; c++) {
    colours += String(c ? "," : "") + "\"" + paletteNames[c] + "\":" +
               (colourOverride[c] >= 0 ? String(lroundf(colourOverride[c] * 100)) : String("null"));
  }
  String sleep = sleepActive ? String((sleepLengthMs - (millis() - sleepStartMs)) / 1000) : String("null");
  return String("\"power\":{\"on\":") + (powerOff ? "false" : "true") +
         ",\"untilMorning\":" + (powerUntilMorning ? "true" : "false") +
         ",\"sleepSecondsLeft\":" + sleep + ",\"dimmer\":" + String(dimmer, 2) + "}," +
         "\"overrides\":{\"brightness\":" + brightnessPct + ",\"speed\":" + adjustSpeed + ",\"pixels\":" + adjustPixels +
         ",\"trail\":" + adjustTrail + ",\"mix\":" + adjustMix + ",\"colors\":{" + colours + "}" +
         ",\"paused\":" + (paused ? "true" : "false") + ",\"until\":" + until + "}";
}
