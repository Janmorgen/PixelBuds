#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "daycycle.h"
#include "islands.h"
#include "rules.h"

// Keeps the schedule, rule and island settings in NVS across reboots. Overrides are never saved.

static Preferences prefs;

void settingsLoad() {
  prefs.begin("pixelbuds", true);
  scheduleWindDown = prefs.getInt("winddown", scheduleWindDown);
  scheduleEmbers = prefs.getInt("embers", scheduleEmbers);
  scheduleWake = prefs.getInt("wake", scheduleWake);
  scheduleGolden = prefs.getInt("golden", scheduleGolden);
  scheduleSunriseShift = prefs.getInt("riseShift", scheduleSunriseShift);
  scheduleSunsetShift = prefs.getInt("setShift", scheduleSunsetShift);
  ruleMix = prefs.getInt("mix", ruleMix);
  ruleMixRate = prefs.getInt("mixRate", ruleMixRate);
  ruleLifeMin = prefs.getInt("lifeMin", ruleLifeMin);
  ruleLifeMax = prefs.getInt("lifeMax", ruleLifeMax);
  ruleExpireFade = prefs.getBool("expireFade", ruleExpireFade);
  ruleRound = prefs.getBool("round", ruleRound);
  islandMode = prefs.getInt("islandMode", islandMode);
  islandBoost = prefs.getFloat("islandBoost", islandBoost);
  islandHoldSetting = prefs.getFloat("islandHold", islandHoldSetting);
  prefs.end();

  // Anything out of range falls back to its group's defaults.
  if (scheduleEmbers < scheduleWindDown || scheduleWindDown < 12 * 60 || scheduleEmbers >= MINUTES_PER_DAY ||
      abs(scheduleWake - 120) > 120 || abs(scheduleGolden - 90) > 90 || abs(scheduleSunriseShift) > 180 ||
      abs(scheduleSunsetShift) > 180) resetScheduleSettings();
  if (ruleMix < 0 || ruleMix > 1 || ruleMixRate < 0 || ruleMixRate > 100 || ruleLifeMin < 1 ||
      ruleLifeMax > RULE_LIFE_LIMIT || ruleLifeMin > ruleLifeMax) resetRules();
  if (islandMode < 0 || islandMode > 3 || !(islandBoost >= 0 && islandBoost <= 5) || islandHoldSetting > 1) {
    resetIslands();
    islandModeDirty = false;
  }
}

static void putInt(const char* key, int v) {
  if (!prefs.isKey(key) || prefs.getInt(key) != v) prefs.putInt(key, v);
}
static void putBool(const char* key, bool v) {
  if (!prefs.isKey(key) || prefs.getBool(key) != v) prefs.putBool(key, v);
}
static void putFloat(const char* key, float v) {
  if (!prefs.isKey(key) || prefs.getFloat(key) != v) prefs.putFloat(key, v);
}

// Writes only the values that differ from what is stored.
void settingsSave() {
  prefs.begin("pixelbuds", false);
  putInt("winddown", scheduleWindDown);
  putInt("embers", scheduleEmbers);
  putInt("wake", scheduleWake);
  putInt("golden", scheduleGolden);
  putInt("riseShift", scheduleSunriseShift);
  putInt("setShift", scheduleSunsetShift);
  putInt("mix", ruleMix);
  putInt("mixRate", ruleMixRate);
  putInt("lifeMin", ruleLifeMin);
  putInt("lifeMax", ruleLifeMax);
  putBool("expireFade", ruleExpireFade);
  putBool("round", ruleRound);
  putInt("islandMode", islandMode);
  putFloat("islandBoost", islandBoost);
  putFloat("islandHold", islandHoldSetting);
  prefs.end();
}
