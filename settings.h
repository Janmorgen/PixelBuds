#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <initializer_list>

#include "daycycle.h"
#include "effects.h"
#include "rules.h"

// Keeps the schedule, rule and effect settings in NVS across reboots. Overrides are never saved.

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
  effectAgeing = prefs.getInt("ageing", effectAgeing);
  effectDeath = prefs.getInt("deathFx", effectDeath);
  effectIsland = prefs.getInt("islandFx", effectIsland);
  effectBoost = prefs.getFloat("fxBoost", effectBoost);
  effectStrengthSetting = prefs.getFloat("fxStrength", effectStrengthSetting);
  prefs.end();

  // Anything out of range falls back to its group's defaults.
  if (scheduleEmbers < scheduleWindDown || scheduleWindDown < 12 * 60 || scheduleEmbers >= MINUTES_PER_DAY ||
      abs(scheduleWake - 120) > 120 || abs(scheduleGolden - 90) > 90 || abs(scheduleSunriseShift) > 180 ||
      abs(scheduleSunsetShift) > 180) resetScheduleSettings();
  if (ruleMix < 0 || ruleMix > 1 || ruleMixRate < 0 || ruleMixRate > 100 || ruleLifeMin < 1 ||
      ruleLifeMax > RULE_LIFE_LIMIT || ruleLifeMin > ruleLifeMax) resetRules();
  if (effectAgeing < 0 || effectAgeing > 1 || (effectDeath & ~DEATH_ALL) || (effectIsland & ~ISLAND_ALL) ||
      !(effectBoost >= 0 && effectBoost <= 5) || effectStrengthSetting > 1) resetEffects();
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
  putInt("ageing", effectAgeing);
  putInt("deathFx", effectDeath);
  putInt("islandFx", effectIsland);
  putFloat("fxBoost", effectBoost);
  putFloat("fxStrength", effectStrengthSetting);
  for (const char* old : {"islandMode", "islandBoost", "islandHold"}) {
    if (prefs.isKey(old)) prefs.remove(old);
  }
  prefs.end();
}
