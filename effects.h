#pragma once

#include <Arduino.h>

// Effect settings, set at /effects and saved by settings.h. The behaviour itself is in islandfx.h.
// Ageing: how contact wears a pixel down. Death effects: run when one pixel dies, on its neighbours.
// Island effects: run when an island (touching pixels) breaks up for good.

#define AGEING_NORMAL 0
#define AGEING_EDGE   1   // slower ageing, slower still with more neighbours: islands wear from the outside

#define DEATH_ORPHAN    (1 << 0)   // a neighbour left alone gets a longer life
#define DEATH_CONTAGION (1 << 1)   // neighbours lose part of their remaining life
#define DEATH_SCATTER   (1 << 2)   // neighbours are pushed a few cells away
#define DEATH_ALL       (DEATH_ORPHAN | DEATH_CONTAGION | DEATH_SCATTER)

#define ISLAND_SEED   (1 << 0)   // the next respawns appear where the island ended, in its colour
#define ISLAND_RIPPLE (1 << 1)   // a ring in the island's colour spreads from where it ended
#define ISLAND_ALL    (ISLAND_SEED | ISLAND_RIPPLE)

#define EFFECT_BOOST_DEFAULT 2.0f

static const char* const ageingNames[] = {"normal", "edge"};
static const char* const deathEffectNames[] = {"orphan", "contagion", "scatter"};
static const char* const islandEffectNames[] = {"seed", "ripple"};
#define DEATH_EFFECT_COUNT 3
#define ISLAND_EFFECT_COUNT 2

static int effectAgeing = AGEING_NORMAL;
static int effectDeath = 0;
static int effectIsland = 0;
static float effectBoost = EFFECT_BOOST_DEFAULT;   // how strongly each effect acts at full strength
static float effectStrengthSetting = -1;           // < 0: strength follows the look (day only); else fixed 0-1
static int islandsNow = 0;                         // islands on the matrix this step, for the status

static const char* const onOffNames[] = {"off", "on"};

void resetEffects() {
  effectAgeing = AGEING_NORMAL;
  effectDeath = 0;
  effectIsland = 0;
  effectBoost = EFFECT_BOOST_DEFAULT;
  effectStrengthSetting = -1;
}

// Old /islands?mode= values, kept for existing clients.
static const char* const islandModeNames[] = {"off", "edge", "orphan"};

// Sets ageing and the orphan effect as an old /islands mode did; other effects are kept.
void applyIslandMode(int mode) {
  bool edge = mode == 1;
  bool orphan = mode == 2;
  effectAgeing = edge ? AGEING_EDGE : AGEING_NORMAL;
  effectDeath = orphan ? (effectDeath | DEATH_ORPHAN) : (effectDeath & ~DEATH_ORPHAN);
}

// The set bits of `mask` as names joined by `sep`, or "none".
static String effectList(int mask, const char* const* names, int count, const char* sep, bool quoted) {
  String out;
  for (int i = 0; i < count; i++) {
    if (!(mask & (1 << i))) continue;
    if (out.length()) out += sep;
    out += quoted ? String("\"") + names[i] + "\"" : String(names[i]);
  }
  return out.length() || quoted ? out : String("none");
}

// Parses "none" or a comma-separated list of names into a bit mask; false on an unknown name.
static bool parseEffectList(const String& s, const char* const* names, int count, int& mask) {
  mask = 0;
  if (s == "none") return true;
  int start = 0;
  while (start <= (int)s.length()) {
    int comma = s.indexOf(',', start);
    String item = s.substring(start, comma < 0 ? s.length() : comma);
    int found = -1;
    for (int i = 0; i < count; i++) if (item == names[i]) found = i;
    if (found < 0) return false;
    mask |= 1 << found;
    if (comma < 0) break;
    start = comma + 1;
  }
  return true;
}

// One status line for the effects.
static String effectsStatus(float strength) {
  String setting = effectStrengthSetting < 0 ? String(" (auto)") : String(" (fixed)");
  return String("effects:     ageing ") + ageingNames[effectAgeing] +
         "  death " + effectList(effectDeath, deathEffectNames, DEATH_EFFECT_COUNT, ",", false) +
         "  island " + effectList(effectIsland, islandEffectNames, ISLAND_EFFECT_COUNT, ",", false) +
         "  strength " + String(strength, 2) + setting + "  boost " + String(effectBoost, 1) +
         "  islands now " + islandsNow + "\n";
}

static String effectsJson(float strength) {
  return String("\"effects\":{\"ageing\":\"") + ageingNames[effectAgeing] + "\",\"death\":[" +
         effectList(effectDeath, deathEffectNames, DEATH_EFFECT_COUNT, ",", true) + "],\"island\":[" +
         effectList(effectIsland, islandEffectNames, ISLAND_EFFECT_COUNT, ",", true) + "],\"strength\":" +
         String(strength, 2) + ",\"strengthSetting\":" +
         (effectStrengthSetting < 0 ? String("\"auto\"") : String(lroundf(effectStrengthSetting * 100))) +
         ",\"boost\":" + String(effectBoost, 1) + ",\"islandsNow\":" + islandsNow + "}";
}
