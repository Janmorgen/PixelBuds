#pragma once

#include <Arduino.h>

// Island-hold experiment: two ways to keep daytime clumps together, selectable over HTTP.
// EDGE: pixels age slower overall and slower still the more neighbours they have.
// ORPHAN: normal ageing, but a pixel left alone by a neighbour's death gets a fresh, longer life.

#define VARIANT_TODAY  0
#define VARIANT_EDGE   1
#define VARIANT_ORPHAN 2

#define ISLANDS_OFF    0   // everyone behaves as before the experiment
#define ISLANDS_SPLIT  1   // left half EDGE, right half ORPHAN
#define ISLANDS_EDGE   2
#define ISLANDS_ORPHAN 3

#define ISLAND_LIFE_BOOST 2.0f   // EDGE lives and ORPHAN bonus lives are (1 + boost * hold) times longer

static int islandMode = ISLANDS_SPLIT;
static int islandModeRequested = ISLANDS_SPLIT;
static bool islandModeDirty = false;
static bool islandRounding = false;   // round colour mixes for EDGE/ORPHAN: brighter but paler islands

static const char* const islandModeNames[] = {"off", "split", "edge", "orphan"};

// Asks the main loop to switch modes; returns false for an unknown name.
static bool requestIslandMode(const String& name) {
  for (int i = 0; i < 4; i++) {
    if (name == islandModeNames[i]) {
      islandModeRequested = i;
      islandModeDirty = true;
      return true;
    }
  }
  return false;
}

// Sets colour-mix rounding from "on"/"off"; returns false for anything else.
static bool requestIslandRounding(const String& value) {
  if (value != "on" && value != "off") return false;
  islandRounding = value == "on";
  return true;
}

static String islandStatus(float hold) {
  const char* layout = islandMode == ISLANDS_SPLIT ? " (left half edge, right half orphan)" : "";
  return String("islands:    ") + islandModeNames[islandMode] + layout + "  hold " + String(hold, 2) +
         "  rounding " + (islandRounding ? "on" : "off") +
         "\n            switch with /islands?mode=off|split|edge|orphan and /islands?round=on|off\n";
}
