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
static bool islandModeDirty = false;
static bool islandRounding = false;   // round colour mixes for EDGE/ORPHAN: brighter but paler islands

static const char* const islandModeNames[] = {"off", "split", "edge", "orphan"};

static const char* const onOffNames[] = {"off", "on"};

// Asks the main loop to switch to mode `mode`; the reply already shows it.
static void requestIslandMode(int mode) {
  islandMode = mode;
  islandModeDirty = true;
}

// One status line for the experiment.
static String islandStatus(float hold) {
  const char* layout = islandMode == ISLANDS_SPLIT ? " (left half edge, right half orphan)" : "";
  return String("islands:     ") + islandModeNames[islandMode] + layout + "  hold " + String(hold, 2) +
         "  rounding " + (islandRounding ? "on" : "off") + "\n";
}

static String islandJson(float hold) {
  return String("\"islands\":{\"mode\":\"") + islandModeNames[islandMode] + "\",\"hold\":" + String(hold, 2) +
         ",\"rounding\":" + (islandRounding ? "true" : "false") + "}";
}
