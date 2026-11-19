#pragma once

#include <Arduino.h>

// How pixels share colour, how long they live and how they die. Set at /rules; saved by settings.h.

#define MIX_POOL 0   // pool own and neighbours' colour, split among the pixels still sharing
#define MIX_PAIR 1   // average with each neighbour in turn, writing into both; drifts toward white

#define RULE_LIFE_MIN_DEFAULT 30
#define RULE_LIFE_MAX_DEFAULT 100
#define RULE_LIFE_LIMIT 2000
#define DEATH_FADE_STEPS  12   // steps a pixel takes to fade out when its life ends with expire=fade

static const char* const mixNames[] = {"pool", "pair"};
static const char* const expireNames[] = {"snap", "fade"};

static int ruleMix = MIX_POOL;
static int ruleMixRate = 100;                     // percent of the way each mix moves toward the shared colour
static int ruleLifeMin = RULE_LIFE_MIN_DEFAULT;   // life range, in steps spent touching another pixel
static int ruleLifeMax = RULE_LIFE_MAX_DEFAULT;
static bool ruleExpireFade = false;               // fade out when life ends in contact, rather than going black
static bool ruleRound = false;                    // round mixes: brighter but paler colours

void resetRules() {
  ruleMix = MIX_POOL;
  ruleMixRate = 100;
  ruleLifeMin = RULE_LIFE_MIN_DEFAULT;
  ruleLifeMax = RULE_LIFE_MAX_DEFAULT;
  ruleExpireFade = false;
  ruleRound = false;
}

// A new pixel's life, in contact steps.
int ruleLifeTime() {
  return random(ruleLifeMin, ruleLifeMax);
}

String rulesStatus() {
  return String("rules:       mix ") + mixNames[ruleMix] + "  mixrate " + ruleMixRate + "%  life " + ruleLifeMin +
         "-" + ruleLifeMax + "  expire " + expireNames[ruleExpireFade] + "  round " + (ruleRound ? "on" : "off") + "\n";
}

String rulesJson() {
  return String("\"rules\":{\"mix\":\"") + mixNames[ruleMix] + "\",\"mixrate\":" + ruleMixRate + ",\"life\":[" +
         ruleLifeMin + "," + ruleLifeMax + "],\"expire\":\"" + expireNames[ruleExpireFade] + "\",\"round\":" +
         (ruleRound ? "true" : "false") + "}";
}
