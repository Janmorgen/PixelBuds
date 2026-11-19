#pragma once
// Shared definitions for the host tests: the virtual clock, the seeded RNG, a show() hook and a check macro.
#include <Arduino.h>
#include <FastLED.h>
#include <cstdio>
uint64_t simMicros = 0;
std::mt19937 simRng(1);
static void (*showHook)() = nullptr;
void simOnShow() { if (showHook) showHook(); }

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
