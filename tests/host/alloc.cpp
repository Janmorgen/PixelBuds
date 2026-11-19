// No heap allocation per frame once warmed up: day look, every effect on.
#include "common.h"
#include <new>
static bool counting = false;
static long allocs = 0;
void* operator new(size_t n) {
  if (counting) allocs++;
  void* p = malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](size_t n) { return operator new(n); }
void operator delete(void* p) noexcept { free(p); }
void operator delete[](void* p) noexcept { free(p); }
void operator delete(void* p, size_t) noexcept { free(p); }
void operator delete[](void* p, size_t) noexcept { free(p); }
#include <ArxContainer.h>
#include "PixelBuds.ino"

int main() {
  effectAgeing = 1; effectDeath = 7; effectIsland = 3; effectStrengthSetting = 1; effectBoost = 2;
  simRng.seed(1);
  setup();
  startTestHold(-1, 0, 0);
  for (int f = 0; f < 500 * FRAMES_PER_STEP; f++) loop();
  counting = true;
  for (int f = 0; f < 500 * FRAMES_PER_STEP; f++) loop();
  counting = false;
  printf("heap allocations in 500 steps (%d frames): %ld\n", 500 * FRAMES_PER_STEP, allocs);
  CHECK(allocs == 0, "%ld heap allocations after warmup", allocs);

  // Stress on the held day look, where the overrides stay in force: 200 % pixels (the most /adjust allows), then
  // 400 %, 400 % speed and mix, every effect at full boost, and a shuffle every 500 steps. Nothing may allocate from
  // the moment the overrides are set, as the pixel count grows.
  for (int pct : {200, 400}) {
    startTestHold(-1, 0, 0);
    adjustPixels = pct; adjustSpeed = 400; adjustMix = 400;
    effectBoost = 5;
    counting = true;
    allocs = 0;
    const int stressSteps = 25000;
    size_t maxPixels = 0;
    for (int s = 0; s < stressSteps; s++) {
      if (s % 500 == 250) shuffleRequested = true;
      for (int f = 0; f < FRAMES_PER_STEP; f++) loop();
      maxPixels = std::max(maxPixels, pixels.size());
    }
    counting = false;
    printf("held day, pixels %d %%: heap allocations in %d steps: %ld (up to %zu pixels, pixels override %d %%)\n", pct,
           stressSteps, allocs, maxPixels, adjustPixels);
    CHECK(adjustPixels == pct, "held day: the pixels override went from %d %% to %d %%", pct, adjustPixels);
    CHECK(maxPixels >= (size_t)(55 * pct / 200), "held day, pixels %d %%: at most %zu pixels, want at least %d", pct,
          maxPixels, 55 * pct / 200);
    CHECK(allocs == 0, "held day, pixels %d %%: %ld heap allocations", pct, allocs);
  }

  // A test day through every transition, the overrides put back after each phase change clears them.
  auto applyOverrides = [] { adjustPixels = 200; adjustSpeed = 400; adjustMix = 400; };
  startTestDay();
  applyOverrides();
  counting = true;
  allocs = 0;
  const int daySteps = 25000;
  size_t maxPixels = 0;
  int phaseChanges = 0;
  char phase[32];
  strncpy(phase, activePhase, sizeof(phase) - 1);
  phase[sizeof(phase) - 1] = 0;
  for (int s = 0; s < daySteps; s++) {
    if (s % 500 == 250) shuffleRequested = true;
    for (int f = 0; f < FRAMES_PER_STEP; f++) {
      loop();
      if (strcmp(phase, activePhase) != 0) { phaseChanges++; strncpy(phase, activePhase, sizeof(phase) - 1); }
      applyOverrides();
    }
    maxPixels = std::max(maxPixels, pixels.size());
  }
  counting = false;
  printf("test day, pixels 200 %%: heap allocations in %d steps: %ld (up to %zu pixels, %d phase changes, test day %s)\n",
         daySteps, allocs, maxPixels, phaseChanges, testActive ? "still running" : "finished");
  CHECK(phaseChanges >= 4, "test day: only %d phase changes", phaseChanges);
  CHECK(maxPixels >= 55, "test day: at most %zu pixels, want at least 55", maxPixels);
  CHECK(allocs == 0, "test day: %ld heap allocations", allocs);
  printf(failures ? "alloc: FAILED\n" : "alloc: ok\n");
  return failures != 0;
}
