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

  // Stress: 200 % pixels, 400 % speed and mix, every effect at full boost, a test day through every transition, and a
  // shuffle every 500 steps. After a short warmup nothing may allocate.
  adjustPixels = 200; adjustSpeed = 400; adjustMix = 400;
  effectBoost = 5;
  startTestDay();
  for (int s = 0; s < 300; s++) for (int f = 0; f < FRAMES_PER_STEP; f++) loop();
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
  printf("stress: heap allocations in %d steps: %ld (up to %zu pixels, test day %s)\n", stressSteps, allocs, maxPixels,
         testActive ? "still running" : "finished");
  CHECK(allocs == 0, "stress: %ld heap allocations after warmup", allocs);
  printf(failures ? "alloc: FAILED\n" : "alloc: ok\n");
  return failures != 0;
}
