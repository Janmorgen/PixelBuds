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
  printf(failures ? "alloc: FAILED\n" : "alloc: ok\n");
  return failures != 0;
}
