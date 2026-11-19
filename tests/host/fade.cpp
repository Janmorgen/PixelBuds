// Trail fade: a step's fade over its four frames must end where fadeToBlackBy(fade) would, and never brighten.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

// One value through fadeBoard, as loop() calls it. A step starts from a value the previous step frame left.
static int fadeOnce(int v, float fade, const bool* stepFlags, int n, int& rises) {
  for (int i = 0; i < RGB_LED_NUM; i++) for (int c = 0; c < 3; c++) { LEDs[i][c] = v; trail[i][c] = v << 8; }
  int last = v;
  for (int f = 0; f < n; f++) {
    fadeBoard(fade, stepFlags[f]);
    int now = LEDs[5][1];
    if (now > last) rises++;
    last = now;
  }
  return last;
}

int main() {
  const float fades[] = {16, 28, 40, 56};
  // loop() order is step frame, then three more, so a value left by a step frame sees n n n S.
  const bool endsAtStep[4] = {false, false, false, true};
  const bool startsAtStep[4] = {true, false, false, false};
  for (float fade : fades) {
    for (int variant = 0; variant < 2; variant++) {
      const bool* flags = variant ? startsAtStep : endsAtStep;
      int mismatches = 0, maxDiff = 0, rises = 0;
      for (int v = 0; v < 256; v++) {
        int want = (v * (256 - (int)fade)) >> 8;
        int got = fadeOnce(v, fade, flags, 4, rises);
        int diff = abs(got - want);
        if (diff) mismatches++;
        maxDiff = std::max(maxDiff, diff);
      }
      printf("fade %2d, %s: mismatches %3d of 256, max difference %d, rises %d\n", (int)fade,
             variant ? "S n n n (step frame first)" : "n n n S (ends on step frame)", mismatches, maxDiff, rises);
      CHECK(rises == 0, "fade %d: value rose from frame to frame %d times", (int)fade, rises);
      if (!variant) CHECK(maxDiff <= 1, "fade %d: step result is %d away from fadeToBlackBy", (int)fade, maxDiff);
    }
  }
  printf(failures ? "fade: %d failure(s)\n" : "fade: ok\n", failures);
  return failures != 0;
}
