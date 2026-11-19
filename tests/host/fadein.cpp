// Fade-in after a move or spawn: writePixel(rampFrames) lifts the cell evenly from its colour at the move to the pixel's
// colour, checked per write against the contract, then through loop() on an embers board with one pixel.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static int contract(int start, int shown, int j, int R) {
  if (R <= 1 || j + 1 >= R) return shown;
  return (int)lroundf(start + (shown - start) * (float)(j + 1) / R);
}

static pixel makePixel(CRGB colour, int x, int y) {
  pixel p;
  p.color = colour;
  p.xPos = x;
  p.yPos = y;
  p.shownPresence = 1.0f;
  p.framesSinceMove = 0;
  return p;
}

static void clearBoard() {
  for (int i = 0; i < RGB_LED_NUM; i++) LEDs[i] = CRGB();
}

// Checks a cell against the contract; reports and returns false on a miss beyond +-1.
static bool checkCell(const char* what, int R, int j, const CRGB& led, const int want[3]) {
  for (int c = 0; c < 3; c++) {
    if (abs((int)led[c] - want[c]) > 1) {
      CHECK(false, "%s, ramp %d, write %d, channel %d: cell shows %d, contract says %d", what, R, j, c, led[c], want[c]);
      return false;
    }
  }
  return true;
}

int main() {
  simRng.seed(1);
  setup();
  startTestHold(-1, 3, 0);
  for (int c = 0; c < PALETTE_SIZE; c++) colourOverride[c] = 0;
  pixels.clear();
  initAvailableIndicies();
  effectsClear();
  const int X = 10, Y = 3, idx = getIndex(X, Y);

  // 1. Black start.
  for (int R : {21, 5, 2, 1}) {
    clearBoard();
    pixel p = makePixel(CRGB(200, 100, 40), X, Y);
    bool ok = true;
    for (int j = 0; j < R + 2; j++) {
      p.writePixel(R);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(0, p.color[c], j, R);
      ok &= checkCell("black start", R, j, LEDs[idx], want);
    }
    printf("black start, ramp %2d: %s\n", R, ok ? "follows the contract" : "MISMATCH");
  }

  // 2. Non-black start, with the cell overwritten between writes: start is taken once, at the first write.
  for (int R : {21, 5, 2}) {
    clearBoard();
    LEDs[idx] = CRGB(60, 60, 60);
    pixel p = makePixel(CRGB(255, 0, 0), X, Y);
    bool ok = true;
    for (int j = 0; j < R + 2; j++) {
      p.writePixel(R);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(60, p.color[c], j, R);
      ok &= checkCell("non-black start", R, j, LEDs[idx], want);
      LEDs[idx] = CRGB((j * 37) % 256, (j * 91 + 17) % 256, 200 - j);   // something else lands in the cell
    }
    printf("non-black start, ramp %2d: %s\n", R, ok ? "follows the contract" : "MISMATCH");
  }

  // 3. Presence changes mid-ramp.
  {
    const int R = 21;
    clearBoard();
    pixel p = makePixel(CRGB(200, 100, 40), X, Y);
    bool ok = true;
    for (int j = 0; j < R + 2; j++) {
      if (j == 6) p.shownPresence = 0.5f;
      if (j == 12) p.shownPresence = 0.8f;
      p.writePixel(R);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(0, (uint8_t)(p.color[c] * p.shownPresence), j, R);
      ok &= checkCell("presence change", R, j, LEDs[idx], want);
    }
    printf("presence change, ramp %2d: %s\n", R, ok ? "follows the contract" : "MISMATCH");
  }

  // 4. Through loop(): embers, one isolated pixel, its head cell's brightness after each move.
  {
    pixels.clear();
    initAvailableIndicies();
    effectsClear();
    clearBoard();
    for (int i = 0; i < RGB_LED_NUM; i++) for (int c = 0; c < 3; c++) trail[i][c] = 0;
    colourOverride[0] = 1;   // one colour in use, so the pixel is kept
    adjustPixels = 1;        // and the look wants a single pixel
    pixel p;
    p.randomize();
    p.color = CRGB(200, 100, 40);   // randomize() picks a colour; fix it
    p.spawnIndex = 0;
    p.role = ROLE_CORE;
    p.setPresence(1.0f);
    p.framesSinceMove = 1000;
    pixels.push_back(p);
    int moves = 0, bad = 0;
    for (int f = 0; f < 1500 && moves < 4; f++) {
      static uint16_t before[RGB_LED_NUM][3];
      memcpy(before, trail, sizeof(before));
      loop();
      if (pixels.size() != 1) { CHECK(false, "loop: the board has %d pixels, want 1", (int)pixels.size()); break; }
      pixel& q = pixels[0];
      if (q.framesSinceMove != 1) continue;
      // A move just wrote its first frame: follow the head cell until the ramp ends.
      const float moveInterval = max(1.0f, activeLook.moveEvery * FRAMES_PER_STEP);
      const int R = max(1, (int)lroundf(moveInterval) - FRAMES_PER_STEP + 1);
      const int x0 = q.getX(), y0 = q.getY(), idx2 = getIndex(x0, y0);
      bool darkStart = true;   // a cell still glowing from an earlier visit starts above black; skip those
      for (int c = 0; c < 3; c++) darkStart &= before[idx2][c] < (2 << 8);
      std::vector<int> trace;
      auto bright = [&]() { return (int)max(LEDs[idx2].r, max(LEDs[idx2].g, LEDs[idx2].b)); };
      trace.push_back(bright());
      for (int k = 1; k < R; k++) {
        loop();
        if (pixels.size() != 1 || pixels[0].getX() != x0 || pixels[0].getY() != y0 || pixels[0].framesSinceMove != k + 1) {
          trace.clear();
          break;
        }
        trace.push_back(bright());
      }
      if ((int)trace.size() != R || !darkStart) continue;   // moved again or got in the way; not a clean ramp
      const pixel& q2 = pixels[0];
      int target = 0;
      for (int c = 0; c < 3; c++) target = max(target, (int)(uint8_t)(q2.color[c] * q2.shownPresence));
      moves++;
      const int even = (target + R - 1) / R;
      int worstStep = 0, drops = 0;
      for (int k = 1; k < R; k++) {
        int d = trace[k] - trace[k - 1];
        if (d < 0) drops++;
        worstStep = max(worstStep, d);
      }
      printf("embers move %d: ramp %d frames, target %d, first %d, last %d, even step %d, biggest step %d, %d drops\n",
             moves, R, target, trace[0], trace.back(), even, worstStep, drops);
      bool ok = true;
      if (drops) { CHECK(false, "move %d: brightness dropped %d times during the ramp", moves, drops); ok = false; }
      if (worstStep > even + 1) { CHECK(false, "move %d: a frame stepped %d, the even step is %d", moves, worstStep, even); ok = false; }
      if (abs(trace.back() - target) > 1) { CHECK(false, "move %d: ended at %d, want %d", moves, trace.back(), target); ok = false; }
      bad += !ok;
    }
    CHECK(moves >= 2, "loop: only %d clean ramps seen", moves);
    printf("loop, embers: %d ramps checked, %d bad\n", moves, bad);
  }

  if (failures) { printf("FAILED: %d\n", failures); return 1; }
  printf("OK\n");
  return 0;
}
