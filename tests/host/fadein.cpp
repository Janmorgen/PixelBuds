// Fade-in after a move or spawn, against the writePixel contract. Sections: 1-3b unit tests of one pixel (black start, kept start
// with the cell overwritten between writes, presence changes, a sweep of ramps/starts/targets); 3c and 3d kept start and kept
// ramp length; 4 a loop() check on an embers board with one pixel; 5 thirty pixels through an evening-to-embers look jump at twelve
// seeds and switch offsets, requiring ramps that cross the switch and no halved heads.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static int contract(int start, int shown, int j, int R) {
  if (R <= 1 || j + 1 >= R) return shown;
  return (int)lroundf(start + (shown - start) * (float)(j + 1) / R);
}

// The pixel's held ramp length; 1 on a sketch that does not keep one.
template <class P> static auto rampLenOf(P& p, int) -> decltype((int)p.rampLen) { return (int)p.rampLen; }
template <class P> static int rampLenOf(P&, long) { return 1; }
template <class P> static int rampLenOf(P& p) { return rampLenOf(p, 0); }

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

// Writes a pixel with the given rampFrames per write; the cell is overwritten between writes. Checks every write against
// the contract with start from the first write and R from the first write.
static bool checkSequence(const char* what, CRGB startCell, CRGB colour, const std::vector<int>& ramps) {
  const int x = 7, y = 3, at = getIndex(x, y);
  clearBoard();
  LEDs[at] = startCell;
  pixel p = makePixel(colour, x, y);
  const int R = max(1, ramps[0]);
  bool ok = true;
  for (size_t j = 0; j < ramps.size(); j++) {
    p.writePixel(ramps[j]);
    int want[3];
    for (int c = 0; c < 3; c++) want[c] = contract(startCell[c], colour[c], (int)j, R);
    ok &= checkCell(what, R, (int)j, LEDs[at], want);
    LEDs[at] = CRGB((j * 13) % 256, 0, 255 - j % 256);
  }
  printf("%s: %s\n", what, ok ? "follows the contract" : "MISMATCH");
  return ok;
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

  // 3b. Sweep: many ramps, starts and targets, every write; +-1 against the contract, and exact for odd ramps
  // against an integer reference that rounds half away from zero.
  {
    int cells = 0, off = 0, inexact = 0;
    for (int R = 2; R <= 24; R++) {
      for (int start = 0; start < 256; start += 17) {
        for (int shown = 0; shown < 256; shown += 5) {
          clearBoard();
          const int s3[3] = {start, 255 - start, start / 2};
          for (int c = 0; c < 3; c++) LEDs[idx][c] = s3[c];
          pixel p = makePixel(CRGB(shown, 255 - shown, shown / 2), X, Y);
          for (int j = 0; j < R + 2; j++) {
            p.writePixel(R);
            for (int c = 0; c < 3; c++) {
              const int t = p.color[c], got = LEDs[idx][c];
              cells++;
              if (abs(got - contract(s3[c], t, j, R)) > 1) {
                if (off++ < 5) CHECK(false, "sweep: ramp %d, start %d, shown %d, write %d, channel %d: cell shows %d, contract says %d", R, s3[c], t, j, c, got, contract(s3[c], t, j, R));
              }
              if (R % 2 == 1) {
                int ref = t;
                if (j + 1 < R) {
                  const int d = t - s3[c], q = ((abs(d) * (j + 1) * 2 + R) / (2 * R));
                  ref = s3[c] + (d < 0 ? -q : q);
                }
                if (got != ref && inexact++ < 5) CHECK(false, "sweep: odd ramp %d, start %d, shown %d, write %d, channel %d: cell shows %d, exact reference %d", R, s3[c], t, j, c, got, ref);
              }
            }
          }
        }
      }
    }
    CHECK(off == 0 && inexact == 0, "sweep: %d cells off the contract, %d odd-ramp cells off the exact reference", off, inexact);
    printf("sweep: %d cells, %d off the contract, %d off the exact odd-ramp reference\n", cells, off, inexact);
  }

  // 3c. Start and ramp length are taken at the first write and kept: a cell from an earlier ramp must not leak in.
  {
    const int X2 = 20, Y2 = 5, idx2 = getIndex(X2, Y2);
    auto reset = [&]() { clearBoard(); LEDs[idx] = CRGB(0, 0, 250); };
    // Stale start: an earlier ramp from a blue cell, then a move to a black cell with later writes at other lengths.
    reset();
    pixel p = makePixel(CRGB(200, 0, 0), X, Y);
    for (int j = 0; j < 25; j++) p.writePixel(21);
    p.xPos = X2; p.yPos = Y2; p.framesSinceMove = 0;
    LEDs[idx2] = CRGB();
    bool ok = true;
    const int writes[] = {13, 1, 40, 5, 13, 21, 2, 13, 13, 13, 13, 13, 13, 13};
    for (int j = 0; j < 14; j++) {
      p.writePixel(writes[j]);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(0, p.color[c], j, 13);
      ok &= checkCell("stale start", 13, j, LEDs[idx2], want);
      if (j + 1 < 13) CHECK(LEDs[idx2].b == 0, "stale start, write %d: blue %d leaked in from the earlier ramp", j, LEDs[idx2].b);
    }
    printf("stale start, ramp 13: %s\n", ok ? "follows the contract" : "MISMATCH");

    // Ramp length 1 at the first write is kept: later writes at 13 show the colour as is.
    clearBoard();
    pixel a = makePixel(CRGB(200, 0, 0), X, Y);
    ok = true;
    for (int j = 0; j < 6; j++) {
      a.writePixel(j == 0 ? 1 : 13);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(0, a.color[c], j, 1);
      ok &= checkCell("kept length 1", 1, j, LEDs[idx], want);
    }
    printf("kept ramp length 1: %s\n", ok ? "follows the contract" : "MISMATCH");

    // Ramp length 21 at the first write is kept: later writes at 1 continue the ramp.
    clearBoard();
    pixel b = makePixel(CRGB(200, 100, 40), X, Y);
    ok = true;
    for (int j = 0; j < 23; j++) {
      b.writePixel(j == 0 ? 21 : 1);
      int want[3];
      for (int c = 0; c < 3; c++) want[c] = contract(0, b.color[c], j, 21);
      ok &= checkCell("kept length 21", 21, j, LEDs[idx], want);
    }
    printf("kept ramp length 21: %s\n", ok ? "follows the contract" : "MISMATCH");
  }

  // 3d. Ramp length 13 at the first write, then 93 on later writes; and a whole 93-frame ramp.
  {
    std::vector<int> v(30, 93);
    v[0] = 13;
    checkSequence("first write 13, later 93", CRGB(30, 30, 30), CRGB(255, 0, 90), v);
    std::vector<int> w(100, 93);
    checkSequence("whole ramp of 93", CRGB(250, 0, 0), CRGB(0, 255, 7), w);
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
    int moves = 0, bad = 0, litRamps = 0;
    for (int f = 0; f < 1500 && (moves < 4 || litRamps < 1); f++) {
      static CRGB before[RGB_LED_NUM], savedLeds[RGB_LED_NUM];
      static uint16_t savedTrail[RGB_LED_NUM][3];
      memcpy(savedLeds, LEDs, sizeof(savedLeds));
      memcpy(savedTrail, trail, sizeof(savedTrail));
      fadeBoard(activeLook.fade, frameInStep == 0);
      memcpy(before, LEDs, sizeof(before));
      memcpy(LEDs, savedLeds, sizeof(savedLeds));
      memcpy(trail, savedTrail, sizeof(savedTrail));
      loop();
      if (pixels.size() != 1) { CHECK(false, "loop: the board has %d pixels, want 1", (int)pixels.size()); break; }
      pixel& q = pixels[0];
      if (q.framesSinceMove != 1) continue;
      // A move just wrote its first frame: follow the head cell until the ramp ends.
      const float moveInterval = max(1.0f, activeLook.moveEvery * FRAMES_PER_STEP);
      const int R = max(1, (int)lroundf(moveInterval) - FRAMES_PER_STEP + 1);
      const int x0 = q.getX(), y0 = q.getY(), idx2 = getIndex(x0, y0);
      const CRGB startColour = before[idx2];   // the cell after that frame's fade, before the pixel writes
      const int startBright = max(startColour.r, max(startColour.g, startColour.b));
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
      if ((int)trace.size() != R) continue;   // moved again or got in the way; not a clean ramp
      const pixel& q2 = pixels[0];
      int target = 0;
      bool lit = false, upward = true;   // lit: the cell starts above black; upward: no channel starts above its target
      for (int c = 0; c < 3; c++) {
        const int t = (uint8_t)(q2.color[c] * q2.shownPresence);
        target = max(target, t);
        upward &= startColour[c] <= t;
      }
      lit = startBright > 2;
      if (lit && !upward) continue;   // only ramps that rise on every channel are checked
      if (lit) litRamps++;
      moves++;
      const int even = (target - startBright + R - 1) / R;
      int worstStep = trace[0] - startBright, leastStep = worstStep, drops = trace[0] < startBright;
      for (int k = 1; k < R; k++) {
        int d = trace[k] - trace[k - 1];
        if (d < 0) drops++;
        worstStep = max(worstStep, d);
        leastStep = min(leastStep, d);
      }
      printf("embers move %d: ramp %d frames, start %d, target %d, first %d, last %d, even step %d, biggest step %d, %d drops\n",
             moves, R, startBright, target, trace[0], trace.back(), even, worstStep, drops);
      bool ok = true;
      if (drops) { CHECK(false, "move %d: brightness dropped %d times during the ramp", moves, drops); ok = false; }
      if (worstStep > even) { CHECK(false, "move %d: a frame stepped %d, the even step is %d", moves, worstStep, even); ok = false; }
      const int lowStep = (target - startBright) / R;
      if (leastStep < lowStep) { CHECK(false, "move %d: a frame stepped only %d, the lower bound is %d", moves, leastStep, lowStep); ok = false; }
      if (abs(trace.back() - target) > 1) { CHECK(false, "move %d: ended at %d, want %d", moves, trace.back(), target); ok = false; }
      bad += !ok;
    }
    CHECK(moves >= 2, "loop: only %d clean ramps seen", moves);
    CHECK(litRamps >= 1, "loop: no ramp from a lit cell was checked");
    printf("loop, embers: %d ramps checked (%d from a lit cell), %d bad\n", moves, litRamps, bad);
  }

  // 5. Through loop(): 30 pixels, evening look then embers through a /test hold switch, at 12 consecutive seeds and switch offsets 5 frames apart.
  // Heads that stayed put (same colour and presence) must not drop by over half from the frame before; and enough heads
  // must have a ramp running across the switch, so the check cannot pass with nothing to see.
  {
    const char* seedEnv = getenv("FADEIN_SEED");
    const int seed0 = seedEnv ? atoi(seedEnv) : 7;
    int checked = 0, dips = 0, spanning = 0;
    auto brightest = [](const CRGB& c) { return (int)max(c.r, max(c.g, c.b)); };
    for (int k = 0; k < 12; k++) {
      simRng.seed(seed0 + k);
      pixels.clear();
      initAvailableIndicies();
      effectsClear();
      clearBoard();
      for (int c = 0; c < PALETTE_SIZE; c++) colourOverride[c] = 0;
      adjustPixels = 100;
      for (int i = 0; i < 30; i++) { pixel np; np.randomize(); pixels.push_back(np); }
      startTestHold(-1, 3, 0);
      for (int f = 0; f < 800 + 5 * k; f++) loop();
      startTestHold(-1, 2, 0);
      for (int f = 0; f < 800 + 5 * k; f++) loop();
      startTestHold(-1, 3, 0);
      for (int f = 0; f < 40; f++) {
        const size_t n = pixels.size();
        std::vector<int> ox(n), oy(n), ob(n), ofs(n);
        std::vector<CRGB> oc(n);
        std::vector<float> op(n);
        for (size_t i = 0; i < n; i++) {
          ox[i] = pixels[i].getX(); oy[i] = pixels[i].getY(); ob[i] = brightest(LEDs[getIndex(ox[i], oy[i])]);
          oc[i] = pixels[i].color; op[i] = pixels[i].shownPresence; ofs[i] = pixels[i].framesSinceMove;
        }
        loop();
        if (pixels.size() != n) continue;
        for (size_t i = 0; i < n; i++) {
          pixel& p = pixels[i];
          if (p.getX() != ox[i] || p.getY() != oy[i]) continue;
          const int now = brightest(LEDs[getIndex(ox[i], oy[i])]);
          if (f == 0 && ofs[i] >= 1 && ofs[i] + 1 < rampLenOf(p)) spanning++;
          if (p.color.r != oc[i].r || p.color.g != oc[i].g || p.color.b != oc[i].b || p.shownPresence != op[i]) continue;
          if (!(p.color.r >= 5 || p.color.g >= 5 || p.color.b >= 5)) continue;
          checked++;
          if (ob[i] >= 40 && now * 2 < ob[i]) dips++;
        }
      }
    }
    CHECK(checked > 0, "look jump: no head-frames were checked");
    CHECK(spanning >= 3, "look jump: only %d heads had a ramp running across the switch, want at least 3", spanning);
    CHECK(dips == 0, "look jump, evening to embers: %d head-frames halved from the frame before", dips);
    printf("look jump, evening to embers: %d head-frames checked, %d ramps across the switch, %d halved\n", checked, spanning, dips);
  }

  if (failures) { printf("FAILED: %d\n", failures); return 1; }
  printf("OK\n");
  return 0;
}
