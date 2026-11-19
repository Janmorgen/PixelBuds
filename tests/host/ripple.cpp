// Ripple drawing through loop() on an empty board: as the ring passes a cell its brightness rises, then falls, with
// no drop from frame to frame on the way up, and the peak lies between a ring drawn once per step's average over a
// step and its step-frame peak.
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static const float CX = 16.0f, CY = 4.0f, RADIUS = 12.0f;
static const int cells[][2] = {{17, 4}, {19, 4}, {21, 4}, {24, 4}, {20, 7}, {16, 0}};

// A cell's red value as a ring drawn on step frames only passes it, the board fading by `fade` per step across the
// step's frames: its brightest step-frame value (`tip`) and its brightest average over a step (`mean`).
struct PerStepRing { float tip, mean; };
static PerStepRing perStepRing(float fade, float dist) {
  float keepFrame = powf(1.0f - fade / 256.0f, 1.0f / FRAMES_PER_STEP);
  float v = 0;
  PerStepRing out = {0, 0};
  for (float r = 0; r < RADIUS + 2; r += RIPPLE_SPEED) {
    float sum = 0;
    for (int f = 0; f < FRAMES_PER_STEP; f++) {
      v *= keepFrame;
      if (f == 0) {
        v = floorf(v);
        float w = r < RADIUS ? 1.0f - fabsf(dist - r) : 0.0f;
        if (w > 0) v = min(255.0f, v + (int)(255 * w * (1.0f - r / RADIUS) * RIPPLE_LEVEL));
        out.tip = max(out.tip, v);
      }
      sum += floorf(v);
    }
    out.mean = max(out.mean, sum / FRAMES_PER_STEP);
  }
  return out;
}

int main() {
  simRng.seed(1);
  setup();
  startTestHold(-1, 0, 0);
  for (int c = 0; c < PALETTE_SIZE; c++) colourOverride[c] = 0;   // no colour: no pixel joins
  pixels.clear();
  initAvailableIndicies();
  effectsClear();

  for (int trailPct : {25, 100, 400}) {
    adjustTrail = trailPct;
    while (frameInStep != 1) loop();   // a ring starts just after a step frame, as trackIslands() adds it then
    for (int i = 0; i < RGB_LED_NUM; i++) { LEDs[i] = CRGB(); for (int c = 0; c < 3; c++) trail[i][c] = 0; }
    ripples.clear();
    ripples.push_back({CX, CY, 0.0f, RADIUS, RIPPLE_LEVEL, CRGB(255, 0, 0)});
    const int frames = (int)(RADIUS / RIPPLE_SPEED + 4) * FRAMES_PER_STEP;
    std::vector<std::vector<int>> trace(sizeof(cells) / sizeof(cells[0]));
    int endedAt = -1;
    for (int f = 0; f < frames; f++) {
      loop();
      if (ripples.empty() && endedAt < 0) endedAt = f;
      for (size_t k = 0; k < trace.size(); k++) trace[k].push_back(LEDs[getIndex(cells[k][0], cells[k][1])].r);
    }
    // The ring lasts radius / RIPPLE_SPEED steps, give or take the frames to the next step.
    const int life = (int)(RADIUS / RIPPLE_SPEED) * FRAMES_PER_STEP;
    CHECK(endedAt >= life - FRAMES_PER_STEP && endedAt <= life + FRAMES_PER_STEP,
          "trail %d %%: the ring ended at frame %d, want %d +-%d", trailPct, endedAt, life, FRAMES_PER_STEP);

    float fade = activeLook.fade;
    for (size_t k = 0; k < trace.size(); k++) {
      const std::vector<int>& t = trace[k];
      size_t peakAt = std::max_element(t.begin(), t.end()) - t.begin();
      int dropsBefore = 0, worstDrop = 0, risesAfter = 0;
      for (size_t f = 1; f < t.size(); f++) {
        int d = t[f] - t[f - 1];
        if (f <= peakAt && d < -1) { dropsBefore++; worstDrop = min(worstDrop, d); }
        if (f > peakAt && d > 1) risesAfter++;
      }
      float dist = hypotf(cells[k][0] - CX, cells[k][1] - CY);
      PerStepRing old = perStepRing(fade, dist);
      printf("trail %3d %% (fade %5.1f), cell (%2d,%d) %4.1f from the centre: peak %3d at frame %3zu (per-step ring: "
             "tip %3.0f, step mean %5.1f), %d drops before the peak (worst %d), %d rises after\n", trailPct, fade,
             cells[k][0], cells[k][1], dist, t[peakAt], peakAt, old.tip, old.mean, dropsBefore, worstDrop, risesAfter);
      CHECK(dropsBefore == 0, "trail %d %%, cell (%d,%d): brightness dropped %d times on the way up, by up to %d",
            trailPct, cells[k][0], cells[k][1], dropsBefore, -worstDrop);
      CHECK(risesAfter == 0, "trail %d %%, cell (%d,%d): brightness rose %d times after the peak", trailPct, cells[k][0],
            cells[k][1], risesAfter);
      CHECK(t[peakAt] >= old.mean - 2 && t[peakAt] <= old.tip + 2,
            "trail %d %%, cell (%d,%d): peak %d, outside the per-step ring's step mean %.1f to tip %.0f", trailPct,
            cells[k][0], cells[k][1], t[peakAt], old.mean, old.tip);
    }
  }
  // Paused: a ring stops growing and lights no cell.
  adjustTrail = 100;
  while (frameInStep != 1) loop();
  ripples.clear();
  ripples.push_back({CX, CY, 0.0f, RADIUS, RIPPLE_LEVEL, CRGB(255, 0, 0)});
  for (int i = 0; i < 8; i++) loop();
  CHECK(ripples.size() == 1 && ripples[0].r > 0.0f, "no growing ring to pause");
  float r0 = ripples[0].r;
  paused = true;
  int rises = 0;
  CRGB prev[RGB_LED_NUM];
  for (int f = 0; f < 40; f++) {
    for (int i = 0; i < RGB_LED_NUM; i++) prev[i] = LEDs[i];
    loop();
    for (int i = 0; i < RGB_LED_NUM; i++)
      for (int c = 0; c < 3; c++) if (LEDs[i][c] > prev[i][c]) rises++;
  }
  paused = false;
  printf("pause: ring r %.3f before, %.3f after 40 paused frames, %d LED rises\n", r0, ripples.empty() ? -1.0f : ripples[0].r, rises);
  CHECK(ripples.size() == 1 && ripples[0].r == r0, "the ring grew while paused (r %.3f -> %.3f)", r0, ripples.empty() ? -1.0f : ripples[0].r);
  CHECK(rises == 0, "%d LED values rose while paused", rises);
  printf(failures ? "ripple: %d failure(s)\n" : "ripple: ok\n", failures);
  return failures != 0;
}
