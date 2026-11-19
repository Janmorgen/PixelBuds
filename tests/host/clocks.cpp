// Move and mix clocks, presence glide, pause and shuffle against the real loop().
#include "common.h"
#include <ArxContainer.h>
#include "PixelBuds.ino"

static void steps(int n) { for (int f = 0; f < n * FRAMES_PER_STEP; f++) loop(); }

// Move and mix ticks per step must follow the look's moveEvery and colorEvery, also mid-transition.
static void rates() {
  const float minutes[] = {1050, 1065, 390, 1350};
  for (float minute : minutes) {
    startTestHold(minute, -1, 0);
    steps(20);
    uint32_t m0 = moveTicks, x0 = mixTicks;
    const int n = 600;
    steps(n);
    double move = (moveTicks - m0) / (double)n, mix = (mixTicks - x0) / (double)n;
    double wantMove = 1.0 / activeLook.moveEvery, wantMix = 1.0 / activeLook.colorEvery;
    printf("minute %4.0f: moves/step %.3f (want %.3f), mixes/step %.3f (want %.3f)\n", minute, move, wantMove, mix, wantMix);
    CHECK(fabs(move - wantMove) <= 0.02 * wantMove, "minute %.0f: %.3f moves per step, want %.3f +-2%%", minute, move, wantMove);
    CHECK(fabs(mix - wantMix) <= 0.02 * wantMix, "minute %.0f: %.3f mixes per step, want %.3f +-2%%", minute, mix, wantMix);
  }
}

// After the interval changes, ticks stay at least the new interval apart; the first may come early.
static void tickGaps() {
  struct Case { float clock, from, to; };
  const Case cases[] = {{27, 48, 3}, {47, 48, 1}, {10, 20, 4}, {0, 100, 5}, {20, 24, 2}};
  for (const Case& c : cases) {
    float clock = c.clock;
    int last = -1000, worst = 1000;
    bool first = true;
    for (int f = 0; f < 200; f++) {
      if (!tickClock(clock, f < 1 ? c.from : c.to)) continue;
      if (!first && f - last < worst) worst = f - last;
      first = false;
      last = f;
    }
    printf("clock %.0f, interval %.0f -> %.0f: shortest gap between later ticks %d\n", c.clock, c.from, c.to, worst);
    CHECK(worst >= (int)c.to, "interval %.0f -> %.0f from clock %.0f: ticks %d frames apart, want >= %d", c.from, c.to, c.clock, worst, (int)c.to);
  }
  float clock = 27;
  bool prev = false;
  int adjacent = 0;
  for (int f = 0; f < 100; f++) {
    bool t = tickClock(clock, f == 0 ? 48.0f : 3.0f);
    if (t && prev) adjacent++;
    prev = t;
  }
  CHECK(adjacent == 0, "%d ticks on consecutive frames after interval 48 -> 3", adjacent);
}

// Frozen while paused: clocks, positions, colours and ages; drawing goes on; ticks resume afterwards.
static int shows = 0;
static void countShow() { shows++; }
static std::vector<float> snap() {
  std::vector<float> v = {moveClock, mixClock, (float)moveTicks, (float)mixTicks, (float)pixels.size()};
  for (pixel& p : pixels)
    for (float x : {(float)p.xPos, (float)p.yPos, (float)p.color.r, (float)p.color.g, (float)p.color.b, p.turnsLived, (float)p.lifeTime}) v.push_back(x);
  return v;
}
static void pause() {
  startTestHold(-1, 1, 0);
  steps(100);
  paused = true;
  loop();
  std::vector<float> before = snap();
  shows = 0;
  showHook = countShow;
  steps(50);
  showHook = nullptr;
  CHECK(snap() == before, "state changed while paused");
  CHECK(shows == 50 * FRAMES_PER_STEP, "%d shows while paused, want %d", shows, 50 * FRAMES_PER_STEP);
  paused = false;
  uint32_t m0 = moveTicks, x0 = mixTicks;
  steps(50);
  CHECK(moveTicks > m0 && mixTicks > x0, "no ticks after resume (%u, %u)", moveTicks - m0, mixTicks - x0);
  printf("pause: state frozen over 50 steps, %d shows, ticks resumed\n", shows);
}

// The drawn presence has reached the simulated one by the last frame of every step.
static int presenceBad = 0, presenceChecked = 0;
static void checkPresence() {
  if (frameInStep != 0) return;
  for (pixel& p : pixels) {
    presenceChecked++;
    if (fabsf(p.shownPresence - p.presence) > 1e-5f) presenceBad++;
  }
}
static void presence() {
  effectAgeing = 1; effectDeath = 7; effectIsland = 3; effectStrengthSetting = 1; effectBoost = 2;
  startTestHold(1050, -1, 0);
  showHook = checkPresence;
  steps(400);
  paused = true;
  steps(20);
  paused = false;
  steps(200);
  showHook = nullptr;
  printf("presence: %d of %d pixel checks off on a step's last frame\n", presenceBad, presenceChecked);
  CHECK(presenceChecked > 0, "no presence checks ran");
  CHECK(presenceBad == 0, "%d pixels had shownPresence != presence on a step's last frame", presenceBad);
}

// Where a step's change in presence was drawn: each pixel moves an equal share per frame, strictly between its old
// and new value on frames 0-2, without overshoot.
static std::vector<float> glideFrom;
static int glideChecked = 0, glideBad = 0;
static void checkGlide() {
  int frame = (frameInStep + FRAMES_PER_STEP - 1) % FRAMES_PER_STEP;   // the frame just drawn
  if (frame == FRAMES_PER_STEP - 1) {
    glideFrom.resize(pixels.size());
    for (size_t i = 0; i < pixels.size(); i++) glideFrom[i] = pixels[i].shownPresence;
    return;
  }
  if (glideFrom.size() != pixels.size()) return;
  for (size_t i = 0; i < pixels.size(); i++) {
    float from = glideFrom[i], to = pixels[i].presence, now = pixels[i].shownPresence;
    if (fabsf(to - from) < 1e-3f) continue;
    glideChecked++;
    float share = (to - from) / FRAMES_PER_STEP;
    bool between = now > fminf(from, to) && now < fmaxf(from, to);
    if (!between || fabsf(now - (from + share * (frame + 1))) > 1e-5f) glideBad++;
  }
}
static void glide() {
  startTestHold(1050, -1, 0);
  steps(100);
  while (frameInStep != 0) loop();
  for (pixel& p : pixels) p.fadingOut = true;   // each step now lowers every pixel's presence by 1 / PRESENCE_FADE_STEPS
  showHook = checkGlide;
  steps(25);
  showHook = nullptr;
  printf("glide: %d of %d changed-presence checks off\n", glideBad, glideChecked);
  CHECK(glideChecked > 100, "only %d presence changes checked", glideChecked);
  CHECK(glideBad == 0, "%d presence changes were not drawn as equal shares per frame", glideBad);
}

// After a shuffle the board shows no trail of the old layout.
static void shuffle() {
  startTestHold(-1, 0, 0);
  steps(100);
  shuffleRequested = true;
  loop();
  bool occupied[RGB_LED_NUM] = {false};
  for (pixel& p : pixels) occupied[getIndex(p.xPos, p.yPos)] = true;
  int stale = 0;
  for (int i = 0; i < RGB_LED_NUM; i++) if (!occupied[i] && (LEDs[i].r || LEDs[i].g || LEDs[i].b)) stale++;
  printf("shuffle: %d lit cells without a pixel on the shuffle frame\n", stale);
  CHECK(stale == 0, "%d stale trail cells after a shuffle", stale);
}

int main() {
  simRng.seed(1);
  setup();
  rates();
  tickGaps();
  pause();
  presence();
  glide();
  shuffle();
  printf(failures ? "clocks: %d failure(s)\n" : "clocks: ok\n", failures);
  return failures != 0;
}
