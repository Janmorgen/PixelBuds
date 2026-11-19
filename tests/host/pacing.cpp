// Frame pacing against the virtual clock: frameBegin()/frameEnd() hold a fixed grid and recover from overruns,
// and loop()'s show() calls stay far enough apart for each LED transfer to finish.
#include "common.h"
#include <ArxContainer.h>
#include "network.h"
#include "frameclock.h"
static uint32_t preShowWorkUs = 0;   // work time loop() spends just before show(), injected below
#define dayCycleReportPixels(n) (simMicros += preShowWorkUs, dayCycleReportPixels(n))
#include "PixelBuds.ino"
#undef dayCycleReportPixels
#ifndef SHOW_GAP_US
#define SHOW_GAP_US 9000
#endif

static std::vector<uint32_t> showTimes;
static void recordShow() { showTimes.push_back(micros()); }

static const uint32_t F = FRAME_US;

static void reset() {
  simMicros = 1000000;
  frameStartUs = frameDeadlineUs = statWindowUs = 0;
  frameClockStarted = false;
  statFrames = statWorkSumUs = statWorkMaxUs = 0;
  frameStats = {0, 0, 0, 0};
}

static uint32_t overheadUs = 0;   // time between frameEnd() and the next frameBegin(), as loop() returning takes

// Runs `n` frames; workUs(k) is frame k's work time. Returns each frame's start time.
static std::vector<uint32_t> run(int n, uint32_t (*workUs)(int)) {
  std::vector<uint32_t> starts;
  for (int k = 0; k < n; k++) {
    frameBegin();
    starts.push_back(micros());
    simMicros += workUs(k);
    frameEnd();
    simMicros += overheadUs;
  }
  return starts;
}

// The late count and the schedule around overruns, with overheadUs between frames.
static void lateChecks() {
  const uint32_t ov = overheadUs;
  // One frame of 1x to 2x FRAME_US: late counts once and the schedule returns to the original grid.
  reset();
  auto b = run(40, [](int k) -> uint32_t { return k == 10 ? 25000 : 5000; });
  CHECK(frameStats.late == 1, "overhead %u us, overrun 25 ms: late = %u, want 1", ov, frameStats.late);
  for (size_t k = 1; k < b.size(); k++) {
    if (k == 11) CHECK(b[k] == b[10] + 25000 + ov, "overhead %u us, overrun 25 ms: the frame after started at +%u, want +%u", ov, b[k] - b[10], 25000 + ov);
    else CHECK(b[k] == b[0] + k * F + ov, "overhead %u us, overrun 25 ms: frame %zu is %d us off the grid", ov, k, (int)(b[k] - b[0] - k * F - ov));
  }

  // More than 2x FRAME_US: the deadline restarts from now, with no burst of back-to-back frames.
  reset();
  auto c = run(40, [](int k) -> uint32_t { return k == 10 ? 50000 : 5000; });
  CHECK(frameStats.late == 1, "overhead %u us, overrun 50 ms: late = %u, want 1", ov, frameStats.late);
  CHECK(c[11] == c[10] + 50000 + ov, "overhead %u us, overrun 50 ms: next frame started at +%u, want +%u", ov, c[11] - c[10], 50000 + ov);
  for (size_t k = 12; k < c.size(); k++) CHECK(c[k] == c[11] + (k - 11) * F, "overhead %u us, overrun 50 ms: frame %zu is off the restarted grid", ov, k);

  // One overrun of 20-37 ms, then a frame long enough to end past its own slot: one late frame.
  static uint32_t over;
  for (over = 20000; over <= 37000; over += 500) {
    reset();
    run(40, [](int k) -> uint32_t { return k == 10 ? over : k == 11 ? 9000 : 5000; });
    CHECK(frameStats.late == 1, "overhead %u us, overrun %u us then 9 ms: late = %u, want 1", ov, over, frameStats.late);
  }

  // A single overrun of any length, restart included, counts once.
  for (over = F + 1; over <= 10 * F; over += 997) {
    reset();
    run(40, [](int k) -> uint32_t { return k == 10 ? over : 5000; });
    CHECK(frameStats.late == 1, "overhead %u us, isolated overrun %u us: late = %u, want 1", ov, over, frameStats.late);
  }

  // Isolated overruns each count once.
  reset();
  run(200, [](int k) -> uint32_t { return k % 50 == 10 ? 25000 : 5000; });
  CHECK(frameStats.late == 4, "overhead %u us, 4 isolated 25 ms overruns: late = %u, want 4", ov, frameStats.late);

  // Back-to-back overruns count once while the clock stays behind, and again after it restarts.
  reset();
  run(40, [](int k) -> uint32_t { return k >= 10 && k < 13 ? 22000 : 5000; });
  CHECK(frameStats.late == 1, "overhead %u us, 3 frames of 22 ms: late = %u, want 1", ov, frameStats.late);
  reset();
  run(40, [](int k) -> uint32_t { return k >= 10 && k < 12 ? 30000 : 5000; });
  CHECK(frameStats.late == 1, "overhead %u us, 2 frames of 30 ms (restart on the second): late = %u, want 1", ov, frameStats.late);
  reset();
  run(40, [](int k) -> uint32_t { return k >= 10 && k < 13 ? 30000 : 5000; });
  CHECK(frameStats.late == 2, "overhead %u us, 3 frames of 30 ms (overrun after the restart): late = %u, want 2", ov, frameStats.late);

  printf("overhead %2u us: late %s\n", ov, failures ? "checks failed" : "counts each overrun once");
}

int main() {
  // Short work: starts exactly FRAME_US apart, never late.
  reset();
  auto a = run(300, [](int) -> uint32_t { return 5000; });
  for (size_t k = 1; k < a.size(); k++) CHECK(a[k] - a[k - 1] == F, "short work: frame %zu started %u us after the last", k, a[k] - a[k - 1]);
  CHECK(frameStats.late == 0, "short work: late = %u", frameStats.late);
  printf("short work: late %u, fps %.2f\n", frameStats.late, frameStats.fps);
  CHECK(fabs(frameStats.fps - 53.3f) <= 0.1f, "fps reads %.3f, want 53.3 +-0.1", frameStats.fps);

  for (uint32_t ov : {0u, 1u, 3u, 10u}) {
    overheadUs = ov;
    lateChecks();
  }
  overheadUs = 3;

  // The reported rate reads the target from the first one-second window on.
  reset();
  std::vector<float> fps;
  float firstFps = 0;
  for (int k = 0; k < 400; k++) {
    frameBegin(); simMicros += 3000; frameEnd(); simMicros += overheadUs;
    if (firstFps == 0) firstFps = frameStats.fps;
    if (k % 54 == 53) fps.push_back(frameStats.fps);
  }
  printf("fps readings:"); for (float f : fps) printf(" %.2f", f); printf("  first %.2f\n", firstFps);
  CHECK(fabs(firstFps - 53.3f) <= 0.1f, "first fps reading %.3f, want 53.3 +-0.1", firstFps);
  CHECK(fabs(frameStats.fps - 53.3f) <= 0.1f, "settled fps %.3f, want 53.3 +-0.1", frameStats.fps);

  // A deadline that lands on micros() == 0 does not restart the stats window.
  reset();
  simMicros = (1ULL << 32) - 10 * (uint64_t)F;
  frameBegin();
  const uint32_t window = statWindowUs;
  frameEnd();
  for (int k = 1; k <= 10; k++) {
    frameBegin();
    if (k == 10) CHECK(frameDeadlineUs == 0, "the deadline is %u at frame 10, want 0", frameDeadlineUs);
    simMicros += 3000;
    frameEnd();
    simMicros += overheadUs;
  }
  CHECK(statWindowUs == window, "stats window moved from %u to %u when the deadline reached 0", window, statWindowUs);
  printf("deadline of exactly 0: stats window kept\n");

  // loop() with one frame's work overrunning its slot by 1 to 30 ms at a time: show() calls never come closer
  // together than SHOW_GAP_US.
  reset();
  setup();
  startTestHold(-1, 0, 0);
  showHook = recordShow;
  int overruns = 0;
  for (uint32_t over = 1000; over <= 30000; over += 1000) {
    for (int k = 0; k < 12; k++) {
      preShowWorkUs = k == 8 ? F + over : 2000;
      loop();
      simMicros += overheadUs;
    }
    overruns++;
  }
  showHook = nullptr;
  uint32_t minGap = UINT32_MAX;
  int tooClose = 0;
  for (size_t k = 1; k < showTimes.size(); k++) {
    uint32_t gap = showTimes[k] - showTimes[k - 1];
    minGap = std::min(minGap, gap);
    if (gap < SHOW_GAP_US) tooClose++;
  }
  printf("show gaps over %d overruns of 1-30 ms: shortest %u us, %d below %u us\n", overruns, minGap, tooClose,
         (unsigned)SHOW_GAP_US);
  CHECK(tooClose == 0, "%d show() calls came less than %u us after the previous one (shortest %u us)", tooClose,
        (unsigned)SHOW_GAP_US, minGap);
  CHECK(SHOW_GAP_US >= 8000, "SHOW_GAP_US %u is shorter than a 256-LED transfer", (unsigned)SHOW_GAP_US);

  printf(failures ? "pacing: %d failure(s)\n" : "pacing: ok\n", failures);
  return failures != 0;
}
