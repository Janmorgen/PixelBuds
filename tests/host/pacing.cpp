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

// Runs `n` frames; workUs(k) is frame k's work time. Returns each frame's start time.
static std::vector<uint32_t> run(int n, uint32_t (*workUs)(int)) {
  std::vector<uint32_t> starts;
  for (int k = 0; k < n; k++) {
    frameBegin();
    starts.push_back(micros());
    simMicros += workUs(k);
    frameEnd();
  }
  return starts;
}

int main() {
  // Short work: starts exactly FRAME_US apart, never late.
  reset();
  auto a = run(300, [](int) -> uint32_t { return 5000; });
  for (size_t k = 1; k < a.size(); k++) CHECK(a[k] - a[k - 1] == F, "short work: frame %zu started %u us after the last", k, a[k] - a[k - 1]);
  CHECK(frameStats.late == 0, "short work: late = %u", frameStats.late);
  printf("short work: late %u, fps %.2f\n", frameStats.late, frameStats.fps);
  CHECK(fabs(frameStats.fps - 53.3f) <= 0.1f, "fps reads %.3f, want 53.3 +-0.1", frameStats.fps);

  // One frame of 1x to 2x FRAME_US: late counts once and the schedule returns to the original grid.
  reset();
  auto b = run(40, [](int k) -> uint32_t { return k == 10 ? 25000 : 5000; });
  CHECK(frameStats.late == 1, "overrun 25 ms: late = %u, want 1", frameStats.late);
  for (size_t k = 0; k < b.size(); k++) {
    if (k == 11) CHECK(b[k] == b[10] + 25000, "overrun 25 ms: the frame after started at +%u, want +25000", b[k] - b[10]);
    else CHECK(b[k] == b[0] + k * F, "overrun 25 ms: frame %zu is %d us off the grid", k, (int)(b[k] - b[0] - k * F));
  }
  printf("overrun 25 ms: late %u, back on grid from frame 12\n", frameStats.late);

  // More than 2x FRAME_US: the deadline restarts from now, with no burst of back-to-back frames.
  reset();
  auto c = run(40, [](int k) -> uint32_t { return k == 10 ? 50000 : 5000; });
  CHECK(frameStats.late == 1, "overrun 50 ms: late = %u, want 1", frameStats.late);
  CHECK(c[11] == c[10] + 50000, "overrun 50 ms: next frame started at +%u, want +50000", c[11] - c[10]);
  for (size_t k = 12; k < c.size(); k++) CHECK(c[k] - c[k - 1] == F, "overrun 50 ms: frame %zu started %u us after the last, want %u", k, c[k] - c[k - 1], F);
  for (size_t k = 12; k < c.size(); k++) CHECK(c[k] == c[11] + (k - 11) * F, "overrun 50 ms: frame %zu is off the restarted grid", k);
  printf("overrun 50 ms: late %u, grid restarted from the overrun frame\n", frameStats.late);

  // The reported rate reads the target from the first one-second window on.
  reset();
  std::vector<float> fps;
  float firstFps = 0;
  for (int k = 0; k < 400; k++) {
    frameBegin(); simMicros += 3000; frameEnd();
    if (firstFps == 0) firstFps = frameStats.fps;
    if (k % 54 == 53) fps.push_back(frameStats.fps);
  }
  printf("fps readings:"); for (float f : fps) printf(" %.2f", f); printf("  first %.2f\n", firstFps);
  CHECK(fabs(firstFps - 53.3f) <= 0.1f, "first fps reading %.3f, want 53.3 +-0.1", firstFps);
  CHECK(fabs(frameStats.fps - 53.3f) <= 0.1f, "settled fps %.3f, want 53.3 +-0.1", frameStats.fps);

  // One overrun of 20-37 ms counts as one late frame.
  for (uint32_t over = 20000; over <= 37000; over += 500) {
    reset();
    for (int k = 0; k < 40; k++) {
      frameBegin();
      simMicros += k == 10 ? over : k == 11 ? 9000 : 5000;
      frameEnd();
    }
    CHECK(frameStats.late == 1, "overrun %u us: late = %u, want 1", over, frameStats.late);
  }
  printf("single overruns of 20-37 ms each count as 1 late frame\n");

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
