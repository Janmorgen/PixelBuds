// Frame pacing against the virtual clock: frameBegin()/frameEnd() hold a fixed grid and recover from overruns.
#include "common.h"
#include "frameclock.h"

static const uint32_t F = FRAME_US;

static void reset() {
  simMicros = 1000000;
  frameStartUs = frameDeadlineUs = statWindowUs = 0;
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

  // The reported rate settles on the target after a few seconds (the first window reads high).
  reset();
  std::vector<float> fps;
  for (int k = 0; k < 400; k++) {
    frameBegin(); simMicros += 3000; frameEnd();
    if (k % 54 == 53) fps.push_back(frameStats.fps);
  }
  printf("fps readings:"); for (float f : fps) printf(" %.2f", f); printf("\n");
  CHECK(fabs(frameStats.fps - 53.3f) <= 0.1f, "settled fps %.3f, want 53.3 +-0.1", frameStats.fps);

  printf(failures ? "pacing: %d failure(s)\n" : "pacing: ok\n", failures);
  return failures != 0;
}
