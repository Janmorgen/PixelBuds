// Speed and mix ladders: the move and mix rates per simulated second must follow /adjust?speed= and ?mix=.
// The rates are the frames per simulated second that run a move or a mix (moveTicks, mixTicks).
// Also checks that one-cell pixel moves and reads of ruleMix happen only on tick frames.
#include "common.h"
#include <unistd.h>
#include <sys/wait.h>
#include <ArxContainer.h>
#include "network.h"
#include "effects.h"
#include "rules.h"
static long mixReads = 0;
#define ruleMix (mixReads++, ruleMix)
#include "PixelBuds.ino"
#undef ruleMix
#ifndef FRAMES_PER_STEP
#define FRAMES_PER_STEP 1
#endif

struct Result { double move, mix; long strayMoves, strayMixes; };

static std::vector<int> positions() {
  std::vector<int> v;
  for (pixel& p : pixels) v.push_back(p.xPos * 8 + p.yPos);
  return v;
}

// True if any pixel moved exactly one cell; a respawn jumps further, so it does not count.
static bool stepped(const std::vector<int>& a, const std::vector<int>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (a[i] == b[i]) continue;
    if (abs(a[i] / 8 - b[i] / 8) + abs(a[i] % 8 - b[i] % 8) == 1) return true;
  }
  return false;
}

// Move and mix tick rates per simulated second over `steps` steps at the given override percentages.
static Result measure(int look, int speed, int mix, int steps) {
  simRng.seed(1);
  adjustSpeed = speed; adjustMix = mix;
  setup();
  startTestHold(-1, look, 0);
  for (int f = 0; f < 200 * FRAMES_PER_STEP; f++) loop();
  uint64_t t0 = simMicros;
  uint32_t move0 = moveTicks, mix0 = mixTicks;
  Result r = {0, 0, 0, 0};
  std::vector<int> prev = positions();
  for (int f = 0; f < steps * FRAMES_PER_STEP; f++) {
    long reads = mixReads;
    uint32_t mt = moveTicks, xt = mixTicks;
    loop();
    std::vector<int> now = positions();
    if (stepped(prev, now) && moveTicks == mt) r.strayMoves++;
    if (mixReads != reads && mixTicks == xt) r.strayMixes++;
    prev = now;
  }
  double secs = (simMicros - t0) / 1e6;
  r.move = (moveTicks - move0) / secs;
  r.mix = (mixTicks - mix0) / secs;
  return r;
}

// Runs measure() in a child process so every run starts from the sketch's boot state.
static Result measureFresh(int look, int speed, int mix, int steps) {
  int fd[2];
  if (pipe(fd) != 0) abort();
  pid_t pid = fork();
  if (pid == 0) {
    Result r = measure(look, speed, mix, steps);
    if (write(fd[1], &r, sizeof r) != (ssize_t)sizeof r) _exit(1);
    _exit(0);
  }
  Result r = {0, 0, 0, 0};
  if (read(fd[0], &r, sizeof r) != (ssize_t)sizeof r) abort();
  waitpid(pid, nullptr, 0);
  close(fd[0]); close(fd[1]);
  return r;
}

int main() {
  const int ladder[] = {25, 50, 75, 100, 150, 200, 300, 400};
  const int n = 8;
  for (int look = 0; look < 4; look++) {
    for (int which = 0; which < 2; which++) {
      const char* name = which ? "mix" : "move";
      double rate[n];
      for (int i = 0; i < n; i++) {
        Result r = measureFresh(look, which ? 100 : ladder[i], which ? ladder[i] : 100, 3000);
        rate[i] = which ? r.mix : r.move;
        CHECK(r.strayMoves <= 60, "look %d %s %d%%: %ld one-cell moves on frames with no move tick", look, name, ladder[i], r.strayMoves);
        CHECK(r.strayMixes == 0, "look %d %s %d%%: %ld mixes on frames with no mix tick", look, name, ladder[i], r.strayMixes);
      }
      printf("look %d %s/s:", look, name);
      for (int i = 0; i < n; i++) printf(" %d%%=%.3f", ladder[i], rate[i]);
      printf("\n");
      for (int i = 1; i < n; i++)
        CHECK(rate[i] > rate[i - 1], "look %d %s: %d%% (%.3f/s) is not above %d%% (%.3f/s)", look, name, ladder[i], rate[i], ladder[i - 1], rate[i - 1]);
      for (int i = 0; i < n; i++) {
        double want = ladder[i] / 100.0 * rate[3];
        CHECK(fabs(rate[i] - want) <= 0.03 * want, "look %d %s: %d%% gives %.3f/s, want %.3f/s +-3%%", look, name, ladder[i], rate[i], want);
      }
    }
  }
  printf(failures ? "ladder: %d failure(s)\n" : "ladder: ok\n", failures);
  return failures != 0;
}
