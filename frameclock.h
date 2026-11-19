#pragma once

#include <Arduino.h>
#include <FastLED.h>

// Frame pacing and timing. The simulation advances in 75 ms steps; the LEDs refresh FRAMES_PER_STEP times per step.

#define STEP_US         75000UL
#define FRAMES_PER_STEP 4
#define FRAME_US        (STEP_US / FRAMES_PER_STEP)
#define SHOW_GAP_US     9000UL   // least time between show() starts: a 256-LED transfer plus margin

// Frame timing over the last full second.
struct FrameStats {
  float fps;
  float workAvgMs;   // loop work per frame, network handling and show() included
  float workMaxMs;
  uint32_t late;     // frames since boot whose work overran their slot
};
static FrameStats frameStats = {0, 0, 0, 0};

static uint32_t frameStartUs = 0, frameDeadlineUs = 0;
static uint32_t statWindowUs = 0, statFrames = 0, statWorkSumUs = 0, statWorkMaxUs = 0;
static uint32_t lastShowUs = 0;     // when the last show() started
static bool showStarted = false;    // whether lastShowUs is set

// Marks the start of a frame's work; closes the stats window once a second has passed since it opened, counting
// frames start to start.
void frameBegin() {
  frameStartUs = micros();
  if (!frameDeadlineUs) frameDeadlineUs = statWindowUs = frameStartUs;
  if (statFrames && frameStartUs - statWindowUs >= 1000000UL) {
    frameStats.fps = statFrames * 1e6f / (frameStartUs - statWindowUs);
    frameStats.workAvgMs = statWorkSumUs / 1000.0f / statFrames;
    frameStats.workMaxMs = statWorkMaxUs / 1000.0f;
    statWindowUs = frameStartUs;
    statFrames = statWorkSumUs = statWorkMaxUs = 0;
  }
}

// Sends the LEDs, first waiting until SHOW_GAP_US has passed since the previous show() started.
void frameShow() {
  if (showStarted) {
    int32_t left;
    while ((left = (int32_t)(lastShowUs + SHOW_GAP_US - micros())) > 0) {
      if (left > 1000) delay(left / 1000);
      else delayMicroseconds(left);
    }
  }
  lastShowUs = micros();
  showStarted = true;
  FastLED.show();
}

// Records the frame's work time, then waits for the next frame's slot. After a stall of more than a frame the
// clock restarts rather than rushing to catch up.
void frameEnd() {
  uint32_t work = micros() - frameStartUs;
  statFrames++;
  statWorkSumUs += work;
  if (work > statWorkMaxUs) statWorkMaxUs = work;

  frameDeadlineUs += FRAME_US;
  int32_t left = (int32_t)(frameDeadlineUs - micros());
  if (left < 0) {
    frameStats.late++;
    if (left < -(int32_t)FRAME_US) frameDeadlineUs = micros();
    return;
  }
  while ((left = (int32_t)(frameDeadlineUs - micros())) > 0) {
    if (left > 1000) delay(left / 1000);
    else delayMicroseconds(left);
  }
}

String frameStatus() {
  char buf[96];
  snprintf(buf, sizeof(buf), "frames:      %.1f fps (target %.1f)  work %.2f ms avg, %.2f ms max  late %lu\n",
           frameStats.fps, 1e6f / FRAME_US, frameStats.workAvgMs, frameStats.workMaxMs, (unsigned long)frameStats.late);
  return String(buf);
}

String frameJson() {
  char buf[128];
  snprintf(buf, sizeof(buf), "\"frames\":{\"fps\":%.1f,\"targetFps\":%.1f,\"workAvgMs\":%.2f,\"workMaxMs\":%.2f,\"late\":%lu}",
           frameStats.fps, 1e6f / FRAME_US, frameStats.workAvgMs, frameStats.workMaxMs, (unsigned long)frameStats.late);
  return String(buf);
}
