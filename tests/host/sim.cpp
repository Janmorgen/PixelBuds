// Host simulation of PixelBuds: runs the real sketch against stubs and records island and frame metrics. Built by parity.sh.
#include <Arduino.h>
#include <FastLED.h>
uint64_t simMicros = 0;
std::mt19937 simRng(1);

#ifdef NEWCODE
struct Island;
void simIslandEnd(const Island&);
#define ISLAND_END_HOOK(is) simIslandEnd(is)
#endif
#include "PixelBuds.ino"

static int warmup = 500, frames = 0, previewFrames = 0;
static long shown = 0, stepsShown = 0, powN = 0, previewed = 0;
static bool looping = false;   // set once setup() is done
static uint64_t ledHash = 1469598103934665603ULL;
static double sumLit = 0, sumSat = 0, sumVal = 0, sumPow = 0, peakPow = 0, sumIslands = 0, sumIslandSize = 0;
static long satN = 0, islandFrames = 0, overCap = 0;
static FILE* out;
static std::vector<std::pair<uint32_t, int>> endsLog;
static long created0 = 0;

#ifdef NEWCODE
void simIslandEnd(const Island& is) {
  if (stepsShown >= warmup) endsLog.push_back({islandStep - is.born, is.peak});
}
#endif

// True when the frame just shown ran a simulation step; loop() has already advanced frameInStep.
static bool stepJustRan() {
  if (!looping) return false;
#ifdef FRAMES_PER_STEP
  return frameInStep == 1 % FRAMES_PER_STEP;
#else
  return true;
#endif
}

// Power and the LED hash sample every frame; the simulation metrics sample once per step.
void simOnShow() {
  shown++;
  for (int i = 0; i < RGB_LED_NUM; i++) for (int c = 0; c < 3; c++) { ledHash ^= LEDs[i][c]; ledHash *= 1099511628211ULL; }
  bool step = stepJustRan();
  if (step) stepsShown++;
  if (stepsShown < warmup) return;
  if (step && stepsShown == warmup) {
#ifdef NEWCODE
    created0 = nextIslandId;
#endif
  }
  uint32_t r = 0, g = 0, b = 0;
  for (int i = 0; i < RGB_LED_NUM; i++) { r += LEDs[i].r; g += LEDs[i].g; b += LEDs[i].b; }
  uint32_t total = ((r * 80) + (g * 55) + (b * 75)) / 256 + RGB_LED_NUM * 5;
  double pw = (double)total * FastLED.brightness / 256 + 125;
  sumPow += pw;
  powN++;
  peakPow = max(peakPow, pw);
  if (pw > 2500) overCap++;
  if (previewed < previewFrames) {
    previewed++;
#ifdef NEWCODE
    fprintf(out, "F %d ", (int)ripples.size());
#else
    fprintf(out, "F 0 ");
#endif
    for (int y = 0; y < 8; y++) for (int x = 0; x < 32; x++) {
      CRGB c = LEDs[getIndex(x, y)];
      fprintf(out, "%02x%02x%02x", c.r, c.g, c.b);
    }
    fprintf(out, "\n");
  }
  if (!step) return;

  long lit = 0;
  for (pixel& p : pixels) {
    if (p.retired || p.dark() || p.presence < 0.05f) continue;
    lit++;
    uint8_t mx = max(p.color.r, max(p.color.g, p.color.b)), mn = min(p.color.r, min(p.color.g, p.color.b));
    sumSat += (mx - mn) / (double)mx;
    sumVal += mx * p.presence;
    satN++;
  }
  sumLit += lit;
#ifdef NEWCODE
  sumIslands += islandsNow;
  for (Island& is : islands) { sumIslandSize += is.size; islandFrames++; }
#endif
}

int main(int argc, char** argv) {
  // new: ageing death island strength boost look frames seed preview out
  // old: mode - - strength boost look frames seed preview out
  if (argc < 11) { fprintf(stderr, "args\n"); return 1; }
  int a = atoi(argv[1]), d = atoi(argv[2]), isl = atoi(argv[3]);
  float strength = atof(argv[4]), boost = atof(argv[5]);
  int look = atoi(argv[6]);
  frames = atoi(argv[7]);
  simRng.seed(atoi(argv[8]));
  previewFrames = atoi(argv[9]);
  out = fopen(argv[10], "w");
#ifdef NEWCODE
  effectAgeing = a; effectDeath = d; effectIsland = isl; effectBoost = boost; effectStrengthSetting = strength;
#else
  islandMode = a; islandBoost = boost; islandHoldSetting = strength; (void)d; (void)isl;
#endif
#ifndef FRAMES_PER_STEP
  const int FRAMES_PER_STEP = 1;
#endif
  setup();
  startTestHold(-1, look, 0);
  looping = true;
  for (long f = 0; f < (long)(warmup + frames) * FRAMES_PER_STEP; f++) loop();   // warmup and frames count steps
  long n = stepsShown - warmup + 1;   // steps sampled
  fprintf(out, "S lit=%.3f sat=%.4f val=%.2f pow=%.1f peakpow=%.1f islands=%.3f islandsize=%.3f frames=%ld hash=%016llx",
          sumLit / n, sumSat / max(1L, satN), sumVal / max(1L, satN), sumPow / max(1L, powN), peakPow, sumIslands / n,
          sumIslandSize / max(1L, islandFrames), n, (unsigned long long)ledHash);
#ifdef NEWCODE
  fprintf(out, " created=%ld over=%ld", (long)(nextIslandId - created0), overCap);
#endif
  fprintf(out, "\n");
  for (auto& e : endsLog) fprintf(out, "E %u %d\n", e.first, e.second);
  fclose(out);
}
