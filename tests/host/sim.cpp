// Host simulation of PixelBuds: runs the real sketch against stubs and records island and frame metrics. Build with build.sh.
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
static long shown = 0;
static uint64_t ledHash = 1469598103934665603ULL;
static double sumLit = 0, sumSat = 0, sumVal = 0, sumPow = 0, peakPow = 0, sumIslands = 0, sumIslandSize = 0;
static long satN = 0, islandFrames = 0, overCap = 0;
static FILE* out;
static std::vector<std::pair<uint32_t, int>> endsLog;
static long created0 = 0;

#ifdef NEWCODE
void simIslandEnd(const Island& is) {
  if (shown >= warmup) endsLog.push_back({islandFrame - is.born, is.peak});
}
#endif

void simOnShow() {
  shown++;
  for (int i = 0; i < RGB_LED_NUM; i++) for (int c = 0; c < 3; c++) { ledHash ^= LEDs[i][c]; ledHash *= 1099511628211ULL; }
  if (shown < warmup) return;
  if (shown == warmup) {
#ifdef NEWCODE
    created0 = nextIslandId;
#endif
  }
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
  uint32_t r = 0, g = 0, b = 0;
  for (int i = 0; i < RGB_LED_NUM; i++) { r += LEDs[i].r; g += LEDs[i].g; b += LEDs[i].b; }
  uint32_t total = ((r * 80) + (g * 55) + (b * 75)) / 256 + RGB_LED_NUM * 5;
  double pw = (double)total * FastLED.brightness / 256 + 125;
  sumPow += pw;
  peakPow = max(peakPow, pw);
  if (pw > 2500) overCap++;
#ifdef NEWCODE
  sumIslands += islandsNow;
  for (Island& is : islands) { sumIslandSize += is.size; islandFrames++; }
#endif
  long k = shown - warmup;
  if (k < previewFrames) {
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
#ifdef FRAMES_PER_STEP
  warmup *= FRAMES_PER_STEP;   // warmup and FRAMES count simulation steps
  frames *= FRAMES_PER_STEP;
#endif
  setup();
  startTestHold(-1, look, 0);
  for (int f = 0; f < warmup + frames; f++) loop();
  long n = shown - warmup + 1;
  fprintf(out, "S lit=%.3f sat=%.4f val=%.2f pow=%.1f peakpow=%.1f islands=%.3f islandsize=%.3f frames=%ld hash=%016llx",
          sumLit / n, sumSat / max(1L, satN), sumVal / max(1L, satN), sumPow / n, peakPow, sumIslands / n,
          sumIslandSize / max(1L, islandFrames), n, (unsigned long long)ledHash);
#ifdef NEWCODE
  fprintf(out, " created=%ld over=%ld", (long)(nextIslandId - created0), overCap);
#endif
  fprintf(out, "\n");
  for (auto& e : endsLog) fprintf(out, "E %u %d\n", e.first, e.second);
  fclose(out);
}
