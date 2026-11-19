#pragma once

#include <math.h>
#include <vector>

#include "effects.h"

// Island tracking and the effects set at /effects. Included by PixelBuds.ino after the pixel class.

#define MAX_RIPPLES   2
#define SEED_FRAMES   40     // steps a seed waits for a pixel to respawn into it
#define SEED_REACH    2      // cells from the island's centre a seed may land
#define RIPPLE_SPEED  0.5f   // ring growth, cells per step
#define RIPPLE_LEVEL  0.25f   // ring brightness at its start, relative to the island's colour
#define RIPPLE_MAX    12.0f  // largest ring radius, cells
#define RIPPLE_MIN_PEAK   3  // only islands that reached this many pixels ripple
#define RIPPLE_MIN_FRAMES 24 // and lasted this many steps (about 2 s)

#ifndef ISLAND_END_HOOK
#define ISLAND_END_HOOK(island)
#endif

// A group of touching lit pixels, followed from step to step.
struct Island {
  int id;
  uint32_t born;    // step it formed
  int size;
  int peak;         // largest size it reached
  float cx, cy;     // centre
  CRGB colour;      // members' mean colour
  int spawnIndex;   // most common birth colour among the members
  bool seen;
};

struct Ripple {
  float cx, cy, r, radius, level;
  CRGB colour;
};

struct Seed {
  int x, y;
  CRGB colour;
  int spawnIndex;
  int framesLeft;
};

static std::vector<Island> islands;
static std::vector<Ripple> ripples;
static std::vector<Seed> seeds;
static int nextIslandId = 1;
static uint32_t islandFrame = 0;

// Effect strength in force: the look's, or the fixed setting.
static float fxStrength() {
  return activeLook.effectStrength;
}

static bool lit(const pixel& p) {
  return !p.retired && !p.fadingOut && !p.dark();
}

static bool cellFree(int x, int y, const pixel* except = nullptr) {
  for (pixel& px : pixels) {
    if (&px != except && !px.retired && px.xPos == x && px.yPos == y) return false;
  }
  return true;
}

// `c` with its grey part removed, as bright as before; black if it has no hue left.
static CRGB pure(CRGB c) {
  uint8_t hi = max(c.r, max(c.g, c.b)), lo = min(c.r, min(c.g, c.b));
  if (hi - lo < 16) return CRGB(0, 0, 0);
  return CRGB((c.r - lo) * hi / (hi - lo), (c.g - lo) * hi / (hi - lo), (c.b - lo) * hi / (hi - lo));
}

// How far one contact step ages a pixel with `contacts` neighbours.
float ageStep(int contacts) {
  if (effectAgeing != AGEING_EDGE) return 1.0f;
  float s = fxStrength();
  return 1.0f / ((1.0f + effectBoost * s) * (1.0f + (contacts - 1) * s));
}

// Runs the death effects as `dying` goes dark, on the neighbours it leaves.
void onPixelDeath(pixel& dying, std::vector<pixel*>& neighbors) {
  float s = fxStrength();
  if (s <= 0.0f) return;
  for (pixel* nb : neighbors) {
    if (!lit(*nb)) continue;

    if (effectDeath & DEATH_ORPHAN) {
      static std::vector<pixel*> around;
      nb->getNeighbors(around);
      int others = 0;
      for (pixel* other : around) {
        if (other != &dying && !other->dark()) others++;
      }
      if (others == 0) {
        int bonusLife = (int)lroundf(ruleLifeTime() * (1.0f + effectBoost * s));
        nb->turnsLived *= 1.0f - s;
        nb->lifeTime = max(nb->lifeTime, (int)lroundf(nb->lifeTime + s * (bonusLife - nb->lifeTime)));
        nb->shareTo = nb->turnsLived <= nb->lifeTime;
      }
    }

    if (effectDeath & DEATH_CONTAGION) {
      float left = nb->lifeTime - nb->turnsLived;
      if (left > 0) nb->turnsLived = nb->lifeTime - left / (1.0f + effectBoost * s);
    }

    if (effectDeath & DEATH_SCATTER) {
      nb->pushDx = (nb->xPos > dying.xPos) - (nb->xPos < dying.xPos);
      nb->pushDy = (nb->yPos > dying.yPos) - (nb->yPos < dying.yPos);
      nb->pushSteps = max(nb->pushSteps, (int)lroundf((1.0f + effectBoost) * s));
    }
  }
}

// Moves a pushed pixel one cell along its push; the push ends at an edge or another pixel.
void pushStep(pixel& p) {
  int x = p.xPos + p.pushDx, y = p.yPos + p.pushDy;
  if (x < p.minX || x >= p.maxX || y < p.minY || y >= p.maxY || !cellFree(x, y)) {
    p.pushSteps = 0;
    return;
  }
  addAvailableIndex(p.xPos, p.yPos);
  removeAvailableIndex(x, y);
  p.xPos = x;
  p.yPos = y;
  p.pushSteps--;
  p.framesSinceMove = 0;
}

// Runs the island effects as `is` breaks up for good.
static void onIslandEnd(const Island& is) {
  ISLAND_END_HOOK(is);
  float s = fxStrength();
  if (s <= 0.0f) return;

  if (effectIsland & ISLAND_SEED) {
    int count = (int)lroundf(min((float)is.peak, (1.0f + effectBoost) * s));
    CRGB colour = pure(is.colour);
    if (!colour.r && !colour.g && !colour.b) colour = paletteColors[is.spawnIndex];
    for (int i = 0; i < count; i++) {
      seeds.push_back({(int)lroundf(is.cx), (int)lroundf(is.cy), colour, is.spawnIndex, SEED_FRAMES});
    }
  }

  if ((effectIsland & ISLAND_RIPPLE) && is.peak >= RIPPLE_MIN_PEAK && islandFrame - is.born >= RIPPLE_MIN_FRAMES) {
    if (ripples.size() >= MAX_RIPPLES) ripples.erase(ripples.begin());
    float radius = min(RIPPLE_MAX, 2.0f + 1.5f * effectBoost * s * sqrtf((float)is.peak));
    CRGB colour = pure(is.colour);
    if (!colour.r && !colour.g && !colour.b) colour = is.colour;
    uint8_t hi = max(colour.r, max(colour.g, colour.b));
    if (hi) colour = CRGB(colour.r * 255 / hi, colour.g * 255 / hi, colour.b * 255 / hi);
    ripples.push_back({is.cx, is.cy, 0.0f, radius, RIPPLE_LEVEL * s, colour});
  }
}

// Moves a just-respawned pixel onto the oldest waiting seed, taking its place and colour.
void plantSeed(pixel& p) {
  while (!seeds.empty()) {
    Seed sd = seeds.front();
    seeds.erase(seeds.begin());
    if (colourAbsent(sd.spawnIndex)) continue;

    int bestX = -1, bestY = -1, bestD = 1000;
    for (int dx = -SEED_REACH; dx <= SEED_REACH; dx++) {
      for (int dy = -SEED_REACH; dy <= SEED_REACH; dy++) {
        int x = sd.x + dx, y = sd.y + dy;
        if (x < p.minX || x >= p.maxX || y < p.minY || y >= p.maxY || !cellFree(x, y, &p)) continue;
        int d = dx * dx + dy * dy;
        if (d < bestD || (d == bestD && random(2))) { bestD = d; bestX = x; bestY = y; }
      }
    }
    if (bestX < 0) continue;

    addAvailableIndex(p.xPos, p.yPos);
    removeAvailableIndex(bestX, bestY);
    p.xPos = bestX;
    p.yPos = bestY;
    p.color = sd.colour;
    p.spawnIndex = sd.spawnIndex;
    p.presence = p.presenceTarget();
    return;
  }
}

// Adds the expanding rings to the matrix; pixels drawn afterwards cover them.
void drawRipples() {
  for (size_t i = 0; i < ripples.size();) {
    Ripple& rp = ripples[i];
    float remaining = 1.0f - rp.r / rp.radius;
    if (remaining <= 0.0f) {
      ripples.erase(ripples.begin() + i);
      continue;
    }
    for (int x = 0; x < 32; x++) {
      for (int y = 0; y < 8; y++) {
        float w = 1.0f - fabsf(hypotf(x - rp.cx, y - rp.cy) - rp.r);
        if (w <= 0.0f) continue;
        float k = w * remaining * rp.level;
        LEDs[getIndex(x, y)] += CRGB(rp.colour.r * k, rp.colour.g * k, rp.colour.b * k);
      }
    }
    rp.r += RIPPLE_SPEED;
    i++;
  }
}

// Groups the lit pixels into islands, carries each island's identity over from the last step
// and runs the island effects for any that broke up without merging into another.
void trackIslands() {
  islandFrame++;
  for (size_t i = 0; i < seeds.size();) {
    if (--seeds[i].framesLeft <= 0) seeds.erase(seeds.begin() + i);
    else i++;
  }

  int grid[32][8];
  static std::vector<bool> member, visited;
  member.assign(pixels.size(), false);
  visited.assign(pixels.size(), false);
  for (int x = 0; x < 32; x++) for (int y = 0; y < 8; y++) grid[x][y] = -1;
  for (size_t i = 0; i < pixels.size(); i++) {
    pixel& p = pixels[i];
    member[i] = lit(p) && p.xPos >= 0 && p.xPos < 32 && p.yPos >= 0 && p.yPos < 8;
    if (member[i]) grid[p.xPos][p.yPos] = i;
    else p.islandId = 0;
  }

  // Connected groups of two or more, largest first; group g is members[spans[g].start ..+ spans[g].size].
  struct Span { int start, size; };
  static std::vector<int> members;
  static std::vector<Span> spans;
  members.clear();
  spans.clear();
  for (size_t i = 0; i < pixels.size(); i++) {
    if (visited[i] || !member[i]) continue;
    int start = members.size();
    members.push_back(i);
    visited[i] = true;
    for (size_t k = start; k < members.size(); k++) {
      const pixel& p = pixels[members[k]];
      const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (auto& st : steps) {
        int x = p.xPos + st[0], y = p.yPos + st[1];
        if (x < 0 || x >= 32 || y < 0 || y >= 8) continue;
        int j = grid[x][y];
        if (j >= 0 && !visited[j]) { visited[j] = true; members.push_back(j); }
      }
    }
    int size = members.size() - start;
    if (size >= 2) spans.push_back({start, size});
    else { pixels[i].islandId = 0; members.resize(start); }
  }
  std::sort(spans.begin(), spans.end(), [](const Span& a, const Span& b) { return a.size > b.size; });

  for (Island& is : islands) is.seen = false;
  static std::vector<int> present;   // last step's islands with a member in some group now
  present.clear();
  for (const Span& span : spans) {
    const int* group = &members[span.start];
    const int* groupEnd = group + span.size;
    // The group continues whichever unclaimed island most of its members came from.
    int bestId = 0, bestCount = 0;
    for (const int* m = group; m != groupEnd; m++) {
      int id = pixels[*m].islandId;
      if (id == 0) continue;
      if (std::find(present.begin(), present.end(), id) == present.end()) present.push_back(id);
      bool claimed = false;
      for (Island& is : islands) if (is.id == id && is.seen) claimed = true;
      if (claimed) continue;
      int count = 0;
      for (const int* o = group; o != groupEnd; o++) count += pixels[*o].islandId == id;
      if (count > bestCount || (count == bestCount && id < bestId)) { bestCount = count; bestId = id; }
    }
    Island* is = nullptr;
    for (Island& cand : islands) if (cand.id == bestId && bestId != 0) is = &cand;
    if (!is) {
      islands.push_back({nextIslandId++, islandFrame, 0, 0, 0, 0, CRGB(0, 0, 0), 0, false});
      is = &islands.back();
    }
    is->seen = true;

    float cx = 0, cy = 0, r = 0, g = 0, b = 0;
    int votes[PALETTE_SIZE] = {0};
    for (const int* m = group; m != groupEnd; m++) {
      const pixel& p = pixels[*m];
      cx += p.xPos; cy += p.yPos;
      r += p.color.r; g += p.color.g; b += p.color.b;
      votes[p.spawnIndex]++;
    }
    float n = span.size;
    is->size = span.size;
    is->peak = max(is->peak, is->size);
    is->cx = cx / n;
    is->cy = cy / n;
    is->colour = CRGB(r / n, g / n, b / n);
    is->spawnIndex = 0;
    for (int c = 1; c < PALETTE_SIZE; c++) if (votes[c] > votes[is->spawnIndex]) is->spawnIndex = c;
    for (const int* m = group; m != groupEnd; m++) pixels[*m].islandId = is->id;
  }

  for (size_t i = 0; i < islands.size();) {
    if (islands[i].seen) { i++; continue; }
    bool merged = std::find(present.begin(), present.end(), islands[i].id) != present.end();
    if (!merged) onIslandEnd(islands[i]);
    islands.erase(islands.begin() + i);
  }
  islandsNow = spans.size();
}

// Forgets every island, ring and seed, as after a rescatter.
void effectsClear() {
  islands.clear();
  ripples.clear();
  seeds.clear();
}
