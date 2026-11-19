#include <FastLED.h>
#include <ArxContainer.h>
#include "frameclock.h"
#include "network.h"
#include "effects.h"
#include "rules.h"

#define RGB_PIN         26     // LED data pin
#define RGB_LED_NUM     256    // 32x8 serpentine matrix
#define BRIGHTNESS      100    // boot brightness, 0-255; the day cycle sets it from the first frame
#define CHIP_SET        WS2812B
#define COLOR_CODE      GRB
#define ONBOARD_LED_PIN 2      // dev board's blue LED, held off
CRGBArray<RGB_LED_NUM> LEDs;

// Matrix index of column x, row y on the serpentine layout.
int getIndex(int x, int y) {
  int yDiff = 0;
  if (x > 31) x = 31;
  if (x < 0) x = 0;
  if (y > 7) y = 7;
  if (y < 0) y = 0;

  if ((x) % 2 == 0) {
    y = abs(y);
    yDiff = 0;
  } else {
    yDiff = 7;
    y = -y;
  }
  return 8 * x + y + yDiff;
}

static uint16_t trail[RGB_LED_NUM][3];   // the LEDs in 8.8 fixed point, so the per-frame fade keeps its fractions

// Fades the board by `perStep` (0-255, as fadeToBlackBy) per simulation step, spread evenly over the step's
// frames. On step frames the fractions are dropped, so a step's fade ends where fadeToBlackBy would.
void fadeBoard(float perStep, bool stepFrame) {
  uint32_t keep = (uint32_t)(powf(1.0f - perStep / 256.0f, 1.0f / FRAMES_PER_STEP) * 65536.0f);
  for (int i = 0; i < RGB_LED_NUM; i++) {
    for (int c = 0; c < 3; c++) {
      uint16_t t = trail[i][c];
      if ((t >> 8) != LEDs[i][c]) t = LEDs[i][c] << 8;
      t = ((uint32_t)t * keep) >> 16;
      if (stepFrame) t &= 0xFF00;
      trail[i][c] = t;
      LEDs[i][c] = t >> 8;
    }
  }
}


// Cells no pixel occupies, in the order they were freed.
struct Cell { int8_t x, y; };
std::vector<Cell> availableIndicies;

void initAvailableIndicies(){
  availableIndicies.clear();
  availableIndicies.reserve(512);
  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 8; y++) availableIndicies.push_back({(int8_t)x, (int8_t)y});
  }
}
// Removes the cell from the free list; the entry after each one removed is not checked.
void removeAvailableIndex(int x, int y){
  for (size_t i = 0; i < availableIndicies.size(); i++) {
    if (availableIndicies[i].x == x && availableIndicies[i].y == y) availableIndicies.erase(availableIndicies.begin() + i);
  }
}

void addAvailableIndex(int x, int y){
  availableIndicies.push_back({(int8_t)x, (int8_t)y});
}

// Claims a random free cell with minX <= x < maxX; false if there is none.
bool takeAvailableIndexIn(int minX, int maxX, int& x, int& y) {
  int count = 0;
  for (const Cell& c : availableIndicies) if (c.x >= minX && c.x < maxX) count++;
  if (count == 0) return false;
  int pick = random(count);
  for (auto it = availableIndicies.begin(); it != availableIndicies.end(); ++it) {
    if (it->x < minX || it->x >= maxX) continue;
    if (pick-- == 0) {
      x = it->x;
      y = it->y;
      availableIndicies.erase(it);
      return true;
    }
  }
  return false;
}


class pixel {
  public:
    pixel();
    int getX();
    int getY();
    CRGB getColor();
    void setColor(CRGB color);
    void move();
    void computeCycle(std::vector<pixel*> & neighbors);
    void writePixel(int rampFrames = 1);
    void randomize();
    bool age(int contacts);
    bool dark() const;
    void getNeighbors(std::vector<pixel*>& output);
    void splitColors(CRGB A, std::vector<pixel*>& neighbors);
    void averageColors(std::vector<pixel*> & neighbors);
    void pairColors(std::vector<pixel*>& neighbors);


    CRGB color = CRGB(0, 0, 0);
    int xPos = 0;
    int yPos = 0;
    float turnsLived = 0;
    int lifeTime = 100;
    int maxX = 32;
    int maxY = 8;
    int minY = 0;
    int minX = 0;
    bool shareTo = true;
    bool retired = false;
    int spawnIndex = 0;
    int framesSinceMove = 1000;
    float presence = 1.0f;       // intensity, 0-1, as the simulation sees it; changes once per step
    float shownPresence = 1.0f;  // intensity drawn, gliding to presence over the step's frames
    bool fadingOut = false;
    bool respawn = false;        // fading out at the end of its life, then respawning in place
    int role = 0;                // ROLE_CORE, ROLE_ARRIVING or ROLE_LEAVING
    int islandId = 0;            // island this pixel belongs to, 0 for none; see islandfx.h
    int pushDx = 0;              // direction and cells left of a scatter push
    int pushDy = 0;
    int pushSteps = 0;
    void expire();
    void retire();
    void updatePresence();
    void setPresence(float level);
    void glidePresence(int framesLeft);
    float presenceTarget();
};
std::vector<pixel> pixels;

#include "islandfx.h"

pixel::pixel() {
  this->xPos = 0;
  this->yPos = 0;
  this->color = CRGB(0, 0, 0);
}

void pixel::randomize() {

  if (!takeAvailableIndexIn(this->minX, this->maxX, this->xPos, this->yPos)) {
    bool positionSet=false;
    for(int x =random(32);x<32;x++){
      for(int y=random(8);y<8;y++){
        bool found=false;

        for(const pixel& px : pixels){
          if(px.xPos==x && px.yPos==y){
            found=true;
            break;
            }
          
          }
          if(!found){
            this->xPos=x;
            this->yPos=y;
            positionSet=true;
            break;
            }
        
        }
        if(positionSet) break;  
      }

  }



  this->lifeTime = ruleLifeTime();
  this->spawnIndex = pickSpawnIndex();
  this->color = paletteColors[this->spawnIndex];
  this->turnsLived = 0;
  this->shareTo = true;
  this->framesSinceMove = 0;
  this->fadingOut = false;
  this->respawn = false;
  this->islandId = 0;
  this->pushSteps = 0;
  this->setPresence(this->presenceTarget());


}
bool pixel::dark() const {
  return this->color[0] < 5 && this->color[1] < 5 && this->color[2] < 5;
}
// Returns true once the pixel has gone dark; otherwise ages it while it touches another pixel, at the
// rate the ageing effect sets. With expire=fade, a pixel whose life runs out fades away before respawning.
bool pixel::age(int contacts) {
  if (this->dark()) return true;

  if (contacts > 0) {
    this->turnsLived += ageStep(contacts);
    this->shareTo = !(this->turnsLived > this->lifeTime);
    if (!this->shareTo && ruleExpireFade && !this->fadingOut) {
      this->fadingOut = true;
      this->respawn = true;
    }
  }
  return false;
}
// Ends the pixel's life early: it fades out, then goes dark.
void pixel::expire() {
  this->turnsLived = this->lifeTime + 1;
  this->shareTo = false;
  this->fadingOut = true;
  this->respawn = false;
}
// Removes the pixel at once and frees its cell.
void pixel::retire() {
  this->retired = true;
  addAvailableIndex(this->xPos, this->yPos);
}
#define PRESENCE_FADE_FRAMES 40   // steps for a fallback fade in or out (~3 s)
#define ROLE_CORE     0           // present in both phases of the transition
#define ROLE_ARRIVING 1           // joins for the next phase, brightening across the transition
#define ROLE_LEAVING  2           // absent from the next phase, dimming across the transition

// Intensity the pixel should show now: the lower of its colour's and its role's transition level.
float pixel::presenceTarget() {
  float level = colourLevel(this->spawnIndex);
  if (this->role == ROLE_ARRIVING) level = min(level, activeSegT);
  else if (this->role == ROLE_LEAVING) level = min(level, 1.0f - activeSegT);
  return level;
}

// Moves the intensity one step toward its target, never faster than the fallback fade; a faded-out pixel goes dark.
void pixel::updatePresence() {
  float target = this->fadingOut ? 0.0f : this->presenceTarget();
  float step = 1.0f / (this->respawn ? DEATH_FADE_FRAMES : PRESENCE_FADE_FRAMES);
  if (this->presence < target) this->presence = min(this->presence + step, target);
  else this->presence = max(this->presence - step, target);
  if (this->fadingOut && this->presence <= 0.0f) this->color = CRGB(0, 0, 0);
}
// Sets the intensity at once, drawn as well as simulated.
void pixel::setPresence(float level) {
  this->presence = this->shownPresence = level;
}
// Moves the drawn intensity one frame toward presence, reaching it after `framesLeft` frames.
void pixel::glidePresence(int framesLeft) {
  this->shownPresence += (this->presence - this->shownPresence) / max(1, framesLeft);
}
// Fills output with the pixels orthogonally next to this one, retired ones included.
void pixel::getNeighbors(std::vector<pixel*>& output) {
  output.clear();
  for (pixel& px : pixels) {
    if (px.getX() < this->maxX && px.getY() < this->maxY && px.getX() >= this->minX && px.getY() >= this->minY) {
      if (((abs(px.getX() - this->xPos) == 1) && this->yPos == px.getY()) || ((abs(px.getY() - this->yPos) == 1) && this->xPos == px.getX())) {

        output.push_back(&px);

      }
    }
  }
}


int pixel::getX() {
  return this->xPos;
}

int pixel::getY() {
  return this->yPos;
}
void pixel::setColor(CRGB color) {
  this->color = color;
}
CRGB pixel::getColor() {
  return this->color;
}


// Sets this pixel to the pooled colour of itself and its neighbours, divided among those with shareTo set.
void pixel::splitColors(CRGB A, std::vector<pixel*>& neighbors) {
  float tempCalc[3] = {(float)A[0], (float)A[1], (float)A[2]};
  float shareToSize = this->shareTo ? 1.0f : 0.0f;

  // Neighbours count in proportion to how visible they are; fading ones have stopped sharing.
  for (pixel* neighbor : neighbors) {
    if (neighbor->fadingOut) continue;
    for (int i = 0; i < 3; i++) {
      tempCalc[i] += neighbor->color[i] * neighbor->presence;
    }
    if (neighbor->shareTo) shareToSize += neighbor->presence;
  }
  if (shareToSize <= 0.0f) return;
  shareToSize = max(shareToSize, 1.0f);
  if (!this->shareTo) {
    this->color = CRGB(0, 0, 0);
    return;
  }

  // Moves mixrate of the way to the pooled colour; rounding keeps colours bright but lets them pale.
  float rate = ruleMixRate / 100.0f;
  float bias = ruleRound ? 0.5f : 0.0f;
  for (int i = 0; i < 3; i++) {
    float pooled = tempCalc[i] / shareToSize;
    this->color[i] = (uint8_t)constrain(this->color[i] + (pooled - this->color[i]) * rate + bias, 0.0f, 255.0f);
  }
}

// Averages with each neighbour in turn and writes the result into both.
void pixel::pairColors(std::vector<pixel*>& neighbors) {
  float rate = ruleMixRate / 100.0f;
  float bias = ruleRound ? 0.5f : 0.0f;
  for (pixel* px : neighbors) {
    if (px->fadingOut) continue;
    for (int i = 0; i < 3; i++) {
      float average = (px->color[i] + this->color[i]) / 2.0f;
      px->color[i] = (uint8_t)constrain(px->color[i] + (average - px->color[i]) * rate + bias, 0.0f, 255.0f);
      this->color[i] = (uint8_t)constrain(this->color[i] + (average - this->color[i]) * rate + bias, 0.0f, 255.0f);
    }
  }
}

// Mixes colours with the neighbours using the mix rule.
void pixel::averageColors(std::vector<pixel*> & neighbors) {
  if (ruleMix == MIX_PAIR) pairColors(neighbors);
  else splitColors(this->color, neighbors);
}

void pixel::move() {
    int oldX = this->xPos;
    int oldY = this->yPos;
    int minDistance = 256;
    int tX = this->xPos;
    int tY = this->yPos;

    // Add available Index
    addAvailableIndex(tX,tY);

    // Determine closest pixel and set a target for a spot around that pixel
    for (pixel& px : pixels) {
      if (px.getX() < this->maxX && px.getY() < this->maxY && px.getX() >= this->minX && px.getY() >= this->minY) {
        if (px.getX() == this->xPos && px.getY() != this->yPos) {
          int calcDist = abs(px.getY() - this->yPos);
          if (calcDist < minDistance) {
            minDistance = calcDist;
            tX = this->xPos;
            tY = px.getY();
          }

        }

        else if (px.getY() == this->yPos && px.getX() != this->xPos) {
          int calcDist = abs(px.getX() - this->xPos);
          if (calcDist < minDistance) {
            minDistance = calcDist;
            tX = px.getX();
            tY = this->yPos;
          }

        }
      }
    }

    // If target pixel is found, move towards it
    if (minDistance != 256) {
      if (tX < this->xPos) {
        this->xPos = (this->xPos) - 1;
      }
      else if (tX > this->xPos) {
        this->xPos = (this->xPos) + 1;
      }
      else if (tY < this->yPos) {
        this->yPos = (this->yPos) - 1;
      }
      else if (tY > this->yPos) {
        this->yPos = (this->yPos) + 1;
      }
    // If target pixel is not found, move a random step in any direction, bouncing off the edges
    } else {
      switch (random(4)) {
        case (0):
          this->xPos += (this->xPos + 1 < this->maxX) ? 1 : -1;
          break;
        case (1):
          this->xPos += (this->xPos - 1 >= this->minX) ? -1 : 1;
          break;
        case (2):
          this->yPos += (this->yPos + 1 < this->maxY) ? 1 : -1;
          break;
        default:
          this->yPos += (this->yPos - 1 >= this->minY) ? -1 : 1;
          break;
      }
    }


    removeAvailableIndex(this->xPos, this->yPos);
    if (this->xPos != oldX || this->yPos != oldY) this->framesSinceMove = 0;
}
// Averages colours with any neighbours; an expired pixel then goes dark so it respawns, unless it is fading out.
void pixel::computeCycle(std::vector<pixel*> & neighbors) {
  if (neighbors.size() != 0 && !this->fadingOut) {
    this->averageColors(neighbors);
  }
  if (!this->shareTo && !this->fadingOut) {
    this->color = CRGB(0, 0, 0);
  }
}
// Writes the pixel's colour at its current intensity, fading it in over rampFrames frames after a move or spawn.
void pixel::writePixel(int rampFrames) {
  CRGB& led = LEDs[getIndex(this->xPos, this->yPos)];
  CRGB shown = this->color;
  for (int i = 0; i < 3; i++) shown[i] = (uint8_t)(this->color[i] * this->shownPresence);
  if (rampFrames <= 1 || this->framesSinceMove + 1 >= rampFrames) {
    led = shown;
  } else {
    led = blend(led, shown, (uint8_t)(255 * (this->framesSinceMove + 1) / rampFrames));
  }
  if (this->framesSinceMove < 1000) this->framesSinceMove++;
}

// Rescatters every pixel into a fresh layout.
void rescatterPixels() {
  shuffleRequested = false;
  initAvailableIndicies();
  FastLED.clear();
  effectsClear();
  for (size_t i = 0; i < pixels.size(); i++) pixels[i].randomize();
}

void setup() {
  Serial.begin(9600);
  settingsLoad();
  FastLED.addLeds<CHIP_SET, RGB_PIN, COLOR_CODE>(LEDs, RGB_LED_NUM);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setDither(0);
  FastLED.clear();
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);
  networkBegin();
#ifndef ESP32
  // ESP32's random() uses the hardware RNG unless randomSeed() is called.
  randomSeed(analogRead(0));
#endif
  initAvailableIndicies();
  effectsClear();
  for (int a = 0; a < 30; a++) {
    pixel newPixel = pixel();
    newPixel.randomize();
    pixels.push_back(newPixel);
  }
  FastLED.show();
}

int frameInStep = 0;                            // 0 on the frames that run a simulation step
float moveClock = FRAMES_PER_STEP - 1;          // frames counted toward the next move tick
float mixClock = FRAMES_PER_STEP - 1;           // and toward the next mix tick
uint32_t moveTicks = 0, mixTicks = 0;

// Counts one frame on a tick clock; true when `interval` frames have passed. At most one tick per frame.
bool tickClock(float& clock, float interval) {
  clock += 1.0f;
  if (clock < interval - 0.001f) return false;
  clock = min(clock - interval, interval);
  return true;
}
static int lastSegment = -2;

// Matches the pixels to the current transition: the extra pixels of the bigger phase arrive or leave
// together across it, and pixels of a colour neither phase uses are recycled.
void reconcilePixels() {
  int from = max(1, (int)lroundf(activeSegFrom->pixels * adjustPixels / 100.0f));
  int to = max(1, (int)lroundf(activeSegTo->pixels * adjustPixels / 100.0f));
  int wantCore = min(from, to);
  int wantArriving = to > from ? to - from : 0;
  int wantLeaving = from > to ? from - to : 0;

  if (activeSegIndex != lastSegment) {
    for (pixel& px : pixels) {
      if (px.retired) continue;
      if (px.role == ROLE_ARRIVING) px.role = ROLE_CORE;
      else if (px.role == ROLE_LEAVING) px.retire();
    }
    lastSegment = activeSegIndex;
  }

  int core = 0, arriving = 0, leaving = 0;
  for (pixel& px : pixels) {
    if (px.retired || (px.fadingOut && !px.respawn)) continue;
    if (colourAbsent(px.spawnIndex)) { px.expire(); continue; }
    if (px.role == ROLE_CORE) core++;
    else if (px.role == ROLE_ARRIVING) arriving++;
    else leaving++;
  }

  // Surplus pixels take a transition role where one is open, otherwise fade out.
  for (pixel& px : pixels) {
    if (px.retired || (px.fadingOut && !px.respawn)) continue;
    if (px.role == ROLE_CORE && core > wantCore) {
      core--;
      if (leaving < wantLeaving) { px.role = ROLE_LEAVING; leaving++; }
      else if (arriving < wantArriving) { px.role = ROLE_ARRIVING; arriving++; }
      else px.expire();
    } else if (px.role == ROLE_ARRIVING && arriving > wantArriving) {
      arriving--;
      px.expire();
    } else if (px.role == ROLE_LEAVING && leaving > wantLeaving) {
      leaving--;
      px.expire();
    }
  }

  // Missing pixels join; arrivals and leavers start at the transition's current intensity.
  bool anyColour = false;
  for (int c = 0; c < PALETTE_SIZE; c++) anyColour |= !colourAbsent(c);
  while (anyColour && (core < wantCore || arriving < wantArriving || leaving < wantLeaving) && !availableIndicies.empty()) {
    pixel newPixel = pixel();
    if (core < wantCore) { newPixel.role = ROLE_CORE; core++; }
    else if (arriving < wantArriving) { newPixel.role = ROLE_ARRIVING; arriving++; }
    else { newPixel.role = ROLE_LEAVING; leaving++; }
    newPixel.randomize();
    if (newPixel.role == ROLE_CORE) newPixel.setPresence(0.0f);
    pixels.push_back(newPixel);
  }
}

void loop() {
  frameBegin();
  networkLoop();
  const Look& look = dayCycleUpdate();
  overridesUpdate();
  if (shuffleRequested) rescatterPixels();
  bool step = frameInStep == 0;
  float moveInterval = max(1.0f, look.moveEvery * FRAMES_PER_STEP);
  float mixInterval = max(1.0f, look.colorEvery * FRAMES_PER_STEP);
  bool moveNow = !paused && tickClock(moveClock, moveInterval);
  bool mixNow = !paused && tickClock(mixClock, mixInterval);
  if (moveNow) moveTicks++;
  if (mixNow) mixTicks++;
  int ramp = max(1, (int)lroundf(moveInterval) - FRAMES_PER_STEP + 1);
  fadeBoard(look.fade, step);
  FastLED.setBrightness(shownBrightness());
  if (step && !paused) drawRipples();

  static std::vector<pixel*> neighbors;
  for (size_t i = 0; i < pixels.size(); i++) {
    if (pixels[i].retired) continue;

    // Paused pixels hold still and keep their colour and age.
    if (paused) {
      if (step) pixels[i].updatePresence();
      pixels[i].glidePresence(FRAMES_PER_STEP - frameInStep);
      pixels[i].writePixel(ramp);
      continue;
    }

    if (step || moveNow || mixNow) pixels[i].getNeighbors(neighbors);

    // A dark pixel respawns in the same role, unless it was fading out for good.
    if (step && pixels[i].age((int)neighbors.size())) {
      onPixelDeath(pixels[i], neighbors);
      if (pixels[i].fadingOut && !pixels[i].respawn) {
        pixels[i].retire();
      } else {
        addAvailableIndex(pixels[i].getX(), pixels[i].getY());
        pixels[i].randomize();
        plantSeed(pixels[i]);
        pixels[i].writePixel(ramp);
      }
      continue;
    }
    if (moveNow) {
      if (pixels[i].pushSteps > 0) pushStep(pixels[i]);
      else if (neighbors.empty()) pixels[i].move();
    }
    if (mixNow) {
      pixels[i].computeCycle(neighbors);
    }
    if (step) pixels[i].updatePresence();
    pixels[i].glidePresence(FRAMES_PER_STEP - frameInStep);

    // A pixel that just went dark is left to fade with the trails rather than blanked.
    const CRGB& c = pixels[i].color;
    if (c.r >= 5 || c.g >= 5 || c.b >= 5) pixels[i].writePixel(ramp);
  }
  if (step) reconcilePixels();
  pixels.erase(std::remove_if(pixels.begin(), pixels.end(), [](const pixel& px) { return px.retired; }), pixels.end());
  if (step && !paused) trackIslands();
  dayCycleReportPixels(pixels.size());
  frameInStep = (frameInStep + 1) % FRAMES_PER_STEP;
  FastLED.show();
  frameEnd();
}
