#include <FastLED.h>
#include <ArxContainer.h>
#include "network.h"
#include "islands.h"
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

void fadeBoard(int amount) {

  fadeToBlackBy(LEDs, RGB_LED_NUM, amount);


}


//arx::vector<arx::vector<int>> queue;
std::vector<std::vector<int>> availableIndicies;

void initAvailableIndicies(){
  availableIndicies.clear();
  for (int x = 0; x < 32;x++){
    for (int y=0;y<8;y++){
      std::vector<int> newIndex;
      newIndex.push_back(x);
      newIndex.push_back(y);
      availableIndicies.push_back(newIndex);
    }
  }
}
void removeAvailableIndex(int x, int y){
    for (std::vector<std::vector<int>>::iterator it = availableIndicies.begin();it<availableIndicies.end();it++){
      std::vector<int> index = *it;
        if(x==index[0] && y == index[1]){
          availableIndicies.erase(it);
        }
      }


}

void addAvailableIndex(int x, int y){
    std::vector<int> newIndex;
    newIndex.push_back(x);
    newIndex.push_back(y);
    availableIndicies.push_back(newIndex);
}

// Claims a random free cell with minX <= x < maxX; false if there is none.
bool takeAvailableIndexIn(int minX, int maxX, int& x, int& y) {
  int count = 0;
  for (auto& index : availableIndicies) if (index[0] >= minX && index[0] < maxX) count++;
  if (count == 0) return false;
  int pick = random(count);
  for (auto it = availableIndicies.begin(); it != availableIndicies.end(); ++it) {
    if ((*it)[0] < minX || (*it)[0] >= maxX) continue;
    if (pick-- == 0) {
      x = (*it)[0];
      y = (*it)[1];
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
    bool dark();
    void assignVariant(int v);
    void rewardOrphans(std::vector<pixel*>& neighbors);
    std::vector<pixel*> getNeighbors();
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
    float presence = 1.0f;       // display intensity, 0-1
    bool fadingOut = false;
    bool respawn = false;        // fading out at the end of its life, then respawning in place
    int role = 0;                // ROLE_CORE, ROLE_ARRIVING or ROLE_LEAVING
    int variant = VARIANT_TODAY; // island-hold behaviour, see islands.h
    void expire();
    void retire();
    void updatePresence();
    float presenceTarget();
};
std::vector<pixel> pixels;

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

        for(pixel px : pixels){
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
  this->presence = this->presenceTarget();


}
bool pixel::dark() {
  return this->color[0] < 5 && this->color[1] < 5 && this->color[2] < 5;
}
// Returns true once the pixel has gone dark; otherwise ages it while it touches another pixel.
// EDGE pixels age slower by day, and slower still with more neighbours, so islands wear from the outside.
// With expire=fade, a pixel whose life runs out fades away before respawning.
bool pixel::age(int contacts) {
  if (this->dark()) return true;

  if (contacts > 0) {
    float step = 1.0f;
    if (this->variant == VARIANT_EDGE) {
      float hold = activeLook.islandHold;
      step = 1.0f / ((1.0f + islandBoost * hold) * (1.0f + (contacts - 1) * hold));
    }
    this->turnsLived += step;
    this->shareTo = !(this->turnsLived > this->lifeTime);
    if (!this->shareTo && ruleExpireFade && !this->fadingOut) {
      this->fadingOut = true;
      this->respawn = true;
    }
  }
  return false;
}
// Sets the behaviour and, in split mode, confines the pixel to its half (EDGE left, ORPHAN right).
void pixel::assignVariant(int v) {
  this->variant = v;
  this->minX = 0;
  this->maxX = 32;
  if (islandMode == ISLANDS_SPLIT) {
    if (v == VARIANT_EDGE) this->maxX = 16;
    else this->minX = 16;
  }
}
// Called as this pixel dies: ORPHAN neighbours it leaves alone get a longer life by day, so their colour carries on.
void pixel::rewardOrphans(std::vector<pixel*>& neighbors) {
  float hold = activeLook.islandHold;
  if (hold <= 0.0f) return;
  for (pixel* nb : neighbors) {
    if (nb->variant != VARIANT_ORPHAN || nb->retired || nb->fadingOut || nb->dark()) continue;
    int others = 0;
    for (pixel* other : nb->getNeighbors()) {
      if (other != this && !other->dark()) others++;
    }
    if (others > 0) continue;
    int bonusLife = (int)lroundf(ruleLifeTime() * (1.0f + islandBoost * hold));
    nb->turnsLived *= 1.0f - hold;
    nb->lifeTime = max(nb->lifeTime, (int)lroundf(nb->lifeTime + hold * (bonusLife - nb->lifeTime)));
    nb->shareTo = nb->turnsLived <= nb->lifeTime;
  }
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
#define PRESENCE_FADE_FRAMES 40   // frames for a fallback fade in or out (~3 s)
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

// Moves the display intensity toward its target, never faster than the fallback fade; a faded-out pixel goes dark.
void pixel::updatePresence() {
  float target = this->fadingOut ? 0.0f : this->presenceTarget();
  float step = 1.0f / (this->respawn ? DEATH_FADE_FRAMES : PRESENCE_FADE_FRAMES);
  if (this->presence < target) this->presence = min(this->presence + step, target);
  else this->presence = max(this->presence - step, target);
  if (this->fadingOut && this->presence <= 0.0f) this->color = CRGB(0, 0, 0);
}
std::vector<pixel*> pixel::getNeighbors() {
  std::vector<pixel*> output;
  for (pixel& px : pixels) {
    if (px.getX() < this->maxX && px.getY() < this->maxY && px.getX() >= this->minX && px.getY() >= this->minY) {
      if (((abs(px.getX() - this->xPos) == 1) && this->yPos == px.getY()) || ((abs(px.getY() - this->yPos) == 1) && this->xPos == px.getX())) {

        output.push_back(&px);

      }
    }


  }
  return output;

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
// Writes the pixel's colour at its current intensity, fading it in over rampFrames after a move or spawn.
void pixel::writePixel(int rampFrames) {
  CRGB& led = LEDs[getIndex(this->xPos, this->yPos)];
  CRGB shown = this->color;
  for (int i = 0; i < 3; i++) shown[i] = (uint8_t)(this->color[i] * this->presence);
  if (rampFrames <= 1 || this->framesSinceMove + 1 >= rampFrames) {
    led = shown;
  } else {
    led = blend(led, shown, (uint8_t)(255 * (this->framesSinceMove + 1) / rampFrames));
  }
  if (this->framesSinceMove < 1000) this->framesSinceMove++;
}

// The behaviour a new pixel gets: in split mode, whichever half has fewer pixels.
int nextIslandVariant() {
  if (islandMode == ISLANDS_OFF) return VARIANT_TODAY;
  if (islandMode == ISLANDS_EDGE) return VARIANT_EDGE;
  if (islandMode == ISLANDS_ORPHAN) return VARIANT_ORPHAN;
  int edge = 0, orphan = 0;
  for (pixel& px : pixels) {
    if (px.retired || px.fadingOut) continue;
    if (px.variant == VARIANT_EDGE) edge++;
    else if (px.variant == VARIANT_ORPHAN) orphan++;
  }
  return edge <= orphan ? VARIANT_EDGE : VARIANT_ORPHAN;
}

// Rescatters every pixel into a fresh layout for the current island mode.
void rescatterPixels() {
  islandModeDirty = false;
  shuffleRequested = false;
  initAvailableIndicies();
  FastLED.clear();
  for (size_t i = 0; i < pixels.size(); i++) {
    int v = VARIANT_TODAY;
    if (islandMode == ISLANDS_EDGE) v = VARIANT_EDGE;
    else if (islandMode == ISLANDS_ORPHAN) v = VARIANT_ORPHAN;
    else if (islandMode == ISLANDS_SPLIT) v = i % 2 ? VARIANT_ORPHAN : VARIANT_EDGE;
    pixels[i].assignVariant(v);
    pixels[i].randomize();
  }
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
  for (int a = 0; a < 30; a++) {
    pixel newPixel = pixel();
    newPixel.assignVariant(nextIslandVariant());
    newPixel.randomize();
    pixels.push_back(newPixel);
  }
  FastLED.show();
}

int frameCount = 1;
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
    newPixel.assignVariant(nextIslandVariant());
    newPixel.randomize();
    if (newPixel.role == ROLE_CORE) newPixel.presence = 0.0f;
    pixels.push_back(newPixel);
  }
}

void loop() {
  networkLoop();
  const Look& look = dayCycleUpdate();
  overridesUpdate();
  if (islandModeDirty || shuffleRequested) rescatterPixels();
  int moveSpeed = max(1, (int)lroundf(look.moveEvery));
  int colorSpeed = max(1, (int)lroundf(look.colorEvery));
  fadeBoard((int)lroundf(look.fade));
  FastLED.setBrightness(shownBrightness());

  for (size_t i = 0; i < pixels.size(); i++) {
    if (pixels[i].retired) continue;
    std::vector<pixel*> neighbors = pixels[i].getNeighbors();

    // Paused pixels hold still and keep their colour and age.
    if (paused) {
      pixels[i].updatePresence();
      pixels[i].writePixel(moveSpeed);
      continue;
    }

    // A dark pixel respawns in the same role, unless it was fading out for good.
    if (pixels[i].age((int)neighbors.size())) {
      pixels[i].rewardOrphans(neighbors);
      if (pixels[i].fadingOut && !pixels[i].respawn) {
        pixels[i].retire();
      } else {
        addAvailableIndex(pixels[i].getX(), pixels[i].getY());
        pixels[i].randomize();
        pixels[i].writePixel(moveSpeed);
      }
      continue;
    }
    if (neighbors.empty() && frameCount % moveSpeed == 0) {
      pixels[i].move();
    }
    if (frameCount % colorSpeed == 0) {
      pixels[i].computeCycle(neighbors);
    }
    pixels[i].updatePresence();

    // A pixel that just went dark is left to fade with the trails rather than blanked.
    const CRGB& c = pixels[i].color;
    if (c.r >= 5 || c.g >= 5 || c.b >= 5) pixels[i].writePixel(moveSpeed);
  }
  reconcilePixels();
  pixels.erase(std::remove_if(pixels.begin(), pixels.end(), [](const pixel& px) { return px.retired; }), pixels.end());
  dayCycleReportPixels(pixels.size());
  frameCount++;
  FastLED.show();
  delay(75);
}
