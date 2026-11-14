#include <FastLED.h>
#include <RotaryEncoder.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArxContainer.h>
#include <stdio.h>
#include <stdlib.h>
#include <numeric>
// #include "alphabet.h"
#include "network.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1 2
#define PIN_IN2 3

#elif defined(ESP8266)
// Example for ESP8266 NodeMCU with input signals on pin D5 and D6
#define PIN_IN1 D5
#define PIN_IN2 D6

#endif
#define MIC_PIN A0



// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:

#define RGB_PIN           26              // LED DATA PIN
#define RGB_LED_NUM    256           // 10 LEDs [0...9]
#define BRIGHTNESS       100           // brightness range [0..255]
#define CHIP_SET       WS2812B      // types of RGB LEDs
#define COLOR_CODE    GRB          //sequence of colors in data stream
#define XMAX 31;
#define YMAX 7;
CRGBArray<RGB_LED_NUM> LEDs;

#define ROTARY_SW 27
#define ROTARY_CK 15
#define ONBOARD_LED_PIN 2   // dev board's blue LED, held off
#define ROTARY_DT 4


//byte c//unt[32][8] = {{'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '1', '1', '1', '1', '0', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '0', '0', '1', '1', '0'}, {'0', '1', '1', '0', '0', '1', '1', '0'}, {'0', '1', '1', '0', '0', '1', '1', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '1', '1', '1', '1', '1', '0', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '0', '0', '0', '0', '1', '1', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '1', '1', '1', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '0', '1', '1', '1', '0', '0', '0'}, {'0', '0', '0', '1', '1', '1', '0', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '1', '1', '0', '0', '0', '0', '0'}, {'0', '1', '1', '0', '0', '0', '0', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '1', '1', '1', '1', '0'}, {'0', '1', '1', '0', '0', '0', '0', '0'}, {'0', '1', '1', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}, {'0', '0', '0', '0', '0', '0', '0', '0'}};// define 3 byte for the random color
byte  a, b, c;
#define UPDATES_PER_SECOND 100

char iByte = 0;

//arx::vector<int>


//int dist[256]={-1};

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
  int res = 8 * x + y + yDiff;

  return res;


}
std::vector<int> getXY(int sequence) {
  std::vector<int> xy = {0, 0};
  int yDiff = 0;
  int y = sequence % 8;

  if ((sequence / 8) % 2 == 0) {
    y = abs(y);
    yDiff = 0;
  } else {
    yDiff = 7;
    y = -y;
  }
  xy[0] = (sequence - y - yDiff) / 8;
  xy[1] = y;
  return xy;
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

std::vector<int> getRandomAvailableIndex(){
    std::vector<int> newRandomIndex;
    int randIndex = random(availableIndicies.size());
    newRandomIndex.push_back(availableIndicies[randIndex][0]);
    newRandomIndex.push_back(availableIndicies[randIndex][1]);
    std::vector<std::vector<int>>::iterator it = availableIndicies.begin() + randIndex;
    availableIndicies.erase(it);

    return newRandomIndex;
}

class pixel {
  public:
    pixel(int x, int y, CRGB color);
    pixel();
    int getX();
    int getY();
    CRGB getColor();
    void setX(int x);
    void setY(int y);
    void setColor(CRGB color);
    void move();
    void computeCycle(std::vector<pixel*> & neighbors);
    void writePixel(int rampFrames = 1);
    void randomize();
    bool age(bool inContact);
    std::vector<pixel*> getNeighbors();
    std::vector<pixel> getNeighbors(std::vector<pixel> prevNeighbors);
    CRGBArray<2> splitColors(CRGB A, CRGB B, int denom, int numer);
    bool operator==( pixel& A);
    bool operator!=( pixel& A);
    void splitColors(CRGB A, std::vector<pixel*>& neighbors);
    void averageColors(std::vector<pixel*> & neighbors);


    CRGB color = (0, 0, 0);
    //    std::vector<pixel> neighbors;
    int xPos = 0;
    int yPos = 0;
    int turnsLived = 0;
    int lifeTime = 100;
    int maxX = 32;
    int maxY = 8;
    int minY = 0;
    int minX = 0;
    bool half = false;
    bool shareTo = true;
    bool retired = false;
    int spawnIndex = 0;
    int framesSinceMove = 1000;
    float presence = 1.0f;       // display intensity, 0-1
    bool fadingOut = false;
    int role = 0;                // ROLE_CORE, ROLE_ARRIVING or ROLE_LEAVING
    void expire();
    void retire();
    void updatePresence();
    float presenceTarget();
};
std::vector<pixel> pixels;
//std::vector<pixel> processed;
bool pixel::operator==( pixel& A) {
  if (A.getX() == xPos && A.getY() == yPos) return true;
  return false;

}
bool pixel::operator!=(pixel& A) {
  if (A.getX() == xPos && A.getY() == yPos) return false;
  return true;

}

pixel::pixel() {
  this->xPos = 0;
  this->yPos = 0;
  this->color = CRGB(0, 0, 0);

}
pixel::pixel(int x, int y, CRGB color) {
  this->xPos = x;
  this->yPos = y;
  this->color = color;

}

void pixel::randomize() {

  if (availableIndicies.size() != 0) {



    //  int rand2 = random(2);


    std::vector<int> newRandomIndex = getRandomAvailableIndex();

    xPos = newRandomIndex[0];
    yPos = newRandomIndex[1];

  } else {
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



  this->lifeTime = random(30, 100);
  this->spawnIndex = pickSpawnIndex();
  this->color = paletteColors[this->spawnIndex];
  this->turnsLived = 0;
  this->shareTo = true;
  this->framesSinceMove = 0;
  this->fadingOut = false;
  this->presence = this->presenceTarget();


}
// Returns true once the pixel has gone dark; otherwise ages it while it touches another pixel.
bool pixel::age(bool inContact) {
  if (this->color[0] < 5 && this->color[1] < 5 && this->color[2] < 5) {
    return true;
  }

  if (inContact) {
    this->turnsLived += 1;
    this->shareTo = !(this->turnsLived > this->lifeTime);
  }
  return false;
}
// Ends the pixel's life early: it fades out, then goes dark.
void pixel::expire() {
  this->turnsLived = this->lifeTime + 1;
  this->shareTo = false;
  this->fadingOut = true;
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
  float step = 1.0f / PRESENCE_FADE_FRAMES;
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
void pixel::setX(int x) {
  this->xPos = x;

}
void pixel::setY(int y) {
  this->yPos = y;
}
void pixel::setColor(CRGB color) {
  this->color = color;
}
CRGB pixel::getColor() {
  return this->color;
}

CRGBArray<2> pixel::splitColors(CRGB A, CRGB B, int denom, int numer) {
  CRGBArray<2> pixel = CRGBArray<2>();
  pixel[0] = CRGB(0, 0, 0);
  pixel[1] = CRGB(0, 0, 0);
  for (int i = 0; i < 3; i++) {
    pixel[0][i] = ((A[i] + B[i]) * denom / numer);
    pixel[1][i] = ((A[i] + B[i]) * (numer - denom) / numer);

  }
  return pixel;
}

//CRGB pixel::splitColors(CRGB A, std::vector<pixel>& neighbors, int denom, int numer) {
//  CRGBArray<2> pixel = CRGBArray<2>();
//  pixel[0] = CRGB(0, 0, 0);
//  pixel[1] = CRGB(0, 0, 0);
//  int tempCalc[3] = {0, 0, 0};
//  CRGB output(0, 0, 0);
//  for (pixel neighbor : neighbors) {
//    for (int i = 0; i < 3; i++) {
//      tempCalc[i] += neighbor.color[i];
//    }
//  }
//  for (int i = 0; i < 3; i++) {
//    tempCalc[i] += A[i];
//    pixel[0][i] = tempCalc[i] *(denum /numer);
//    pixel[1][i] = tempCalc[i] * (numer - denom) / numer);
//  }
//  for (pixel neighbor : neighbors) {
//    neighbor.color = pixel[1];
//  }
////  return outpuSt;
//  return pixel[0];
//}

// Sets this pixel to the pooled colour of itself and its neighbours, divided among those with shareTo set.
void pixel::splitColors(CRGB A, std::vector<pixel*>& neighbors) {
  float tempCalc[3] = {(float)A[0], (float)A[1], (float)A[2]};
  float shareToSize = this->shareTo ? 1.0f : 0.0f;

  // Neighbours count in proportion to how visible they are.
  for (pixel* neighbor : neighbors) {
    for (int i = 0; i < 3; i++) {
      tempCalc[i] += neighbor->color[i] * neighbor->presence;
    }
    if (neighbor->shareTo) shareToSize += neighbor->presence;
  }
  if (shareToSize <= 0.0f) return;
  shareToSize = max(shareToSize, 1.0f);

  CRGB output(0, 0, 0);
  for (int i = 0; i < 3; i++) {
    output[i] = (uint8_t)min(tempCalc[i] / shareToSize, 255.0f);
  }
  this->color = this->shareTo ? output : CRGB(0, 0, 0);
}

//void pixel::splitColors(CRGB A, std::vector<pixel>& neighbors) {
////  CRGBArray<2> colorSplit = splitColors(px.getColor(), this->color, 1, neighbors.size());
//
//  int tempCalc[3] = {0, 0, 0};
//  CRGB output(0, 0, 0);
//
//  for (pixel neighbor : neighbors) {
//    for (int i = 0; i < 3; i++) {
//      tempCalc[i] += neighbor.color[i];
//    }
//
//
//  }
//
//  for (int i = 0; i < 3; i++) {
//    tempCalc[i] += A[i];
//
//    output[i] = tempCalc[i] / (((int) neighbors.size())+1);
//
//
//  }
//  for (pixel neighbor : neighbors) {
//    neighbor.color = output;
//  }
//  this->color = output;
//
//
//}

// If half is set to false for the pixel, simply take the average of the pixel and each neighbor and assign it to both iteratively
// Otherwise take the average of all neighboring pixels and assign all at once
void pixel::averageColors(std::vector<pixel*> & neighbors) {
  //  if (neighbors.size()==0) return;
  if (this->half) {
    splitColors(this->color, neighbors);
    //    this->color =output;
  } else {
    //      Best bang for buck funciton, nice visuals, good diffusion, low effort
    for (pixel* px : neighbors) {

      CRGBArray<2> colorSplit = splitColors(px->getColor(), this->color, 1, 2);

      CRGB average = colorSplit[0];
      px->setColor(average);
      this->color = average;

    }
  }
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

    //     delay(10);
    // If target pixel is found, move towards it
    if (minDistance != 256) {
      //     LEDs[getIndex(this->xPos,this->yPos)] = CRGB(0,0,0);
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
  if (neighbors.size() != 0) {
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

RotaryEncoder *encoder = nullptr;

boolean encoderPress(){
  while(digitalRead(ROTARY_SW)==LOW){
    delay(100);
    if(digitalRead(ROTARY_SW) == HIGH){
      return true;
      }
    }
    return false;
  }
void readEncoder(){
  encoder->tick();

//  rotPs = encoder.getPosition();

  }
  
int moveSpeed = 1;
int colorSpeed= 1;
int ageValue  = 30;
int brightness= 100;
void changePixelCount() {
  Serial.println("changePixelCount");

    int pixelSize=pixels.size();
    
    while (true){
      readEncoder();
      int direction = (int) encoder->getDirection();
      pixelSize += direction;
      if (direction == -1 && pixelSize > 1) {
        addAvailableIndex(pixels[pixels.size()-1].getX(),pixels[pixels.size()-1].getY());
        pixels[pixels.size()-1].color = CRGB (0,0,0);
        pixels[pixels.size()-1].writePixel();
        pixels.pop_back();
        
        }
      else if (direction == 1 && pixelSize < 256){
          pixel newPixel=pixel();

          newPixel.half = true;
          newPixel.randomize();

      //  newPixel.getNeighbors();
          pixels.push_back(newPixel);
          newPixel.writePixel();
        }
      FastLED.show();
      Serial.println(pixelSize);
      if (pixelSize < 1) pixelSize=1;
      if (pixelSize > 256) pixelSize=256;
      if (encoderPress()) break;
      }
//     for (int a = 0; a < pixelSize; a++) {
//      pixel newPixel=pixel();
//
//      newPixel.half = true;
//      newPixel.randomize();
//
//      //  newPixel.getNeighbors();
//      pixels.push_back(newPixel);
//      
//      }
     
    
  }

void changeMoveSpeed(){
  Serial.println("changeMoveSpeed");
  while (true){
    readEncoder();
    int direction = (int) encoder->getDirection();
    moveSpeed+=direction;
    if (moveSpeed < 1) moveSpeed = 1;
    Serial.println(moveSpeed);
    if (encoderPress()) break;  
    }
  }
void changeColorSpeed(){
  Serial.println("changeColorSpeed");
  while (true){
    readEncoder();
    int direction = (int) encoder->getDirection();
    colorSpeed+=direction;
    if (colorSpeed < 1) colorSpeed = 1;
    Serial.println(colorSpeed);
    if (encoderPress()) break;  
    }
  }
void changeBrightness(){
  Serial.println("changeBrightness");
  while (true){
    readEncoder();
    int direction = (int) encoder->getDirection();
    brightness+=direction;
    if (brightness < 1) brightness = 1;
    if (brightness > 255) brightness = 255;
    Serial.println(brightness);
    if (encoderPress()) break;  
    }
  }
  
void handleModes(){
  int setting=0;
  FastLED.clear();
  while (true){
    readEncoder();
    int direction = (int) encoder->getDirection();
    setting += direction;
    
    if (setting < 0) setting=0;
    if (setting > 4) setting=4;
    Serial.println(setting);
    for (int px=-1; px < 4; px++){
      if (px < setting){
        LEDs[px] = CRGB(255,0,0);
        }else{
        LEDs[px] = CRGB(0,0,0);  
          }
      }
      FastLED.show();
    if (encoderPress()) break;
    }
  if (setting == 0){
    changePixelCount();
    }
  else if (setting == 1){
    changeMoveSpeed();
    }
  else if (setting == 2){
    changeColorSpeed();
    }
  else if (setting == 3){
    changeBrightness();
    }
  
  
  }
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}



void setup() {

  Serial.begin(9600);
  Serial.println("WS2812B LEDs strip Initialize");
  //  Serial.println("Please enter the 1 to 6 value.....Otherwise no any effect show");
  FastLED.addLeds<CHIP_SET, RGB_PIN, COLOR_CODE>(LEDs, RGB_LED_NUM);
  //  randomSeed(analogRead(0));
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setDither(0);
  FastLED.clear();
//  FastLED.show();/
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    pinMode(ROTARY_SW,INPUT_PULLUP);
    networkBegin();
    encoder = new RotaryEncoder(ROTARY_CK, ROTARY_DT, RotaryEncoder::LatchMode::TWO03);
  //  pinMode(MIC_PIN,INPUT);
#ifndef ESP32
  // ESP32's random() uses the hardware RNG unless randomSeed() is called.
  randomSeed(analogRead(0));
#endif
  initAvailableIndicies();
      attachInterrupt(digitalPinToInterrupt(ROTARY_CK), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), checkPosition, CHANGE);
  for (int a = 0; a < 30; a++) {
//    LEDs[a]= CRGB(255,180,40);/
//        Serial.println(a);
      
    pixel newPixel = pixel();

    //    if (random(2) == 1) {
    //      newPixel.minY = 0;
    //      newPixel.maxY = 8;
    //      newPixel.minX = 16;
    //      newPixel.maxX = 32;
    //    }
    //    else {
    //      newPixel.minY = 0;
    //      newPixel.maxY = 8;
    //      newPixel.minX = 0;
    //      newPixel.maxX = 16;
    //      newPixel.half = true;
    //    }
    newPixel.half = true;
    newPixel.randomize();

    //  newPixel.getNeighbors();
    pixels.push_back(newPixel);

  }
  FastLED.show();
  //  for (int x = 0; x < 32; x++) {
  //    for (int y = 0; y < 8; y++) {
  //      if (cunt[x][y] == '1') {
  //        pixel newPixel = pixel();
  //        newPixel.randomize();
  //        newPixel.xPos = x;
  //        newPixel.yPos = y;
  //        newPixel.half = true;
  //        pixels.push_back(newPixel);
  //      }
  //    }
  //  }
  Serial.println(pixels.size());




}








int x = 0;
int y = 0;
int noise = 0;
int maxNoise = 0;
CRGB color = CRGB(0, 0, 0);
int frameCount=1;
static int lastSegment = -2;

// Matches the pixels to the current transition: the extra pixels of the bigger phase arrive or leave
// together across it, and pixels of a colour neither phase uses are recycled.
void reconcilePixels() {
  int from = max(1, (int)lroundf(activeSegFrom->pixels));
  int to = max(1, (int)lroundf(activeSegTo->pixels));
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
    if (px.retired || px.fadingOut) continue;
    if (colourAbsent(px.spawnIndex)) { px.expire(); continue; }
    if (px.role == ROLE_CORE) core++;
    else if (px.role == ROLE_ARRIVING) arriving++;
    else leaving++;
  }

  // Surplus pixels take a transition role where one is open, otherwise fade out.
  for (pixel& px : pixels) {
    if (px.retired || px.fadingOut) continue;
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
  while ((core < wantCore || arriving < wantArriving || leaving < wantLeaving) && !availableIndicies.empty()) {
    pixel newPixel = pixel();
    newPixel.half = true;
    if (core < wantCore) { newPixel.role = ROLE_CORE; core++; }
    else if (arriving < wantArriving) { newPixel.role = ROLE_ARRIVING; arriving++; }
    else { newPixel.role = ROLE_LEAVING; leaving++; }
    newPixel.randomize();
    if (newPixel.role == ROLE_CORE) newPixel.presence = 0.0f;
    pixels.push_back(newPixel);
  }
}

void loop() {
  networkLoop();
  const Look& look = dayCycleUpdate();
  moveSpeed = max(1, (int)lroundf(look.moveEvery));
  colorSpeed = max(1, (int)lroundf(look.colorEvery));
  fadeBoard((int)lroundf(look.fade));
  FastLED.setBrightness((uint8_t)lroundf(look.brightness));
//   if(encoderPress()){
// //    handleEncoder(0);/
//      handleModes();
//     }

  for (size_t i = 0; i < pixels.size(); i++) {
    if (pixels[i].retired) continue;
    std::vector<pixel*> neighbors = pixels[i].getNeighbors();

    // A dark pixel respawns in the same role, unless it was fading out for good.
    if (pixels[i].age(!neighbors.empty())) {
      if (pixels[i].fadingOut) {
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
  
  //  pixel newPixel = pixel();


  
  //  pixel nPixel=pixel();
  //
  //  if (random(2)==1){
  //    nPixel.minY=0;
  //    nPixel.maxY=8;
  //    nPixel.minX=16;
  //    nPixel.maxX=32;
  //    }
  //    else{
  //      nPixel.minY=0;
  //      nPixel.maxY=8;
  //      nPixel.minX=0;
  //      nPixel.maxX=16;
  //      nPixel.half=true;
  //      }
  //       nPixel.randomize();
  //
  ////  newPixel.getNeighbors();
  //  pixels.push_back(nPixel);
   delay(75);

}
