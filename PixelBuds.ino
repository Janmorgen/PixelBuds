#include <FastLED.h>
#include <RotaryEncoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArxContainer.h>
#include <stdio.h>
#include <stdlib.h>
#include <numeric>
#include "alphabet.h"

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

RotaryEncoder *encoder = nullptr;

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
    void computeCycle();
    void writePixel();
    void randomize();
    void age(bool fade, std::vector<pixel> & neighbors);
    std::vector<pixel> getNeighbors();
    std::vector<pixel> getNeighbors(std::vector<pixel> prevNeighbors);
    CRGBArray<2> splitColors(CRGB A, CRGB B, int denom, int numer);
    bool operator==( pixel& A);
    bool operator!=( pixel& A);
    void splitColors(CRGB A, std::vector<pixel>& neighbors);
    void averageColors(std::vector<pixel> & neighbors);


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



  int rand3 = random(3);
  this->lifeTime = random(30, 200);
  //  this->half = rand2;/
  switch (rand3) {
    case (0): {
        this->color = CRGB(255, 0, 0);
        break;
      }
    case (1): {
        this->color = CRGB(0, 255, 0);
        break;
      }
    case (2): {
        this->color = CRGB(0, 0, 255);
        break;
      }
    default: {
        Serial.println("Uncaught Error");
      }
  }
  this->turnsLived = 0;


}
void pixel::age(bool fade, std::vector<pixel> & neighbor) {

  this->turnsLived += 1;
  this->shareTo = !(this->turnsLived > this->lifeTime);


  if (this->color[0] < 5 && this->color[1] < 5 && this->color[2] < 5) {
    // Serial.print("AvailableIndicies Size: ");
    // Serial.println(availableIndicies.size());
    std::vector<int> newAvailableIndex;

    newAvailableIndex.push_back(this->xPos);
    newAvailableIndex.push_back(this->yPos);
    // availableIndicies.reserve(newAvailableIndex.size()*2);
    availableIndicies.push_back(newAvailableIndex);
    this->randomize();
    //        delete pixels.back();


    //        pixels.push_back(newPixel);
  }



}
std::vector<pixel> pixel::getNeighbors() {
  std::vector<pixel> output;
  //    this->neighbors.clear();
  //    this->neighbors.swap(newNeighbors);
  for (pixel px : pixels) {
    if (px.getX() < this->maxX && px.getY() < this->maxY && px.getX() > this->minX && px.getY() > this->minY) {
      if ((abs(px.getX() - this->xPos) == 1) && this->yPos == px.getY() || (abs(px.getY() - this->yPos) == 1) && this->xPos == px.getX()) {

        output.push_back(px);

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

void pixel::splitColors(CRGB A, std::vector<pixel>& neighbors) {

  int tempCalc[3] = {0, 0, 0};
  CRGB output(0, 0, 0);
  int shareToSize = 0;

  for (pixel neighbor : neighbors) {
    for (int i = 0; i < 3; i++) {
      tempCalc[i] += neighbor.color[i];
    }
    if (neighbor.shareTo) shareToSize += 1;


  }
  if (shareToSize != 0) {
    if (this->shareTo) shareToSize += 1;

    for (int i = 0; i < 3; i++) {
      tempCalc[i] += A[i];

      int outputVal = tempCalc[i] / shareToSize;

      if (outputVal > 255) {
        //        Serial.print("OutputVal Greater than 255! on: ");/
        //        Serial.println(i);/
        outputVal = 128;
      }

      output[i] = outputVal;

    }
    for (pixel neighbor : neighbors) {
      if (neighbor.shareTo)
        neighbor.color = output;
      //      else neighbor.color.fadeLightBy(70);
    }
    if (this->shareTo) this->color = output;
    else {
      this->color = CRGB(0, 0, 0);
    }
  } else {
    //    this->color.fadeLightBy(100);
  }
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
void pixel::averageColors(std::vector<pixel> & neighbors) {
  //  if (neighbors.size()==0) return;
  if (this->half) {
    splitColors(this->color, neighbors);
    //    this->color =output;
  } else {
    //      Best bang for buck funciton, nice visuals, good diffusion, low effort
    for (pixel px : neighbors) {

      CRGBArray<2> colorSplit = splitColors(px.getColor(), this->color, 1, 2);

      CRGB average = colorSplit[0];
      px.setColor(average);
      this->color = average;

    }
  }
}


void pixel::computeCycle() {

  std::vector<pixel> neighbors = this->getNeighbors();
  int neighborSize = neighbors.size() + 1;

  if (neighbors.size() == 0) {
    int minDistance = 256;
    int tX = this->xPos;
    int tY = this->yPos;

    // Add available Index
    addAvailableIndex(tX,tY);



    for (pixel px : pixels) {
      if (px.getX() < this->maxX && px.getY() < this->maxY && px.getX() > this->minX && px.getY() > this->minY) {
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

    } else {
      int randomDir = random(2);
      switch (randomDir) {
        case (0): {
            if (this->yPos + 1 < this->maxY) this->yPos += 1;
            else this->yPos -= 1;
            break;
          }
        case (1): {
            if (this->yPos - 1 > this->minY) this->yPos -= 1;
            else this->yPos += 1;

            break;
          }


      }


    }


    neighbors = this->getNeighbors();

    // Remove available index after moving pixel
    removeAvailableIndex(this->xPos, this->yPos);


    //  this->age(false);
  }
  this->age(!(neighbors.size() == 0), neighbors);

  if (neighbors.size() != 0) {
    this->averageColors(neighbors);
  }

}
void pixel::writePixel() {
  LEDs[getIndex(this->xPos, this->yPos)] = this->color;

}






void setup() {

  Serial.begin(9600);
  Serial.println("WS2812B LEDs strip Initialize");
  //  Serial.println("Please enter the 1 to 6 value.....Otherwise no any effect show");
  FastLED.addLeds<CHIP_SET, RGB_PIN, COLOR_CODE>(LEDs, RGB_LED_NUM);
  //  randomSeed(analogRead(0));
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.clear();
  FastLED.show();
  //  pinMode(4,INPUT);
  //  pinMode(MIC_PIN,INPUT);
  randomSeed(analogRead(0));
  initAvailableIndicies();

  for (int a = 0; a < 200; a++) {
    //    Serial.println(a);
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


void loop() {
  //  FastLED.clear();/
  fadeBoard(150);

  for (int i = 0; i < pixels.size(); i++) {
    pixels[i].computeCycle();
    //      delay(90);
    pixels[i].writePixel();
    //      FastLED.show();
  }

  //  pixel newPixel = pixel();


  FastLED.show();
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
