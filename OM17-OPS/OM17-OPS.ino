#include <SPI.h>
#include <Pixy.h>

const byte CATEGORY_TOP = 0b00000001;
const byte CATEGORY_BOTTOM = 0b00000010;
const byte CATEGORY_LEFT = 0b00000100;
const byte CATEGORY_RIGHT = 0b00001000;

Pixy pixy;

long serialTimer = 0;

void setup()
{
  Serial.begin(9600);
  pixy.init();
}

void loop()
{
  uint16_t blocks;
  
  blocks = pixy.getBlocks();
  
  if (blocks == 4) {
    serialTimer++;
    analyzeBlocks();
  }

  delay(1);
}

void analyzeBlocks()
{
  int tl = -1;
  int tr = -1;
  int bl = -1;
  int br = -1;
  
  for (int i = 0; i < 4; ++i) {
    int leftOf = 0;
    int topOf = 0;
    
    for (int j = 0; j < 4; ++j) {
      if (i == j) continue;
      if (pixy.blocks[i].x < pixy.blocks[j].x) {
        leftOf++;
      }
      if (pixy.blocks[i].y < pixy.blocks[j].y) {
        topOf++;
      }
    }

    if ((leftOf >= 2) && (topOf >= 2)) {
      tl = i;
    }

    if ((leftOf < 2) && (topOf >= 2)) {
      tr = i;
    }

    if ((leftOf >= 2) && (topOf < 2)) {
      bl = i;
    }

    if ((leftOf < 2) && (topOf < 2)) {
      br = i;
    }
  }

  if (serialTimer % 50 == 0) {
    int lu = pixy.blocks[tr].x - pixy.blocks[tl].x;
    int ll = pixy.blocks[br].x - pixy.blocks[bl].x;
  
    float lavg = ((float)lu + (float)ll) / 2.0f;
  
    int wl = pixy.blocks[bl].y - pixy.blocks[tl].y;
    int wr = pixy.blocks[br].y - pixy.blocks[tr].y;
  
    float wavg = ((float)wl + (float)wr) / 2.0f;

    float angle = -23256500.0f + (48.4422f - -23256500.0f)/(1 + pow(((lavg / wavg)/8.270154f), 7.675144f));
    float dist = 120.5 / (float)wavg;

    Serial.println(angle);
    Serial.println(dist);
  }
}

