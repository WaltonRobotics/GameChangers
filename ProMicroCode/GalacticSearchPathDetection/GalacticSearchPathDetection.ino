#include <Pixy2.h>
#include <float.h>

#define NUM_PATHS 4
#define REGIONS_PER_PATH 3

#define GALACTIC_SEARCH_RED_A_INDEX  0
#define GALACTIC_SEARCH_RED_B_INDEX  1
#define GALACTIC_SEARCH_BLUE_A_INDEX 2
#define GALACTIC_SEARCH_BLUE_B_INDEX 3

#define NO_DETERMINATION_DUTY_CYCLE 0.0
#define GALACTIC_SEARCH_RED_A_DUTY_CYCLE  0.25
#define GALACTIC_SEARCH_RED_B_DUTY_CYCLE  0.5
#define GALACTIC_SEARCH_BLUE_A_DUTY_CYCLE 0.75
#define GALACTIC_SEARCH_BLUE_B_DUTY_CYCLE 1.0

#define NUM_BALLS_THRESHOLD 2
#define MATCH_SCORE_THRESHOLD 50

#define NO_DETERMINATION_BYTE_MESSAGE 0x0A
#define GALACTIC_SEARCH_RED_A_BYTE_MESSAGE 0x0B
#define GALACTIC_SEARCH_RED_B_BYTE_MESSAGE 0x0C
#define GALACTIC_SEARCH_BLUE_A_BYTE_MESSAGE 0x0D
#define GALACTIC_SEARCH_BLUE_B_BYTE_MESSAGE 0x0E

#define BAUD_RATE 9600
struct Target {
  int x;
  int y;
  int width;
  int height;

  Target(int x, int y, int width, int height) {
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
  }

  Target() {
    this->x = 0;
    this->y = 0;
    this->width = 0;
    this->height = 0;
  }

  float getDistanceTo(Target otherTarget) {
    return sqrt(pow(otherTarget.x - this->x, 2) + pow(otherTarget.y - this->y, 2));
  }
};

class PathID {
  public:
    PathID() {
      this->nominalRegions = new Target[REGIONS_PER_PATH];
      this->matchScore = 0;
      this->name = "";
    }

    PathID(String name) {
      this->nominalRegions = new Target[REGIONS_PER_PATH];
      this->matchScore = 0;
      this->name = name;
    }

    ~PathID() {
      delete nominalRegions;
    }

  public:
    Target *nominalRegions;
    float matchScore;
    String name;
    byte byteMessage;
};

// This is the main Pixy object
Pixy2 pixy;

PathID galacticSearchPathIDs[NUM_PATHS];

PathID &getBestPath() {
  float bestScore = FLT_MAX;
  int bestIndex = 0;

  for (int i = 0; i < NUM_PATHS; i++) {
    float currentScore = galacticSearchPathIDs[i].matchScore;

    if (currentScore < bestScore) {
      bestScore = currentScore;
      bestIndex = i;
    }
  }

  return galacticSearchPathIDs[bestIndex];
}

void setup() {
  Serial.begin(BAUD_RATE);

  pixy.init();

  galacticSearchPathIDs[GALACTIC_SEARCH_RED_A_INDEX].name = "Galactic Search Red A";
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_A_INDEX].nominalRegions[0] = Target(178, 120, 44, 38);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_A_INDEX].nominalRegions[1] = Target(251, 38, 22, 18);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_A_INDEX].nominalRegions[2] = Target(34, 39, 12, 10);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_A_INDEX].byteMessage = GALACTIC_SEARCH_RED_A_BYTE_MESSAGE;

  galacticSearchPathIDs[GALACTIC_SEARCH_RED_B_INDEX].name = "Galactic Search Red B";
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_B_INDEX].nominalRegions[0] = Target(251, 39, 22, 16);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_B_INDEX].nominalRegions[1] = Target(109, 25, 14, 9);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_B_INDEX].nominalRegions[2] = Target(6, 131, 12, 8);
  galacticSearchPathIDs[GALACTIC_SEARCH_RED_B_INDEX].byteMessage = GALACTIC_SEARCH_RED_B_BYTE_MESSAGE;

  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_A_INDEX].name = "Galactic Search Blue A";
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_A_INDEX].nominalRegions[0] = Target(297, 24, 14, 12);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_A_INDEX].nominalRegions[1] = Target(108, 26, 12, 9);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_A_INDEX].nominalRegions[2] = Target(160, 12, 8, 8);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_A_INDEX].byteMessage = GALACTIC_SEARCH_BLUE_A_BYTE_MESSAGE;

  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_B_INDEX].name = "Galactic Search Blue B";
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_B_INDEX].nominalRegions[0] = Target(232, 28, 16, 12);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_B_INDEX].nominalRegions[1] = Target(116, 19, 12, 9);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_B_INDEX].nominalRegions[2] = Target(195, 6, 6, 7);
  galacticSearchPathIDs[GALACTIC_SEARCH_BLUE_B_INDEX].byteMessage = GALACTIC_SEARCH_BLUE_B_BYTE_MESSAGE;
}

void loop() {
  const int numBallsDetected = pixy.ccc.getBlocks();

  if(Serial.available()) {
    
    byte value = Serial.read();
    
    if (numBallsDetected)
    {
      for (PathID &path : galacticSearchPathIDs) {
        path.matchScore = 0;
  
        for (int i = 0; i < numBallsDetected; i++)
        {
          Target blockTarget = Target(pixy.ccc.blocks[i].m_x,
                                      pixy.ccc.blocks[i].m_y,
                                      pixy.ccc.blocks[i].m_width,
                                      pixy.ccc.blocks[i].m_height);
  
          float bestScore = FLT_MAX;
  
          for (int j = 0; j < REGIONS_PER_PATH; j++) {
            float currentScore = blockTarget.getDistanceTo(path.nominalRegions[j]);
  
            if (currentScore < bestScore) {
              bestScore = currentScore;
            }
          }
  
          path.matchScore += bestScore;
        }
      }
  
      PathID &bestPath = getBestPath();
      //Serial.println(bestPath.name + " - score: " + bestPath.matchScore);
      
      if (bestPath.matchScore < MATCH_SCORE_THRESHOLD && numBallsDetected >= NUM_BALLS_THRESHOLD) {
        Serial.write(bestPath.byteMessage);
        // digitalWrite(PIXY_CAM_WRITE_PIN, HIGH);
        // analogWrite(PIXY_CAM_WRITE_PIN, (int)(bestPath.dutyCycle * 255.0));
      } else {
        Serial.write(NO_DETERMINATION_BYTE_MESSAGE);
      } 
      // analogWrite(PIXY_CAM_WRITE_PIN, (int)(NO_DETERMINATION_DUTY_CYCLE * 255.0));
    } else {
      Serial.write(NO_DETERMINATION_BYTE_MESSAGE);
    }
  }

  delay(50);

}
