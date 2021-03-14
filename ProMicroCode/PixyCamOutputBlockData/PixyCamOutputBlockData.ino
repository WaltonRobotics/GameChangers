#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

/*
 * Analog write outputs (duty cycle) and their corresponding messages:
 * 0 (0%) - no determination
 * 64 (25%) - red A
 * 128 (50%) - red B 
 * 192 (75%) - blue A
 * 255 (100%) - blue B
 */
 
void setup()
{
  Serial.begin(115200);
  
  pixy.init();
}

void loop()
{
  const int numBallsDetected = pixy.ccc.getBlocks();

  if (numBallsDetected)
  {
    Serial.print("Detected ");
    Serial.println(numBallsDetected);
    for (int i = 0; i < numBallsDetected; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }
}
