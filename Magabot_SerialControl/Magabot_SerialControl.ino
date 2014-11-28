#include <Wire.h>
#include <Magabot.h>

Magabot robot;

int inByte = 'p';
char order = '1'; 

void setup()
{
  Serial.begin(9600);
  robot.actuateMotors(0,0);
  robot.actuateLEDs(255,255,255);
}

void loop() 
{  
  if (Serial.available() > 0) 
  {
    inByte = Serial.read();
    
    
    lowLevelProtocol();
  }
    
  update();
  robot.update();
}

