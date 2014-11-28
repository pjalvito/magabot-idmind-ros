int dir1=0,dir2=0;
int veloc1=0,veloc2=0;
int velocity = 15;
int turnVel = 8;
int reverseState =0;
boolean forward = true;
boolean stopped = true;
boolean turning = false;
boolean right = false;
boolean manual = false;
int red;
int green;
int blue;
unsigned long bTime = 0;
// for full low level control serial protocol
void lowLevelProtocol()
{
  robot.readSonars();
  //Sonars 'S'
  if (inByte==0x83)
  {
    for (int o=0;o<5;o++)
    {
      Serial.write((unsigned char) (robot.sonarRead[o].read() >> 8));
      Serial.write((unsigned char) (robot.sonarRead[o].read() & 0xFF));
    }
  }
  //Odometer 'J'
  else if (inByte==0x74)
  {
    robot.readClicks();
    Serial.write((unsigned char) (robot.leftClicks >> 8));
    Serial.write((unsigned char) (robot.leftClicks & 0xFF));
    Serial.write((unsigned char) (robot.rightClicks >> 8));
    Serial.write((unsigned char) (robot.rightClicks & 0xFF));
  }
  //Infra-Red Sensors 'I'
  else if (inByte==0x49) //I IR read
  {
    robot.readIR();
    for (int i=0;i<3;i++)
    {
      Serial.write((unsigned char) (robot.irRead[i] >> 8));
      Serial.write((unsigned char) (robot.irRead[i] & 0xFF));
    }
  } 
  //Battery Sensor 'K'
  else if (inByte==0x4B)
  {
    robot.readBattery();
    Serial.write((unsigned char) (robot.batteryRead >> 8));
    Serial.write((unsigned char) (robot.batteryRead & 0xFF));        
  }  
  //Bumpers 'B'
  else if (inByte==0x66)
  {
    robot.readBumpers();
    for (int b=0;b<2;b++)
    {
      Serial.write((unsigned char) robot.bumperRead[b]);
    }
  }  
  //LEDs 'v'
  else if (inByte==0x76)
  {
    while(Serial.available()==0);
    red = Serial.read();
    while(Serial.available()==0);
    green = Serial.read();
    while(Serial.available()==0);
    blue = Serial.read();
    robot.actuateLEDs(red,green,blue);
  }
  //Motors '†'
  else if (inByte==0x86)
  {
    while(Serial.available()==0);
      veloc1 = Serial.read();
    while(Serial.available()==0);
      dir1 = Serial.read();
    while(Serial.available()==0);
      veloc2 = Serial.read();
    while(Serial.available()==0);
      dir2 = Serial.read();
   
     
   int vel1 = 0;
   int vel2 = 0;
   
   
   
    if((int)dir1 == 0)
      vel1 = (int)veloc1;
    else
      vel1 = -(int)veloc1;
      
      
    if((int)dir2 == 0)
      vel2 = (int)veloc2;
    else
      vel2 = -(int)veloc2;
    
    robot.actuateMotors(vel1,vel2);
  }
  
  if (inByte=='a' || inByte == 'A')
  {
    right = false;
    turn();
    turning = true;
    manual = true;   
  }
  //d
  else if(inByte=='d'|| inByte=='D')
  {
    right=true;
    turn();
    manual = true;
  }
  //s
  else if(inByte=='s'|| inByte=='S')
  {
    veloc1 = -velocity;
    veloc2 = -velocity;
    forward = false;
    turning = false;
    stopped = false;
    manual = true;
  }
  //w
  else if(inByte=='w'|| inByte=='W')
  {
    veloc1 = (velocity);
    veloc2 = (velocity);
    forward = true;
    turning = false;
    stopped = false; 
    manual = true;
  }
  //p
  else if(inByte=='p'|| inByte=='P' || inByte=='G' || inByte=='g')
  {
    veloc1 = 0;
    veloc2 = 0;
    turning = false;
    stopped = true;
    manual = false;
    robot.actuateMotors(veloc1,veloc2);
  }
  else if(inByte== '+' || inByte == '-')//ox43 ='+' 0x45='-'
  {
    velocity= (inByte== '+') ? velocity+1 : velocity-1; 
    if (!stopped)
    {
      if(turning)
      {
        turn();
      }
      else if (forward)
      {
        veloc1 = velocity;
        veloc2 = velocity;
      }  
      else//backward
      {
        veloc1 = -velocity;
        veloc2 = -velocity;
      }
    }
  }
  if(manual)
  {
    robot.actuateMotors(veloc1,veloc2);
  }
  
}

void update()
{
  
    robot.readBumpers();
    if( (robot.bumperRead[0]||robot.bumperRead[1]) && millis()>bTime)
    {
      
      bTime = millis()+500;
    }
  if(millis()<bTime )
  {
    robot.actuateMotors(-velocity,-velocity);
    robot.actuateLEDs(255,0,0);
    reverseState = 1;
  }
  else
  {
    robot.actuateLEDs(0,0,255);    

    if (reverseState == 1)
    {
      veloc1 = 0;
      veloc2 = 0;
      robot.actuateMotors(veloc1,veloc2);
      reverseState = 0;
    }
  }
}
void turn()
{  
 if (!right)
 {
    if (stopped)
    {
       veloc1 = -turnVel;
       veloc2 = turnVel;
    }
    else if (forward) 
    {
      veloc1 = velocity-turnVel;
      veloc2 = velocity;
    }
    else
    {
      veloc1 = -velocity;
      veloc2 = -velocity+turnVel;
    }
  }
  else 
  {
    if (stopped)
    {
       veloc1 = turnVel;
       veloc2 = -turnVel;
    }
    else if (forward) 
    {
      veloc1 = velocity;
      veloc2 = velocity-turnVel;
    }
    else
    {
      veloc1 = -velocity+turnVel;
      veloc2 = -velocity;
    }
  }
}

