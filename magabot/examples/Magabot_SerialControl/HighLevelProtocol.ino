int veloc1=0,veloc2=0;
int velocity = 15;
int turnVel = 8;

boolean forward = true;
boolean stopped = true;
boolean turning = false;
boolean right = false;

int reverseState =0;
unsigned long becoTime;
unsigned long bTime;
unsigned long iTime;


void highLevelProtocol()
{
  if(order == '1') AssistedNavigation();
  if(order == '2') obstacleAvoid();
  
  robot.irMaxValue = 1023;
  robot.readBattery();
  //robot.readClicks(); 
  //Serial.println(robot.leftMotorVelocity);
}

//Simple characters instructions Serial control protocol
int SerialAnalyze()
{
  if (inByte=='a' || inByte == 'A')
  {
    right = false;
    turn();
    turning = true;   
  }
  //d
  else if(inByte=='d'|| inByte=='D')
  {
    right=true;
    turn();
  }
  //s
  else if(inByte=='s'|| inByte=='S')
  {
    veloc1 = -velocity;
    veloc2 = -velocity;
    forward = false;
    turning = false;
    stopped = false;
  }
  //w
  else if(inByte=='w'|| inByte=='W')
  {
    veloc1 = (velocity);
    veloc2 = (velocity);
    forward = true;
    turning = false;
    stopped = false;
    
  }
  //p
  else if(inByte=='p'|| inByte=='P' || inByte=='G' || inByte=='g')
  {
    veloc1 = 0;
    veloc2 = 0;
    turning = false;
    stopped = true;
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
  else if (inByte == 'v' || inByte =='V')
  {
    Serial.print("r:");
    for (int i=0;i<3;i++)
    {
      Serial.print(robot.irRead[i]);
      Serial.print(';');
    }
    Serial.println();
    Serial.print("s:");
    for (int o=0;o<5;o++)
    {
      Serial.print(robot.sonarRead[o].read());
      Serial.write(';');
    }
    Serial.println();
  }
  robot.actuateMotors(veloc1,veloc2);
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



void AssistedNavigation()
{
  robot.readIR();
  robot.readBumpers();
  robot.readSonars();
  
  if( (robot.bumperRead[0]||robot.bumperRead[1]) && millis()>bTime)
  {
    Serial.println('b');
    bTime = millis()+500;
  }
  
  if( (robot.irState[0]||robot.irState[1]||robot.irState[2]) && millis()>iTime) 
  {
    Serial.println('i');
    iTime = millis()+500;
  }
  if(millis()<bTime || millis()<iTime)
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


//**************************************//
//***********Obstacle avoidance*********//
//*********For autonomous control*******//
///*************************************//
void obstacleAvoid()
{
  robot.readIR();
  robot.readBumpers();
  robot.readSonars();
  
  veloc1 = velocity;
  veloc2 = velocity;
 
  int frontDistance = 100;
  int sideDistance = 50;


  if(robot.bumperRead[0])
  {
    reverseState = 1;

    robot.actuateMotors(-5,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(robot.bumperRead[1])
  {
    reverseState = 1;
    robot.actuateMotors(-veloc1,-5);  
    becoTime = millis()+1000;  
  }
  if(robot.irState[1])
  {
    reverseState = 1;
    robot.actuateMotors(-10,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(robot.irState[1])
  {
    reverseState = 1;
    robot.actuateMotors(-10,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(robot.irState[2])
  {
    reverseState = 1;

    robot.actuateMotors(-veloc1,-10);  
    becoTime = millis()+1000;  
  }
  
  if (reverseState ==1 && millis()>becoTime)
  {
    reverseState = 0;
  }
  int bestDirection = 0;
  int bestDirectionValue = 0;

  for (int count = 0;count<5;count++)
  {
    if(robot.sonarRead[count].read() > bestDirectionValue)
    {
      bestDirectionValue = robot.sonarRead[count].read();
      bestDirection = count;
    }
  }
  String dir;
  if (reverseState == 0)
  {    
    if(robot.sonarRead[2].read() > frontDistance)
    {
      robot.actuateMotors(veloc1,veloc2);
      robot.actuateLEDs(0,255,0);
      if(robot.sonarRead[0].read() < sideDistance || robot.sonarRead[1].read() < sideDistance)
      {
        robot.actuateMotors(veloc1,0);  
        robot.actuateLEDs(255,255,0);
      }
      else if(robot.sonarRead[3].read() < sideDistance || robot.sonarRead[4].read() < sideDistance)
      {
        robot.actuateMotors(0,veloc2);  
        robot.actuateLEDs(255,255,0);
      } 
    }
    else
    {
      robot.actuateMotors(veloc1,0);
      robot.actuateLEDs(255,255,0);
    }
  }
  else
  {
    robot.actuateLEDs(255,0,0);
  }
}

