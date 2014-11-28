int dir1=0,dir2=0;

int red;
int green;
int blue;

int ponteiro;

// for full low level control serial protocol
void lowLevelProtocol()
{
  robot.readSonars();
  if (ponteiro==0)
  {
    //Sonars 'S'
    if (inByte==0x83)
    {
     // robot.readSonars();
      Serial.write(inByte);
      for (int o=1;o<6;o++)
      {
        Serial.write((unsigned char) (robot.sonarRead[o].read() >> 8));
        Serial.write((unsigned char) (robot.sonarRead[o].read() & 0xFF));
      }
    }
    
    //Odometer 'J'
    else if (inByte==0x74)
    {
      robot.readClicks();
      Serial.write(inByte);   
      Serial.write((unsigned char) (robot.leftClicks >> 8));
      Serial.write((unsigned char) (robot.leftClicks & 0xFF));
      Serial.write((unsigned char) (robot.rightClicks >> 8));
      Serial.write((unsigned char) (robot.rightClicks & 0xFF));
    }
    
    //Infra-Red Sensors 'I'
    else if (inByte==0x73) //I IR read
    {
      robot.readIR();
      Serial.write(inByte);
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
      Serial.write(inByte);
      Serial.write((unsigned char) (robot.batteryRead >> 8));
      Serial.write((unsigned char) (robot.batteryRead & 0xFF));        
    }
    
    //Bumpers 'B'
    else if (inByte==0x66)
    {
      robot.readBumpers();
      Serial.write(inByte);
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
      Serial.write(inByte);
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
      int vel1 = (int)((veloc1<<8)+ dir1);
      int vel2 = (int)((veloc2<<8)+ dir2);
      robot.actuateMotors(vel1,-vel2);
      Serial.write(inByte);
        //ponteiro=1;
    }
  }
/*  else if (ponteiro==1) { veloc1 = inByte;ponteiro=2;}
  else if (ponteiro==2) { dir1 = inByte;ponteiro=3;}
  else if (ponteiro==3) { veloc2 = inByte;ponteiro=4;}
  else if (ponteiro==4) 
  {
    dir2 = inByte;ponteiro=0;
    
    int vel1 = (int)((veloc1<<8)+ dir1);
    int vel2 = (int)((veloc2<<8)+ dir2);
    
    robot.actuateMotors(vel1,-vel2);
    Serial.write(0x86);
  }
  
  else if (ponteiro==5) { red = inByte; ponteiro=6;}
  else if (ponteiro==6) { green = inByte; ponteiro=7;}
  else if (ponteiro==7) { blue = inByte; ponteiro=0; robot.actuateLEDs(red,green,blue);}*/
}
