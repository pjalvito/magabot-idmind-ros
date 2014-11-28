/*
  Magabot.h - Library for controlling Magabot robotics platform.
  Created by Francisco Dias, December 23, 2011.
  Released into the public domain.
  http://magabot.cc/
*/

#ifndef Magabot_h
#define Magabot_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#include <Wire.h>
#else
#include <WProgram.h>
#include <Wire.h>
#endif

#define REGISTER_CONFIG (16)
#define REGISTER_OUTPUT (16)

class Sonar 
{
	private:
		uint8_t id;
		int value;
		int state;
		unsigned long time;
	public:
		Sonar();
		uint8_t attach(int address);
		void detach();
		int read();
		int update();
		bool attached();
};

class Magabot
{
	public:
		Magabot();

		void actuateMotors(int vel1, int vel2);
		void update();
		void actuateLEDs(int Red, int Green, int Blue);
		void readBattery();
		void readSonars();
		void changeSonarAddress(char oldAddress, char newAddress);
		void readIR();
		void readBumpers();
		void readClicks();
	
		int irMaxValue;
		int sonarMaxValue;

		bool irState[3];
		int irRead[3];
		bool bumperRead[2];
		int sonarState[6];
		Sonar sonarRead[6];
		int batteryRead;
		short leftClicks;
		short rightClicks;
		float leftMotorVelocity;
		float rightMotorVelocity;		
		int clicksPerTurn;

	private:
	
		unsigned long _time;
		unsigned long motorUpdateTime;

		int v1;
		int v2;
		int _sonarReading;
		int _sonarNumber;
		int _sonarId;
};


#endif