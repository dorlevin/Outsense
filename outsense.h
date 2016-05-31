#ifndef __OUTSENSE__
#define __OUTSENSE__
#include "mraa.hpp"
#include <ctime>
#include <array>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


#define DEATHTIME 2100
#define RESOLUTION "640x480";
#define SECSPDAY 24*60*60
#define DEATHSEC DEATHTIME*36
#define STD_RECIP "dorlevin@gmail.com"
using namespace std;

class USBCam;

enum class Power_state{
        polling = 0,
        Active,
        NightSleep,
        PowerDown,

};




class Logger{
	friend class Outsensor;
public:

	Logger();
	~Logger();
	void write(string message);

protected:
	string m_name;
	unsigned long m_creation;
	ofstream m_file;
};

class Outsensor{
	friend class Logger;
public:
        Outsensor();
        Outsensor(unsigned sensorPin, unsigned sensorEnPin, unsigned led1, unsigned led2, unsigned led3, unsigned led4);
        ~Outsensor(void);
        int setupIO(unsigned sensorPin, unsigned sensorEnPin, unsigned led1, unsigned led2, unsigned led3, unsigned led4);
        int Cycle(void);
        Power_state getPstate(void) const;
        void S3(void);
        void S4(void);
        void setup(void);
        bool sense(void);
        void nightSleep(void);
        void powerOff(bool reduceQuality);
        void writeLog(string message);
        void enableSensor(void);
        void disableSensor(void);
        void registerWakeUp(void);
        bool sendImages(bool reduceQuality);

protected:
        Power_state m_pstate;
        array<mraa::Gpio*, 4> m_ledArray;
        mraa::Gpio* m_sensor;
        mraa::Gpio* m_sensorEn;
		Logger m_log;
};




void signalHandler(int signum);

#endif
