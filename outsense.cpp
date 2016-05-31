/*
 * Author: Dor Levin <dor.levin@intel.com>
 * Copyright (c) 2015 - 2016 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file outsense.cpp
 * @brief outsense sensor
 *
 * code for product based on Intel Edison, integrating camera, leds and proximity sensor
 * and implementing low-power algorithms, for high efficiency, and low power consumption.
 *
 * @date 31/05/2016
 */
#include "outsense.h"
#include "mraa.hpp"
#include <ctime>
#include <array>
#include <iostream>
#include <unistd.h>
#include "stdio.h"
#include "stdlib.h"
#include <csignal>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <linux/kernel.h>
#include <thread>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
bool debug = false;
bool dropbox = false;

typedef unsigned long long uint64_t;

using namespace std;
using namespace cv;


time_t now;
static uint64_t imageNum = 0;


//c-tor, pin numbers can be changed as you wish
Outsensor::Outsensor(void)
:Outsensor(13, 3, 6, 9, 10, 11)
{}
Outsensor::Outsensor(unsigned sensorPin, unsigned sensorEnPin, unsigned led1, unsigned led2, unsigned led3, unsigned led4)
:m_pstate(Power_state::polling), m_ledArray(), m_sensor(), m_sensorEn()
{
        setupIO(sensorPin, sensorEnPin, led1, led2, led3, led4);
}

Outsensor::~Outsensor(void)
{
        delete m_sensor;
        for (int i=0;i<m_ledArray.size();++i) delete m_ledArray[i];
        delete m_sensorEn;
}

//interrupt handler, for when waking up from low-power mode
void interruptHandler(void *args){
//	if (debug) cout<<"woke up due to sensor interrupt"<<endl;
//	g_outsensor->Cycle();
	//m_pstate = Power_state::polling;
	signal(SIGINT, signalHandler);
	sleep(0.5);
	return;
}
array<mraa::Gpio*, 4> g_leds;//for clean exit purpuse
mraa::Gpio* g_sensor;
mraa::Gpio* g_sensorEn;
//setup GPIO pins as connected by HW:
//sensorPin - Digital input from proximity sensor
//sensorEnPin - digital output pin, connected to the sensor's Vcc, in order to disable it
//led1-4 - Digital output pins, connected to the Anodes.
int Outsensor::setupIO(unsigned sensorPin, unsigned sensorEnPin, unsigned led1, unsigned led2, unsigned led3, unsigned led4){
    // create a analog input object from Analog pin 0
    m_sensor = new mraa::Gpio(sensorPin);
    if (m_sensor->dir(mraa::DIR_IN)!=mraa::SUCCESS)
        std::cerr << "Can't set digital pin as input, exiting" << std::endl;
    m_sensor->isr(mraa::EDGE_RISING, &interruptHandler, NULL );
    g_sensor = m_sensor;
    // create a GPIO object from MRAA using it
    m_sensorEn = new mraa::Gpio(sensorEnPin);
    if (m_sensorEn->dir(mraa::DIR_OUT)!=mraa::SUCCESS)
	    cerr<<"Can't set digital pin enable as output"<<endl;
    mraa::Gpio* led_white = new mraa::Gpio(led1,true,false);
    mraa::Gpio* led_green = new mraa::Gpio(led2,true,false);
    mraa::Gpio* led_amber = new mraa::Gpio(led3,true,false);
    mraa::Gpio* led_lime = new mraa::Gpio(led4,true,false);
    g_sensorEn = m_sensorEn;
    m_ledArray = {led_white, led_green, led_amber, led_lime};
    g_leds = m_ledArray;
    // set the pin as output
    if    ((led_white->dir(mraa::DIR_OUT) |
                    led_green->dir(mraa::DIR_OUT) |
                    led_amber->dir(mraa::DIR_OUT) |
                    led_lime->dir(mraa::DIR_OUT)) != mraa::SUCCESS) {
            std::cerr << "Can't set digital pin as output, exiting" << std::endl;
            return MRAA_ERROR_UNSPECIFIED;
    }
    return mraa::SUCCESS;
}
Power_state Outsensor::getPstate(void) const{
        return m_pstate;
}
//this function is in charge of doing a cycle when in active mode. in charge of leds and camera usage
int Outsensor::Cycle(void){
	VideoCapture cap(0);
	if (!cap.isOpened()){
		cout<<"error openning the cam"<<endl;
		writeLog("there was an error openning the camera");
	}
	disableSensor();

    vector<int> compression_params; //vector that stores the compression parameters of the image
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
    compression_params.push_back(98); //specify the compression quality
	m_pstate = Power_state::Active;
	if (debug) cout<<"cycling"<<endl;
	string pre = "/media/sdcard/";
	string numPre = to_string(m_log.m_creation);
	writeLog("cycling");
	Mat waste;
	Mat frame[4];

	int i = 0;
	//flash leds, and capture frames
	for (auto led: m_ledArray)
	{
		led->write(1);
		sleep(0.025);
		cap >> frame[i];
		led->write(0);
		cap >> waste;
		++i;
	}
	//save frames to files
	for (auto f : frame){
		string num = numPre + "_" + to_string(++imageNum);
		string path= pre + num + ".jpg";
		bool bSuccess = imwrite(path, f, compression_params); //write the image to file
	}
	string mess ="captured images " + to_string(m_log.m_creation) + "_" + to_string(imageNum-3) + " - " + to_string(imageNum);
	writeLog(mess);
	m_pstate = Power_state::polling;
	cap.release();
	if (dropbox) sendImages(false);
	return 1;
}
//kill Wifi and wpa, and enable power down sleep, in order to let the Edison go to S3
void Outsensor::setup(void){
        system("ifconfig wlan0 down");//shut down wifi for battery
        system("systemctl stop wpa_supplicant");
        system ("echo auto > /sys/devices/pci0000\:00/0000\:00\:04.3/power/control");
	//system("systemctl disable wpa_supplicant");


}
//registers a wakeup call with the MCU, and then takes the Edison into S3 power state
void Outsensor::S3(){
		signal(SIGINT, signalHandler);
        m_pstate=Power_state::PowerDown;
        string cmd = "echo -n ";
        cmd+= '"';
        cmd.append("mem");
        cmd+='"';
        cmd.append(" > /sys/power/state");
        //echo -n "mem" > /sys/power/state > log.txt
        registerWakeUp();
        system(cmd.c_str());
        sleep(0.2);
}
//takes Edison into S4 power state
void Outsensor::S4(){
        m_pstate = Power_state::NightSleep;
//      echo -n "disk" > /sys/power/state > log.txt
        string cmd = "echo -n ";
        cmd+= '"';
        cmd.append("disk");
        cmd+='"';
        cmd.append(" > /sys/power/state");
        system(cmd.c_str());
}

//Digital value of sensor output
inline bool Outsensor::sense(void){
        return m_sensor->read();
}
//poweroffs machine at night time
void Outsensor::nightSleep(void){
        time_t t = time(0);
        t%=SECSPDAY;//seconds today
        time_t deathSec = DEATHTIME*36;//time of death in seconds from midnight
        if (debug) cout <<"t:"<< t <<" death:"<<deathSec<<endl;
        if (t>=deathSec){
                if (debug) cout<<"trying to go to S4"<<endl;
//                S4();
                powerOff(!debug);
        }
}
void Outsensor::writeLog(string message){
	m_log.write(message);

}

//poweroffs machine at night time, after sending some images back, for Quality assurance
void Outsensor::powerOff(bool reduceQuality){
	signal(SIGINT, signalHandler);

	writeLog("power off");
	sendImages(reduceQuality);
	string cmd = "poweroff";
	system(cmd.c_str());
}

//function to send images back to recipient Dropbox account. should wake back up the Wifi and wpa
bool Outsensor::sendImages(bool reduceQuality){
	signal(SIGINT, signalHandler);
	if (imageNum<=1){
		writeLog("no pictures to send");
		return false;
	}
    system("systemctl restart wpa_supplicant");
    system("ifconfig wlan0 up");//resume wifi
	if (debug) cout<<"sending some photos"<<endl;
	sleep(10);

	system ("./dropbox.py start");
	sleep(10);
	string file;

	for (unsigned i=0;i<4;++i){

		file = "/media/sdcard/" + to_string(m_log.m_creation) + "_" + to_string(imageNum-i) + ".jpg";
		writeLog("trying to convert " + file);
		string cmd;
		if (reduceQuality)
			cmd = "convert " + file + " -quality 50 /home/root/Dropbox/sample_"+to_string(m_log.m_creation) + "_" + to_string(imageNum-i) + ".jpg";
		else
			cmd = "cp " + file + " /home/root/Dropbox/sample_"+to_string(m_log.m_creation) + "_" + to_string(imageNum-i) + ".jpg";
		writeLog(cmd);
		system(cmd.c_str());
	}
	system("./dropbox.py status");
	sleep(20);
	system("./dropbox.py stop");
    system("ifconfig wlan0 down");//shut down wifi for battery
    system("systemctl stop wpa_supplicant");
	return true;
}
//signal handler for exiting cleanly after receiving CTRL+C
void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    cout << "exiting cleanly" << endl;
    system("systemctl restart wpa_supplicant");
    system("ifconfig wlan0 up");//resume wifi
    // cleanup and close up stuff here
    for (auto led: g_leds) led->write(0);
    
    g_sensor->isrExit();
    g_sensorEn->write(0);
   exit(signum);
}
//power on the sensor, by pushing "1" through the sensorEn pin
void Outsensor::enableSensor(void){
	if (debug) cout<<"enabling sensor"<<endl;
	m_sensorEn->write(1);
}
//power off the sensor, by pushing "0" through the sensorEn pin
void Outsensor::disableSensor(void){
	if (debug) cout<<"disabling sensor"<<endl;
	m_sensorEn->write(0);
}
//register a wakeup call with the MCU, using the ttymcu port
void Outsensor::registerWakeUp(void){
	if (debug) cout<<"registering a wake up call"<<endl;
	string cmd = "echo ";
	cmd+='"';
	cmd+="start";
	cmd+='"';
	cmd+="> /dev/ttymcu0";

	system(cmd.c_str());
}

Logger::Logger()
{
	time_t now = time(0);
	m_creation = now;
	m_name = string("/media/sdcard/logs/") + std::to_string(m_creation) +string(".log");
	m_file.open(m_name.c_str());
}

Logger::~Logger(){
	m_file.close();
}

void Logger::write(string message){
	unsigned long now = time(0);
	m_file<< "[ +"<<(now-m_creation) <<" ]: "<<message<<endl;

}


int main( int argc, char *argv[])
{

	signal(SIGINT, signalHandler);
	for (int i=1; i<argc;++i){
		if (!strncmp(argv[i],"--debug", 10)) debug = true;
		if (!strncmp(argv[i],"--dropbox", 10)) dropbox = true;
	}
	Outsensor outsensor;
	outsensor.setup();
	outsensor.enableSensor();
	time_t t;
	while (1)
	{
			//cycle if proximity sensor is detecting
			if (outsensor.sense()) outsensor.Cycle();
			outsensor.disableSensor();
			if (debug) cout<<"trying to go to S3"<<endl;
			outsensor.writeLog("going to S3");
			t = time(0);
			t%=SECSPDAY;
			if (debug) cout <<"t: "<<t<<", deathtime:"<<DEATHSEC<<endl;
			//after cycle/if not detecting, take Edison into S3 power state, or poweroff, depands on time of day
			if (t<=DEATHSEC)
				outsensor.S3();
			else
				outsensor.powerOff(!debug);
			signal(SIGINT, signalHandler);
			if (debug) cout<<"woke up from S3"<<endl;
			//re-enable the sensor, for next sense
			outsensor.enableSensor();
			outsensor.disableSensor();
			outsensor.enableSensor();
			sleep(1);
	}
	return mraa::SUCCESS;
}

