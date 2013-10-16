/*
 * ServoControl.h
 *
 *  Created on: Sep 14, 2013
 *      Author: sk
 */

#pragma once

#include "ofMain.h"
#include "ofxGui.h"

class ServoControl {
public:

	static const int default_min = 500;
	static const int default_max = 2500;

	void setNumber(unsigned char number);
	bool sendTarget(ofSerial *serial);
	int getTargetCommand(ofSerial *serial);
	unsigned char getErrorCommand(ofSerial *serial);

    void setSerial(ofSerial *serial);

    // value change callback
    void updateTarget(int &placeholder);

    // action callbacks
	void resetRange();
    void getTarget();
	void getError();

	ofParameterGroup parameters;
	ofParameter<int> target;
	ofParameter<int> set_min;
	ofParameter<int> set_max;

    ofxButton reset_range;
	ofxButton get_target;
	ofxButton get_error;


private:
	ofSerial *serial;
	string port;
	unsigned char number;
};
