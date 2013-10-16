/*
 * ServoControl.cpp
 *
 *  Created on: Sep 14, 2013
 *      Author: sk
 */

#include "ServoControl.h"

void ServoControl::setSerial(ofSerial *serial) {
	this->serial = serial;
}

void ServoControl::setNumber(unsigned char number) {
	this->number = number;
}

bool ServoControl::sendTarget(ofSerial *serial) {

	uint32_t target_value = (uint32_t) (target.get());

    serial->flush();

	serial->writeByte(0xAA); //start byte
	serial->writeByte(0x0C); //device id
	serial->writeByte(0x04); //command number
	serial->writeByte(number); //servo number
	serial->writeByte(target_value & 0x7F);
	serial->writeByte((target_value >> 7) & 0x7F);
}

int ServoControl::getTargetCommand(ofSerial *serial) {

	return 0;
}

unsigned char ServoControl::getErrorCommand(ofSerial *serial) {

	return 0;
}

void ServoControl::updateTarget(int &placeholder) {

	cout << (int)number << " updateTarget " << target.get() << endl;

	sendTarget(serial);
}

void ServoControl::getTarget() {

	int target = getTargetCommand(serial);

	cout << (int)number << " getTarget " << target << endl;
}

void ServoControl::getError() {

	unsigned char error = getErrorCommand(serial);

	cout << (int)number << " getError " << (int)error << endl;
}

void ServoControl::resetRange() {

	set_min.set(default_min);
	set_max.set(default_max);

	if (target.get() < default_min) {
		target.set(default_min);
	}
	if (target.get() > default_max) {
		target.set(default_max);
	}
}
