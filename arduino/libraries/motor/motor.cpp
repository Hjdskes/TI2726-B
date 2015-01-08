/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include "motor.h"
#include "Arduino.h"

Motor::Motor(int fwd, int rev, int en) : fwd(fwd), rev(rev), en(en) {
	pinMode(fwd, OUTPUT);
	pinMode(rev, OUTPUT);
	pinMode(en, OUTPUT);
	this->start();
}

/**
 * Given a certain percentage of the maximum speed,
 * this method will return the correct value to set
 * using analogWrite().
 */
static int getPWMValue(int speed) {
	return (255 * speed) / 100;
}

void Motor::forward(int speed) {
	int pwm = getPWMValue(speed);
	digitalWrite(this->rev, LOW); // FIXME: needed?
	analogWrite(this->fwd, pwm);
}

void Motor::backward(int speed) {
	int pwm = getPWMValue(speed);
	digitalWrite(this->fwd, LOW); // FIXME: needed?
	analogWrite(this->rev, pwm);
}

void Motor::start() {
	digitalWrite(this->en, HIGH);
}

void Motor::stop() {
	digitalWrite(this->en, LOW);
}
