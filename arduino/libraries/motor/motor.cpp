/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include "motor.h"
#include "Arduino.h"

static const uint8_t MAX_DUTY_CYCLE = 255;

Motor::Motor(const int fwd, const int rev, const int en) : fwd(fwd), rev(rev), en(en) {
	pinMode(fwd, OUTPUT);
	pinMode(rev, OUTPUT);
	pinMode(en, OUTPUT);
	this->start();
}

/**
 * Given a certain percentage of the maximum speed,
 * this method will return the correct value to set
 * using analogWrite(). Enable possible optimization
 * for the compiler, since this function is so small.
 */
inline int getPWMValue(const int speed) {
	return (MAX_DUTY_CYCLE * speed) / 100;
}

static int capSpeed(const int speed) {
	if (speed > 100) {
		return 100;
	} else if (speed < 0) {
		return 0;
	}
	return speed;
}

void Motor::forward(const int speed) {
	int pwm = getPWMValue(capSpeed(speed));
	digitalWrite(this->rev, LOW);
	analogWrite(this->fwd, pwm);
}

void Motor::backward(const int speed) {
	int pwm = getPWMValue(capSpeed(speed));
	digitalWrite(this->fwd, LOW);
	analogWrite(this->rev, pwm);
}

void Motor::start() {
	digitalWrite(this->en, HIGH);
}

void Motor::stop() {
	digitalWrite(this->en, LOW);
}

bool Motor::isStopped() {
	return digitalRead(this->en) == LOW;
}
