/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

Motor::Motor(int fwd, int rev, int en) : fwd(fwd), rev(rev), en(en) {
	pinMode(fwd, OUTPUT);
	pinMode(rev, OUTPUT);
	pinMode(en, OUTPUT);
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
	analogWrite(this->rev, 0); // FIXME: needed?
	analogWrite(this->fwd, pwm);
}

void Motor::backward(int speed) {
	int pwm = getPWMValue(speed);
	analogWrite(this->fwd, 0); // FIXME: needed?
	analogWrite(this->rev, pwm);
}

void Motor::stop() {
	// FIXME: ENABLE pin?
	analogWrite(this->fwd, 0);
	analogWrite(this->rev, 0);
}
