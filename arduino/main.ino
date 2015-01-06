/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

Motor *motor;

void setup() {
	enum {
		FORWARD = 0;
		REVERSE,
		ENABLE
	};
	int LEFT_MOTOR[] = { 6, 7, 24 };
	int RIGHT_MOTOR[] = { 2, 3, 25 };

	Serial.begin(9600);

	/* Setup motors. */
	Engine left(LEFT_MOTOR[FORWARD], LEFT_MOTOR[REVERSE], LEFT_MOTOR[ENABLE]);
	Engine right(RIGHT_MOTOR[FORWARD], RIGHT_MOTOR[REVERSE], RIGHT_MOTOR[ENABLE]);
	motor = motor(&left, &right);
}

void loop() {
	motor.forward();
	delay(1000);
	motor.stop();

	motor.backward();
	delay(1000);
	motor.stop();

	motor.left();
	delay(1000);
	motor.stop();

	motor.right();
	delay(1000);
	motor.stop();

	delay(2000);
}

