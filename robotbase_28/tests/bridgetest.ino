/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

const int FWD1 = 6;

void setup() {
	pinMode(FWD1, OUTPUT); 
}

void loop() {
	digitalWrite(FWD1, HIGH);
	delay(2000);
	digitalWrite(FWD1, LOW);
	delay(2000);
}

