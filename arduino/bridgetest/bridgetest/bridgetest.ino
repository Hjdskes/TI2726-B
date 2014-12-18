int fwd1 = 6;

void setup() {
  Serial.begin(9600);

  pinMode(fwd1, OUTPUT); 
}

void loop() {
  digitalWrite(fwd1, HIGH);
  delay(2000);
  digitalWrite(fwd1, LOW);
  delay(2000);
}

