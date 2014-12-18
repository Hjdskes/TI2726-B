int FWD = 0;
int REV = 1;
int motor_left[] = {
  6, 7};
int motor_right[] = {
  2, 3};

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 2; i++){
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == 'W' || inChar == 'w') {
      drive_forward();
    } else if (inChar == 'A' || inChar == 'a') {
      turn_left();
    } else if (inChar == 'S' || inChar == 's') {
      drive_backward();
    } else if (inChar == 'D' || inChar == 'd') {
      turn_right();
    }
    delay(500);
  } 
}

void drive_forward(){
  digitalWrite(motor_left[FWD], HIGH);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], HIGH);
  digitalWrite(motor_right[REV], LOW);
}

void drive_backward(){
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], HIGH);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], HIGH);
}

void turn_left(){
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], HIGH);

  digitalWrite(motor_right[FWD], HIGH);
  digitalWrite(motor_right[REV], LOW);
}

void turn_right(){
  digitalWrite(motor_left[FWD], HIGH);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], HIGH);
}
