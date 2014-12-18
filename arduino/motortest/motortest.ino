// Use this code to test your motor with the Arduino board:

// if you need PWM, just use the PWM outputs on the Arduino
// and instead of digitalWrite, you should use the analogWrite command

// —————————————————————————  Motors
int FWD = 0;
int REV = 1;
int motor_left[] = {6, 7};
int motor_right[] = {2, 3};

// ————————————————————————— Setup
void setup() {
  Serial.begin(9600);

  // Setup motors
  for(int i = 0; i < 2; i++){
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }
}

// ————————————————————————— Loop
void loop() {

  drive_forward();
  delay(1000);
  motor_stop();

  drive_backward();
  delay(1000);
  motor_stop();

  turn_left();
  delay(1000);
  motor_stop();

  turn_right();
  delay(1000);
  motor_stop();

  motor_stop();
  delay(1000);
  motor_stop();

  delay(2000);                  // wait for a second
}

// ————————————————————————— Drive

void motor_stop(){
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], LOW);
  delay(25);
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

