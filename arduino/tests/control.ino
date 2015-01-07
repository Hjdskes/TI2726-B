#include <ros.h>
#include <std_msgs/String.h>

class NewHardware : 
public ArduinoHardware { 
public: 
  NewHardware() : 
  ArduinoHardware(&Serial1, 57600) {
  };
};

ros::NodeHandle_<NewHardware> nh;

int FWD = 0;
int REV = 1;
int motor_left[] = {
  6, 7};
int motor_right[] = {
  2, 3};

void messageCb(const std_msgs::String& msg) {
  if (msg.data == "fwd") {
    drive_forward();
  }
}

ros::Subscriber<std_msgs::String> sub("direction", &messageCb);

void setup() {
  Serial.begin(9600);

  // Setup motors
  for(int i = 0; i < 2; i++) {
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(500);
}

void motor_stop() {
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], LOW);
  delay(25);
}

void drive_forward() {
  digitalWrite(motor_left[FWD], HIGH);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], HIGH);
  digitalWrite(motor_right[REV], LOW);
}

void drive_backward() {
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], HIGH);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], HIGH);
}

void turn_left() {
  digitalWrite(motor_left[FWD], LOW);
  digitalWrite(motor_left[REV], HIGH);

  digitalWrite(motor_right[FWD], HIGH);
  digitalWrite(motor_right[REV], LOW);
}

void turn_right() {
  digitalWrite(motor_left[FWD], HIGH);
  digitalWrite(motor_left[REV], LOW);

  digitalWrite(motor_right[FWD], LOW);
  digitalWrite(motor_right[REV], HIGH);
}


