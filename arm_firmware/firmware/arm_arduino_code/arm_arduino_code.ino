#include <Servo.h>

#define WAIST_PIN 11
#define SHOULDER_PIN 9
#define ELBOW_PIN 10
#define WRIST_PIN 6
#define WRIST_TWIST_PIN 5
#define TOOL_PIN 3

#define WAIST_HOME 0
#define SHOULDER_HOME 0
#define ELBOW_HOME 0
#define WRIST_HOME 0
#define WRIST_TWIST_HOME 0
#define TOOL_TIP_HOME 0

Servo waist;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo wrist_twist;
Servo tool_tip;

char joint;
int value;

void setup() {
  waist.attach(WAIST_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);
  wrist.attach(WRIST_PIN);
  wrist_twist.attach(WRIST_TWIST_PIN);
  tool_tip.attach(TOOL_PIN);

  waist.write(WAIST_HOME);
  shoulder.write(SHOULDER_HOME);
  elbow.write(ELBOW_HOME);
  wrist.write(WRIST_HOME);
  wrist_twist.write(WRIST_TWIST_HOME);
  tool_tip.write(TOOL_TIP_HOME);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  while (Serial.available()) {
    String msg = Serial.readStringUntil(',');

    // Extract joint identifier and value from the message
    joint = msg.charAt(0);
    value = msg.substring(1).toInt();
  }

  // Process the final command received
  switch (joint) {
    case 'b':
      moveServo(waist, value);
      break;
    case 's':
      moveServo(shoulder, value);
      break;
    case 'e':
      moveServo(elbow, value);
      break;
    case 'w':
      moveServo(wrist, value);
      break;
    case 't':
      moveServo(wrist_twist, value);
      break;
    case 'g':
      moveServo(tool_tip, value);
      break;
    default:
      // Invalid joint identifier
      break;
  }
}

void moveServo(Servo& servo, int goal) {
  int currentPos = servo.read();
  if (goal != currentPos) {
    if (goal > currentPos) {
      for (int pos = currentPos; pos <= goal; pos++) {
        servo.write(pos);
        delayMicroseconds(1); // Adjust speed if needed
      }
    } else {
      for (int pos = currentPos; pos >= goal; pos--) {
        servo.write(pos);
        delayMicroseconds(1); // Adjust speed if needed
      }
    }
  }
}
