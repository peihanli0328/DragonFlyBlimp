
//////// Define const ////////////
// #define SERVOMIN  150
// #define SERVOMAX  600
#define SERVOMIN 100
#define SERVOMAX 445
#define SERVOMID 275
#define SERVO_FREQ 50
#define SERVO_R 3
#define SERVO_L 7
#define CATCHSPEED 200
#define HOLDSPEED 150


int capAt255(int value, int motorSpeed) {
  if (abs(value) > motorSpeed) {
    return (value > 0) ? motorSpeed : -motorSpeed;
  }
  return value;
}




void motorSetup() {
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
                        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotorL->setSpeed(50);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(50);
  myMotorR->run(FORWARD);
  // turn on motor
  myMotorL->run(RELEASE);
  myMotorR->run(RELEASE);

  Serial.println("Motor Setup Completed!");
}

void servoSetup() {
  // Serial.begin(115200);
  Serial.println("2 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(SERVO_1, 0, SERVOMID);
  pwm.setPWM(SERVO_2, 0, SERVOMID);
  Serial.println("Servo setup completed!");
  delay(10);
}
int LM = 0;
int RM = 0;

void motorControl(int throttlePress, int flag, int yaw) {
  int motorSpeed = map(throttlePress, 0, 100, 0, 255);

  // yaw = capAt255(yaw, 255);
  // Serial.print("yaw:");
  // Serial.println(yaw);
  switch (flag) {
    case 0:  // stop
      myMotorL->run(FORWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(0);
      myMotorR->setSpeed(0);
      break;
    case 1:  // go up
      LM = capAt255(motorSpeed, 255);
      RM = capAt255(motorSpeed, 255);
      myMotorL->run(BACKWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(LM);
      myMotorR->setSpeed(RM);
      break;
    case 2:  //DOWN
      LM = capAt255(motorSpeed, 255);
      RM = capAt255(motorSpeed, 255);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
      myMotorL->setSpeed(LM);
      myMotorR->setSpeed(RM);
      break;
    case 3:  // Left
      myMotorL->run(BACKWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(motorSpeed);
      myMotorR->setSpeed(motorSpeed);
      break;
    case 4:  //Right
      myMotorL->run(BACKWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(motorSpeed);
      myMotorR->setSpeed(motorSpeed);
      break;
    case 5:  // forward
      LM = capAt255(motorSpeed - yaw, 255);
      RM = capAt255(motorSpeed + yaw, 255);
      // int motorSpeed = map(throttlePress, 0, 100, 0, 255);
      myMotorL->run(BACKWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(LM);
      myMotorR->setSpeed(RM);
      break;
    case 6:  // Backward
      LM = capAt255(motorSpeed - yaw, 255);
      RM = capAt255(motorSpeed + yaw, 255);
      // int motorSpeed = map(throttlePress, 0, 100, 0, 255);
      myMotorL->run(BACKWARD);
      myMotorR->run(BACKWARD);
      myMotorL->setSpeed(LM);
      myMotorR->setSpeed(RM);
      break;
  }
  // if(flag = 0){ //forward
  // int motorSpeed = map(throttlePress, 0, 100, 0, 255);
  // // if (motorSpeed < 60 || throttlePress == 0) {
  // //   motorSpeed = 0;
  // // }
  // myMotorL->setSpeed(motorSpeed);
  // myMotorR->setSpeed(motorSpeed);
  // }else if(flag = 1){ //backword

  // }
  // if (throttlePress < 50) {
  //   throttlePress = 0;
  // }
  // int motorSpeed = map(throttlePress, 0, 100, 0, 255);
  // if (motorSpeed < 60 || throttlePress == 0) {
  //   motorSpeed = 0;
  // }
  // myMotorL->setSpeed(motorSpeed);
  // myMotorR->setSpeed(motorSpeed);
  // }
}

// void setThrustDir(bool leftDirection, bool rightDirection) {
//   // set left motor thrust direction based on the flag
//   if (leftDirection) {
//     myMotorL->run(FORWARD);
//     // myMotorL->setSpeed(100);
//     // Serial.printf("FORWARD\n");
//     // delay(100);
//   } else {
//     myMotorL->run(BACKWARD);
//     // Serial.printf("BACKWARD\n");
//     delay(100);
//     // myMotorL->setSpeed(100);
//   }
//   // set right motor thrust direction based on the flag
//   if (rightDirection) {
//     myMotorR->run(BACKWARD);
//     // delay(100);
//     // myMotorR->setSpeed(100);
//   } else {
//     myMotorR->run(FORWARD);
//     // delay(100);
//     // myMotorR->setSpeed(100);
//   }
// }

int servoAngle;
// Arduino setup function. Runs in CPU 1

void updateServoAngles(int value) {

  // Neutral position (90 deg) when left joystick in middle
  if (abs(value) < 10) {
    servoAngle = 90;
  }
  // Pointing up (180 deg) when left joystick pushed up (-511 - 0)
  else if (value <= -10) {
    servoAngle = map(abs(value), 0, 100, 90, 180);
  }
  // Pointing down (0 deg) when left joystick pushed down (0 - 512)
  else if (value >= 10) {
    servoAngle = map(value, 0, 100, 90, 0);
  }
  pwm.setPWM(SERVO_R, 0, map(servoAngle, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(SERVO_L, 0, map(servoAngle, 0, 180, SERVOMIN, SERVOMAX));
  // delay(10);
}
void updateServoAngles_fw(int value) {

  // Neutral position (90 deg) when left joystick in middle
  if (abs(value) < 10) {
    servoAngle = 90;
  }
  // Pointing up (180 deg) when left joystick pushed up (-511 - 0)
  else if (value <= -10) {
    servoAngle = map(abs(value), 0, 100, 90, 180);
  }
  // Pointing down (0 deg) when left joystick pushed down (0 - 512)
  else if (value >= 10) {
    servoAngle = map(value, 0, 100, 90, 0);
  }
  pwm.setPWM(SERVO_R, 0, map(servoAngle, 180, 0, SERVOMIN, SERVOMAX));
  pwm.setPWM(SERVO_L, 0, map(servoAngle, 0, 180, SERVOMIN, SERVOMAX));
  // delay(10);
}
