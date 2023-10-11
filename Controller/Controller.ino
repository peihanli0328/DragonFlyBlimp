#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>



// const int numServos = 2;
// int servoMin = 0;  // Minimum servo position
// int servoMax = 255;  // Maximum servo position
// int servoPins[numServos] = {0, 7};  // Corresponding channels on the PWM board

//////// Define objects from packages//////////
// Controller (GamePad)
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
// Servo PWM
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Motor I2C
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorL = AFMS.getMotor(3);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(4);


//////// Define const ////////////
// #define SERVOMIN  150
// #define SERVOMAX  600
#define SERVOMIN  100
#define SERVOMAX  450
#define SERVO_FREQ 50
#define SERVO_1   0
#define SERVO_2   4

int satrting_servoAngle = ((SERVOMAX-SERVOMIN)/2)+SERVOMIN; 

void motorSetup(){
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
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

void servoSetup(){
  // Serial.begin(115200);
  Serial.println("2 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(SERVO_1, 0,satrting_servoAngle);
  pwm.setPWM(SERVO_2, 0,satrting_servoAngle);
  Serial.println("Servo setup completed!");
  delay(10);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

/**
Function for control motor when given a throttle value from the controller

Input:
    int   throttlePress --> 0 - 1023, reading of the throttle press from the controller
    bool  isForward       --> 1 / 0, indicates if the press if from throttle or brake(reverse)
*/
void motorControl(int throttlePress) {
  // if (isForward){
  //   myMotorL->run(FORWARD);
  //   myMotorR->run(BACKWARD);
  // }else{
  //   myMotorL->run(BACKWARD);
  //   myMotorR->run(FORWARD);
  // }
  // convert the throttle press (0 - 1023) to actual motor speed (0 - 255)
  int motorSpeed = map(throttlePress, 0, 1023, 0, 255);
  Serial.printf("motor:%3d\n",motorSpeed);
  myMotorL->setSpeed(motorSpeed);
  myMotorR->setSpeed(motorSpeed);
  myMotorL->run(RELEASE);
  myMotorR->run(RELEASE);
}

void setThrustDir(bool leftDirection, bool rightDirection){
  // set left motor thrust direction based on the flag
  if (leftDirection){
    myMotorL->run(FORWARD);
  }
  else{
    myMotorL->run(BACKWARD);
  }
  // set right motor thrust direction based on the flag
  if (rightDirection){
    myMotorR->run(BACKWARD);
  }
  else{
    myMotorR->run(FORWARD);
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // motorSetup();
  servoSetup();

}

// Arduino loop function. Runs in CPU 1
void loop() {
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];
    if (myGamepad && myGamepad->isConnected()) {

      
      ////////////// Servo control /////////////////////////
      // for (int angle = 0; angle <= 180; angle++) {
      //   pwm.setPWM(0, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
      //   pwm.setPWM(4, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
      //   Serial.printf("Angle: %3d\n", angle);
      //   delay(15); 
      // }

      // for (int angle = 180; angle >= 0; angle--) {
      //   pwm.setPWM(0, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
      //   pwm.setPWM(4, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
      //   Serial.printf("Angle: %3d\n", angle);
      //   delay(15); 
      // }

      int leftJoystickY = myGamepad->axisY();   // range from -511 - 512
      // Serial.printf("\n", leftJoystickY);
      int servoAngle;
      // Neutral position (90 deg) when left joystick in middle
      if (abs(leftJoystickY) <100){
        servoAngle = 90;
      }
      // Pointing up (180 deg) when left joystick pushed up (-511 - 0)
      else if (leftJoystickY <= -100){
        servoAngle = 180;
      }
      // Pointing down (0 deg) when left joystick pushed down (0 - 512)
      else if (leftJoystickY >= 100){
        servoAngle = 0;
      }
      pwm.setPWM(SERVO_1, 0, map(servoAngle, 0, 180, SERVOMIN, SERVOMAX));
      pwm.setPWM(SERVO_2, 0, map(servoAngle, 180, 0, SERVOMIN, SERVOMAX));

      // Serial.printf("left joystick: %4d, servoAngle: %3d\n", leftJoystickY, servoAngle);
      ////////////////////////////////////////////////////

      ///////////// DC Motor Thrust Direction ////////////
      // int rightJoystickX = myGamepad->axisRX();
      // bool leftDirection;
      // bool rightDirection;
      // // Neutral Position;
      // if (abs(rightJoystickX) < 100){
      //   leftDirection = 1;
      //   rightDirection = 0;
      // }
      // // Left turn, right joystick pushed left (negative)
      // else if (rightJoystickX <= -100){
      //   leftDirection = 1;
      //   rightDirection = 1;
      // }
      // // Right turn, right joystick pushed right (position)
      // else if (rightJoystickX >= 100){
      //   leftDirection = 0;
      //   rightDirection = 0;
      // }
      /////////////////////////////////////////////////////

      ////////////// Throttle and Reverse control /////////////
      // // Both throttle and brake are pressed
      // if (myGamepad->throttle() > 10 && myGamepad->brake() > 10){
      //   Serial.println("Please do not press both triggers!");
      // } 
      // else if (myGamepad->throttle() > 10){
      //   // Throttle --> forward   (0 - 1023)
      //   setThrustDir(leftDirection, rightDirection);
      //   motorControl(myGamepad->throttle()); // isForward = 1
      // }
      // else if (myGamepad->brake() > 10){
      //   // Brake --> backward     (0 - 1023)
      //   // reverse the direction of thrust
      //   setThrustDir(!leftDirection, !rightDirection);
      //   motorControl(myGamepad->brake());    // isForward = 0
      // }
      /////////////////////////////////////////////////////////
    }
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
  // delay(150);
}
