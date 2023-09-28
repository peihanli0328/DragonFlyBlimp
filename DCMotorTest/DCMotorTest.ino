#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M3 and M4. 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);

void setup() {
  Serial.begin(115200);           
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(50);
  myMotor1->run(FORWARD);
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(50);
  myMotor2->run(FORWARD);
  myMotor2->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.print("tick");

  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i);
    delay(10);
  }

  Serial.print("tock");

  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i);
    delay(10);
  }

  Serial.print("tech");
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  delay(1000);
}
