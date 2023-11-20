//////// Define const ////////////
#define SERVOMIN 150
// #define SERVOMAX  600
#define SERVOMIN 100
#define SERVOMAX 445
#define SERVOMID 275
#define SERVO_FREQ 50
#define SERVO_1 3
#define SERVO_2 7
#define CATCHSPEED 240
#define HOLDSPEED 180



SensorOutputs checkSensorsAndUpdate() {
  SensorOutputs outputs;
  sensors_event_t temp_event, pressure_event;
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
  }
  // while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
  //   delay(10);  // wait until there's something to read
  // }
  dps.getEvents(&temp_event, &pressure_event);
  // Assign the sensor data to the struct
  // outputs.current_yaw = ypr.yaw ;
  outputs.current_yaw = ypr.yaw;



  outputs.current_pressure = pressure_event.pressure;

  return outputs;
}




// void loop() {

int satrting_servoAngle = ((SERVOMAX - SERVOMIN) / 2) + SERVOMIN;
int pre_throttlePress = 0;

double goal_yaw_set = 0;
double goal_yaw = 0;
double goal_pressure_set = 0;
double goal_pressure = 0;
SensorOutputs sensorData;


int fowardpess = 0;
int catchState = 0;
int enableStopHold = 0;
// int stableValue = 0;     // the current stable value
int buttonState;  // the current reading from the input pin
// int lastButtonState = LOW;   // the previous reading from the input pin
int stableValue = 0;
int lastButtonState = LOW;           // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers



// Arduino loop function. Runs in CPU 1
void loop() {

  // delay(10);
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        char typeIdentifier = client.read();
        if (typeIdentifier == 'I') {
          String result = readInteger(client);
          processData(result, ptrRightJoystickX, ptrLeftJoystickY, ptrThrust, ptrBreak, ptrcatchs, ptrstable);
          if (stable != lastButtonState) {
            // reset the debouncing timer
            lastDebounceTime = millis();
          }


          // Update Sensor every 0.5 S
          unsigned long currentMillis = millis();
          // if (currentMillis - lastUpdate >= 500) {
          if (currentMillis - lastUpdate >= 250) {
            lastUpdate = currentMillis;
            sensorData = checkSensorsAndUpdate();
            float measuredvbat = analogReadMilliVolts(VBATPIN);
            measuredvbat *= 2;     // we divided by 2, so multiply back
            measuredvbat /= 1000;  // convert to volts!
            // Serial.print("VBat: " );
            if (measuredvbat < 3.3) {
              digitalWrite(led, HIGH);

            } else {
              digitalWrite(led, LOW);
            }
            // Serial.println(measuredvbat);
          }
          if ((millis() - lastDebounceTime) > debounceDelay) {
            // if the button state has changed:
            if (stable != buttonState) {
              buttonState = stable;

              // only toggle the stable value if the new button state is HIGH
              if (buttonState == HIGH) {
                stableValue = !stableValue;
              }
            }
          }
          lastButtonState = stable;
          if (leftJoystickY > 0) {  // go up
            // if (fowardpess == 0) {
            //   sensorData = checkSensorsAndUpdate();
            //   goal_yaw_set = sensorData.current_yaw;
            //   goal_pressure_set = (sensorData.current_pressure * 100.0);
            //   fowardpess = 1;
            // }
            // sensorData = checkSensorsAndUpdate();
            // int output_yaw = computePID_yaw(goal_yaw_set, sensorData.current_yaw);
            // if (output_yaw > 0) {
            // updateServoAngles(output_yaw / 3.0);
            motorControl(leftJoystickY, 1, 0);
            enableStopHold = 0;

          } else if (leftJoystickY < 0) {  // go down
            motorControl(abs(leftJoystickY), 2, 0);
            enableStopHold = 0;

            updateServoAngles_fw(0);        //
          } else if (rightJoystickX < 0) {  //left
            updateServoAngles((-rightJoystickX));
            motorControl(abs(rightJoystickX) / 1.5, 3, 0);
            enableStopHold = 0;
          } else if (rightJoystickX > 0) {  //right
            updateServoAngles((-rightJoystickX));
            motorControl(abs(rightJoystickX) / 1.5, 4, 0);
            enableStopHold = 0;
          } else if (Thrust > 0) {  // go forward
            if (fowardpess == 0) {
              sensorData = checkSensorsAndUpdate();
              goal_yaw_set = sensorData.current_yaw;
              fowardpess = 1;
            }
            // int output_yaw = 0;
            sensorData = checkSensorsAndUpdate();
            int output_yaw = computePID_yaw_fw(goal_yaw_set, sensorData.current_yaw);
            // // Serial.print("currentyaw:");
            // // Serial.println(sensorData.current_yaw);
            // // Serial.print("goalyaw:");
            // // Serial.println(goal_yaw_set);
            // // Serial.print("output_yaw:");
            // // Serial.println(output_yaw);
            updateServoAngles_fw(90);
            Serial.println("Thrusting");
            Serial.println(Thrust * 0.7);
            motorControl(Thrust * 0.8, 5, output_yaw);
            enableStopHold = 0;
          }

          else if (Break > 0) {  //backward
            updateServoAngles_fw(-90);
            motorControl(Break, 6, 0);
            enableStopHold = 0;
          }

          else if (stableValue == 1) {  //holding mode
            digitalWrite(led, HIGH);
            if (fowardpess == 0) {
              goal_pressure_set = 0;
              sensorData = checkSensorsAndUpdate();
              goal_yaw_set = sensorData.current_yaw;
              goal_pressure_set = (sensorData.current_pressure * 100.0);
              fowardpess = 1;
              previous_error_pressure = 0;
            }



            // onePixel.setPixelColor(0, 0, 0, 255);   //  Set pixel 0 to (r,g,b) color value
            // onePixel.show();
            enableStopHold = 0;

            sensorData = checkSensorsAndUpdate();

            int output_yaw = computePID_yaw(goal_yaw_set, sensorData.current_yaw);
            int goal_pressure = computePID_pressure(goal_pressure_set, (sensorData.current_pressure * 100.0));
            // goal_pressure = capAt255(goal_pressure * 3, 100);
            // Serial.print("stablemode:");
            // Serial.println(sensorData.current_yaw);
            // Serial.print("current_pressure:");
            // Serial.println(sensorData.current_pressure);
            // Serial.print("goal_yaw_set:");
            // Serial.println(goal_yaw_set);
            // Serial.print("output_yaw:");
            // Serial.println(output_yaw);
            Serial.print("sensorData.current_pressure");
            Serial.println(sensorData.current_pressure);
            Serial.print("goal_pressure:");
            Serial.println(goal_pressure);

            if (goal_pressure < -1) {
              updateServoAngles(0);
              Serial.print("Goal < -1: ");
              Serial.println(goal_pressure);
              motorControl(50, 2, 0);
            } else if (goal_pressure > 3) {
              // goal_pressure = 0;
              //  sensorData = checkSensorsAndUpdate();
              //   goal_pressure_set = (sensorData.current_pressure * 100.0);
              //  goal_pressure = computePID_yaw(goal_pressure_set, (sensorData.current_pressure * 100.0));
              Serial.print("goal > 0:  ");
              Serial.println(goal_pressure);
              motorControl(50, 1, 0);
            }
            // } else {
            //   if (output_yaw > 0) {
            //     updateServoAngles(output_yaw);
            //     Serial.print("Go left:");
            //     Serial.println(output_yaw);
            //     motorControl(output_yaw, 1, 0);
            //   } else if (output_yaw < 0) {
            //     Serial.print("Go right:");
            //     Serial.println("output_yaw");
            //     updateServoAngles(output_yaw);
            //     motorControl(output_yaw, 1, 0);
            //   }
            // }
            digitalWrite(led, LOW);
          } else if (Thrust >= 0 && Break >= 0 && rightJoystickX == 0 && stable == 0) {  //stop
            Serial.println("STOP");
            motorControl(0, 0, 0);
            updateServoAngles(0);
            // Serial.print("Stopped");
            fowardpess = 0;
            ptrprevious_error_yaw = 0;
            ptrprevious_error_pressure = 0;
            goal_yaw_set = 0;
            goal_pressure_set = 0;
            goal_pressure = 0;
            // onePixel.clear();

            ////////// New Change ////////////
            /*
            Attempting to add a PID controller to cancel out the yaw rate?
            */
            // if (enableStopHold == 0){
            //   // reset the values when stop commanding
            //   fowardpess = 0;
            //   ptrprevious_error_yaw = 0;
            //   ptrprevious_error_pressure = 0;
            //   // goal_yaw_set = 0;
            //   goal_pressure_set = 0;
            //   goal_pressure = 0;
            //   sensorData = checkSensorsAndUpdate();
            //   goal_yaw_set = sensorData.current_yaw;
            //   enableStopHold = 1;
            // }
            // int output_yaw = computePID_yaw(goal_yaw_set, sensorData.current_yaw);
            // if (output_yaw > 0) {
            //   updateServoAngles(output_yaw);
            //   Serial.print("Go left:");
            //   Serial.println(output_yaw);
            //   motorControl(0.7*output_yaw, 1, 0);
            // } else if (output_yaw < 0) {
            //   Serial.print("Go right:");
            //   Serial.println(output_yaw);
            //   updateServoAngles(output_yaw);
            //   motorControl(abs(0.7*output_yaw), 1, 0);
            // }


          }
          if (catchs == 1) {  //catch
            catchState = 1;
          }
          // Release
          if (catchs == 2) {
            catchState = 2;
          }
          // Neutral
          if (catchs == 0) {
            catchState = 0;
          }

          switch (catchState) {
            case 0:
              myMotorC->setSpeed(0);
              myMotorC->run(FORWARD);
              //  Serial.println("Neutral");
              break;

            case 1:

              if ((millis() - firstPress) < 5000) {
                myMotorC->setSpeed(CATCHSPEED);
                // Serial.println("High Catchs:");
              } else {
                myMotorC->setSpeed(HOLDSPEED);
                // Serial.println("HOLDSPEED:");
              }
              // myMotorC->setSpeed(CATCHSPEED);
              myMotorC->run(FORWARD);
              break;

            case 2:
              myMotorC->setSpeed(CATCHSPEED);
              myMotorC->run(BACKWARD);
              // Serial.println("Release");
              if ((millis() - firstPress) > 5000) {
                catchState = 0;
              }
              break;
          }
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
// if (fowardpess == 0) {
//   sensorData = checkSensorsAndUpdate();
//   goal_yaw_set = sensorData.current_yaw;
//   fowardpess = 1;
// }
// sensorData = checkSensorsAndUpdate();
// int output_yaw = computePID_yaw(goal_yaw_set, sensorData.current_yaw);
// Serial.print("currentyaw:");
// Serial.println(sensorData.current_yaw);
// Serial.print("goalyaw:");
// Serial.println(goal_yaw_set);
// Serial.print("output_yaw:");
// Serial.println(output_yaw);
// motorControl(Thrust, 1, output_yaw);

// sensorData = checkSensorsAndUpdate();
// Serial.print("currentyaw:");
// Serial.println(sensorData.current_yaw);
// goal_yaw_set = sensorData.current_yaw - rightJoystickX / 3.8;
// // }
// if (goal_yaw_set >= 360) {
//   goal_yaw_set -= 360;
// } else if (goal_yaw_set < 0) {
//   goal_yaw_set += 360;
// }
// int output_yaw = computePID_yaw(goal_yaw_set, sensorData.current_yaw);
// Serial.print("goal_yaw_set:");
// Serial.println(goal_yaw_set);
// Serial.print("output_yaw:");
// Serial.println(output_yaw);