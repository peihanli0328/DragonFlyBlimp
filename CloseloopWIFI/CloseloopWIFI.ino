// This example shows how to read temperature/pressure
// #include <Adafruit_TestBed.h>
// extern Adafruit_TestBed TB;
#define VBATPIN A13
// #define DEFAULT_I2C_PORT &Wire
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
// #include <Adafruit_NeoPixel.h>    //  Library that provides NeoPixel functions

// Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

// Set the IP address and port to match the server you're connecting to
IPAddress serverIP(192, 168, 50, 30);
const uint16_t serverPort = 10000;
const char* ssid = "ZhouLab";
const char* password = "ZhouRobotics917";
long firstPress;
// Create an instance of the WiFiServer class
WiFiServer server(serverPort);
// #define BP32_MAX_GAMEPADS 1
// GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
// Servo PWM
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Motor I2C
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorL = AFMS.getMotor(3);  // left motor
Adafruit_DCMotor *myMotorR = AFMS.getMotor(4);  // right motor
Adafruit_DCMotor *myMotorC = AFMS.getMotor(1);  // catcher motor
int rightJoystickX = 0;
int leftJoystickY = 0;
int* ptrRightJoystickX = &rightJoystickX;
int* ptrLeftJoystickY = &leftJoystickY;
int Thrust =0 ;
int* ptrThrust= &Thrust;
int Break = 0;
int* ptrBreak= &Break;
int catchs = 0;
int* ptrcatchs= &catchs;
int stable =0;
int* ptrstable = &stable;


int led = LED_BUILTIN;
// Global variables for the PID
double integral_yaw = 0;
double integral_pressure = 0;
double previous_error_yaw = 0;
double previous_error_pressure = 0;
double* ptrprevious_error_yaw= &previous_error_yaw;
double* ptrprevious_error_pressure= &previous_error_pressure;


unsigned long lastUpdate = 0; // Stores the last update time
const unsigned long updateInterval = 3000; // Update interval in milliseconds (500ms)

#define BNO08X_RESET -1
Adafruit_DPS310 dps;
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
struct SensorOutputs {
    double current_yaw;
    double current_pressure;
};


#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


// Can also use SPI!
#define DPS310_CS 10

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

