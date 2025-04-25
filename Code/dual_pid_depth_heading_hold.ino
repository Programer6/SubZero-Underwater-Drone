#include <Wire.h>
#include 
#include 

//   Pins  
const int csPin = 12;            // Pressure sensor chip select
const int upThrusterPin1 = 8;    // Up/down thruster 1
const int upThrusterPin2 = 9;    // Up/down thruster 2
const int escLeft = 10;          // Yaw (left) thruster
const int escRight = 11;         // Yaw (right) thruster

//   PID Params for Depth 
float kpD = 0.7, kiD = 0.1, kdD = 0.15;
float targetDepth = 2.0;
float prevErrorD = 0, integralD = 0;

//  PID Params for Heading  
float kpH = 1.0, kiH = 0.1, kdH = 0.2;
float targetHeading = 0.0;
float prevErrorH = 0, integralH = 0;

// Thruster PWM limits
const int minThrusterPower = 1000;
const int maxThrusterPower = 2000;

//   Sensors  
ms5540c pressureSensor;
HMC5883L compass;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  digitalWrite(csPin, LOW);
  pressureSensor.begin();
  digitalWrite(csPin, HIGH);

  compass = HMC5883L();
  compass.SetScale(1.3);
  compass.SetMeasurementMode(Measurement_Continuous);

  pinMode(upThrusterPin1, OUTPUT);
  pinMode(upThrusterPin2, OUTPUT);
  pinMode(escLeft, OUTPUT);
  pinMode(escRight, OUTPUT);

  analogWrite(upThrusterPin1, minThrusterPower);
  analogWrite(upThrusterPin2, minThrusterPower);
  analogWrite(escLeft, minThrusterPower);
  analogWrite(escRight, minThrusterPower);

  Serial.println(F("Depth & Heading Hold PID Controller Ready"));
  Serial.println(F("Commands: SETD,KP,KI,KD,DEPTH | SETH,KP,KI,KD,HEADING"));
}

void loop() {
  //   Read Current Depth  
  digitalWrite(csPin, LOW);
  float currentDepth = getPressureInMeters();
  digitalWrite(csPin, HIGH);

  //   PID for Depth  
  float errorD = targetDepth - currentDepth;
  integralD += errorD * 0.5;
  float derivativeD = errorD - prevErrorD;
  float outputD = kpD * errorD + kiD * integralD + kdD * derivativeD;
  prevErrorD = errorD;
  int thrusterPower = constrain((int)(1500 + outputD * 200), minThrusterPower, maxThrusterPower);
  analogWrite(upThrusterPin1, thrusterPower);
  analogWrite(upThrusterPin2, thrusterPower);

  //   Read Current Heading  
  MagnetometerRaw raw = compass.ReadRawAxis();
  float currentHeading = atan2(raw.YAxis, raw.XAxis) * 180.0 / PI;
  if (currentHeading < 0) currentHeading += 360.0;

  //   PID for Heading  
  float errorH = targetHeading - currentHeading;
  // Handle wrap-around for shortest rotation
  if (errorH > 180) errorH -= 360;
  if (errorH < -180) errorH += 360;
  integralH += errorH * 0.5;
  float derivativeH = errorH - prevErrorH;
  float outputH = kpH * errorH + kiH * integralH + kdH * derivativeH;
  prevErrorH = errorH;
  // Map outputH to left/right thrusters for yaw
  int leftPWM = constrain((int)(1500 + outputH * 4), minThrusterPower, maxThrusterPower);
  int rightPWM = constrain((int)(1500 - outputH * 4), minThrusterPower, maxThrusterPower);
  analogWrite(escLeft, leftPWM);
  analogWrite(escRight, rightPWM);

  //   Telemetry Output  
  Serial.print("DEPTH:"); Serial.print(currentDepth, 2);
  Serial.print(", SETPOINT:"); Serial.print(targetDepth, 2);
  Serial.print(", HEADING:"); Serial.print(currentHeading, 1);
  Serial.print(", HDG_SET:"); Serial.print(targetHeading, 1);
  Serial.print(", KP_D:"); Serial.print(kpD, 2);
  Serial.print(", KI_D:"); Serial.print(kiD, 2);
  Serial.print(", KD_D:"); Serial.print(kdD, 2);
  Serial.print(", KP_H:"); Serial.print(kpH, 2);
  Serial.print(", KI_H:"); Serial.print(kiH, 2);
  Serial.print(", KD_H:"); Serial.print(kdH, 2);
  Serial.print(", PWM_UP:"); Serial.print(thrusterPower);
  Serial.print(", PWM_L:"); Serial.print(leftPWM);
  Serial.print(", PWM_R:"); Serial.println(rightPWM);

  //   Command Parsing for PID/Setpoints  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("SETD")) {
      float kpt, kit, kdt, tgt;
      int n = sscanf(cmd.c_str(), "SETD,%f,%f,%f,%f", &kpt, &kit, &kdt, &tgt);
      if (n == 4) {
        kpD = kpt; kiD = kit; kdD = kdt; targetDepth = tgt;
        Serial.print(F("Depth PID updated: ")); Serial.print(kpD); Serial.print(", ");
        Serial.print(kiD); Serial.print(", "); Serial.print(kdD);
        Serial.print(F(" | Depth setpoint: ")); Serial.println(targetDepth);
        integralD = 0;
      } else {
        Serial.println(F("Parse error. Format: SETD,KP,KI,KD,DEPTH"));
      }
    }
    else if (cmd.startsWith("SETH")) {
      float kpt, kit, kdt, tgt;
      int n = sscanf(cmd.c_str(), "SETH,%f,%f,%f,%f", &kpt, &kit, &kdt, &tgt);
      if (n == 4) {
        kpH = kpt; kiH = kit; kdH = kdt; targetHeading = tgt;
        Serial.print(F("Heading PID updated: ")); Serial.print(kpH); Serial.print(", ");
        Serial.print(kiH); Serial.print(", "); Serial.print(kdH);
        Serial.print(F(" | Heading setpoint: ")); Serial.println(targetHeading);
        integralH = 0;
      } else {
        Serial.println(F("Parse error. Format: SETH,KP,KI,KD,HEADING"));
      }
    }
  }
  delay(200);
}

float getPressureInMeters() {
  float pressure = pressureSensor.getPressure();
  return pressure / 9806.65; // freshwater
}