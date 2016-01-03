#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

// Uncommenting following line will enable debugging output
#define DEBUG

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

const int ledPin = 11;

int inityaw;  // Store initial heading

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

void fail(void) {
  while (1) {
    digitalWrite(ledPin, HIGH);
    delay(250);
    digitalWrite(ledPin, LOW);
    delay(250);
  }

}

void setup(void) {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("Init"));
#endif

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  if (!accel.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
#ifdef DEBUG
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
#endif
    fail();
  }

  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
#ifdef DEBUG
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
#endif
    fail();
  }

  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    inityaw = orientation.heading;  // Grab an initial heading
    if (inityaw < 0) {
      inityaw += 360;
    }
  }
  delay(500);

  digitalWrite(ledPin, HIGH);
}

void loop(void) {
  float pitch = 0.0, roll = 0.0, yaw = 0.0;

  accel.getEvent(&accel_event); // Calculate pitch and roll from the raw accelerometer data
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    roll = orientation.roll;
    pitch = orientation.pitch;
  }
  
  mag.getEvent(&mag_event); // Calculate the heading using the magnetometer
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    yaw = orientation.heading;
  }

  if (pitch >= 0) {
    pitch -= 90;
    pitch = fscale(0, 90, 1023, 512, pitch, 0);
  } else {
    pitch += 90;
    pitch = fscale(-90, 0, 512, 0, pitch, 0);
  }
  if (pitch < 0) {
    pitch = 0;
  }
  if (pitch > 1023) {
    pitch = 1023;
  }
  Joystick.Y(pitch);
/*
   yaw = yaw - inityaw;
   if (yaw > 180) {
    yaw = -360 + yaw;
  }
  else if (yaw < -180) {
    yaw = 360 + yaw;
  }
  if (yaw < 0) {
    yaw = fscale(-40, 0, 0, 512, yaw, 0);
  }
  else {
    yaw = fscale(0, 40, 512, 1023, yaw, 0);
  }
  if (yaw < 0) {
    yaw = 0;
  }
  if (yaw > 1023) {
    yaw = 1023;
  }
  Joystick.X(yaw);


  if (roll < 0) {
    roll = fscale(-25, 0, 0, 512, roll, 0);
  }
  else {
    roll = fscale(0, 25, 512, 1023, roll, 0);
  }
  if (roll < 0) {
    roll = 0;
  }
  if (roll > 1023) {
    roll = 1023;
  }
  Joystick.Z(roll);
*/
#ifdef DEBUG
  Serial.print(F("Yaw: "));
  Serial.print(yaw);
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(pitch);
  Serial.print(F("; "));
  Serial.print(F("Roll: "));
  Serial.print(roll);
  Serial.print(F("; "));
  Serial.println(F(""));
#endif

#ifdef DEBUG
  delay(250);
#else
  delay(10);
#endif
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  } else { // invert the ranges
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}
