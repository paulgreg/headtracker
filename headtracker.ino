#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <LSM303.h>

// Uncommenting following line will enable debugging output
//#define DEBUG

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

LSM303 compass;

const int ledPin = 11;

int inityaw;  // Store initial heading
float yaw = 0.0, pitch = 0.0;

sensors_event_t accel_event;
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

  Wire.begin();

  if (!accel.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
#ifdef DEBUG
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
#endif
    fail();
  }

  compass.init();
  compass.enableDefault();
  
  // Calibration values. Use the Calibrate example program to get the values for your compass.
  compass.m_min.x = -517; compass.m_min.y = -554; compass.m_min.z = -527;
  compass.m_max.x = +501; compass.m_max.y = +457; compass.m_max.z = 320;


  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);

  compass.read();
  yaw = compass.heading((LSM303::vector){0.0,-1.0,0.0});
  inityaw = yaw;  // Grab an initial heading
    if (inityaw < 0) {
      inityaw += 360;
    }
  delay(500);

  digitalWrite(ledPin, HIGH);
}

void loop(void) {

  accel.getEvent(&accel_event); // Calculate pitch and roll from the raw accelerometer data
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    pitch = orientation.pitch;
  }

  compass.read();
  yaw = compass.heading((LSM303::vector){0.0,-1.0,0.0});

  yaw = normalize(yaw - inityaw, 90);
  Joystick.X(yaw);

  pitch = normalize(pitch, 90);
  Joystick.Y(pitch);

#ifdef DEBUG
  Serial.print(F("Yaw: "));
  Serial.print(yaw);
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(pitch);
  Serial.print(F("; "));
  Serial.println(F(""));
#endif

#ifdef DEBUG
  delay(250);
#else
  delay(10);
#endif
}

float normalize(float value, float angle) {
  float v = value;

  if (v >= 0) {
    v = fscale(0, angle, 512, 1023, v, 0);
  } else {
    v = fscale(-angle, 0, 0, 512, v, 0);
  }
  if (v < 0) v = 0;
  if (v > 1023) v = 1023;

  return v;
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
