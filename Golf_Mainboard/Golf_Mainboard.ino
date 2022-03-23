#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(48, 53); // CE, CSN


#include <NMEAGPS.h>
#define NEOGPS_USE_SERIAL1
#include <GPSport.h>
#include <Adafruit_BNO08x.h>
#define BNO08X_RESET -1

NMEAGPS gps;
gps_fix fix;
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

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

float distance;
float myAngle;
float angle;


#define USIters 1
#define USPULSES 10

const int trigPinL = 35;
const int echoPinL = 34;
const int trigPinR = 32;
const int echoPinR = 33;
const int motorL = 7;
const int motorR = 6;
const uint64_t pipe = 0xE6E6E6E6E6E6;
const byte address[6] = "00001";

char c, mode;
float x, y;
float v, w;
int waitedL, waitedR;
int LIters, RIters;
long startT;
long temp;
long durationL, durationR;
long distanceL, distanceR;
bool ping = true;

void setup() {
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  Serial.begin(9600);
  gpsPort.begin(9600);
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  setReports(reportType, reportIntervalUs);
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

long msToIn(long ms) {
  return ms / 74/* / 2*/;
}

long usToIn(long us) {
  return us * 1125L / 1000000L;
}

double getAngle(double x, double y) {
  return (map(atan2(y, x) * 180 / PI, 180, -180, 0, 360) + 90) % 360;
}

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

void writeL(double mSpeed) {
  Serial.print(mSpeed);
  Serial.print(", ");
  if (mSpeed >= 1) {
    analogWrite(motorL, 255);
  }
  else if (mSpeed >= 0) {
    analogWrite(motorL, mSpeed * 255);
  }
}

void writeR(double mSpeed) {
  Serial.print(mSpeed);
  Serial.println();
  if (mSpeed >= 1) {
    analogWrite(motorR, 255);
  }
  else if (mSpeed >= 0) {
    analogWrite(motorR, mSpeed * 255);
  }
}

void navManual() {
  x = map(x, 0, 1023, 100, -100);
  y = map(y, 0, 1023, -100, 100);
  v = (100 - abs(x)) * (y / 100) + y;
  w = (100 - abs(y)) * (x / 100) + x;
  writeL((v - w) / 200.0); //LEFT ---- Algorithm: https://home.kendra.com/mauser/Joystick.html
  writeR((v + w) / 200.0); //RIGHT
}

NeoGPS::Location_t target ((long)x * 10000000L, (long)y * 10000000L);

void navGPS() { 
  distance = (float) fix.location.DistanceMiles(target) / 5280.0f;
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
  }
  Serial.print("My angle: ");
  Serial.println(ypr.yaw);
  myAngle = ypr.yaw;
  angle = myAngle - fix.location.BearingToDegrees(target); //!! Check clockwise vs counter
  Serial.print("Offset angle: ");
  Serial.print(angle);
  Serial.print("   Distance: ");
  Serial.println(distance);
  Serial.println(fix.dateTime.seconds);
  if (distance < 2) { //Within 10 ft, just rotate
    writeL(0); //! Calibrate turn speed
    writeR(0);
  }
  else if (distance < 10) { //Within 10 ft, just rotate
    writeL(angle * 0.05); //! Calibrate turn speed
    writeR(-angle * 0.05);
  }
  else if (distance < 50) { //Within 50 ft, move to it
    writeL(angle * 0.03 + .5); //! Calibrate
    writeR(-angle * 0.03 + .5);
  }
  else { //Out of 50 ft, consider itself lost
    Serial.println("Out of range");
    writeL(0);
    writeR(0);
  }
}

void loop() {
  if (radio.available()) {
    radio.read(&mode, sizeof(mode)); //Receive RF Message
    delay(10);
    while (c != '(') {
      radio.read(&c, sizeof(c));
      delay(10);
    }
    delay(10);
    radio.read(&x, sizeof(x));
    delay(10);
    while (c != ',') {
      radio.read(&c, sizeof(c));
      delay(10);
    }
    delay(10);
    radio.read(&y, sizeof(y));
    while (c != ')') {
      radio.read(&c, sizeof(c));
      delay(10);
    }
    Serial.print(mode);
    Serial.print("\t");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);

    if (mode == 'J') { //Joystick Mode
      navManual();
    }
    else if (mode == 'F') {
      navGPS();
    }
    else {
      writeL(0);
      writeR(0);
    }
  }
  while (gps.available(gpsPort)) { //UPDATE GPS !!!
    fix = gps.read();
  }
  if (bno08x.wasReset()) {
    Serial.print("IMU was reset ");
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
  delay(10);
}
