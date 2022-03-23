#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(48, 53); // CE, CSN


#include <NMEAGPS.h>
#include <GPSport.h>
#include <Adafruit_BNO08x.h>
#define BNO08X_RESET -1
#define NEOGPS_USE_SERIAL1

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


#define USIters 2

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
int waited;
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
    while (1) { delay(10); }
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
  if (mSpeed >= 0) {
    analogWrite(motorL, mSpeed * 255);
  }
}

void writeR(double mSpeed) {
  Serial.print(mSpeed);
  Serial.println();
  if (mSpeed >= 0) {
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

bool navUS() {
  durationL = 0;
  waited = 0;
  radio.openWritingPipe(pipe);
  radio.stopListening();
  ping = true;
  for (int i = 0; i < USIters; i++) {
    radio.write(&ping, sizeof(ping));
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    temp = pulseIn(echoPinL, HIGH);
    if (temp > 30000 && waited < USIters * 5) {
      i--;
      waited++;
    }
    else if (temp > 30000) {
      durationL = USIters * 40000;
      break;
    }
    else {
      durationL += temp;
    }
    delay(57);
  }
  durationL /= USIters;
  distanceL = msToIn(durationL);
  Serial.print(durationL);
  Serial.print(" L ");
  Serial.println(distanceL);

  durationR = 0;
  waited = 0;
  for (int i = 0; i < USIters; i++) {
    radio.write(&ping, sizeof(ping));
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
    temp = pulseIn(echoPinR, HIGH);
    if (temp > 30000 && waited < USIters * 5) {
      i--;
      waited++;
    }
    else if (temp > 30000) {
      durationR = USIters * 40000;
      break;
    }
    else {
      durationR += temp;
    }
    delay(57);
  }
  durationR /= USIters;
  distanceR = msToIn(durationR);
  Serial.print(durationR);
  Serial.print(" R ");
  Serial.println(distanceR);

  ping = false;
  radio.write(&ping, sizeof(ping));
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  if (durationL > 30000 && durationR > 30000) {
    Serial.println("Out of range!");
    return false;
  }
  else if (durationR > 30000) {
    writeL(-1);
    writeR(1);
    Serial.println("RS is lost!");
  }
  else if (durationL > 30000) {
    writeL(1);
    writeR(-1);
    Serial.println("LS is lost!");
  }
  else {
    writeL((double) (distanceL - 50) / 50.0 + (double) (distanceL - distanceR) / 40.0);
    writeR((double) (distanceR - 50) / 50.0 + (double) (distanceR - distanceL) / 40.0);
    //writeL(distanceL * 2 - distanceR);
    //writeR(distanceR * 2 - distanceL);
    Serial.println("Good!");
  }
  delay(100);
  return true;
}

void navGPS() {
  Serial.println("US Failed, Using GPS");
  NeoGPS::Location_t target ((long)x * 10000000L, (long)y * 10000000L);
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
  Serial.println(ypr.yaw);
  myAngle = ypr.yaw;
  angle = myAngle - fix.location.BearingToDegrees(target); //!! Check clockwise vs counter
  if (distance < 10) { //Within 10 ft, just rotate
    writeL(angle * 0.05); //! Calibrate turn speed
    writeR(-angle * 0.05);
  }
  else if (distance < 50) { //Within 50 ft, move to it
    writeL(angle * 0.03 + .5); //! Calibrate
    writeR(-angle * 0.03 + .5);
  }
  else { //Out of 50 ft, consider itself lost
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
      if (!navUS()) { //Try Ultrasonic
        navGPS(); //If no US reception, use GPS
      }
    }
    else {
      writeL(0);
      writeR(0);
    }
  }
  while (gps.available(gpsPort)) { //UPDATE GPS !!!
    gps_fix fix = gps.read();
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
