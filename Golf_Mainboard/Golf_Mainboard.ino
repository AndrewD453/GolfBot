#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "GyroMath.h"
#include "GPSMath.h"
#include "Motors.h"

RF24 radio(48, 53); // CE, CSN

const uint64_t pipe = 0xE6E6E6E6E6E6;
const byte address[6] = "00001";

char c, mode;
float x, y;
float v, w;
bool ping = true;

void setup() {
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

void navManual() {
  x = map(x, 0, 1023, 100, -100);
  y = map(y, 0, 1023, -100, 100);
  v = (100 - abs(x)) * (y / 100) + y;
  w = (100 - abs(y)) * (x / 100) + x;
  writeL((v - w) / 200.0); //LEFT ---- Algorithm: https://home.kendra.com/mauser/Joystick.html
  writeR((v + w) / 200.0); //RIGHT
}

void navGPS() {
  readGPS();
  target = NeoGPS::Location_t((long)x * 10000000L, (long)y * 10000000L); //Check this line
  distance = (float) fix.location.DistanceMiles(target) / 5280.0f;
  readIMU();
  Serial.print("My angle: ");
  Serial.println(ypr.yaw);
  myAngle = ypr.yaw;
  angle = myAngle - fix.location.BearingToDegrees(target); //!! Check clockwise vs counter
  angle = (int)(myAngle - fix.location.BearingToDegrees(target) + 720) % 360;
  if (angle > 180) {
    angle -= 360;
  }
  Serial.print("Offset angle: ");
  Serial.print(angle);
  Serial.print("   Distance: ");
  Serial.println(distance);
  Serial.println(fix.dateTime.seconds);
  if (distance < 2) { //Within 10 ft, just rotate
    writeL(0); //! Calibrate turn speed
    writeR(0);
  }
  else if (distance < 5) { //Within 5 ft, just rotate
    writeL(angle / 90.0); //! Calibrate turn speed
    writeR(-angle / 90.0);
  }
  else if (distance < 50) { //Within 50 ft, move to target
    writeL(angle / 45.0 + .5); //! Calibrate
    writeR(-angle / 45.0 + .5);
  }
  else { //Out of 50 ft, consider itself lost
    Serial.println("Out of range");
    writeL(0);
    writeR(0);
  }
}

void readRadio() {
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
}

void loop() {
  if (radio.available()) {
    readRadio();
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
  readGPS();
  readIMU();
  delay(20);
}
