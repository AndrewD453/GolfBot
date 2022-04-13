#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 10); // CE, CSN

#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <GPSport.h>

NMEAGPS gps;

gps_fix fix;

const int RPin = A5;
const int GPin = A6;
const int BPin = A7;
const int buttonPin = 3;
const int xPin = A0;
const int yPin = A1;
const uint64_t pipe = 0xE6E6E6E6E6E6;
const byte address[6] = "00001";

long startT;
volatile int mode = 1;
long x, y;
char modeC;
bool ping;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(RPin, OUTPUT);
  pinMode(GPin, OUTPUT);
  pinMode(BPin, OUTPUT);
  radio.begin(); // Start the NRF24L01
  radio.openWritingPipe(pipe);
  radio.stopListening();
  Serial.println("GPS Initializing...");
  gpsPort.begin(9600);
  while (!gps.available(gpsPort));
  Serial.println("GPS Worked");
  gps_fix fix = gps.read();
  attachInterrupt(digitalPinToInterrupt(buttonPin), nextMode, FALLING);
  //delay(5000);
  while (!gps.available(gpsPort));
  fix = gps.read();
}

void displayMode() { //Show mode on RGB LED
  //Colors: 1 = Follow = Blue       2 = Joystick = Green          3 = Manual = Red
  if (mode == 1) {
    digitalWrite(RPin, LOW);
    digitalWrite(GPin, LOW);
    digitalWrite(BPin, HIGH);
  }
  if (mode == 2) {
    digitalWrite(RPin, LOW);
    digitalWrite(GPin, HIGH);
    digitalWrite(BPin, LOW);
  }
  if (mode == 0) {
    digitalWrite(RPin, HIGH);
    digitalWrite(GPin, LOW);
    digitalWrite(BPin, LOW);
  }
}

void writeChar(char c) {
  radio.write(&c, sizeof(c));
}

void nextMode() {
  Serial.println("Next Mode");
  mode = (mode + 1) % 3;
  displayMode();
}

void loop() {
  // Set values to send
  if (mode == 0) { //Stopped
    x = 0;
    y = 0;
    modeC = 'S';
  }
  else if (mode == 1) { //Follow
    x = fix.latitudeL();
    y = fix.longitudeL();
    modeC = 'F';
  }
  else if (mode == 2) { //Joystick
    x = (long) analogRead(xPin);
    y = (long) (1023.0f - (float) analogRead(yPin));
    modeC = 'J';
  }
  Serial.print(modeC);
  Serial.print(" ");
  Serial.print(x);
  Serial.print(" ");
  Serial.println(y);

  //Send values
  writeChar(modeC);
  writeChar('(');
  radio.write(&x, sizeof(x));
  writeChar(',');
  radio.write(&y, sizeof(y));
  writeChar(')');
  
  while (gps.available(gpsPort)) { //UPDATE GPS (and figure out port) !!!
    fix = gps.read();
    Serial.println("GPS!");
    Serial.println(fix.latitude());
  }

  delay(100);
}
