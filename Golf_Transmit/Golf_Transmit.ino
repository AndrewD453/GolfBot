#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 10); // CE, CSN


#include <NMEAGPS.h>
#include <GPSport.h>

NMEAGPS gps;

gps_fix fix;


const int trigPin = 4;
const int echoPin = 5;
const int RPin = A5;
const int GPin = A6;
const int BPin = A7;
const int LEDPin = 9;
const int buttonPin = 2;
const int xPin = A0;
const int yPin = A1;
const uint64_t pipe = 0xE6E6E6E6E6E6;
const byte address[6] = "00001";

long startT;
int mode = 0;
float x, y;
char modeC;
bool wasPressed = false;
bool ping;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  radio.begin(); // Start the NRF24L01
  radio.openWritingPipe(pipe);
  radio.stopListening();
  displayMode();
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

void loop() {
  if (digitalRead(buttonPin) == LOW && !wasPressed) {
    mode = (mode + 1) % 3;
    displayMode();
    wasPressed = true;
    Serial.println(mode);
  }
  if (digitalRead(buttonPin) == HIGH) {
    wasPressed = false;
  }
  if (mode == 0) { //Stopped
    x = 0;
    y = 0;
    modeC = 'S';
  }
  else if (mode == 1) { //Follow
    //x = fix.latitude();
    x = 20.0;
    //y = fix.longitude();
    y = 30.0;
    modeC = 'F';
  }
  else if (mode == 2) { //Joystick
    x = (float) analogRead(xPin);
    y = 1023.0f - (float) analogRead(yPin);
    modeC = 'J';
  }
  Serial.println(modeC);
  writeChar(modeC);
  writeChar('(');
  radio.write(&x, sizeof(x));
  writeChar(',');
  radio.write(&y, sizeof(y));
  writeChar(')');
  if (mode == 1) {
    ping = true;
    radio.openReadingPipe(1, pipe);
    radio.startListening();
    startT = millis();
    while (!radio.available() && millis() < startT + 2000) {
      delay(20);
    }
    if (millis() < startT + 2000) {
      while (ping == true) {
        if (radio.available()) {
          radio.read(&ping, sizeof(ping));
          digitalWrite(trigPin, HIGH);
          delayMicroseconds(10);
          digitalWrite(trigPin, LOW);
        }
        delay(20);
      }
    }
    radio.openWritingPipe(pipe);
    radio.stopListening();
  }
  while (gps.available(gpsPort)) { //UPDATE GPS (and figure out port) !!!
    gps_fix fix = gps.read();
  }
}
