#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(8, 10); // CE, CSN

#define iters 5

const int trigPinL = A0;
const int echoPinL = A1;
const int trigPinR = A2;
const int echoPinR = A3;
const int m1A = 4;
const int m1B = 5;
const int m1Spd = 6;
const int m2A = 7;
const int m2B = 2;
const int m2Spd = 3;
int waited;
long temp;
long durationL, durationR;
long distanceL, distanceR;
int buttonState = 1;
int x, y;
char c;
const uint64_t pipe = 0xE6E6E6E6E6E6;
bool written = false;
void setup() {
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(m1A, OUTPUT);
  pinMode(m1B, OUTPUT);
  pinMode(m2A, OUTPUT);
  pinMode(m2B, OUTPUT);
  pinMode(m1Spd, OUTPUT);
  pinMode(m2Spd, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

long msToIn(long ms) {
  return ms / 74/* / 2*/;
}

void write1(double mSpeed) {
  Serial.print("L: ");
  Serial.println(mSpeed * 255);
  if (mSpeed > 1) {
    mSpeed = 1;
  }
  if (mSpeed < -1) {
    mSpeed = -1;
  }
  if (mSpeed == 0) {
    digitalWrite(m1A, HIGH);
    digitalWrite(m1B, HIGH);
  }
  else if (mSpeed > 0) {
    digitalWrite(m1A, HIGH);
    digitalWrite(m1B, LOW);
    analogWrite(m1Spd, mSpeed * 255);
  }
  else {
    digitalWrite(m1A, LOW);
    digitalWrite(m1B, HIGH);
    analogWrite(m1Spd, -mSpeed * 255);
  }
}

void write2(double mSpeed) {
  Serial.print("R: ");
  Serial.println(mSpeed * 255);
  if (mSpeed > 1) {
    mSpeed = 1;
  }
  if (mSpeed < -1) {
    mSpeed = -1;
  }
  if (mSpeed == 0) {
    digitalWrite(m2A, HIGH);
    digitalWrite(m2B, HIGH);
  }
  else if (mSpeed > 0) {
    digitalWrite(m2A, HIGH);
    digitalWrite(m2B, LOW);
    analogWrite(m2Spd, mSpeed * 255);
  }
  else {
    digitalWrite(m2A, LOW);
    digitalWrite(m2B, HIGH);
    analogWrite(m2Spd, -mSpeed * 255);
  }
}

void loop() {
  if (radio.available()) {
    radio.read(&c, sizeof(c));
    delay(10);
    while (c != 'x') {
      radio.read(&c, sizeof(c));
      delay(10);
    }
    delay(10);
    radio.read(&x, sizeof(x));
    delay(10);
    while (c != 'y') {
      radio.read(&c, sizeof(c));
      delay(10);
    }
    delay(10);
    radio.read(&y, sizeof(y));
    Serial.println(x);
    Serial.println(y);
    //Serial.println(buttonState);
    if (x > 512) {
      //write1(map(y, 0, 1023, -1, 1) - map(x, 0, 512, 1, 0));
      write1((double) y / 1023 * 2 - 1 - (1 - (double) x / 512));
      //write2(map(y, 0, 1023, -1, 1));
      write2((double) y / 1023 * 2 - 1);
    }
    else {
      //write1(map(y, 0, 1023, -1, 1));
      write1((double) y / 1023 * 2 - 1);
      //write2(map(y, 0, 1023, -1, 1) - map(x, 512, 1023, 0, 1));
      write2((double) y / 1023 * 2 - 1 - (((double) x - 512) / 512));
    }
  }
  delay(10);
}
