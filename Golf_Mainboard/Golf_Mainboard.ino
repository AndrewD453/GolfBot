#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(8, 10); // CE, CSN

#define iters 2

const int trigPinL = A5;
const int echoPinL = A4;
const int trigPinR = A1;
const int echoPinR = A0;
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
bool buttonState = 1;
const byte address[6] = "00001";
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
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}

long msToIn(long ms) {
  return ms / 74/* / 2*/;
}

void write1(double mSpeed) { // right
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

void write2(double mSpeed) { // left
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
  durationL = 0;
  waited = 0;
  for (int i = 0; i < iters; i++) {
    radio.write(&buttonState, sizeof(buttonState));
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    temp = pulseIn(echoPinL, HIGH);
    if (temp > 30000 && waited < iters * 5) {
      i--;
      waited++;
    }
    else if (temp > 30000) {
      durationL = iters * 40000;
      break;
    }
    else {
      durationL += temp;
    }
    delay(57);
  }
  durationL /= iters;
  distanceL = msToIn(durationL);
  Serial.print(durationL);
  Serial.print(" L ");
  Serial.println(distanceL);

  durationR = 0;
  waited = 0;
  for (int i = 0; i < iters; i++) {
    radio.write(&buttonState, sizeof(buttonState));
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
    temp = pulseIn(echoPinR, HIGH);
    if (temp > 30000 && waited < iters * 5) {
      i--;
      waited++;
    }
    else if (temp > 30000) {
      durationR = iters * 40000;
      break;
    }
    else {
      durationR += temp;
    }
    delay(57);
  }
  durationR /= iters;
  distanceR = msToIn(durationR);
  Serial.print(durationR);
  Serial.print(" R ");
  Serial.println(distanceR);

  if (durationL > 30000 && durationR > 30000) {
    write1(0);
    write2(0);
    Serial.println("Out of range!");
  }
  else if (durationL > 30000) {
    write1(1);
    write2(-1);
    Serial.println("LS is lost!");
  }
  else if (durationR > 30000) {
    write1(-1);
    write2(1);
    Serial.println("RS is lost!");
  }
  else {
    //write1((double) (distanceL - 50) / 50.0 + (double) (distanceL - distanceR) / 40.0);
    //write2((double) (distanceR - 50) / 50.0 + (double) (distanceR - distanceL) / 40.0);
    write1(distanceR * 2 - distanceL);
    write2(distanceL * 2 - distanceR);
    Serial.println("Good!");
  }
  delay(100);
  Serial.println("\n");
}
