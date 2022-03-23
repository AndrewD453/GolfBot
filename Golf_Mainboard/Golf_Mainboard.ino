#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN

const int trigPinL = 4;
const int echoPinL = 5;
const int trigPinR = 0;
const int echoPinR = 1;
const int m1A = 2;
const int m1B = 3;
const int m2A = 6;
const int m2B = 7;
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
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}

long msToIn(long ms) {
  return ms / 74/* / 2*/;
}

void write1(int mSpeed) {
  switch (mSpeed) {
    case 0:
      digitalWrite(m1A, HIGH);
      digitalWrite(m1B, HIGH);
      break;
    case 1:
      digitalWrite(m1A, HIGH);
      digitalWrite(m1B, LOW);
      break;
    case -1:
      digitalWrite(m1A, LOW);
      digitalWrite(m1B, HIGH);
      break;
  }
}

void write2(int mSpeed) {
  switch (mSpeed) {
    case 0:
      digitalWrite(m2A, HIGH);
      digitalWrite(m2B, HIGH);
      break;
    case 1:
      digitalWrite(m2A, LOW);
      digitalWrite(m2B, HIGH);
      break;
    case -1:
      digitalWrite(m2A, HIGH);
      digitalWrite(m2B, LOW);
      break;
  }
}

void loop() {
  radio.write(&buttonState, sizeof(buttonState));
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  durationL = pulseIn(echoPinL, HIGH);
  distanceL = msToIn(durationL);
  Serial.print(durationL);
  Serial.print(" L ");
  Serial.println(distanceL);
  
  radio.write(&buttonState, sizeof(buttonState));
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH);
  distanceR = msToIn(durationR);
  Serial.print(durationR);
  Serial.print(" R ");
  Serial.println(distanceR);
  
  if (durationL > 40000) {
    write1(0);
    write2(0);
    Serial.println("Out of range!");
  }
  else if (distanceL > 50) {
    write1(1);
    write2(1);
    Serial.println("Too Far!");
  }
  else if (distanceL < 20) {
    write1(-1);
    write2(-1);
    Serial.println("Too Close!");
  }
  else {
    write1(0);
    write2(0);
    Serial.println("Good!");
  }
  delay(100);
}
