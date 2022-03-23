#include <Servo.h>

Servo escL, escR;

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN

int waited;
long temp;
long durationL, durationR;
long distanceL, distanceR;
int buttonState = 1;
int x, y;
float v, w;
char c;
const uint64_t pipe = 0xE6E6E6E6E6E6;
const byte address[6] = "00001";
bool written = false;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  escL.write(90);
  escR.write(90);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

void write1(double mSpeed) {
  Serial.print(mSpeed);
  Serial.print(", ");
  if (mSpeed >= 0) {
    analogWrite(3, mSpeed * 255);
  }
}

void write2(double mSpeed) {
  Serial.print(mSpeed);
  Serial.println();
  if (mSpeed >= 0) {
    analogWrite(5, mSpeed * 255);
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
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);
    x = map(x, 0, 1023, 100, -100);
    y = map(y, 0, 1023, -100, 100);
    v = (100 - abs(x)) * (y / 100.0) + y;
    w = (100 - abs(y)) * (x / 100.0) + x;
    write1((v-w)/200.0); //LEFT ---- Algorithm: https://home.kendra.com/mauser/Joystick.html
    write2((v+w)/200.0); //RIGHT
  }
  delay(10);
}
