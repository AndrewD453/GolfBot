#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
boolean buttonState = 0;
const int trigPin = 4;
const int echoPin = 5;
long duration;
long distance;
bool wasHigh = false;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  //Serial.println("Begin");
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}

long msToIn(long ms) {
  return ms / 74/* / 2*/;
}

void loop() {
  if (radio.available()) {
    Serial.println("Data recieved, beginning");
    radio.read(&buttonState, sizeof(buttonState));
    //Serial.println(buttonState);
    //if (buttonState == HIGH) {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }
  delay(20);
}
