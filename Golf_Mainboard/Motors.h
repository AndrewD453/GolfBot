const int motorL = 7;
const int motorR = 6;

void writeL(double mSpeed) {
  Serial.print(mSpeed);
  Serial.print(", ");
  if (mSpeed >= 1) {
    analogWrite(motorL, 255);
  }
  else if (mSpeed >= 0) {
    analogWrite(motorL, mSpeed * 255);
  }
  else {
    analogWrite(motorL, 0);
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
  else {
    analogWrite(motorR, 0);
  }
}
