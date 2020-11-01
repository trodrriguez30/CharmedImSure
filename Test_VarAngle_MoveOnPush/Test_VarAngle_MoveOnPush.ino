
//This code causes the servo to move with a button and move back with the second press

//Next goal: move servos based on different input

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Estimates
// difference of 50 => 15 deg
// difference of 150 => 45 deg
// difference of 300 => 90 deg
// difference of 600 => 180 deg
// I'm guessing difference of 50 leads to change in just about 15 degrees
// caps out at 180 deg
#define SERVOMIN  150
#define SERVOMAX  300
uint8_t servonum = 0;


uint8_t buttonState1 = HIGH;
uint8_t previousButtonState1 = HIGH;
uint8_t buttonState2 = HIGH;
uint8_t previousButtonState2 = HIGH;

int servoState1 = SERVOMIN;
int servoState2 = SERVOMIN;

int pin1 = 2;
int pin2 = 5;


void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
 
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

}

void loop() {
  //Check button for servo group 1
  buttonState1 = debounceRead(pin1);
  if ((LOW == buttonState1) and (HIGH == previousButtonState1))
  {
    if (servoState1 == SERVOMIN) {
      moveServo1(SERVOMIN, SERVOMAX);
      servoState1 = SERVOMAX;
    }
    else {
      moveServo1(SERVOMAX, SERVOMIN);
      servoState1 = SERVOMIN;
    }
  }
  previousButtonState1 = buttonState1;
  
  buttonState2 = debounceRead(pin2);
  if ((LOW == buttonState2) and (HIGH == previousButtonState2))
  {
    if (servoState2 == SERVOMIN) {
      moveServo2(SERVOMIN, SERVOMAX);
      servoState2 = SERVOMAX;
    }
    else {
      moveServo2(SERVOMAX, SERVOMIN);
      servoState2 = SERVOMIN;
    }
  }
  previousButtonState2 = buttonState2;
}
uint8_t debounceRead(int pin)
{
  uint8_t pinState = digitalRead(pin);
  uint32_t timeout = millis();
  while (millis() < timeout+10)
  {
    if (digitalRead(pin) != pinState)
    {
      pinState = digitalRead(pin);
      timeout = millis();
    }
  }

  return pinState;
}

void moveServo1(int from, int to)
{
if (from < to)
  for (uint16_t pulselen = from; pulselen < to; pulselen++) {
    pwm.setPWM(0, 0, pulselen);
  }
else
  for (uint16_t pulselen = from; pulselen > to; pulselen--) {
    pwm.setPWM(0, 0, pulselen);
  }
}

void moveServo2(int from, int to)
{
if (from < to)
  for (uint16_t pulselen = from; pulselen < to; pulselen++) {
    pwm.setPWM(1, 0, pulselen);
  }
else
  for (uint16_t pulselen = from; pulselen > to; pulselen--) {
    pwm.setPWM(1, 0, pulselen);
  }
}
