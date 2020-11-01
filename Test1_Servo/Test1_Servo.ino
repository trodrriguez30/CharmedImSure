/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
#define SERVOMIN 150
#define SERVOMAX 600

int pos = 0;    // variable to store the servo position

int servonum = 0;

void setup() {
  pwm.begin();
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
}

void loop() {
  
  /*for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  */
  int finaldeg = 60;
  for (int degrees = 0; degrees < finaldeg; degrees++){
    uint16_t pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(500);
  
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
