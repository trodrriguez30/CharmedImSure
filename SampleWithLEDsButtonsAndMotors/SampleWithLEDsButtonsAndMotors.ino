#define DEBOUNCE 10  // how many ms to debounce, 5+ ms is usually plenty

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 5 States
#define PLAY_LIVE 0
#define RECORD 1
#define PLAY_RECORDING 2
#define STOP 3
#define RESET 4

//Musical Notes
#define A2 110
#define B2 123.471
#define C3 130.813
#define D3 146.832
#define E3 164.814
#define F3 174.614
#define G3 195.998
#define G_sharp3 207.652

const int ledPin_PlayLive = 2; 
const int ledPin_Record = 3;    
const int ledPin_PlayRecording = 4;
const int ledPin_Stop = 5;    
const int ledPin_Reset = 6;
const int ledPin_StripLight= 7;
const int buttonPin_PlayLive = 8;
const int buttonPin_Record = 9;
const int buttonPin_PlayRecording = 10;
const int buttonPin_Stop = 11;                                
const int buttonPin_Reset = 12;        
const int speakerPin = 13;      

//Servo code
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int SERVOMIN = 150;
int SERVOMAX = 600;
int USMIN = 600;
int USMAX = 2400;
int currentServoAng1 = 150;
int currentServoAng2 = 150;
int currentServoAng3 = 150;
int ServoState1 = SERVOMIN;
int ServoState2 = SERVOMIN;
int ServoState3 = SERVOMIN;


//define the buttons that we'll use.
byte buttons[] = {buttonPin_PlayLive, buttonPin_Record, buttonPin_PlayRecording, buttonPin_Stop, buttonPin_Reset}; 
 
//determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)
 
//track if a button is just pressed, just released, or 'currently pressed' 
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];


void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  byte i;
  Serial.begin(9600); //set up serial port
  Serial.print("Button checker with ");
  Serial.print(NUMBUTTONS, DEC);
  Serial.println(" buttons");
  // Make input & enable pull-up resistors on switch pins
  for (i=0; i< NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }
}
 
void loop() {
  byte thisSwitch=thisSwitch_justPressed();
  switch(thisSwitch)
  {  
  case 0: 
    Serial.println("PLAY LIVE"); 
    ledSwitch(thisSwitch);
    sound(thisSwitch);
    break;
  case 1: 
    Serial.println("RECORD"); 
    ledSwitch(thisSwitch);
    sound(thisSwitch);
    break;
  case 2: 
    Serial.println("PLAY RECORDING"); 
    ledSwitch(thisSwitch);
    sound(thisSwitch);
    break;
  case 3: 
    Serial.println("STOP"); 
    ledSwitch(thisSwitch);
    sound(thisSwitch);
    break;
  case 4: 
    Serial.println("RESET"); 
    ledSwitch(thisSwitch);
    sound(thisSwitch);
    break;
  }
}
 
void check_switches()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;
  if (millis() < lasttime) {
    // we wrapped around, lets just try again
    lasttime = millis();
  }
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return; 
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  for (index = 0; index < NUMBUTTONS; index++) {
    justpressed[index] = 0;       //when we start, we clear out the "just" indicators
    justreleased[index] = 0;
    currentstate[index] = digitalRead(buttons[index]);   //read the button
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
        // just pressed
        justpressed[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
        justreleased[index] = 1; // just released
      }
      pressed[index] = !currentstate[index];  //remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index]; //keep a running tally of the buttons
  }
}
 
byte thisSwitch_justPressed() {
  byte thisSwitch = 255;
  check_switches();  //check the switches &amp; get the current state
  for (byte i = 0; i < NUMBUTTONS; i++) {
    current_keystate[i]=justpressed[i];
    if (current_keystate[i] != previous_keystate[i]) {
      if (current_keystate[i]) thisSwitch=i;
    }
    previous_keystate[i]=current_keystate[i];
  }  
  return thisSwitch;
}

void ledSwitch(int switchNum){
  switch(switchNum)
  {
  case 0:
    Serial.println("PLAY LIVE and STRIP LED ON"); 
    digitalWrite(ledPin_PlayLive, HIGH);
    digitalWrite(ledPin_Record, LOW);
    digitalWrite(ledPin_PlayRecording, LOW);
    digitalWrite(ledPin_Stop, LOW); 
    digitalWrite(ledPin_Reset, LOW);
    digitalWrite(ledPin_StripLight, HIGH);  
    break;
  case 1:
    Serial.println("RECORD LED ON"); 
    digitalWrite(ledPin_PlayLive, LOW);
    digitalWrite(ledPin_Record, HIGH);
    digitalWrite(ledPin_PlayRecording, LOW);
    digitalWrite(ledPin_Stop, LOW); 
    digitalWrite(ledPin_Reset, LOW);  
    break; 
  case 2:
    Serial.println("PLAY RECORDING and STRIP LED ON");  
    digitalWrite(ledPin_PlayLive, LOW);
    digitalWrite(ledPin_Record, LOW);
    digitalWrite(ledPin_PlayRecording, HIGH);
    digitalWrite(ledPin_Stop, LOW); 
    digitalWrite(ledPin_Reset, LOW);  
    break;
  case 3:
    Serial.println("STOP LED ON");  
    digitalWrite(ledPin_PlayLive, LOW);
    digitalWrite(ledPin_Record, LOW);
    digitalWrite(ledPin_PlayRecording, LOW);
    digitalWrite(ledPin_Stop, HIGH); 
    digitalWrite(ledPin_Reset, LOW);
    digitalWrite(ledPin_StripLight, HIGH);    
    break;
  case 4:
    Serial.println("RESET LED ON");  
    digitalWrite(ledPin_PlayLive, LOW);
    digitalWrite(ledPin_Record, LOW);
    digitalWrite(ledPin_PlayRecording, LOW);
    digitalWrite(ledPin_Stop, LOW); 
    digitalWrite(ledPin_Reset, HIGH);  
    break; 
  }
}

void sound(int switchNum){
  switch(switchNum)
  {
  case 0:
    Serial.println("PLAY LIVE SOUND ON"); 
    /*IMPLEMENT */
    break;
  case 1:
    Serial.println("RECORD SOUND ON"); 
    /*IMPLEMENT */
    break; 
  case 2:
    Serial.println("PLAY RECORDING SOUND ON");  
    sampleMelody();
    break;
  case 3:
    Serial.println("STOP LED ON");  
    noTone(speakerPin);
    break;
  case 4:
    Serial.println("RESET LED ON");  
    noTone(speakerPin);
    for (uint16_t pulselen = currentServoAng1; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(0, 0, pulselen);
      }
    for (uint16_t pulselen = currentServoAng2; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(1, 0, pulselen);
      }
    for (uint16_t pulselen = currentServoAng3; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
      }
    break; 
  }
}

void sampleMelody(){
  int sampleMelody[] = { C3, C3, G3, C3, C3, G_sharp3, F3, D3, B2, A2, E3, E3, E3, E3, G3, G3, D3, C3, B2, A2, A2, B2, C3, D3, G3, G3,E3, E3, E3, E3, A2, B2, D3, F3, G_sharp3, C3, C3, G3, C3, C3};
  int noteDurations[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 , 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 , 2, 2, 2, 2, 2};
  
  for (int thisNote = 0; thisNote < 40; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/4, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    /*
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(0, 0, pulselen);
    }
  
    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(0, 0, pulselen);
    }
  
    delay(500);
    */
  
    // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
    // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
    
    
    //servo movement
    //Serial.println(sampleMelody[thisNote]);
    //It truncates the value of the note
    if (sampleMelody[thisNote] == floor(C3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng1; pulselen < currentServoAng1 + 100; pulselen++) {
        pwm.setPWM(0, 0, pulselen);
      }
      //moveServo(currentServoAng1, 150, 0);
      currentServoAng1 += 100;
    }
    else if (sampleMelody[thisNote] == floor(G3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng2; pulselen < currentServoAng2 + 100; pulselen++) {
        pwm.setPWM(1, 0, pulselen);
      }
      //moveServo(currentServoAng2, 150, 1);
      currentServoAng2 += 100;
    }
    else if(sampleMelody[thisNote] == floor(G_sharp3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng1; pulselen < currentServoAng1 + 100; pulselen++) {
        pwm.setPWM(0, 0, pulselen);
      }
      //moveServo(currentServoAng1, 50, 0);
      currentServoAng1 += 100;
    }
    else if (sampleMelody[thisNote] == floor(B2)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng3; pulselen < currentServoAng3 + 100; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
      }
      //moveServo(currentServoAng2, 150, 1);
      currentServoAng3 += 100;
    }
    else if(sampleMelody[thisNote] == floor(F3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng1; pulselen > currentServoAng1 - 50; pulselen--) {
        pwm.setPWM(0, 0, pulselen);
      }
      //moveServo(currentServoAng1, currentServoAng1 - 150, 0);
      currentServoAng1 -= 50;
    }
    else if (sampleMelody[thisNote] == floor(A2)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng3; pulselen > currentServoAng3 - 50; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
      }
      //moveServo(currentServoAng2, 150, 1);
      currentServoAng3 -= 50;
    }
    else if (sampleMelody[thisNote] == floor(D3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng3; pulselen < currentServoAng3 + 100; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
      }
      for (uint16_t pulselen = currentServoAng2; pulselen < currentServoAng2 + 100; pulselen++) {
        pwm.setPWM(1, 0, pulselen);
      }
      //moveServo(currentServoAng2, 150, 1);
      currentServoAng2 += 100;
      currentServoAng3 += 100;
    }
    else if (sampleMelody[thisNote] == floor(E3)){
      //Serial.println(sampleMelody[thisNote]);
      //Serial.println("Made It");
      for (uint16_t pulselen = currentServoAng2; pulselen > currentServoAng2 - 50; pulselen--) {
        pwm.setPWM(1, 0, pulselen);
      }
      //moveServo(currentServoAng2, 150, 1);
      currentServoAng2 -= 50;
    }

    tone(speakerPin, sampleMelody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 3.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);

   
    
  }
}


void moveServo(int from, int to, int servonum){
  if (from < to){
    for (uint16_t pulselen = from; pulselen < to; pulselen++) {
      pwm.setPWM(servonum, 0, pulselen);
    }
  }
  else{
    for (uint16_t pulselen = from; pulselen > to; pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);
    }
  }
}
