//Code for push buttons and LEDS- 5 States Framework
#include <Servo.h>

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

//Pin Numbers
const int ledPin_PlayLive = 2; 
const int ledPin_Record = 3;    
const int ledPin_PlayRecording = 4;    
const int ledPin_Stop = 5;    
const int ledPin_Reset = 6;
const int ledPin_StripLight= 7;
const int buttonPin_Reset = 8;        
const int buttonPin_Stop = 9;        
const int buttonPin_PlayRecording = 10;        
const int buttonPin_Record = 11;        
const int buttonPin_PlayLive = 12;        
const int ledPin = 13; //test purposes       

//Variables
int ledPin_PlayLive_State = LOW; // the current state of the output pin
int ledPin_Record_State = LOW; // the current state of the output pin
int ledPin_PlayRecording_State = LOW; // the current state of the output pin
int ledPin_Stop_State = LOW; // the current state of the output pin
int ledPin_Reset_State = LOW; // the current state of the output pin

int currentState;
int newState;

int button_PlayLive_State;
int button_Record_State;
int button_PlayRecording_State;
int button_Stop_State;
int button_Reset_State;

//Funtion to move motors
void moveMotors(){
  
}

//Function to turn on respective LEDS
void turnLED() {
  if (button_PlayLive_State == HIGH) {
    // turn LED on:
    digitalWrite(ledPin_PlayLive, HIGH);
    digitalWrite(ledPin_StripLight, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin_PlayLive, LOW);
  }
  
  if (button_Record_State == HIGH) {
    // turn LED on:
    digitalWrite(ledPin_Record, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin_Record, LOW);
  }
  
  if (button_PlayRecording_State == HIGH) {
    // turn LED on:
    digitalWrite(ledPin_PlayRecording, HIGH);
    digitalWrite(ledPin_StripLight, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin_PlayRecording, LOW);
  }

  if (button_Stop_State == HIGH) {
    // turn LED on:
    digitalWrite(ledPin_Stop, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin_Stop, LOW);
  }

  if (button_Reset_State == HIGH) {
    // turn LED on:
    digitalWrite(ledPin_Reset, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin_Reset, LOW);
  }

  
}
// Funtion to determine current state
int getCurrentState(){
   if (button_PlayLive_State == HIGH) {
    currentState= PLAY_LIVE;
    return currentState;
  } 
  
  if (button_Record_State == HIGH) {
    currentState= RECORD;
    return currentState;
    } 
  
  if (button_PlayRecording_State == HIGH) {
    currentState= PLAY_RECORDING;
    return currentState;
  } 
  if (button_Stop_State == HIGH) {
    currentState= STOP;
    return currentState;
      } 

  if (button_Reset_State == HIGH) {
    currentState= RESET;
    return currentState;
  }  
  }
//Function to determine if the state was changed
bool changeOfState() {
  
  int newState= -1;

  if (digitalRead(12) == HIGH) {
    newState = PLAY_LIVE;
  } else if (digitalRead(11) == HIGH) {
    newState = RECORD;
  } else if (digitalRead(10) == HIGH) {
    newState = PLAY_RECORDING;
  } else if (digitalRead(9) == HIGH) {
    newState = STOP;
  } else if (digitalRead(8) == HIGH) {
    newState = RESET;
  }

  // To check for state change
  if (newState != -1 && currentState != newState){
    currentState = newState;
    return true;
  }

  return false;
 }

  // Functions for 5 Modes
void mode_PlayLive(){
  Serial.println("PLAY LIVE STATE");
  turnLED();
  moveMotors();
}
 
void mode_Record(){
  Serial.println("RECORD STATE");
  turnLED();
  moveMotors();
}

void mode_PlayRecording(){
  Serial.println("PLAY RECORDING STATE");
  turnLED();
  moveMotors();
}

void mode_Reset(){
  Serial.println("RESET STATE");
  turnLED();
  moveMotors();
}

void mode_Stop(){
  Serial.println("STOP STATE");
  turnLED();
  moveMotors();
}

void setup() {
  //Initiate Serial communication.
  Serial.begin(9600);
  
  // Start in PLAY_LIVE
  currentState = PLAY_LIVE;

  //Set pins
  pinMode(buttonPin_PlayLive, INPUT); // PLAY_LIVE
  pinMode(buttonPin_Record, INPUT); // RECORD
  pinMode(buttonPin_PlayRecording, INPUT); // PLAY_RECORDING
  pinMode(buttonPin_Stop, INPUT); // STOP
  pinMode(buttonPin_Reset, INPUT); //RESET


 // set initial LED state
  digitalWrite(ledPin_PlayLive, ledPin_PlayLive_State); 
  digitalWrite(ledPin_Record, ledPin_Record_State); 
  digitalWrite(ledPin_PlayRecording, ledPin_PlayRecording_State); 
  digitalWrite(ledPin_Stop, ledPin_Stop_State); 
  digitalWrite(ledPin_Reset, ledPin_Reset_State); 

}

void loop() {
  // See if State Changed
  //changeOfState();
  button_PlayLive_State = digitalRead(12);
  button_Record_State = digitalRead(11);
  button_PlayRecording_State = digitalRead(10);
  button_Stop_State = digitalRead(9);
  button_Reset_State = digitalRead(8);


  digitalWrite(ledPin_PlayLive, 1);

  getCurrentState();
  //5 Modes Switch
  switch(currentState){
    case PLAY_LIVE:
      mode_PlayLive();
      break;
    case RECORD:
      mode_Record();
      break;
    case PLAY_RECORDING:
      mode_PlayRecording();
      break;
    case STOP:
      mode_Stop();
      break;
    case RESET:
      mode_Reset();
      break;
    }

  

 

}
