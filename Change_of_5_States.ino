#define DEBOUNCE 10  // how many ms to debounce, 5+ ms is usually plenty

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
const int ledPin = 13; //test purposes           

//define the buttons that we'll use.
byte buttons[] = {buttonPin_PlayLive, buttonPin_Record, buttonPin_PlayRecording, buttonPin_Stop, buttonPin_Reset}; 
 
//determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)
 
//track if a button is just pressed, just released, or 'currently pressed' 
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];


void setup() {
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
    break;
  case 1: 
    Serial.println("RECORD"); 
    ledSwitch(thisSwitch);
    break;
  case 2: 
    Serial.println("PLAY RECORDING"); 
    ledSwitch(thisSwitch);
    break;
  case 3: 
    Serial.println("STOP"); 
    ledSwitch(thisSwitch);
    break;
  case 4: 
    Serial.println("RESET"); 
    ledSwitch(thisSwitch);
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
