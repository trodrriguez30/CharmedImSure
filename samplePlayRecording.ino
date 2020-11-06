#define DEBOUNCE 10  // how many ms to debounce, 5+ ms is usually plenty

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

    tone(speakerPin, sampleMelody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}
