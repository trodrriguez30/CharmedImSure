#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define DEBOUNCE 10  // how many ms to debounce, 5+ ms is usually plenty

// 256/128 = 2 which means +/- 1 Hz 
#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 256 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.

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

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 60

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

int counter = 0;

arduinoFFT FFT = arduinoFFT();
 
unsigned int samplingPeriod;
unsigned long microSeconds;
 
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

//servo code
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

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, ledPin_StripLight, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

int melody[]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int noteDurations[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 , 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 , 2, 2, 2, 2, 2};

void setup() {
  byte i;
  Serial.begin(9600); //set up serial port
  Serial.print("Button checker with ");
  Serial.print(NUMBUTTONS, DEC);
  Serial.println(" buttons");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // Make input & enable pull-up resistors on switch pins
  for (i=0; i< NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}
 
void loop() {
  // Fill along the length of the strip in various colors...
  colorWipe(strip.Color(255,   0,   0), 50); // Red
  colorWipe(strip.Color(  0, 255,   0), 50); // Green
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  rainbow(10);             // Flowing rainbow cycle along the whole strip
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
  
  byte thisSwitch=thisSwitch_justPressed();
  switch(thisSwitch)
  {  
  case 0: 
    Serial.println("PLAY LIVE"); 
    ledSwitch(thisSwitch);
    //sound(thisSwitch);
    break;
  case 1: 
    Serial.println("RECORD"); 
    ledSwitch(thisSwitch);
    //sound(thisSwitch);
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
    resetMelody();
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
    playLiveFFT();
    break;
  case 1:
    Serial.println("RECORD SOUND ON"); 
    recordFFT();
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
    //resets the motors
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

void playLiveFFT(){
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 

   for(int j=0; j< 40; j++){
    for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
     
        vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
        vImag[i] = 0; //Makes imaginary term 0 always

        /*remaining wait time between samples if necessary*/
        while(micros() < (microSeconds + samplingPeriod))
        {
          //do nothing
        }
    }
 
    /*Perform FFT on samples*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    /*Find peak frequency and print peak*/
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    Serial.println(peak);     //Print out the most dominant frequency.

    if ((peak >= A2) && (peak <= (G_sharp3 + 1))){
      melody[j] = peak;
    }
    }
 }

 void recordFFT(){
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 

   for(int j=counter; j< 40; j++){
    for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
     
        vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
        vImag[i] = 0; //Makes imaginary term 0 always

        /*remaining wait time between samples if necessary*/
        while(micros() < (microSeconds + samplingPeriod))
        {
          //do nothing
        }
    }
 
    /*Perform FFT on samples*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    /*Find peak frequency and print peak*/
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    Serial.println(peak);     //Print out the most dominant frequency.

    if ((peak >= A2) && (peak <= (G_sharp3 + 1))){
      melody[j] = peak;
      counter++
    }
    }
 }

void sampleMelody(){
  int sampleMelody[] = { C3, C3, G3, C3, C3, G_sharp3, F3, D3, B2, A2, E3, E3, E3, E3, G3, G3, D3, C3, B2, A2, A2, B2, C3, D3, G3, G3,E3, E3, E3, E3, A2, B2, D3, F3, G_sharp3, C3, C3, G3, C3, C3};
  
  for (int thisNote = 0; thisNote < 40; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/4, etc.
    int noteDuration = 1000 / noteDurations[thisNote];

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
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void resetMelody(){
  for(int i= 0; i< 40; i++){
    melody[i] = 0;
    counter = 0;
  }
}

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
