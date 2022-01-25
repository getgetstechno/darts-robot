/*
 * Code made by a student
 * last edited : 8/1/2022
 */
 
#include <IRremote.h> // include library for infrared controller

const int RECV_PIN = 4; // pin number 4 for IR receiver
IRrecv irrecv(RECV_PIN); // create object irrecv
decode_results results; // name of outcome IR data = results

int period = 100;   // every 100ms the esp32 will check if the IR received somthing -> see underneed
unsigned long time_now = 0;   // initilise the amount of miliseconds the esp32 is running -> see underneed

// pins for SN74hc595n (shift register) number 1 : control canon 1 and 2
const byte latchPin1 = 18;    
const byte clockPin1 = 5;         
const byte dataPin1 = 19;        

// pins for SN74hc595n (shift register) number 2 : control canon 3 and trigger
const byte latchPin2 = 33;
const byte clockPin2 = 25;
const byte dataPin2 = 32;

// pins for SN74hc595n (shift register) number 3 : control UpDown and Y-wheels (4 wheels)
const byte latchPin3 = 2;
const byte clockPin3 = 15;
const byte dataPin3 = 0;

// pins for SN74hc595n (shift register) number 4 : control angle and X-wheels (6 wheels)
const byte latchPin4 = 14;
const byte clockPin4 = 12;
const byte dataPin4 = 27;


// here come the array to control the buttons. That means that if you send the first row data to the shift register the first motor will sturn clockwise,
// the second row will turn the second motor clockwise
// the third one control the first motor conuter-clockwise --> it actually just the inverse of the first one
// the fourth one control the second motor counter-clockwise --> it actually just the inverse of the second one
const char motorDrive[4][8] = {{B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000},    //CW1
                               {B00001000, B00001100, B00000100, B00000110, B00000010, B00000011, B00000001, B00001001},    //CW2
                               {B10010000, B00010000, B00110000, B00100000, B01100000, B01000000, B11000000, B10000000},    //CCW1
                               {B00001001, B00000001, B00000011, B00000010, B00000110, B00000100, B00001100, B00001000}};    //CCW2

byte numberOfStateEnable = 0; // see how much buttons on the IR are pressed at the same moment

// it's important for me to know the number of buttons on the IR that is enable.
// That's why I will light up the amount of numberOfStateEnable. if one --> just enable1Pin, if two --> enable1Pin and enable2Pin, ...
// why only 4 and not more. I think that 5 motors or more, running at the same time, will destroy all the motors due to high current.
// 4 at the same moment is for now enough I think. 
byte enable1Pin = 21; // pin number on which the led is installed
byte enable2Pin = 3;  // pin number on which the led is installed
byte enable3Pin = 22; // pin number on which the led is installed 
byte enable4Pin = 23; // pin number on which the led is installed

// here is an array of all the hexadecimal values of the buttons of the IR. They are sorted in this way : 
// 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, star, hashtag, upper arrow, left arrow, center, right arrow, downer arrow
const int IRReceiverCodes[] = {0xFF6897, 0xFF9867, 0xFFB04F, 0xFF30CF, 0xFF18E7, 0xFF7A85, 0xFF10EF, 0xFF38C7, 
                              0xFF5AA5, 0xFF4AB5, 0xFF42BD, 0xFF52AD, 0xFF629D, 0xFF22DD, 0xFF02FD, 0xFFC23D, 0xFFA857};
                              
// for all buttons on the IR there is a value 0 or 1, if the button is one time pressed then the value is 1, if you then press again, the value is 0
// 18 zero's but there are 17 buttons ? --> buttonState[0] is skipped because we start which button 1                              
char buttonState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// the 3 control pins for the shift register are set in an array the be controllable easely in the code --> see later
const char SR[4][3] = {{latchPin1, clockPin1, dataPin1},
                  {latchPin2, clockPin2, dataPin2},
                  {latchPin3, clockPin3, dataPin3},
                  {latchPin4, clockPin4, dataPin4}};
                  
char Array2[2] = {0, 0}; // if there are only 2 buttons pressed, then I make an arraw where I store which button is pressed --> see why important later
char Array3[3] = {0, 0, 0}; // same but when there are 3 buttons pressed
char Array4[4] = {0, 0, 0, 0}; // same but when there are 4 buttons pressed


// one I know which button is pressed, I need to action the right motor, therefore I need to know on which shift register the motor is connected (whichSR),
// I also need to know in which direction the motor is turning. (whichDirection) --> see later but first I have to create these variables
// when one button is pressed, whichSR1 and whichDirection1 is used
// when two buttons are pressed, whichSR1, whichDirection1, whichSR2 and whichDirection2 are used. And so on for 3 and 4
byte whichSR1; 
int whichDirection1;
byte whichSR2;
int whichDirection2;
byte whichSR3;
int whichDirection3;
byte whichSR4;
int whichDirection4;


// must important array of the code, it collect all the buttons that are at that moment on. But in order in time. 
// That means that if you pressed 1, 4, 2 and 3 the array will be {1, 4, 2, 3} but when you pressed again on value 2 the array become {1, 4, 3, 0}
byte buttonPressedArray[] = {0, 0, 0, 0};

// end declaring variables, time for real code! ----------------------------------------------------------------------------------

void setup() { // when the arduino is power up, he will execute the following block of code only one time. 
  irrecv.enableIRIn(); // activate the IR as a receiver and not as a transmitter
  Serial.begin(115200); // start the serial communication at 115200 bits per seconds --> only used for debugging 

  // declare all pins as outputs : 
  pinMode(latchPin1, OUTPUT);
  pinMode(clockPin1, OUTPUT);
  pinMode(dataPin1, OUTPUT);
  pinMode(latchPin2, OUTPUT);
  pinMode(clockPin2, OUTPUT);
  pinMode(dataPin2, OUTPUT);
  pinMode(latchPin3, OUTPUT);
  pinMode(clockPin3, OUTPUT);
  pinMode(dataPin3, OUTPUT);
  pinMode(latchPin4, OUTPUT);
  pinMode(clockPin4, OUTPUT);
  pinMode(dataPin4, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  pinMode(enable3Pin, OUTPUT);
  pinMode(enable4Pin, OUTPUT);
}

void loop() {
  if(millis() > time_now + period) { // everything in this braces is executes every period=100ms -> 10x per seconds 
    time_now = millis(); // one's whe past the delay of 100ms whe declare the new time_now

    // firstly check if the IR receiver collected something
    if(irrecv.decode(&results)) {
      // He detected something, but what ?
      for(int i = 0; i < 17; i++) {
        if(results.value == IRReceiverCodes[i]) {
          // button i + 1 is pressed
          //Serial.println(i + 1);
          // here I change the value in the buttonState array, I do it to change a pushbuttons into a switch button 
          if(buttonState[i + 1] == 0) {
            buttonState[i + 1] = 1;
            numberOfStateEnable++;  // if button is pressed then the the number of button pressed is +1
          } else {
            buttonState[i + 1] = 0;
            numberOfStateEnable--;  // if a button is pressed again then the number of buttons pressed is -1
          }

          // now start an important and complex code. It can probably be coded in a shorter way but ... it works!
          // the objecive here is to fill my buttonPressedArray[] with the buttons that are on at this moment. 
          // But I have to be carefull for sublings --> bug
          if(buttonPressedArray[0] == 0) {
            // cool the value is 0 --> we can replace with i + 1
            // check firstly if another value is == i+1 -> yes? -> replace then the value with 0
            if(buttonPressedArray[1] == i + 1) {
              buttonPressedArray[1] = 0;
            } else if (buttonPressedArray[2] == i + 1) {
              buttonPressedArray[2] = 0;
            } else if (buttonPressedArray[3] == i + 1) {
              buttonPressedArray[3] = 0;
            } else {
              buttonPressedArray[0] = i + 1; 
            } 
          } else { // buttonPressedArray[0] != 0
            // not cool, the value already exist --> if value = i + 1 --> 0, if something else --> then continue
            if(buttonPressedArray[0] == i + 1) {
              // we dubble clicked on the buttons --> value must be 0
              buttonPressedArray[0] = 0;
            } else {
              // the value is not 0 and not i + 1 --> means that two buttons are pressed --> need continue
              if(buttonPressedArray[1] == 0) {
                // cool the value is 0 --> we can replace with i + 1
                // check firstly if another value is == i+1 -> yes? -> replace then the value with 0
                if (buttonPressedArray[2] == i + 1) {
                  buttonPressedArray[2] = 0;
                } else if (buttonPressedArray[3] == i + 1) {
                  buttonPressedArray[3] = 0;
                } else {
                  buttonPressedArray[1] = i + 1; 
                }  
              } else { // buttonPressedArray[1] != 0
                // not cool, the value already exist --> if value = i + 1 --> 0, if something else --> do nothing and continue
                if(buttonPressedArray[1] == i + 1) {
                  // we dubble clicked on the buttons --> value must be 0
                  buttonPressedArray[1] = 0;
                } else {
                  // the value is not 0 and not i + 1 --> means that three buttons are pressed --> need to continue
                  if(buttonPressedArray[2] == 0) {
                    // cool the value is 0 --> we can replace with i + 1
                    // check firstly if another value is == i+1 -> yes? -> replace then the value with 0
                    if (buttonPressedArray[3] == i + 1) {
                      buttonPressedArray[3] = 0;
                    } else {
                      buttonPressedArray[2] = i + 1; 
                    }  
                  } else { // buttonPressedArray[2] != 0
                    // not cool, the value already exist --> if value = i + 1 --> 0, if something else --> do nothing and continue
                    if(buttonPressedArray[2] == i + 1) {
                      // we dubble clicked on the buttons --> value must be 0
                      buttonPressedArray[2] = 0;
                    } else {
                      // the value is not 0 and not i + 1 --> means that four buttons are pressed --> need to continue
                      if(buttonPressedArray[3] == 0) {
                        // cool the value is 0 --> we can replace with i + 1
                        buttonPressedArray[3] = i + 1; 
                      } else { // buttonPressedArray[2] != 0
                        // not cool, the value already exist --> if value = i + 1 --> 0, if something else then continue
                        if(buttonPressedArray[3] == i + 1) {
                          // we dubble clicked on the buttons --> value must be 0
                          buttonPressedArray[3] = 0;
                        } else {
                          // the value is not 0 and not i + 1 --> means that five buttons are pressed --> dangerous!
                          // Do nothing, eventually it's not possible to be here because there are only 4 buttons that can be pressed
                        }
                      }
                    }
                  }
                }
              }
            }
          }
          
          //Serial.println(numberOfStateEnable);
          //Here I now get the possible 4 buttons that are pressed, I get the order, the number and I can remove them.
          /*
          for(int i = 0; i < 4 ; i++) {
            Serial.print(buttonPressedArray[i]);
          }
          Serial.println();
          */
        }
      /*
      for(int k = 1; k < 18; k++) {
        Serial.print(buttonState[k], DEC);
      }
      Serial.println();
      */
      }
      //Serial.println(numberOfStateEnable);
      irrecv.resume(); //IR receiver wil forget the value he just collected, otherwise he will allways give the same value
    } // end if
  } // end if
// now we want to control the motors with the array we created -------------------------------------------------------

  switch(numberOfStateEnable) { // number of buttons that are pressed at the same moment
    case 0:
      // do nothing because no buttons is pressed
      //Serial.println("number = 0");
      digitalWrite(enable1Pin, LOW);
      digitalWrite(enable2Pin, LOW);
      digitalWrite(enable3Pin, LOW);
      digitalWrite(enable4Pin, LOW);
      break;
    case 1: 
      // light up the first led
      digitalWrite(enable1Pin, HIGH);
      digitalWrite(enable2Pin, LOW);
      digitalWrite(enable3Pin, LOW);
      digitalWrite(enable4Pin, LOW);
      // action only one motor
      for(int m = 0; m < 4; m++) {
        if(buttonPressedArray[m] != 0) {
          // OK, we find our index we need
          whichSR1 = getNumberOfSH(buttonPressedArray[m]); // return the shift register of the motor that has to run. See function below
          //Serial.println(whichSR);
          whichDirection1 = getDirection(buttonPressedArray[m]); // return the direction the motor has to turn --> see function below
          //Serial.println(whichDirection1);
          // now it's time to action the motor!
          for(int i = 0; i < 8; i++) {
            digitalWrite(SR[whichSR1][0], LOW);
            shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, motorDrive[whichDirection1][i]);
            digitalWrite(SR[whichSR1][0], HIGH);
            delay(1);
          }
        }
      }
      break;
    case 2:
      //light up the 2 first leds
      digitalWrite(enable1Pin, HIGH);
      digitalWrite(enable2Pin, HIGH);
      digitalWrite(enable3Pin, LOW);
      digitalWrite(enable4Pin, LOW);
      // here I need to use the array2 to know which buttons is pressed.
      Array2[0] = 0;
      Array2[1] = 0;
      // action two motors
      // we need 2 whichSR, 2 whichDirection and 2 buttonPressed
      for(int m = 0; m < 4; m++) {
        if(buttonPressedArray[m] != 0) {
          if(Array2[0] == 0) {
            Array2[0] = buttonPressedArray[m];
          } else {
            Array2[1] = buttonPressedArray[m];
          } // now our Array2 is completed with the 2 buttons pressed
        }
        /*
        Serial.println("Array 2 : ");
        Serial.print(Array2[0], DEC);
        Serial.print(Array2[1], DEC);
        Serial.println();
        */
      }
      // we have our two values, now we need the motors to turn!
      whichSR1 = getNumberOfSH(Array2[0]);
      whichSR2 = getNumberOfSH(Array2[1]);
      //Serial.println(whichSR2);
      whichDirection1 = getDirection(Array2[0]);
      whichDirection2 = getDirection(Array2[1]);
      for(int i = 0; i < 8; i++) {
        // here I have to see if I have to action two motors on the same shift register or not.
        if(whichSR1 != whichSR2) { 
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, motorDrive[whichDirection1][i]);
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, motorDrive[whichDirection2][i]);
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          delay(1);
        } else { // whichSR1 == whichSR2
          // problem if same SR then need to combine bites together;
          digitalWrite(SR[whichSR1][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection2][i]));
          digitalWrite(SR[whichSR2][0], HIGH);
          delay(1);
        }
      }
      break;
    case 3:
      // light up 3 leds
      digitalWrite(enable1Pin, HIGH);
      digitalWrite(enable2Pin, HIGH);
      digitalWrite(enable3Pin, HIGH);
      digitalWrite(enable4Pin, LOW);
      // action now 3 motors at the same time
      // here I will copie the buttons that are pressed into Array3
      Array3[0] = 0;
      Array3[1] = 0;
      Array3[2] = 0;
      for(int m = 0; m < 4; m++) {
        if(buttonPressedArray[m] != 0) {
          if(Array3[0] == 0) {
            Array3[0] = buttonPressedArray[m];
          } else if(Array3[1] == 0) {
            Array3[1] = buttonPressedArray[m];
          } else {
            Array3[2] = buttonPressedArray[m];
          }
        }
        //Serial.println("Array 3 : ");
        //Serial.print(Array3[0], DEC);
        //Serial.print(Array3[1], DEC);
        //Serial.print(Array3[2], DEC);
        //Serial.println();
      }
      // we have our three values, now we need the 3 motors to turn!
      whichSR1 = getNumberOfSH(Array3[0]);
      whichSR2 = getNumberOfSH(Array3[1]);
      whichSR3 = getNumberOfSH(Array3[2]);
      //Serial.println(whichSR1);
      whichDirection1 = getDirection(Array3[0]);
      whichDirection2 = getDirection(Array3[1]);
      whichDirection3 = getDirection(Array3[2]);
      
      if(whichSR1 != whichSR2 && whichSR2 != whichSR3 && whichSR1 != whichSR3) {  
        // ze zijn allemaal verschillend
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          digitalWrite(SR[whichSR3][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, motorDrive[whichDirection1][i]);
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, motorDrive[whichDirection2][i]);
          shiftOut(SR[whichSR3][2], SR[whichSR3][1], MSBFIRST, motorDrive[whichDirection3][i]);
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          digitalWrite(SR[whichSR3][0], HIGH);
          delay(1);
        }
      } 
      if(whichSR1 == whichSR2 && whichSR1 != whichSR3) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR3][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection2][i]));
          shiftOut(SR[whichSR3][2], SR[whichSR3][1], MSBFIRST, (motorDrive[whichDirection3][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR3][0], HIGH);
          delay(1);
        }
      }
      if(whichSR2 == whichSR3 && whichSR2 != whichSR1) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR2][0], LOW);
          digitalWrite(SR[whichSR1][0], LOW);
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, (motorDrive[whichDirection2][i] | motorDrive[whichDirection3][i]));
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i]));
          digitalWrite(SR[whichSR2][0], HIGH);
          digitalWrite(SR[whichSR1][0], HIGH);
          delay(1);
        }
      }
      if(whichSR1 == whichSR3 && whichSR1 != whichSR2) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection3][i]));
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, (motorDrive[whichDirection2][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          delay(1);
        }
      }
      break;
    case 4:
      //light up the 4 leds
      digitalWrite(enable1Pin, HIGH);
      digitalWrite(enable2Pin, HIGH);
      digitalWrite(enable3Pin, HIGH);
      digitalWrite(enable4Pin, HIGH);
      // action now 4 motors at the same time
      // now that I'm checking the code again, I see that it is stupid to just copy an array into an other.
      Array4[0] = 0;
      Array4[1] = 0;
      Array4[2] = 0;
      Array4[3] = 0;
      
      for(int m = 0; m < 4; m++) {
        if(buttonPressedArray[m] != 0) {  // will always be the case --> stupid
          if(Array4[0] == 0) {
            Array4[0] = buttonPressedArray[m];
          } else if(Array4[1] == 0) {
            Array4[1] = buttonPressedArray[m];
          } else if(Array4[2] == 0) {
            Array4[2] = buttonPressedArray[m];
          } else {
            Array4[3] = buttonPressedArray[m];
          }
        }
        //Serial.println("Array 4 : ");
        //Serial.print(Array4[0], DEC);
        //Serial.print(Array4[1], DEC);
        //Serial.print(Array4[2], DEC);
        //Serial.print(Array4[3], DEC);
        //Serial.println();
      }
      // we have our four values, now we need the 4 motors to turn!
      whichSR1 = getNumberOfSH(Array4[0]);
      whichSR2 = getNumberOfSH(Array4[1]);
      whichSR3 = getNumberOfSH(Array4[2]);
      whichSR4 = getNumberOfSH(Array4[3]);
      //Serial.println(whichSR4);
      whichDirection1 = getDirection(Array4[0]);
      whichDirection2 = getDirection(Array4[1]);
      whichDirection3 = getDirection(Array4[2]);
      whichDirection4 = getDirection(Array4[3]);
      
      if(whichSR1 != whichSR2 && whichSR2 != whichSR3 && whichSR3 != whichSR4 && whichSR1 != whichSR4) {
        // ze zijn allemaal verschillend
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          digitalWrite(SR[whichSR3][0], LOW);
          digitalWrite(SR[whichSR4][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, motorDrive[whichDirection1][i]);
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, motorDrive[whichDirection2][i]);
          shiftOut(SR[whichSR3][2], SR[whichSR3][1], MSBFIRST, motorDrive[whichDirection3][i]);
          shiftOut(SR[whichSR4][2], SR[whichSR4][1], MSBFIRST, motorDrive[whichDirection4][i]);
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          digitalWrite(SR[whichSR3][0], HIGH);
          digitalWrite(SR[whichSR4][0], HIGH);
          delay(1);
        }
      }
      // 1 = 2 en 3 = 4 
      if(whichSR1 == whichSR2 && whichSR3 == whichSR4) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR3][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection2][i]));
          shiftOut(SR[whichSR3][2], SR[whichSR3][1], MSBFIRST, (motorDrive[whichDirection3][i] | motorDrive[whichDirection4][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR3][0], HIGH);
          delay(1);
        }
      }
      // 1 = 3 en 2 = 4
      if(whichSR1 == whichSR3 && whichSR2 == whichSR4) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection3][i]));
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, (motorDrive[whichDirection2][i] | motorDrive[whichDirection4][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          delay(1);
        }
      }
      // 1 = 4 en 2 = 3
      if(whichSR1 == whichSR4 && whichSR2 == whichSR3) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR2][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection4][i]));
          shiftOut(SR[whichSR2][2], SR[whichSR2][1], MSBFIRST, (motorDrive[whichDirection2][i] | motorDrive[whichDirection3][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR2][0], HIGH);
          delay(1);
        }
      }
      if(whichSR1 == whichSR2 && whichSR2 != whichSR3 && whichSR3 != whichSR4) {
        // problem if same SR then need to combine bites together;
        for(int i = 0; i < 8; i++) {
          digitalWrite(SR[whichSR1][0], LOW);
          digitalWrite(SR[whichSR3][0], LOW);
          digitalWrite(SR[whichSR4][0], LOW);
          shiftOut(SR[whichSR1][2], SR[whichSR1][1], MSBFIRST, (motorDrive[whichDirection1][i] | motorDrive[whichDirection2][i]));
          shiftOut(SR[whichSR3][2], SR[whichSR3][1], MSBFIRST, (motorDrive[whichDirection3][i]));
          shiftOut(SR[whichSR4][2], SR[whichSR4][1], MSBFIRST, (motorDrive[whichDirection4][i]));
          digitalWrite(SR[whichSR1][0], HIGH);
          digitalWrite(SR[whichSR3][0], HIGH);
          digitalWrite(SR[whichSR4][0], HIGH);
          delay(1);
        }
      }
      // here are only some of the possibilities because --> I only use the combinations that are frequently happening.
      break;
  }  
}
// end void loop() --------------------------------------------------------------------------------------------

byte getNumberOfSH(byte a) {    // return the shift register number 
  switch(a) {
    case 1: return 0;  //0 = SN74hc595n number 1
    case 2: return 0;  //1 = SN74hc595n number 2 
    case 3: return 1;  //2 = SN74hc595n number 3
    case 4: return 0;  //3 = Sn74hc595n number 4
    case 5: return 0;
    case 6: return 1;
    case 7: return 2;
    case 8: return 1;
    case 9: return 2;
    case 10: return 1;
    case 11: return 2;
    case 12: return 2;
    case 13: return 3;
    case 14: return 3;
    // case 15 still no function  because it's button center
    case 16: return 3;
    case 17: return 3;
  }
}

byte getDirection(byte a) {   // return the direction the motor turn
  switch(a) {
    case 1: return 0;     //0 = CW1
    case 2: return 1;     //1 = CW2
    case 3: return 1;     //2 = CCW1
    case 4: return 2;     //3 = CCW2
    case 5: return 3;
    case 6: return 3;
    case 7: return 2;
    case 8: return 0;
    case 9: return 1;
    case 10: return 2;
    case 11: return 0;
    case 12: return 3;
    case 13: return 2;
    case 14: return 3;
    // case 15 still no function because center button
    case 16: return 1;
    case 17: return 0;
  }
}
