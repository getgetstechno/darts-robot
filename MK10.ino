// INCLUDE ALL LIBRARIES
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

#include "NewPing.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

// ==========================================================================
// ALL VARIABLES CONCERNING THE ACCELEROMETRE ===============================
// ==========================================================================

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

float pitch;
float generalAngleOffset = 0.2;

// ==========================================================================
// ALL VARIABLES CONCERNING THE ULTRASONIC SENSORS ==========================
// ==========================================================================

const byte Trig1 = 3;
const byte Echo1 = 3;
const byte Trig3 = 17;
const byte Echo3 = 17;
const byte Trig4 = 16;
const byte Echo4 = 16;
const byte max_distance = 144;
 
float duration1, distance1;
float duration2, distance2;
float duration3, distance3;
float duration4, distance4;

int iterations = 2;

NewPing sonar1(Trig1, Echo1, max_distance);
NewPing sonar3(Trig3, Echo3, max_distance);
NewPing sonar4(Trig4, Echo4, max_distance);

#define DHTPIN 23        // pin number of sensor
#define DHTTYPE DHT22   // type of sensor (AM2302)

int maxHum = 80;
int maxTemp = 50;

DHT dht(DHTPIN, DHTTYPE);

// =========================================================================
// REAL VARIABELS IN CODE ==================================================
// =========================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
#define DECODE_NEC

const char motorDrive[5][8] = {{B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000},   //CW1
                               {B00001000, B00001100, B00000100, B00000110, B00000010, B00000011, B00000001, B00001001},   //CW2
                               {B10010000, B00010000, B00110000, B00100000, B01100000, B01000000, B11000000, B10000000},   //CCW1
                               {B00001001, B00000001, B00000011, B00000010, B00000110, B00000100, B00001100, B00001000},   //CCW2
                               {B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000}};  //nothing

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
const byte clockPin3 = 4;
const byte dataPin3 = 0;
// pins for SN74hc595n (shift register) number 4 : control angle and X-wheels (6 wheels)
const byte latchPin4 = 14;
const byte clockPin4 = 12;
const byte dataPin4 = 27;

const char SR[4][3] = {{latchPin1, clockPin1, dataPin1},
                  {latchPin2, clockPin2, dataPin2},
                  {latchPin3, clockPin3, dataPin3},
                  {latchPin4, clockPin4, dataPin4}};

byte butCannon1 = 36;
byte butCannon2 = 35;
byte butCannon3 = 34;
byte triggerBut = 39;

int period = 100;   // every 100ms the esp32 will check if the IR received somthing -> see underneed
unsigned long time_now = 0;   // initilise the amount of miliseconds the esp32 is running -> see underneed

bool chargePart1 = false;
bool chargePart2 = false;

byte target = 10;
float generalOffset = 0.5;
float distanceLeft;
float distanceLeftNeeded = 0;
float angleNeeded = 0;
int chargingStatus = 1;

bool distanceFound = false;
bool angleFound = false;

int speeed = 0;

// ==========================================================================
// END VARIABELS START SETUP ================================================
// ==========================================================================

void setup() {
  // ==============================================================
  // code for the infrared sensor =================================
  // ==============================================================
  
  Serial.begin(115200);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.print(F("at pin "));
  Serial.println(IR_RECEIVE_PIN);
  dht.begin();
  
  // ===============================================================
  // setup section for the accelerometre ===========================
  // ===============================================================
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(52);
  mpu.setYGyroOffset(-51);
  mpu.setZGyroOffset(29);
  mpu.setXAccelOffset(-6767);
  mpu.setYAccelOffset(-1513);
  mpu.setZAccelOffset(5117);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // ==============================================================
  // declare all pins as inputs/outputs ===========================
  // ==============================================================
  
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
  
  pinMode(butCannon1, INPUT_PULLUP);
  pinMode(butCannon2, INPUT_PULLUP);
  pinMode(butCannon3, INPUT_PULLUP);
  pinMode(triggerBut, INPUT_PULLUP);

  // ===============================================================
  // useful setup code =============================================
  // ===============================================================
  
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("  HELLO  WORLD  ");
  lcd.setCursor(0,1);
  lcd.print("WAKE UP MY ROBOT");
  delay(1000);
  lcd.clear();

  /*
  bool settingsDone = false;
  while(settingsDone == false) {
    
  }
  */
  bool finishSetup = false;
  while(finishSetup == false) {
    if(IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.command);
      IrReceiver.resume(); // Enable receiving of the next value
      if (IrReceiver.decodedIRData.command == 0x46) {
        chargingStatus++;
      } else if (IrReceiver.decodedIRData.command == 0x15) {
        chargingStatus--;
      } else if (IrReceiver.decodedIRData.command == 0x40) {
        finishSetup = true;
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("CHAR. STATUS");
    lcd.setCursor(14,0);
    lcd.print(chargingStatus); 
  }
}

void loop() {
  if(chargingStatus == 1) {
    lcd.setCursor(14,0);
    lcd.print(chargingStatus);
    lcd.setCursor(0,1);
    lcd.print("CHARGING PART 1");  
    chargePartOne();
  } else if (chargingStatus == 2) {
    lcd.setCursor(14,0);
    lcd.print(chargingStatus);
    lcd.setCursor(0,1);
    lcd.print("CHARGING PART 2");  
    chargePartTwo();
  } else if (chargingStatus == 3) {
    lcd.setCursor(14,0);
    lcd.print(chargingStatus);
    lcd.setCursor(0,1);
    lcd.print("CHARGING PART 3");
    chargePartThree();
  } else if (chargingStatus >= 4) {
    lcd.setCursor(14,0);
    lcd.print(chargingStatus);
    lcd.setCursor(0,1);
    lcd.print("CHARGING DONE");
    delay(1000);
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("CHANGING DIST. ?");
    lcd.setCursor(0,1);
    lcd.print("CHANGING HEIG. ?");
    
    bool goFor = false;
    bool goBack = false;
    bool goDown = false;
    bool goUp = false;
    bool whichHeight = false;
    while(whichHeight == false) {
      if(IrReceiver.decode()) {
        Serial.println(IrReceiver.decodedIRData.command);
        IrReceiver.resume(); // Enable receiving of the next value
        if (IrReceiver.decodedIRData.command == 0x46) {
          goUp = true;
        } else if (IrReceiver.decodedIRData.command == 0x15) {
          goDown = true;
        } else if (IrReceiver.decodedIRData.command == 0x40) {
          whichHeight = true;
        } else if (IrReceiver.decodedIRData.command == 0x44) {
          goBack = true;
        } else if (IrReceiver.decodedIRData.command == 0x43) {
          goFor = true;
        }
      }
      if(goFor == true) {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("FOR");
        for(int j=0; j<200; j++) {
          for(int i = 0; i < 8; i++) {
            digitalWrite(SR[2][0], LOW);
            shiftOut(SR[2][2], SR[2][1], MSBFIRST, motorDrive[3][i]);
            digitalWrite(SR[2][0], HIGH);
            delay(1);
          }
        }
      }
      if(goBack == true) {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("BACK");
        for(int j=0; j<200; j++) {
          for(int i = 0; i < 8; i++) {
            digitalWrite(SR[2][0], LOW);
            shiftOut(SR[2][2], SR[2][1], MSBFIRST, motorDrive[1][i]);
            digitalWrite(SR[2][0], HIGH);
            delay(1);
          }
        }
      }
      if(goUp == true) {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("UP");
        for(int j=0; j<500; j++) {
          for(int i = 0; i < 8; i++) {
            digitalWrite(SR[2][0], LOW);
            shiftOut(SR[2][2], SR[2][1], MSBFIRST, motorDrive[0][i]);
            digitalWrite(SR[2][0], HIGH);
            delay(1);
          }
        }
      }
      if(goDown == true) {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("DOWN");
        for(int j=0; j<500; j++) {
          for(int i = 0; i < 8; i++) {
            digitalWrite(SR[2][0], LOW);
            shiftOut(SR[2][2], SR[2][1], MSBFIRST, motorDrive[2][i]);
            digitalWrite(SR[2][0], HIGH);
            delay(1);
          }
        }
      }
      goDown = false;
      goUp = false;
      goFor = false;
      goBack = false;
    }
    
    // now find target please...
    for(int canon = 0; canon <= 2; canon++) {
      // we have to select the distance we want to use for the angle and the right distance, therefore we will select it using the IR
      // We will do it by selecting first the hundreds and then the tens and the units
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DISTANCE : ");

      int distanceT = 0;
      int distanceU = 0;
      distanceFound = false;
      while(distanceFound == false) {
        if(IrReceiver.decode()) {
          Serial.println(IrReceiver.decodedIRData.command);
          IrReceiver.resume(); // Enable receiving of the next value
          if (IrReceiver.decodedIRData.command == 0x46) {
            distanceT++;
          } else if (IrReceiver.decodedIRData.command == 0x15) {
            distanceT--;
          } else if (IrReceiver.decodedIRData.command == 0x40) {
            distanceFound = true;
          } else if (IrReceiver.decodedIRData.command == 0x44) {
            distanceU--;
          } else if (IrReceiver.decodedIRData.command == 0x43) {
            distanceU++;
          }
          lcd.setCursor(12, 0);
          distanceLeftNeeded = (10*distanceT) + (distanceU);
          Serial.println(distanceLeftNeeded);
          if(distanceLeftNeeded == 0) {
            distanceFound = false;
          }
          lcd.print(distanceLeftNeeded);
        } 
      }
      distanceLeft = getRightDistance();
      Serial.println(distanceLeftNeeded);
      bool distanceOk = false;
      while (distanceOk == false) {
        float value1 = distanceLeftNeeded - generalOffset;
        while(distanceLeft <= value1) {
          if(value1 - distanceLeft > 5) {
            speeed = (value1 - distanceLeft)*7;  
          } else if (value1 - distanceLeft <= 5 && value1 - distanceLeft > 2) {
            speeed = (value1 - distanceLeft)*4;
          } else {
            speeed = 3;
          }
          // move robot to the right
          //Serial.println("Move robot to the left");
          int j = 0;
          while (j < 5*speeed) {
            for(int i = 0; i < 8; i++) {
              digitalWrite(SR[3][0], LOW);
              shiftOut(SR[3][2], SR[3][1], MSBFIRST, motorDrive[3][i]);
              digitalWrite(SR[3][0], HIGH);
              delay(1);
            }
            j++;
          }
          delay(200);
          distanceLeft = getRightDistance();
          lcd.setCursor(6,1);
          lcd.print(distanceLeft);
        }
        float value2 = distanceLeftNeeded + generalOffset;
        while(distanceLeft > value2) {
          if(distanceLeft - value2 > 5) {
            speeed = (distanceLeft - value2)*7;  
          } else if (distanceLeft - value2 <= 5 && distanceLeft-value2 > 2) {
            speeed = (distanceLeft - value2)*4;
          } else {
            speeed = 3;
          }
          // move robot to the left
          //Serial.println("Move robot to the right");
          int j = 0;
          while (j < 5*speeed) {
            for(int i = 0; i < 8; i++) {
            digitalWrite(SR[3][0], LOW);
            shiftOut(SR[3][2], SR[3][1], MSBFIRST, motorDrive[1][i]);
            digitalWrite(SR[3][0], HIGH);
            delay(1);
            }
            j++;
          }
          delay(200);
          distanceLeft = getRightDistance();
          lcd.setCursor(6,1);
          lcd.print(distanceLeft);
        }
        delay(1000);
        distanceLeft = getRightDistance();
        lcd.setCursor(6,1);
        lcd.print(distanceLeft);
        Serial.println(distanceOk);
        if((distanceLeft > distanceLeftNeeded - generalOffset) && (distanceLeft < distanceLeftNeeded + generalOffset)) {
          // ok we are in our inbteval
          distanceOk = true;
        }
      }

      // position left right done
      // angle start

      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ANGLE : ");
      
      int angleT = 0;
      int angleU = 0;
      angleFound = false;
      while(angleFound == false) {
        if(IrReceiver.decode()) {
          Serial.println(IrReceiver.decodedIRData.command);
          IrReceiver.resume(); // Enable receiving of the next value
          if (IrReceiver.decodedIRData.command == 0x46) {
            angleT++;
          } else if (IrReceiver.decodedIRData.command == 0x15) {
            angleT--;
          } else if (IrReceiver.decodedIRData.command == 0x40) {
            angleFound = true;
          } else if (IrReceiver.decodedIRData.command == 0x44) {
            angleU--;
          } else if (IrReceiver.decodedIRData.command == 0x43) {
            angleU++;
          }
          
          lcd.setCursor(9, 0);
          angleNeeded = (angleT*10)+(angleU);
          Serial.println(angleNeeded);
          Serial.println(angleNeeded / 10);
          if(angleNeeded == 0) {
            angleFound = false;
          }
          lcd.print(angleNeeded/10);
        } 
      }
      
      pitch = getAngle();
      Serial.println(angleNeeded);
      bool angleOk = false;
      while (angleOk == false) {
        float value1 = (angleNeeded/10) - generalAngleOffset;
        Serial.println("value 1 : ");
        Serial.println(value1);
        while(pitch <= value1) {
          if((value1 - pitch) > 4) {
            speeed = abs((value1 - pitch)*20);  
          } else if (((value1 - pitch) <= 4) && ((value1 - pitch) > 2)) {
            speeed = abs((value1 - pitch)*10);
          } else {
            speeed = 6;
          }
          Serial.println(speeed);
          // angle down
          //Serial.println("angle down");
          int j = 0;
          while (j < speeed*5) {
            for(int i = 0; i < 8; i++) {
            digitalWrite(SR[3][0], LOW);
            shiftOut(SR[3][2], SR[3][1], MSBFIRST, motorDrive[2][i]);
            digitalWrite(SR[3][0], HIGH);
            delay(1);
            }
            j++;
          }
          delay(300);
          pitch = getAngle();
          lcd.setCursor(6, 1);
          lcd.print(pitch);
        }
        float value2 = (angleNeeded/10) + generalAngleOffset;
        Serial.println("value 2 : ");
        Serial.println(value2);
        while(pitch > value2) {
          // angle up
          if((pitch - value2) > 4) {
            speeed = abs((pitch - value2)*20);  
          } else if(((pitch - value2) <= 2) && ((pitch - value2) > 2)) {
            speeed = abs((pitch - value2)*10);
          } else {
            speeed = 4;
          }
          Serial.println(speeed);
          //Serial.println("angle up");
          int j = 0;
          while (j < speeed*5) {
            for(int i = 0; i < 8; i++) {
            digitalWrite(SR[3][0], LOW);
            shiftOut(SR[3][2], SR[3][1], MSBFIRST, motorDrive[0][i]);
            digitalWrite(SR[3][0], HIGH);
            delay(1);
            }
            j++;
          }
          delay(300);
          pitch = getAngle();
          lcd.setCursor(6, 1);
          lcd.print(pitch);
        }
        delay(1000);
        pitch = getAngle();
        lcd.setCursor(6, 1);
        lcd.print(pitch);
        if(pitch > (angleNeeded/10) - generalAngleOffset && pitch < (angleNeeded/10) + generalAngleOffset) {
          // ok we are in our interval
          angleOk = true;
        }
      }
      delay(1000);
      // angle done
      // ready to shoot !!!
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("READY TO SHOOT");
      distanceLeft = getRightDistance();
      lcd.setCursor(0,0);
      lcd.print(distanceLeft);
      pitch = getAngle();
      lcd.setCursor(10, 0);
      lcd.print(pitch);
      bool shoot = false;
      while(shoot == false) {
        if(IrReceiver.decode()) {
          Serial.println(IrReceiver.decodedIRData.command);
          IrReceiver.resume(); // Enable receiving of the next value
          if (IrReceiver.decodedIRData.command == 0x46) {
            int t = 0;
            while(t < 900) {
              for(int i = 0; i < 8; i++) {
                digitalWrite(SR[1][0], LOW);
                shiftOut(SR[1][2], SR[1][1], MSBFIRST, (motorDrive[2][i]));
                digitalWrite(SR[1][0], HIGH);
                delay(1);
              }
              t++;
            }
          }
          else if (IrReceiver.decodedIRData.command == 0x15) {
            int t = 0;
            while(t < 300) {
              for(int i = 0; i < 8; i++) {
                digitalWrite(SR[1][0], LOW);
                shiftOut(SR[1][2], SR[1][1], MSBFIRST, (motorDrive[2][i]));
                digitalWrite(SR[1][0], HIGH);
                delay(1);
              }
              t++;
            }
          }
          else if (IrReceiver.decodedIRData.command == 0x43) {
            shoot = true;
          }
        } 
      }
    // shoot done
    }
  chargingStatus = 1;
  }
}

void chargePartOne() {
  bool endPart1 = false;
  byte value1 = 4;
  byte value2 = 4;
  byte value3 = 4;
  while(endPart1 == false) {
    if(IrReceiver.decode()) {
      IrReceiver.resume(); // Enable receiving of the next value
      if (IrReceiver.decodedIRData.command == 0x52) {
        endPart1 = true;
        chargingStatus++;
      }
    } 
    // load here the canon
    if(digitalRead(butCannon1) == 0) {
      value1 = 1;
    } else {
      value1 = 4;
    }
    if(digitalRead(butCannon2) == 0) {
      value2 = 0;
    } else {
      value2 = 4;
    }
    if(digitalRead(butCannon3) == 0) {
      value3 = 3;
    } else {
      value3 = 4;
    }
    //Serial.println(value1);
    //Serial.println(value2);
    //Serial.println(value3);
    if(value1 != 4 || value2 != 4 || value3 != 4) {
      for(int i = 0; i < 8; i++) {
        digitalWrite(SR[0][0], LOW);
        digitalWrite(SR[1][0], LOW);
        shiftOut(SR[0][2], SR[0][1], MSBFIRST, (motorDrive[value1][i] | motorDrive[value2][i]));
        shiftOut(SR[1][2], SR[1][1], MSBFIRST, (motorDrive[value3][i]));
        digitalWrite(SR[0][0], HIGH);
        digitalWrite(SR[1][0], HIGH);
        delay(1);
      } 
    } else {
      endPart1 = true;
      chargingStatus++;
    }
  }
}

void chargePartTwo() {
  bool endPart2 = false;
  while(endPart2 == false) {
    // spinning the wheels up is ready -> go putting the trigger down
    byte valueTrigger = 4;
    if(digitalRead(triggerBut) == 0) {
      valueTrigger = 0;
    } else {
      valueTrigger = 4;
    }
    if(valueTrigger != 4) {
      for(int i = 0; i < 8; i++) {
        digitalWrite(SR[1][0], LOW);
        shiftOut(SR[1][2], SR[1][1], MSBFIRST, (motorDrive[valueTrigger][i]));
        digitalWrite(SR[1][0], HIGH);
        delay(1);
      } 
    } else {
      endPart2 = true;
      chargingStatus++;
    }  
  }
}
  

void chargePartThree() {
  int steps = 0;
  while(steps < 7000) {
    for(int i = 0; i < 8; i++) {
      digitalWrite(SR[0][0], LOW);
      digitalWrite(SR[1][0], LOW);
      shiftOut(SR[0][2], SR[0][1], MSBFIRST, (motorDrive[3][i] | motorDrive[2][i]));
      shiftOut(SR[1][2], SR[1][1], MSBFIRST, (motorDrive[1][i]));
      digitalWrite(SR[0][0], HIGH);
      digitalWrite(SR[1][0], HIGH);
      delayMicroseconds(750);
    }
    steps++;
  }
  chargingStatus++;
}

float getRightDistance() {
  
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  float soundcm = 0.0343;
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    soundcm = ((331.4 + (0.606 * temperature) + (0.0124 * humidity))/10000);
  }
  float distance;
  duration3 = sonar3.ping_median(iterations);
  duration4 = sonar4.ping_median(iterations);
  //Use 343 metres per second as speed of sound
  distance3 = (duration3 / 2) * soundcm;
  distance4 = (duration4 / 2) * soundcm;
  float difference = distance3 - distance4;
  if(abs(difference) < 10) {
    distance = ((distance3 + distance4) / 2);
    if (distance >= 400 || distance <= 2) {
      Serial.println("out of range");
    } else {
      return distance; 
    }
  } else {
    Serial.println("oeps we got an error or the robot is not possitioned perpandicular to the board");
    return distance;
  }
  Serial.println(distance);
}

float getAngle() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    pitch = ypr[2] * 180 / M_PI;
    return pitch;
    #endif
  }
}
