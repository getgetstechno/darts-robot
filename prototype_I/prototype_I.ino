#include <Stepper.h>
#include <Servo.h> 

#define STEPS1 2048 // number of steps needed to make 1 turn
#define STEPS2 2048 

Stepper stepper1(STEPS1, 8, 10, 9, 11);
Stepper stepper2(STEPS2, 4, 6, 5, 7);

Servo myServo;

int potVal = 0;
int potVal2 = 0;
int potVal3 = 0;

void setup() {
  //Serial.begin(9600);
  stepper1.setSpeed(12);
  stepper2.setSpeed(12);

  myServo.attach(3);
  myServo.write(80);
  pinMode(2, INPUT_PULLUP);
}

void loop() {
  potVal = map(analogRead(A0),0,1024,0,500);
  potVal2 = map(analogRead(A1),0,1024,0,500);
  potVal3 = map(analogRead(A2), 0, 1024, 118, 130);
  myServo.write(potVal3);
    if (potVal>300) {
      stepper1.step(20);
    }  
    if (potVal<200) {
      stepper1.step(-20);
    } 
    if (potVal2>300) {
      stepper2.step(20);  
    }
    if (potVal2<200) {
      stepper2.step(-20);
    } 
    if (digitalRead(2) == LOW) {
      myServo.write(80);
    } 
    delay(30);
  //Serial.println(potVal); //for debugging
}
