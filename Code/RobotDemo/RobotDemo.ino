#include "BaseAngle.h"
#include "ArmAngles.h"
#include <Servo.h>
#include <Math.h>

#define sampleFrequency 1000
#define datapoints 1501

static uint8_t BaseServoPin = 10;
static uint8_t ArmServoPin = 11;
volatile uint16_t counter = 0;

Servo BaseMotor;
Servo ArmMotor;

long timenow = 0;
long timestart = 0;

void setup() {
  Serial.begin(115200);
  //Setup Servo
  BaseMotor.attach(BaseServoPin); 
  ArmMotor.attach(ArmServoPin);
  delay(5000);
  Serial.print("\nStarting now\n");
  timestart = micros(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  timenow = micros();
  if((timenow - timestart > sampleFrequency) && (counter < datapoints)){
    counter++;
    
    //move the servo 
    BaseMotor.write(BA[counter]);
    ArmMotor.write(AA[counter]);
    timestart = micros();
  }
  else if(counter >= datapoints){
    counter = 0;
    Serial.print("\nRepeating Now\n");
    //move the servo 
    BaseMotor.write(abs(BA[counter]));
    ArmMotor.write(abs(AA[counter]));
    delay(5000);
    timestart = micros();
  }
}
