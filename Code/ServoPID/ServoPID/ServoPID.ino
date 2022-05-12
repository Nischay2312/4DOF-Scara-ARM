#include "PID.h"
#include <Wire.h>
#include <Servo.h>
#include <Math.h>

#define pi 3.14285714286

void InverseKin(float Xc, float Yc);
void ForwardKin(float T1, float T2); 

// Declare the Servo pin 
int servoPin = 3; 
// Create a servo object 
Servo Servo1;
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

double PIDstructure[11];
double PIDstructure1[11];

#define ControlFrequency 250.0
#define Setpoint 0.0

long timestart;
int counter = 0;
#define rep 5000
 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(19200);
PIDInit(PIDstructure, Setpoint, ControlFrequency);
PIDInit(PIDstructure1, Setpoint, ControlFrequency);
Servo1.attach(servoPin); 
timestart = millis();
Serial.print("\nStart Measuring\n");

}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//delay(400);

//Compute Dir. Kin current position of Robot
double Cords[2];
DirectKin(x, y, Cords);

//Compute InvKin
float Thetas[2];
InverseKin(0.0, 80.0, Thetas);

updateSetpoint(PIDstructure, Thetas[0]);
updateSetpoint(PIDstructure, Thetas[1]);
  
char outputBuffer[150];

//Get the current sensor reading and compute PID
double pv = x, pv1 = y;
PIDcalculate(PIDstructure, pv);
PIDcalculate(PIDstructure, pv1);

//WRITE TO SERVOS
int Sangle = map(PIDstructure[8],-255, 255, 0,180);
int Sangle1 = map(PIDstructure1[8],-255, 255, 0,180);
Servo1.write(Sangle);
Servo1.write(Sangle1); 

  
//sprintf(outputBuffer, "Pv: %d \t Error: %d \t Output: %d\t Intrgral: %d \tServo: %d\n", (int)pv, (int)PIDstructure[7], (int)PIDstructure[8], (int)PIDstructure[10], Sangle); 
//Serial.print(outputBuffer);

//delay(10);
counter++;
if(counter > rep)
{
  long timenow = millis() - timestart;
  while(1){
    Serial.print(timenow);
    Serial.print("\n");
    delay(5000);
  }
}
}

void InverseKin(float Xpos, float Ypos, float * Thetas){
    float L1 = 30;
    float L2 = 15;
    float L3 = sqrt(pow(Xpos,2) + pow(Ypos,2));
    float a = acos((L1*L1 + L3*L3 - L2*L2)/(2*L1*L3));
    float b = acos((L1*L1 + L2*L2 - L3*L3)/(2*L2*L1));
    float Theta = atan2(Ypos,Xpos);
    Thetas[0] = ((Theta + a));
    Thetas[1] = (((b - (pi - Thetas[0])))); 
}

void DirectKin(double T1, double T2, double * Cords){
    float Larm1 = 30;
    float Larm2 = 15;
    
    float arm1X = Larm1 * cos(T1*pi/180);
    float arm1Y = Larm1 * sin(T1*pi/180);

    float arm2X = Larm2 * cos(T2*pi/180) + arm1X;
    float arm2Y = Larm2 * sin(T2*pi/180) + arm1Y;
    
    Cords[1] = arm2Y;
    Cords[0] = arm2X;
}
