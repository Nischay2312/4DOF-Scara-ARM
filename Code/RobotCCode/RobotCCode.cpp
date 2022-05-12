#include "PID.h"            //Own Custom PID C++ file Header
#include <Wire.h>           //Arduino/Teensy General I2C Library
#include <Math.h>           //Used to compute complex mathematical funtions.

#define pi 3.14285714286
#define d2r pi/180.0
#define r2d 180.0/pi

//Function Definitions
void InverseKin(float Xc, float Yc);
void ForwardKin(float T1, float T2);
void BaseHoming();
void ArmHoming();
void PlanPath(float * Xcords, float * Ycords, float A1, float A2, float B1, float B2, float time, bool Polar);
void InverseKinematics(float * IKinData, float * Xcords, float * Ycords);
void DirectKinematics(float *DKinData, float BaseAngle, float ArmAngle);
void PIDControl();

//Pin Assignments
//Using 4 motors/actuators Base, Arm, Yaw and Up/Down and 2 Fans to cool robot.
//I2C is used to obtain sensor data.
//2 Limit Switches for Base and Arm Homing.
int BaseMotorL  =  2;
int BaseMotorR  =  3;
int ArmMotorL   =  4;
int ArmMotorR   =  5;
int YawMotorL   =  6;
int YawMotorR   =  7;
int UpDnU       =  8;
int UPDnD       =  9;
int Fan         = 10;
int SDA         = 18;       //Used For I2C
int SCL         = 19;       //Used For I2C
int SW1         = 20;
int SW2         = 21;

//Sensor I2C Addresses
const uint8_t BaseEn = 0x44;
const uint8_t ArmEn  = 0x45;
const uint8_t YawEn  = 0x46;

//PID 
float BasePID[11];
float ArmPID[11];
float YawPID[11]; 
#define ControlFrequency 1000.0

//Angle Data (Stores the Desired Angles) and Motor Inputs
volatile float BaseAngle = 0.0;
volatile float ArmAngle = 0.0;
volatile float YawAngle = 0.0;
volatile float UpDn = 0.0;
volatile float BasePIDout = 0.0;
volatile float ArmPIDout = 0.0;
volatile float YawPIDout = 0.0;
volatile float BasePV = 0.0, ArmPV = 0.0, YawPV = 0.0;      //These are the actual angles obtained from Encoder.

//Path Planning Array length
#define maxPathTime 10.0                            //The Max time the robot can take to complete a certain path. (seconds)
#define Pathfrequency 500.0                         //Frequency at which we feed new path data to the PID controller
#define maxLength = maxPathTime*Pathfrequency       //Max Size of array that will store the path data
float stepTime = 1/Pathfrequency;
float Xcords[maxLength];
float Ycords[maxLength];
float IKinData[2];
bool Modifying = false;
bool PathComplete = false;
volatile uint16_t PositionCounter = 0;
int PositionCounterLimit = 0;

//Using Teensy 4.1's Interval timer to generate interrupts for PID control
IntervalTimer PIDTimer;

void setup(){
    //SETUP I2C Communication
    Wire.begin(SDA, SCL);
    
    //Pin Initialization
    //Motors and Actuators
    pinMode(BaseMotorL, OUTPUT);
    pinMode(BaseMotorR, OUTPUT);
    pinMode(ArmMotorL, OUTPUT);
    pinMode(ArmMotorR, OUTPUT);
    pinMode(YawMotorL, OUTPUT);
    pinMode(YawMotorR, OUTPUT);
    pinMode(UpDnL, OUTPUT);
    pinMode(UpDnR, OUTPUT);    
    pinMode(Fan, OUTPUT);
    //Switches
    pinMode(SW1, INPUT);
    pinMode(SW2, INPUT);

    //PID Initialization
    PIDInit(BasePID, 1500.0, 200.0, 280.0, 0.0, ControlFrequency);
    PIDInit(ArmPID, 1250.0, 150.0, 50.0, 0.0, ControlFrequency);
    PIDInit(YawPID, 40.0, 6.1, 3.4, 0.0, ControlFrequency);

    //Homing
    BaseHoming();
    ArmHoming();

    //Start PID Interrupts
     PIDTimer.begin(PIDControl, 1/ControlFrequency*1000000);  // Run the Control at Control Frequency, time is in microseconds

     //Start the FANS to cool the system
     //Don't want to run them at full speed (danger of wearing them out)
     analogWrite(FAN, 180);
}

void loop(){
    //This the main loop, we do direct kinematics here along with path planning.
    
    //First Plan a Path, for the sake of simplicity we are only going to a straight line from Point A to B
    //We have decided to use multiple ways to input positions for the path, user can enter Polar or Cartesian Coordinates.

    //For now we use Polar. We go from A(45cm, 30degrees) to B(25cm, 60degrees)
    PathPlan(Xcords, Ycords, 45.0, 30.0, 25.0, 60.0, 2.0, true);

    //Now we calculate the Current X and Y postion using Direct Kinematics
    float DKinData[2];
    bool path1 = true;

    while(1){
    DirectKinematics(DKinData, BaseAngle, ArmAngle);
        //Our current path is finished, lets give a new one
        //To keep things simple and sane, we just make the robot go back to inital position.
        if(PathComplete && path1){
            PathPlan(Xcords, Ycords, 25.0, 60.0, 45.0, 30.0, 2.0, true);
            path1 = false;
        }
        else if(PathComplete && path1){
            PathPlan(Xcords, Ycords, 45.0, 30.0, 25.0, 60.0, 2.0, true);
            path1 = true;
        }
    }
}

void PIDControl(){
    //The Main PID Control Loop
    //Do Inverse Kinematics and get the Base, arm and Yaw angles
    if(!Modifying && !PathComplete){     //To verify that the path is not being modified currently or completed, otherwise don't change the current angles.
        InverKinematics(IKinData ,Xcords, Ycords);
        BaseAngle = IKinData[0];
        ArmAngle = IKinData[1];
    }
    //First Read the Sensor Data
    Wire.requestFrom(BaseEn, 4);    //Requesting Base encoder for 4 bytes (size of Float)
    //Since Wire.Read stores data sequentially and in 8 bits so we do this to get the angle.
    BasePV = Wire.Read()<<32|Wire.Read()<<16|Wire.Read()<<8|Wire.Read();        

    Wire.requestFrom(ArmEn, 4);    //Requesting Base encoder for 4 bytes (size of Float)
    //Since Wire.Read stores data sequentially and in 8 bits so we do this to get the angle.
    ArmPV = Wire.Read()<<32|Wire.Read()<<16|Wire.Read()<<8|Wire.Read();       
    Wire.requestFrom(YawEn, 4);    //Requesting Base encoder for 4 bytes (size of Float)
    //Since Wire.Read stores data sequentially and in 8 bits so we do this to get the angle.
    YawPV = Wire.Read()<<32|Wire.Read()<<16|Wire.Read()<<8|Wire.Read();        

    //Now Update the PID setponts incase the desired angles have been changed
    updateSetpoint(BasePID, BaseAngle);
    updateSetpoint(ArmPID, ArmAngle);
    updateSetpoint(YawPID, YawAngle);

    //Now Do the PID control Algorithm
    BasePIDout = PIDcalculate(BasePID, BasePV);
    ArmPIDout = PIDcalculate(ArmPID, BasePV);
    YawPIDout = PIDcalculate(YawPID, YawPV);

    //Output to the motors now.

    //Base Motor
    //If PID out value > 0 , then motors move right and left otherwise.
    if(BasePIDout > 0){
        analogWrite(BaseMotorR, BasePIDout);
    }
    else{
        analogWrite(BaseMotorL, abs(BasePIDout));
    }

    //Arm Motor
    //If PID out value > 0 , then motors move right and left otherwise.
    if(ArmPIDout > 0){
        analogWrite(ArmMotorR, ArmPIDout);
    }
    else{
        analogWrite(ArmMotorL, abs(ArmPIDout));
    }

    //Yaw Motor
    //If PID out value > 0 , then motors move right and left otherwise.
    if(YawPIDout > 0){
        analogWrite(YawMotorR, YawPIDout);
    }
    else{
        analogWrite(YawMotorL, abs(YawPIDout));
    }

    //Linear UP/DN Actuator
    //If UpDn value > 0 , then actuator go down and up otherwise.
    //The needs manual output as the actuator wokrs at 12V, and at output of 180 we get 12 V
    if(UpDn > 0){
        analogWrite(UpDnU, 180);
    }
    else{
        analogWrite(UPDnD, 180));
    }
}

void PlanPath(float * Xcords, float * Ycords, float A1, float A2, float B1, float B2, float time, bool Polar){
    //First set the Modifying Flag as 1;
    Modifying = true;

    float X1 = A1;
    float Y1 = A2;
    float X2 = B1;
    float Y2 = B2;
    //Now we need see if out data is in Cartessan or Polar if Polar then convert them to cartesisan.
    if(Polar){
        X1 = A1 * cos(A2*d2r);
        Y1 = A1 * sin(A2*d2r);
        X2 = B1 * cos(B2*d2r);
        Y2 = B1 * sin(B2*d2r);
    }

    //Now we plan path.
    //now we generate a straight line of the from y = mx + c
    float slope = (Y2 - Y1)/(X2 - X1);
    //now get c
    float constant = Y2 - slope * X2;
    
    //Now we calculate the number of steps need to complete the path in the required time.
    float timeSteps = time / stepTime;
    float Xstep = (X2 - X1) / timeSteps;
    float Ystep = (Y2 - Y1) / timeSteps;

    PositionCounterLimit = timeSteps;
    //Intialize the X and Y coordinate array
    Xcords[0] = X1;
    Ycords[1] = Y1; 

    //This loop will fill the array with the position data
    for(int i = 1; i < PositionCounterLimit; i++){
        if(Xstep == 0){         //This checks if we have a vertical line.
                Ycords[i] = Ycords[i-1] + Ystep;
                Xpos[i] = Xpos[i-1];
        }    
        else{
                Xcords[i] = Xcords[i - 1] + Xstep;
                Ycords[i] = Xcords[i] * slope + constant;
        }
    }

    //Now our Path is set so we set modifying flag back to false.
    Modifying = false;
    //We just updated the path so let the PID controller loop know that
    PathComplete = false;

}

void InverseKinematics(float * IKinData, float * Xcords, float * Ycords){
    //These are the arm lengths and L3 is the shortest distance b/w base and the endeffector.
    float L1 = 30.0;
    float L2 = 15.0;
    float L3 = sqrt(Xcords[PositionCounter]^2 + Ycords[PositionCounter]^2);
    
    //These mathematical expressions converr the desired X adn Y cordinates to Desired Base and Arm motor angles. MATH IS MAGIC!!!
    float a = acos(complex(pow(L1,2) + pow(L3,2) - pow(L2,2))/(2*L1*L3)));
    float b = acos(complex((pow(L1,2) + pow(L2,2) - pow(L3,2))/(2*L2*L1)));
    
    float Theta = atan2(y,x);
    IKinData[0] = real((Theta + a));
    IKinData[1] = -real(pi-b); 

    //Now time to update the Position Counter but first we check if we have finished sedning all the cordinates
    if(PositionCounter == PositionCounterLimit){     //We are done
        //We set the Path Complete flag as 1 to let the main loop know that the robot has reached its destination.
        PathComplete = true;
        PositionCounter = 0;        //Reset the position counter for next path data.
    }
    else{
        PositionCounter++;
    }
}

void DirectKinematics(float *DKinData, float BaseAngle, float ArmAngle){
    //This will use the current motor angles and get current X and Y coordinates of Endeffector    
    //Arm Lengths
    float Larm1 = 30;
    float Larm2 = 15;
    
    //These Mathematical Expressions Convert Current motor angles to Current X and Y coordinates. 
    float arm1X = Larm1 * cos(BaseAngle*pi/180);
    float arm1Y = Larm1 * sin(BaseAngle*pi/180);

    float arm2X = Larm2 * cos((ArmAngle + BaseAngle)*pi/180) + arm1X;
    float arm2Y = Larm2 * sin((ArmAngle + BaseAngle)*pi/180) + arm1Y;
    
    DKinData[0] = arm2Y;
    DKinData[1] = arm2X;
end
}

void BaseHoming(){
    //The idea is to keep moving the arm right until
    //the Limit Switch is pressed

    //Move the arm, (not at full speed to ensure nothing breaks) 
    analogWrite(BaseMotorR, 100);
    while(digitalRead(SW1 == 0)){

    }
    //If we are here then the switch is pressed
    //Send The current base arm angle to the encoder which is 30 degrees (determined using design tools) 
    BaseAngle = 30.0;

    //Now transmit this angle to Encoder Board, it will set this as the current motor angle. 
    Wire.BeginTransmission(BaseEn);
    Wire.write("Sending Current Angle");
    Wire.write(BaseAngle);
    Wire.endTransmission();
}


void ArmHoming(){
    //The idea is to keep moving the arm right until
    //the Limit Switch is pressed

    //Move the arm, (not at full speed to ensure nothing breaks) 
    analogWrite(ArmMotorR, 100);
    while(digitalRead(SW2 == 0)){

    }
    //If we are here then the switch is pressed
    //Send The current base arm angle to the encoder which is -120 degrees (determined using design tools) 
    ArmAngle = -120.0;

    //Now transmit this angle to Encoder Board, it will set this as the current motor angle. 
    Wire.BeginTransmission(ArmEn);
    Wire.write("Sending Current Angle");
    Wire.write(ArmAngle);
    Wire.endTransmission();
}