/*
 * Author: Nischay Joshi
 * Date: Feb 26, 2022
 * Notes: 
 *          PID Controller    
 *          Bunch of functions to Initialize, calculate the controller and to change the setpoint. 
 *
 */
 
#include "PID.h"

/*
PID Structure MAP: [ 0      1       2       3       4       5       6            7         8         9             10]      
                     dt    max     min      kp      ki      kd     setpoint     error    output   preverror     integral
*/

#define max 255.0
#define min -255.0
#define ki 1.8
#define kd 0.6
#define kp 1.0

/*
 * Initialize PID Function
 * Parameters: PIDstructure     - A pointer to double array of 11 items that store the current status of the controller 
 *             setpoint         - Value at which we need to set the controller to.
 *             controlFrequenct - The frequency at which the controler is running.
 */
void PIDInit(double *PIDstructure, double setpoint, double controlFrequency){
    PIDstructure[0] = 1/controlFrequency;
    PIDstructure[1] = max;
    PIDstructure[2] = min;
    PIDstructure[3] = kp;
    PIDstructure[4] = ki;
    PIDstructure[5] = kd;
    PIDstructure[6] = setpoint;
    PIDstructure[7] = 0.0;
    PIDstructure[8] = 0.0;
    PIDstructure[9] = 0.0;
    PIDstructure[10] = 0.0;
}

/*
 * PID Calculate function
 * Parameters:  PIDstructure    - A pointer to an array that stores the current state of the controller
 *              processVariable - The reading from the sensor, i.e the feedback signal.
 */
void PIDcalculate(double *PIDstructure, double processVariable){
    //Set Error
    PIDstructure[7] = PIDstructure[6] - processVariable;
    
    //Calculate the K,I and D Terms 
    double KTerm = PIDstructure[7] * PIDstructure[3];
    double ITerm = (PIDstructure[10] + PIDstructure[7] * PIDstructure[0]) * PIDstructure[4];
    double DTerm = ((PIDstructure[7] - PIDstructure[9]) / PIDstructure[0]) * PIDstructure[5];

    //Compute Output
    PIDstructure[8] = KTerm + ITerm + DTerm;
    
    //Check if we are in the output limit
    if(PIDstructure[8] < min){
      PIDstructure[8] = min;
    }
    if(PIDstructure[8] > max){
      PIDstructure[8] = max;
    }
    
    //Set Intgeral and Previous Error
    PIDstructure[9] = PIDstructure[7];
    PIDstructure[10] = PIDstructure[10] + PIDstructure[7] * PIDstructure[0];
}

/*
 * Function to Update the setpoint for the controller
 * Parameters: PIDstructure - A pointer to the array that stores the stae of the controller
 *             setpoint     - The new value of the setpoint that we want to change to.
 */
void updateSetpoint( double *PIDstructure, double setpoint){
    PIDstructure[6] = setpoint; 
}
