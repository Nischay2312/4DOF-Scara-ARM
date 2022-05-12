/*
 * Author: Nischay Joshi
 * Date: Feb 26, 2022
 * Notes: 
 *          PID Controller Header File    
 *          Bunch of functions to Initialize, calculate the controller and to change the setpoint. 
 *
 */
#ifndef _PID_H    
#define _PID_H

void PIDInit(double *PIDstructure, double setpoint, double controlFrequency);
void PIDcalculate(double *PIDstructure, double processVariable);
void updateSetpoint( double *PIDstructure, double setpoint);

#endif
