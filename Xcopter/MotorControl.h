//
//  MotorControl.h
//  Xcopter
//
//  Description:  This class implements motor control though a PID loop.  The output of the PID loop is fed to logic that produces PWM signals to appropriately control each speed controller.
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__MotorControl__
#define __Xcopter__MotorControl__

#include <stdio.h>
#include "IO.h"
#include "GPIOPin.h"
#include "MotionSensors.h"
#include "PIDControl.h"

/*
 INPUTS:
 gyro inputs
 desired orientation
 
 OUTPUTS:
 motor enable PWM signals
*/
typedef struct{
    char throttle;
    char roll, pitch, yaw;
}Orientation;

typedef struct{
    int front_left, front_right, back_left, back_right;
}Throttle;



class MotorControl {

private:
    //might not return void later
    void RunPID();
    void DutyCycleToSpeedController();
    void TurnMotorOn(GPIOPin *m);
    void TurnMotorOff(GPIOPin *m);
    
    GPIOPin *motor_1;
    GPIOPin *motor_2;
    GPIOPin *motor_3;
    GPIOPin *motor_4;
    
    PID_OUTPUT pid_output;
    PID_REFERENCES reference_values;
    
    Throttle throttle;
    
    PIDController pid_controller;
    
    

    
public:
    MotorControl();//should this have information about IO?
    ~MotorControl();
    
    //ControlMotors method runs the PID loop and adjusts the output pwm signals
    void ControlMotors();
    void SetOrientation(Orientation *orientation, SensorData *sensor_data);
    
};

#endif /* defined(__Xcopter__MotorControl__) */
