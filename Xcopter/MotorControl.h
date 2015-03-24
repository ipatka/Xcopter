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



class MotorControl {

private:
    //might not return void later
    void RunPID();
    void DutyCycleToSpeedController();
    
    
    
    
public:
    MotorControl();//should this have information about IO?
    ~MotorControl();
    
    //ControlMotors method runs the PID loop and adjusts the output pwm signals
    void ControlMotors();
    void SetOrientation(Orientation *o);
    
};

#endif /* defined(__Xcopter__MotorControl__) */
