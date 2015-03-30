//
//  Quadcopter.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "Quadcopter.h"

//motors
//motor control

//peripherals



Quadcopter::Quadcopter(){
    //constructor initializes all IO and peripherals
    
    // Do these need to be here? Or does the constructor get called since we declare them in the quadcopter class?
    this->motor_control = new MotorControl();
    this->motion_sensor = new MotionSensor();
    
}


Quadcopter::~Quadcopter(){
    //deconstructor
}

void Quadcopter::Fly(){
   
    //main quadcopter loop
    //system has already been initialized
    
    while (1) {
        
        //check for warnings
        
        //check for an instruction from rf peripheral
        
        //check current orientation from MPU
        
        //Send a struct with the orientation to a PID method. PID method then runs PidController for each one and returns the throttle values in a struct to be sent to motors
        this->motion_sensor->GetSensorReadings(&sensor_data);
        this->motor_control->SetOrientation(&this->orientation, &sensor_data);

        
        //set motor control object with current and desired orientation
        
        //tell motor control to run loop
        
        
    }
    
}

