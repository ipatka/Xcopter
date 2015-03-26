//
//  MotorControl.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "MotorControl.h"
#include "PIDControl.h"


MotorControl::MotorControl(){
    //Initialize the PIDs
    new PIDController();
}

MotorControl::~MotorControl(){
    
}

