//
//  MotorControl.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "MotorControl.h"


MotorControl::MotorControl(){
    
    //initialize motors
    this->motor_1 = new GPIOPin(MOTOR_DDR, MOTOR_PORT, MOTOR_WRITE_1, OUTPUT);
    this->motor_2 = new GPIOPin(MOTOR_DDR, MOTOR_PORT, MOTOR_WRITE_2, OUTPUT);
    this->motor_3 = new GPIOPin(MOTOR_DDR, MOTOR_PORT, MOTOR_WRITE_3, OUTPUT);
    this->motor_4 = new GPIOPin(MOTOR_DDR, MOTOR_PORT, MOTOR_WRITE_4, OUTPUT);
}

MotorControl::~MotorControl(){
}





//low level motor control
void MotorControl::TurnMotorOn(GPIOPin *m){
    m->Write(1);
}

void MotorControl::TurnMotorOff(GPIOPin *m){
    m->Write(0);
}
