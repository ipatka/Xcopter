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
    
    InitializeMotors();
    InitializePeripherals();
    //MotorControl motor_control = new MotorControl();
}

Quadcopter::~Quadcopter(){
    
}

void Quadcopter::Fly(){
   
    //main quadcopter loop
    //system has already been initialized
    
    while (1) {
        
        //check for warnings
        
        //check for an instruction from rf peripheral
        
        //check current orientation from gyro peripheral
        
        //set motor control object with current and desired orientation
        
        //tell motor control to run loop
        
        
    }
    
}

void Quadcopter::SetOrientation(Orientation *o){
    orientation.throttle = o->throttle;
    orientation.roll = o->roll;
    orientation.pitch = o->pitch;
    orientation.yaw = o->yaw;
}

Orientation Quadcopter::GetOrientation(){
    return orientation;
}

void Quadcopter::InitializeMotors(){
    //set up output pins for each motor's enable signal
    SetPinAsOutput(DATA_DIR_REG_B, MOTOR_WRITE_1);
    SetPinAsOutput(DATA_DIR_REG_B, MOTOR_WRITE_2);
    SetPinAsOutput(DATA_DIR_REG_B, MOTOR_WRITE_3);
    SetPinAsOutput(DATA_DIR_REG_B, MOTOR_WRITE_4);
}

void Quadcopter::InitializePeripherals(){
    //gyro
    SetPinAsInput(DATA_DIR_REG_D, GYRO_READ_1);
    SetPinAsOutput(DATA_DIR_REG_D, GYRO_WRITE_1);
    
}

void Quadcopter::SetPinAsInput(volatile uint8_t *DDR, const unsigned char pin_number){
    Binary::clearBit(DDR, pin_number);
}

void Quadcopter::SetPinAsOutput(volatile uint8_t *DDR, const unsigned char pin_number){
    Binary::setBit(DDR, pin_number);
}