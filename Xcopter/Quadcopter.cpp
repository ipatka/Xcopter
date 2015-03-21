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
    InitializeMotors();
    InitializePeripherals();
    //MotorControl motor_control = new MotorControl();
}

Quadcopter::~Quadcopter(){
    
}

void Quadcopter::Fly(Orientation *o){
    orientation.throttle = o->throttle;
    orientation.roll = o->roll;
    orientation.pitch = o->pitch;
    orientation.yaw = o->yaw;
    
    //give this to motor control
    
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