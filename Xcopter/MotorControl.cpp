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
    
    //Initialize the PIDs
    this->pid_controller = new PIDController();
}

MotorControl::~MotorControl(){
}


void MotorControl::ControlMotors() {
    
}



void MotorControl::SetOrientation(Orientation *orientation, SensorData *sensor_data) {
    reference_values.roll = orientation->roll;
    reference_values.yaw = orientation->yaw;
    reference_values.pitch = orientation->pitch;
    
    this->pid_controller->QuadPidController(&reference_values, sensor_data, &pid_output);
    
    throttle.front_left = orientation->throttle - pid_output.roll_output - pid_output.pitch_output - pid_output.yaw_output;
    
    throttle.back_left = orientation->throttle - pid_output.roll_output + pid_output.pitch_output + pid_output.yaw_output;
    
    throttle.front_right = orientation->throttle + pid_output.roll_output - pid_output.pitch_output + pid_output.yaw_output;
    
    throttle.back_right = orientation->throttle + pid_output.roll_output + pid_output.pitch_output - pid_output.yaw_output;
    
}



//low level motor control
void MotorControl::TurnMotorOn(GPIOPin *m){
    m->Write(1);
}

void MotorControl::TurnMotorOff(GPIOPin *m){
    m->Write(0);
}
