//
//  Quadcopter.h
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__Quadcopter__
#define __Xcopter__Quadcopter__

#include <stdio.h>
#include "Binary.h"
#include "Peripheral.h"
#include "MotorControl.h"
#include "MotionSensors.h"


#include <avr/io.h>

#define DATA_DIR_REG_B &DDRB
#define DATA_DIR_REG_C &DDRC
#define DATA_DIR_REG_D &DDRD
#define DATA_DIR_REG_E &DDRE
/***********************/

//motors
#define MOTOR_PORT &PORTB
#define MOTOR_WRITE_1 PORTB0
#define MOTOR_WRITE_2 PORTB1
#define MOTOR_WRITE_3 PORTB2
#define MOTOR_WRITE_4 PORTB3
/***********************/

//gyro
#define GYR0_PORT &PORTD
#define GYRO_WRITE_1 PORTD0

#define GYR0_PIN &PIND
#define GYRO_READ_1 PIND0





class Quadcopter {
    
private:
    
    /*Private Instance Variables*/
    Orientation orientation;
    MotorControl motor_control;
    SensorData sensor_data;
    
    
    /*Private Methods*/
    
    void EmergencyLand();
    void SetOrientation(Orientation *o);
    Orientation GetOrientation();
    
    void InitializeMotors(void);
    void InitializePeripherals(void);
    SensorData InitializeSensors(char type, MotionSensor motion_sensor);
    

    
    void SetPinAsOutput(volatile uint8_t *DDR, const unsigned char pin_number);
    void SetPinAsInput(volatile uint8_t *DDR, const unsigned char pin_number);
    

public:
    
    //io is initialized in the constructor
    Quadcopter();
    ~Quadcopter();
    
    void Fly();
   
   
};


#endif /* defined(__Xcopter__Quadcopter__) */
