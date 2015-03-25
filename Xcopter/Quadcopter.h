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

#include "IO.h"

#include "Binary.h"
#include "Peripheral.h"
#include "MotorControl.h"


class Quadcopter {
    
private:
    
    /*Private Instance Variables*/
    Orientation orientation;
    MotorControl motor_control;
    
    
    /*Private Methods*/
    
    void EmergencyLand();
    void SetOrientation(Orientation *o);
    Orientation GetOrientation();
    
    void InitializeMotors(void);
    void InitializePeripherals(void);
    

    
    void SetPinAsOutput(volatile uint8_t *DDR, const unsigned char pin_number);
    void SetPinAsInput(volatile uint8_t *DDR, const unsigned char pin_number);
    

public:
    
    //io is initialized in the constructor
    Quadcopter();
    ~Quadcopter();
    
    void Fly();
   
   
};


#endif /* defined(__Xcopter__Quadcopter__) */
