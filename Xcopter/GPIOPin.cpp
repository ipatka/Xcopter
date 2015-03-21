//
//  GPIOPin.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "GPIOPin.h"

GPIOPin::GPIOPin(volatile uint8_t *ddr, volatile uint8_t *reg, io_type t){
    //constructor method
    data_direction_register = ddr;
    
    gpio_type = t;
    if (gpio_type == INPUT)
        pin_reg = reg;
    else
        port_reg = reg;
    
}


GPIOPin::GPIOPin(){
    //deconstructor method
}


void GPIOPin::Write(const unsigned char *data){
    if (gpio_type == OUTPUT) {

    }
}