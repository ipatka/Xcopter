//
//  Peripheral.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "Peripheral.h"

Peripheral::Peripheral(volatile uint8_t *data_direction_register, const unsigned char input_bit, const unsigned char output_bit){
    //constructor method
    DDR = data_direction_register;

}

Peripheral::~Peripheral(){
    //deconstructor method
}


void Peripheral::getIO(volatile uint8_t *data_direction_register, const unsigned char bit){
    return DDR;
}