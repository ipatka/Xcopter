//
//  Peripheral.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "Peripheral.h"

Peripheral::Peripheral(volatile uint8_t *data_direction_register, const unsigned char in_bit, const unsigned char out_bit){
    //constructor method
    DDR = data_direction_register;
    input_bit = in_bit;
    output_bit = out_bit;

}

Peripheral::~Peripheral(){
    //deconstructor method
}


volatile uint8_t Peripheral::getIO(void){
    return *DDR;
}