//
//  GPIOPin.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "GPIOPin.h"

GPIOPin::GPIOPin(volatile uint8_t *ddr, volatile uint8_t *reg, const unsigned char b, io_type t){
  
    this->data_direction_register = ddr;
    this->gpio_type = t;
    this->bit_num = b;
    
    if (this->gpio_type == INPUT)
        this->pin_reg = reg;
    else
        this->port_reg = reg;

    this->Initialize();
}

GPIOPin::~GPIOPin(){
    //deconstructor method
}


void GPIOPin::Initialize(){
    
    if (this->gpio_type == INPUT) {
        Binary::clearBit(this->data_direction_register, this->bit_num);
    } else {
        Binary::setBit(this->data_direction_register, this->bit_num);
    }
    
}


//GPIOPin public methods

void GPIOPin::Write(const unsigned char *data){
    if (this->gpio_type == OUTPUT)
        Binary::setBit(this->port_reg, this->bit_num);
}

unsigned char GPIOPin::Read(void){
    if (this->gpio_type == INPUT)
        return Binary::readBit(this->pin_reg, this->bit_num);
    else
        return 2;//this sucks
}

void GPIOPin::Toggle(){
    if (this->gpio_type == OUTPUT)
        Binary::toggleBit(this->port_reg, this->bit_num);
}

