//
//  Binary.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "Binary.h"


Binary::Binary(){
    //constructor method
}

Binary::~Binary(){
    //deconstructor method
}

void Binary::setBit(volatile uint8_t *byte, const unsigned char bit){
    *byte = *byte | (1 << bit);
}

void Binary::clearBit(volatile uint8_t *byte, const unsigned char bit){
    *byte &= ~(1 << bit);
}

unsigned char Binary::readBit(unsigned char byte, const unsigned char bit){
    unsigned char ValueOfBit;
    ValueOfBit = (byte & (1 << bit)) >> bit;
    return ValueOfBit;
}

void Binary::toggleBit(volatile uint8_t *byte, const unsigned char bit){
    *byte = (*byte ^= (1 << bit));
}