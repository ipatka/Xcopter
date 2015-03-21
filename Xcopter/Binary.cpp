//
//  Binary.cpp
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "Binary.h"


void Binary::Binary(){
    //constructor method
}

void Binary::~Binary(){
    //deconstructor method
}

void Binary::setBit(volatile uint8_t *byte, const unsigned char bit){
    *Byte = *Byte | (1 << BitToSet);
}

void Binary::clearBit(volatile uint8_t *byte, const unsigned char bit){
    *Byte &= ~(1 << BitToClear);
}

void Binary::readBit(unsigned char byte, const unsigned char bit){
    unsigned char ValueOfBit;
    ValueOfBit = (Byte & (1 << BitToRead)) >> BitToRead;
    return ValueOfBit;
}

void Binary::toggleBit(volatile uint8_t *byte, const unsigned char bit){
    *Byte = (*Byte ^= (1 << BitToToggle));
}