//
//  Binary.h
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__Binary__
#define __Xcopter__Binary__

#include <stdint.h>
#include <avr/io.h>


class Binary {
    
    
public:

    
    Binary(); //constructor
    ~Binary(); //deconstructor
    
    static void setBit(volatile uint8_t *byte, const unsigned char bit);
    static void clearBit(volatile uint8_t *byte, const unsigned char bit);
    static unsigned char readBit(unsigned char byte, const unsigned char bit);
    static void toggleBit(volatile uint8_t *byte, const unsigned char bit);
    
};



#endif /* defined(__Xcopter__Binary__) */
