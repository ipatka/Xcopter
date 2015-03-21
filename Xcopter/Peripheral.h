//
//  Peripheral.h
//  Xcopter
//
//  Created by Zach Lite on 3/20/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__Peripheral__
#define __Xcopter__Peripheral__

#include <stdio.h>
#include <avr/io.h>

class Peripheral {

    
private:
    volatile uint8_t *DDR;
    unsigned char input_bit;
    unsigned char output_bit;


public:
    Peripheral(volatile uint8_t *data_direction_register, const unsigned char in_bit, const unsigned char out_bit); //constructor
    ~Peripheral(); //deconstructor
    
    int initialize();
    
    volatile uint8_t getIO(void);
    
};



#endif /* defined(__Xcopter__Peripheral__) */
