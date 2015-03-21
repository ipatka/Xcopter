//
//  GPIOPin.h
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__GPIOPin__
#define __Xcopter__GPIOPin__

#include <stdio.h>
#include "Binary.h"

typedef enum {INPUT, OUTPUT} io_type;

class GPIOPin {
    volatile uint8_t *data_direction_register;
    volatile uint8_t *port_reg;
    volatile uint8_t *pin_reg;
    volatile uint8_t *bit_num; //the pin or port bit number you plan to read or write from
    io_type gpio_type;

public:
    GPIOPin(volatile uint8_t *ddr, volatile uint8_t *reg, io_type t);
    GPIOPin();
    
    void Write(const unsigned char *data);
    const unsigned char Read(void);
    
    
};

#endif /* defined(__Xcopter__GPIOPin__) */
