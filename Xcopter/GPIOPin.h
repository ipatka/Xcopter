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
    
private:
    volatile uint8_t *data_direction_register;
    volatile uint8_t *port_reg;
    volatile uint8_t *pin_reg;
    unsigned char bit_num; //the pin or port bit number you plan to read or write from
    io_type gpio_type;
    
    void Initialize();

    
public:
    GPIOPin(volatile uint8_t *ddr, volatile uint8_t *reg, const unsigned char b, io_type t);
    ~GPIOPin();
    
    void Write(const unsigned char data);
    unsigned char Read(void);
    void Toggle(void);
    
};

#endif /* defined(__Xcopter__GPIOPin__) */
