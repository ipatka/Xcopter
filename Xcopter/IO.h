//
//  IO.h
//  Xcopter
//
//  Created by Zach Lite on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef Xcopter_IO_h
#define Xcopter_IO_h

#include <avr/io.h>

#define DATA_DIR_REG_B &DDRB
#define DATA_DIR_REG_C &DDRC
#define DATA_DIR_REG_D &DDRD
#define DATA_DIR_REG_E &DDRE
/***********************/

//motors
#define MOTOR_DDR &DDRB
#define MOTOR_PORT &PORTB
#define MOTOR_WRITE_1 PORTB0
#define MOTOR_WRITE_2 PORTB1
#define MOTOR_WRITE_3 PORTB2
#define MOTOR_WRITE_4 PORTB3
/***********************/

//gyro
#define GYR0_PORT &PORTD
#define GYRO_WRITE_1 PORTD0

#define GYR0_PIN &PIND
#define GYRO_READ_1 PIND0


#endif
