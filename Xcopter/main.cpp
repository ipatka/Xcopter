#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Quadcopter.h"

using namespace std;


int main (void){


    Quadcopter *xcopter = new Quadcopter();
    
    if (xcopter) {
        xcopter->Fly();
    }
    
    
    
    //should init peripherals...without need to make a peripheral object
    
    //decide on DDRs, ports, and pins for each peripheral now
    
    
    return 0;
}