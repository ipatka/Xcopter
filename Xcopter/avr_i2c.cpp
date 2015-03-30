//
//  avr_i2c.cpp
//  Xcopter
//
//  Created by Isaac Patka on 3/29/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "avr_i2c.h"

/** Create an I2C Master interface, connected to the specified pins
 *
 *  @param sda I2C data line pin
 *  @param scl I2C clock line pin
 */
I2C::I2C() {
    //set SCL to 100kHz
    TWSR = 0x00;
    TWBR = 0x20;
    //enable TWI
    TWCR = (1<<TWEN);
}

/** Set the frequency of the I2C interface
 *
 *  @param hz The bus frequency in hertz
 */
//void frequency(int hz);

/** Read from an I2C slave
 *
 *  Performs a complete read transaction. The bottom bit of
 *  the address is forced to 1 to indicate a read.
 *
 *  @param address 8-bit I2C slave address [ addr | 1 ]
 *  @param data Pointer to the byte-array to read data in to
 *  @param length Number of bytes to read
 *  @param repeated Repeated start, true - don't send stop at end
 *
 *  @returns
 *       0 on success (ack),
 *   non-0 on failure (nack)
 */
int I2C::read(int address, char *data, int length, bool repeated) {
    int stop = (repeated) ? 0 : 1;
    
    int count;
    char dummy_read, *ptr;
    
    this->start();
    
    this->write(address | 0x01);

    
    // Read in bytes
    for (count = 0; count < (length); count++) {
        ptr = (count == 0) ? &dummy_read : &data[count - 1];
        // shouldn't this line be opposite? do ACK reads until the last one then do a nack read?
        // this line says do nack reads until the last then do ack. need to test
        uint8_t stop_ = (count == (length - 1)) ? 1 : 0;
        *ptr = this->read(stop_);
    }
    
    // If not repeated start, send stop.
    if (stop)
        this->stop();
    
    // last read
    data[count-1] = this->read(0);
    
    
    
    return true;
}

/** Read a single byte from the I2C bus
 *
 *  @param ack indicates if the byte is to be acknowledged (1 = acknowledge)
 *
 *  @returns
 *    the byte read
 */
int I2C::read(int ack) {
    //ack
    if (ack == 1) {
        TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
        while ((TWCR & (1<<TWINT)) == 0);
        return TWDR;
    }
    //nack
    else {
        TWCR = (1<<TWINT)|(1<<TWEN);
        while ((TWCR & (1<<TWINT)) == 0);
        return TWDR;
    }
}

/** Write to an I2C slave
 *
 *  Performs a complete write transaction. The bottom bit of
 *  the address is forced to 0 to indicate a write.
 *
 *  @param address 8-bit I2C slave address [ addr | 0 ]
 *  @param data Pointer to the byte-array data to send
 *  @param length Number of bytes to send
 *  @param repeated Repeated start, true - do not send stop at end
 *
 *  @returns
 *       0 on success (ack),
 *   non-0 on failure (nack)
 */
int I2C::write(int address, const char *data, int length, bool repeated) {
    int stop = (repeated) ? 0 : 1;
    
    for (int i = 0; i < length; i++) {
        this->write(data[i]);
    }
    
    if (stop)
        this->stop();
    

    int written = length;
    
    return length != written;
}

/** Write single byte out on the I2C bus
 *
 *  @param data data to write out on bus
 *
 *  @returns
 *    '1' if an ACK was received,
 *    '0' otherwise
 */
int I2C::write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
    return 1;
}

/** Creates a start condition on the I2C bus
 */
void I2C::start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

/** Creates a stop condition on the I2C bus
 */
void I2C::stop(void) {
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}