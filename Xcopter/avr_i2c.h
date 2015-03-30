//
//  avr_i2c.h
//  Xcopter
//
//  Created by Isaac Patka on 3/29/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__avr_i2c__
#define __Xcopter__avr_i2c__

#include <stdio.h>
#include <avr/io.h>

class I2C {
    
public:
    
    enum RxStatus {
        NoData
        , MasterGeneralCall
        , MasterWrite
        , MasterRead
    };
    
    enum Acknowledge {
        NoACK = 0
        , ACK   = 1
    };
    
    
    
    
    /** Create an I2C Master interface, connected to the specified pins
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     */
    I2C();
    
    /** Set the frequency of the I2C interface
     *
     *  @param hz The bus frequency in hertz
     */
//    void frequency(int hz);
    
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
    int read(int address, char *data, int length, bool repeated);
    
    /** Read a single byte from the I2C bus
     *
     *  @param ack indicates if the byte is to be acknowledged (1 = acknowledge)
     *
     *  @returns
     *    the byte read
     */
    int read(int ack);
    
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
    int write(int address, const char *data, int length, bool repeated);
    
    /** Write single byte out on the I2C bus
     *
     *  @param data data to write out on bus
     *
     *  @returns
     *    '1' if an ACK was received,
     *    '0' otherwise
     */
    int write(uint8_t data);
    
    /** Creates a start condition on the I2C bus
     */
    void start(void);
    
    /** Creates a stop condition on the I2C bus
     */
    void stop(void);
    
protected:
    
    void aquire();
    
    //I2CName     _i2c;
    static I2C  *_owner;
    int         _hz;
    
};

#endif /* defined(__Xcopter__avr_i2c__) */
