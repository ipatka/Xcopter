//
//  PIDControl.h
//  Xcopter
//
//  Created by Isaac Patka on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__PIDControl__
#define __Xcopter__PIDControl__


#include "stdint.h"



typedef struct{
    //! Last process value, used to find derivative of process value.
    int16_t lastProcessValue;
    //! Summation of errors, used for integrate calculations
    int32_t sumError;
    //! The Proportional tuning constant, multiplied with SCALING_FACTOR
    int16_t P_Factor;
    //! The Integral tuning constant, multiplied with SCALING_FACTOR
    int16_t I_Factor;
    //! The Derivative tuning constant, multiplied with SCALING_FACTOR
    int16_t D_Factor;
    //! Maximum allowed error, avoid overflow
    int16_t maxError;
    //! Maximum allowed sumerror, avoid overflow
    int32_t maxSumError;
}PID;

/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         32767
#define MAX_LONG        2147483647
#define MAX_I_TERM      (MAX_LONG / 2)

#define SCALING_FACTOR  128



// Boolean values
#define FALSE           0
#define TRUE            1

// Coefficientes
#define K_P     1.00
#define K_I     0.00
#define K_D     0.00

class PIDController {
    
private:
    
    /*Private Instance Variables*/
    PID pid_pitch_rate;
    PID pid_pitch_stab;
    PID pid_roll_rate;
    PID pid_roll_stab;
    PID pid_yaw_rate;
    PID pid_yaw_stab;
    
    
    
    
    /*Private Methods*/
    
    void PidResetIntegrator(PID *pid);

    
    
public:
    
    PIDController();
    
    void PidInit(int16_t p_factor, int16_t i_factor, int16_t d_factor, PID *pid);
    
    int16_t PidController(int16_t setPoint, int16_t processValue, PID *pid);

    
    
};

#endif /* defined(__Xcopter__PIDControl__) */
