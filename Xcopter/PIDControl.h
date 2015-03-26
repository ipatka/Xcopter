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
#include "MotionSensors.h"



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

typedef struct{
    int pitch_output, roll_output, yaw_output;
}PID_OUTPUT;

typedef struct{
    int roll, pitch, yaw;
}PID_REFERENCES;

/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         32767
#define MAX_LONG        2147483647
#define MAX_I_TERM      (MAX_LONG / 2)

#define SCALING_FACTOR  128


//PIDs
#define PID_PITCH_RATE 0
#define PID_PITCH_STAB 1
#define PID_ROLL_RATE 3
#define PID_ROLL_STAB 4
#define PID_YAW_RATE 5
#define PID_YAW_STAB 6

// Coefficientes
#define K_P_PITCH_RATE     1.00
#define K_I_PITCH_RATE     0.00
#define K_D_PITCH_RATE     0.00

#define K_P_PITCH_STAB     1.00
#define K_I_PITCH_STAB     0.00
#define K_D_PITCH_STAB     0.00

#define K_P_ROLL_RATE     1.00
#define K_I_ROLL_RATE     0.00
#define K_D_ROLL_RATE     0.00

#define K_P_ROLL_STAB     1.00
#define K_I_ROLL_STAB     0.00
#define K_D_ROLL_STAB     0.00

#define K_P_YAW_RATE     1.00
#define K_I_YAW_RATE     0.00
#define K_D_YAW_RATE     0.00

#define K_P_YAW_STAB     1.00
#define K_I_YAW_STAB     0.00
#define K_D_YAW_STAB     0.00


class PIDController {
    
private:
    
    /*Private Instance Variables*/
    
    //PID controllers
    PID pid_pitch_rate;
    PID pid_pitch_stab;
    PID pid_roll_rate;
    PID pid_roll_stab;
    PID pid_yaw_rate;
    PID pid_yaw_stab;
    
    //Coefficients
    double quad_pid_coefficients[6][3] = {
        {   K_P_PITCH_RATE * SCALING_FACTOR,
            K_I_PITCH_RATE * SCALING_FACTOR,
            K_D_PITCH_RATE * SCALING_FACTOR
        },
        {   K_P_PITCH_STAB * SCALING_FACTOR,
            K_I_PITCH_STAB * SCALING_FACTOR,
            K_D_PITCH_STAB * SCALING_FACTOR
        },
        {   K_P_ROLL_RATE * SCALING_FACTOR,
            K_I_ROLL_RATE * SCALING_FACTOR,
            K_D_ROLL_RATE * SCALING_FACTOR
        },
        {   K_P_ROLL_STAB * SCALING_FACTOR,
            K_I_ROLL_STAB * SCALING_FACTOR,
            K_D_ROLL_STAB * SCALING_FACTOR
        },
        {   K_P_YAW_RATE * SCALING_FACTOR,
            K_I_YAW_RATE * SCALING_FACTOR,
            K_D_YAW_RATE * SCALING_FACTOR
        },
        {   K_P_YAW_STAB * SCALING_FACTOR,
            K_I_YAW_STAB * SCALING_FACTOR,
            K_D_YAW_STAB * SCALING_FACTOR
        }
    };
    
    PID pid_array[6] = {
        pid_pitch_rate,
        pid_pitch_stab,
        pid_roll_rate,
        pid_roll_stab,
        pid_yaw_rate,
        pid_yaw_stab
    };
    
    
    /*Private Methods*/
    
    void PidResetIntegrator(PID *pid);
    void QuadPidInit(double (&quad_pid_coefficients)[6][3], PID (&pid_array)[6]);
    void PidInit(int16_t p_factor, int16_t i_factor, int16_t d_factor, PID *pid);
    int16_t PidController(int16_t setPoint, int16_t processValue, PID *pid);
    
    
public:
    
    PIDController();
    void QuadPidController(PID_REFERENCES *reference_values, SensorData *measurement_value, PID_OUTPUT *pid_output);
    

    
    
};

#endif /* defined(__Xcopter__PIDControl__) */
