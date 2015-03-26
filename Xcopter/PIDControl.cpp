//
//  PIDControl.cpp
//  Xcopter
//
//  Created by Isaac Patka on 3/21/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "PIDControl.h"


PIDController::PIDController() {
    

    
    this->QuadPidInit(quad_pid_coefficients, pid_array);
    
    

}

void PIDController::PidResetIntegrator(PID *pid) {
    pid->sumError = 0;
}

void PIDController::PidInit(int16_t p_factor, int16_t i_factor, int16_t d_factor, PID *pid) {
    // Start values for PID controller
    pid->sumError = 0;
    pid->lastProcessValue = 0;
    // Tuning constants for PID loop
    pid->P_Factor = p_factor;
    pid->I_Factor = i_factor;
    pid->D_Factor = d_factor;
    // Limits to avoid overflow
    pid->maxError = MAX_INT / (pid->P_Factor + 1);
    pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

void PIDController::QuadPidInit(double (&quad_pid_coefficients)[6][3], PID (&pid_array)[6]) {
    
    for (int i = 0; i < 6; i++) {
        this->PidInit(quad_pid_coefficients[i][0], quad_pid_coefficients[i][1], quad_pid_coefficients[i][2], &pid_array[i]);
    }
    
    
}

int16_t PIDController::PidController(int16_t setPoint, int16_t processValue, PID *pid_st)
{
    int16_t error, p_term, d_term;
    int32_t i_term, ret, temp;
    
    error = setPoint - processValue;
    
    // Calculate Pterm and limit error overflow
    if (error > pid_st->maxError){
        p_term = MAX_INT;
    }
    else if (error < -pid_st->maxError){
        p_term = -MAX_INT;
    }
    else{
        p_term = pid_st->P_Factor * error;
    }
    
    // Calculate Iterm and limit integral runaway
    temp = pid_st->sumError + error;
    if(temp > pid_st->maxSumError){
        i_term = MAX_I_TERM;
        pid_st->sumError = pid_st->maxSumError;
    }
    else if(temp < -pid_st->maxSumError){
        i_term = -MAX_I_TERM;
        pid_st->sumError = -pid_st->maxSumError;
    }
    else{
        pid_st->sumError = temp;
        i_term = pid_st->I_Factor * pid_st->sumError;
    }
    
    // Calculate Dterm
    d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);
    
    pid_st->lastProcessValue = processValue;
    
    ret = (p_term + i_term + d_term) / SCALING_FACTOR;
    if(ret > MAX_INT){
        ret = MAX_INT;
    }
    else if(ret < -MAX_INT){
        ret = -MAX_INT;
    }
    
    return((int16_t)ret);
}

PID_OUTPUT PIDController::QuadPidController(PID (&pid_array)[6]) {
    
}