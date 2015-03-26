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

void PIDController::QuadPidController(PID_REFERENCES *reference_values, SensorData *measurement_value, PID_OUTPUT *pid_output) {
    
    int16_t pitch_stab_output = this->PidController(reference_values->pitch, measurement_value->pitch_stab, &pid_array[PID_PITCH_STAB]);
    int16_t roll_stab_output = this->PidController(reference_values->roll, measurement_value->roll_stab, &pid_array[PID_ROLL_STAB]);
    int16_t yaw_stab_output = this->PidController(reference_values->yaw, measurement_value->yaw_stab, &pid_array[PID_YAW_STAB]);
    
    
    pid_output->pitch_output = this->PidController(pitch_stab_output, measurement_value->pitch_rate, &pid_array[PID_PITCH_RATE]);
    pid_output->roll_output = this->PidController(roll_stab_output, measurement_value->roll_rate, &pid_array[PID_ROLL_RATE]);
    pid_output->yaw_output = this->PidController(yaw_stab_output, measurement_value->yaw_rate, &pid_array[PID_YAW_RATE]);
    
}