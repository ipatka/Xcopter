//
//  MotionSensors.h
//  Xcopter
//
//  Created by Isaac Patka on 3/26/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#ifndef __Xcopter__MotionSensors__
#define __Xcopter__MotionSensors__

#include <stdio.h>
#include "MPU9150.h"
#include "Quaternion.h"





typedef struct{
    int roll_stab, roll_rate, pitch_stab, pitch_rate, yaw_stab, yaw_rate;
}SensorData;

class MotionSensor {
    
private:
    
    /*Private Instance Variables*/
    MPU9150 imu;
    Quaternion q1;
    char buffer[200];

    /*Private Methods*/
    
public:
    /*Public Instance Variables*/
    
    /*Public Methods*/
    MotionSensor();
    void GetSensorReadings(SensorData *sensor_readings);
    
};

#endif /* defined(__Xcopter__MotionSensors__) */