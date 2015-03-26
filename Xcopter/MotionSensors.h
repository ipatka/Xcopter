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


#define MPU9150 0


typedef struct{
    int roll, pitch, yaw, rotational_rate;
}SensorData;

class MotionSensor {
    
private:
    
    /*Private Instance Variables*/

    /*Private Methods*/
    
public:
    /*Public Instance Variables*/
    
    /*Public Methods*/
    MotionSensor();
    SensorData GetSensorReadings(SensorData sensor_readings);
    
};

#endif /* defined(__Xcopter__MotionSensors__) */