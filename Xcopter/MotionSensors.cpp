//
//  motion_sensors.cpp
//  Xcopter
//
//  Created by Isaac Patka on 3/26/15.
//  Copyright (c) 2015 Isaac Patka. All rights reserved.
//

#include "MotionSensors.h"

MotionSensor::MotionSensor() {
    // Initialize contact with the sensor
    
    new MPU9150();
    imu.initialiseDMP();
    imu.setFifoReset(true);
    imu.setDMPEnabled(true);
    


}

void MotionSensor::GetSensorReadings(SensorData *sensor_readings) {
    
    
    if(imu.getFifoCount() >= 48){
        imu.getFifoBuffer(buffer,  48);
    }
    
    q1.decode(buffer);
    q1.decode_rate(buffer);
    //debug.printf("%f, %f, %f, %f\r\n", q1.w, q1.v.x, q1.v.y, q1.v.z);
    
    sensor_readings->pitch_stab = q1.v.x;
    sensor_readings->pitch_rate = q1.r.x;
    sensor_readings->roll_stab = q1.v.y;
    sensor_readings->roll_rate = q1.r.y;
    sensor_readings->yaw_stab = q1.v.z;
    sensor_readings->yaw_rate = q1.r.z;
    
    //return sensor_readings;
}