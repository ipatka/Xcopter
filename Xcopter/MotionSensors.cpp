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

}

void MotionSensor::GetSensorReadings(SensorData *sensor_readings) {
    
    sensor_readings->pitch_stab = 0;
    sensor_readings->pitch_rate = 0;
    sensor_readings->roll_stab = 0;
    sensor_readings->roll_rate = 0;
    sensor_readings->yaw_stab = 0;
    sensor_readings->yaw_rate = 0;
    
    //return sensor_readings;
}