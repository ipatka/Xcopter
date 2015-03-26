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

SensorData MotionSensor::GetSensorReadings(SensorData sensor_readings) {
    
    sensor_readings.pitch = 0;
    sensor_readings.roll = 0;
    sensor_readings.yaw = 0;
    sensor_readings.rotational_rate = 0;
    
    return sensor_readings;
}