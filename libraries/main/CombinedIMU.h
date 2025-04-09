#ifndef __SENSOR_IMU_H__
#define __SENSOR_IMU_H__

#include <Arduino.h>
#include "SensorIMU.h"
#include "OtherIMU.h"
#include "DataSource.h"

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
    float roll;
    float pitch;
    float heading;
} combined_state;

// Define a sensor that holds the data from all 3 IMUs
typedef struct COMBINED_IMU{

} COMBINED_IMU;

class CombinedIMU : public DataSource {
public:
    CombinedIMU()
}


#endif 