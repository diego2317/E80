#ifndef __SENSOR_IMU_H__
#define __SENSOR_IMU_H__

#include <Arduino.h>
#include "SensorIMU.h"
#include "OtherIMU.h"
#include "DataSource.h"

#define IMU_COUNT 3

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
// typedef struct COMBINED_IMU{

// } COMBINED_IMU;

class CombinedIMU : public DataSource {
public:
    CombinedIMU(OtherIMU& imu1, OtherIMU& imu2, SensorIMU& imu);

    void read(void);


    
    void getOrientation(const float* ax, const float* ay, const float* az,
        const float* mx, const float* my, const float* mz,
        const float* gx, const float* gy, const float* gz);


    // Weights for the accelerometers [a1, a2, a3]
    void setAccelWeights(double w1, double w2, double w3);

    // Weights for the gyroscopes [g1, g2] (since only 2 have gyros)
    void setGyroWeights(double w1, double w2);

    combined_state state;

    String printRollPitchHeading(void);
    String printAccels(void);

    size_t writeDataBytes(unsigned char * buffer, size_t idx);

private:
    OtherIMU& imu1;
    OtherIMU& imu2;
    SensorIMU& imu;

    void fuseGyro(float * );

}


#endif 