#ifndef __COMBINED_IMU_2_H__
#define __COMBINED_IMU_2_H__

#include <Arduino.h>
#include <Wire.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include "DataSource.h"
#include <Adafruit_LSM6DS3TRC.h>
#include "Adafruit_I2CDevice.h"
#include <array>
#include <cmath>
// TODO: we need a state struct. Do we want floats or to use Eigen?
// Do we need quaternion stuff in the state?
typedef struct {
    float accelX;   // m/s^2
    float accelY;   // m/s^2
    float accelZ;   // m/s^2
    float gyroX;    // rad/s
    float gyroY;    // rad/s
    float gyroZ;    // rad/s
    float magX;     // [Gauss]
    float magY;     // [Gauss]
    float magZ;     // [Gauss]
    float roll;     // [degrees]
    float pitch;    // [degrees]
    float heading;  // [degrees]
} combined_imu_state;

// Struct for Non-Board IMUs
// Copied from OtherIMU.h
typedef struct LSM6DS3TRC {
    TwoWire *dev_i2c = &Wire1;
    //uint8_t address = 0b01101010;
    //Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(address, dev_i2c);
    Adafruit_LSM6DS3TRC *IMU = new Adafruit_LSM6DS3TRC();
} LSM6DS3TRC;

// Struct for Board IMU
// Copied from SensorIMU.h
typedef struct LSM303AGR{
    TwoWire *dev_i2c = &Wire;
    LSM303AGR_ACC_Sensor *Acc = new LSM303AGR_ACC_Sensor(dev_i2c);
    LSM303AGR_MAG_Sensor *Mag = new LSM303AGR_MAG_Sensor(dev_i2c);
} LSM303AGR;
  
class CombinedIMU2 : public DataSource {
public:
    CombinedIMU2(void);

    // Start connection to all three sensors?
    void init(int deviceID);

    // Reads data from sensor
    void read(void);

    // Computes the weighted mean acceleration for a sample
    float weightedMean(const std::array<float, 3>& raw);

    // Determines the orientation of the "combined" IMU
    // Uses a Madgwick orientation algorithm
    // TODO: Determine parameters
    void getOrientation();

    // Stores "current" IMU state
    combined_imu_state state;

    // prints state to serial
    String printRollPitchHeading(void);
    String printAccels(void);

    // from DataSource
    size_t writeDataBytes(unsigned char * buffer, size_t idx);


    int getDeviceID();

    int lastExecutionTime = -1;

private:

    // Create sensor instances for all three IMUs
    LSM303AGR boardIMU;
    LSM6DS3TRC IMU_1;
    LSM6DS3TRC IMU_2;

    // Offsets applied to raw x/y/z accel values
    std::array<float, 3> accel_offsets = {1.0F, 1.0F, 1.0F};
    std::array<float, 3> mag_offsets   = {1.0F, 1.0F, 1.0F};
    std::array<std::array<float, 3>, 3> mag_ironcomp = {{
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F}
    }};

    // Order is board, other, other
    // Units are mg
    std::array<float, 3> accRMS = {3.0F * 0.00980665, 1.7F * 0.00980665, 1.7F * 0.00980665};
    std::array<float, 3> accWeight = {1.0f/(accRMS[0]*accRMS[0]), 1.0f/(accRMS[1]*accRMS[1]), 1.0f/(accRMS[2]*accRMS[2])};

    const float dt = 0.099f; // 99 ms sample period
    const float tau_a = 0.3f; // tau for accel + mag
    const float tau_g = 0.05f; // tau for gyro
    const float alpha_a = dt (tau_a + dt); 
    const float alpha_g = dt (tau_g + dt);
    bool initialized = false;


}

#endif