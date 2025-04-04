#ifndef __OTHER_IMU_H__
#define __OTHER_IMU_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "Adafruit_I2CDevice.h"
#include "DataSource.h"
#include <sstream>

typedef struct {
    float accelX; // m/s^2
    float accelY; // m/s^2
    float accelZ; // m/s^2
    float gyroX; // rad/s
    float gyroY; // rad/s
    float gyroZ; // rad/s
    float q0;
    float q1;
    float q2;
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float roll;     // [degrees]
    float pitch;    // [degrees]
    float heading;  // [degrees]
} imu_state;

typedef struct LSM6DS3TRC {
    TwoWire *dev_i2c = &Wire1;
    //uint8_t address = 0b01101010;
    //Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(address, dev_i2c);
    Adafruit_LSM6DS3TRC *IMU = new Adafruit_LSM6DS3TRC();
} LSM6DS3TRC;

class OtherIMU : public DataSource {
public:
    OtherIMU();
    OtherIMU(int deviceID);

    // Starts the connection to the sensor
    void init(int deviceID);

    // Reads data from the sensor
    void read(void);

    // Function for Madgwick orientation stuff
    void getOrientation(float& q0, float& q1, float& q2, float& q3,
        float ax, float ay, float az,
        float gx, float gy, float gz,
        float beta);

    // Latest reported orientation data is stored here
    imu_state state; 

    // prints state to serial
    String printRollPitchHeading(void);
    String printAccels(void);

    // from DataSource
    size_t writeDataBytes(unsigned char * buffer, size_t idx);


    int getDeviceID();

    int lastExecutionTime = -1;
    
private:
    LSM6DS3TRC myIMU;
    // Offsets applied to raw x/y/z accel values
    float accel_offsets[3] = { 1.0F, 1.0F, 1.0F };

    // Offsets applied to raw x/y/z gyro values
    float gyro_offsets[3] = { 1.00, 1.00, 1.00 };
};

#endif