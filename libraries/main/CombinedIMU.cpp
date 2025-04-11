#include "CombinedIMU.h"
#include "Printer.h"
#include <cmath>
#include <algorithm>
extern Printer printer;

CombinedIMU::CombinedIMU(OtherIMU& imu1, OtherIMU& imu2, SensorIMU& imu)
    : imu1{imu1}, imu2{imu2}, imu{imu} {}

void CombinedIMU::read(void) {
    float raw_x_accel[3];
    float raw_y_accel[3];
    float raw_z_accel[3];
    raw_x_accel[0] = imu.state.accelX;
    raw_x_accel[1] = imu1.state.accelX;
    raw_x_accel[2] = imu2.state.accelX;
    raw_y_accel[0] = imu.state.accelY;
    raw_y_accel[1] = imu1.state.accelY;
    raw_y_accel[2] = imu2.state.accelY;
    raw_z_accel[0] = imu.state.accelZ;
    raw_z_accel[1] = imu1.state.accelZ;
    raw_z_accel[2] = imu2.state.accelZ;


    float raw_gyro_x[2];
    float raw_gyro_y[2];
    float raw_gyro_z[2];
    raw_gyro_x[0] = imu1.state.gyroX;
    raw_gyro_x[1] = imu2.state.gyroX;
    raw_gyro_y[0] = imu1.state.gyroY;
    raw_gyro_y[1] = imu2.state.gyroY;
    raw_gyro_z[0] = imu1.state.gyroZ;
    raw_gyro_z[1] = imu2.state.gyroZ;

    // Now, transform the acceleration and gyro data using the Board
    // IMU as the reference frame



    float outlier_threshould = 2.0f;
    float default_accel_weights[3] = {0.34,0.33,0.33};
    // only do outlier rejection for the accelerometers
    
    // Now, fuse the data using weighted average
    float averaged_accel_x = 0.34 * raw_x_accel[0] + 0.33 * raw_x_accel[1] + 0.33 * raw_x_accel[2];
    
    float averaged_accel_y = 0.34 * raw_y_accel[0] + 0.33 * raw_y_accel[1] + 0.33 * raw_y_accel[2];

    float averaged_accel_z = 0.34 * raw_z_accel[0] + 0.33 * raw_z_accel[1] + 0.33 * raw_z_accel[2];

    float averaged_gyro_x = 0.5 * raw_gyro_x[0] + 0.5 * raw_gyro_x[1];
    float averaged_gyro_y = 0.5 * raw_gyro_y[0] + 0.5 * raw_gyro_y[1];
    float averaged_gyro_z = 0.5 * raw_gyro_z[0] + 0.5 * raw_gyro_z[1];
    

    // Now, do filtering stuff





    // 
}