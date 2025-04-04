#include "OtherIMU.h"
#include "Printer.h"
#include <math.h>
#include <sstream>
extern Printer printer;

// Helper function to fill in the label string
static const char* formatLabels(char* dest, size_t size, int deviceID) {
    snprintf(dest, size,
             "rollIMU_%d,pitchIMU_%d,headingIMU_%d,"
             "accelX_%d,accelY_%d,accelZ_%d,"
             "gyroX_%d,gyroY_%d,gyroZ_%d",
             deviceID, deviceID, deviceID,
             deviceID, deviceID, deviceID,
             deviceID, deviceID, deviceID);
    return dest;
}

OtherIMU::OtherIMU(int deviceID)
    : DataSource(formatLabels(varNames_, sizeof(varNames_), deviceID),
                 "float,float,float,float,float,float,float,float,float"),
                 deviceID_(deviceID),
                 varNames_{} {
    // Constructor body if needed
}



void OtherIMU::init(void) {
    Serial.print("Initializing other IMU... ");

    // create i2c interface
    myIMU.dev_i2c->begin();

    if (!myIMU.IMU->begin_I2C()) {
        Serial.println("failed to initialize I2C");
    }
}

void OtherIMU::read(void) {
    // Get new raw data
    // TODO: Determine if IMU uses RHR
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp; // unused
    myIMU.IMU->getEvent(&accel,&gyro,&temp);
    //float raw_acc_data[3];
    //myIMU.IMU->readAcceleration(raw_acc_data[0], raw_acc_data[1], raw_acc_data[2]);

    //float raw_gyro_data[3];
    //myIMU.IMU->readGyroscope(raw_gyro_data[0], raw_gyro_data[1], raw_gyro_data[2]);

    // Remove offsets from acceleration measurements?
    state.accelX = accel.acceleration.x;
    state.accelY = accel.acceleration.y;
    state.accelZ = accel.acceleration.z;

    // remove offsets from gyroscope measurements
    state.gyroX = gyro.gyro.x;
    state.gyroY = gyro.gyro.y;
    state.gyroZ = gyro.gyro.z;

    getOrientation(state.q0, state.q1, state.q2, state.q3, 
        state.accelX, state.accelY, state.accelZ, 
        state.gyroX, state.gyroY, state.gyroZ, 0.1f);
}

String OtherIMU::printRollPitchHeading(void) {
    String printString = "IMU:"; 
    printString += " roll: ";
    printString += String(state.roll);
    printString += "[deg],";
    printString += " pitch: "; 
    printString += String(state.pitch);
    printString += "[deg],";
    printString += " heading: ";
    printString += String(state.heading);
    printString += "[deg]";
    printString += " gyroX: ";
    printString += String(state.gyroX);
    printString += "[rad/s], ";
    return printString; 
}

String OtherIMU::printAccels(void) {
    String printString = "IMU:";
  
    printString += " accelX: ";
    printString += String(state.accelX);
    printString += "[m/s^2], ";
    printString += " accelY: ";
    printString += String(state.accelY);
    printString += "[m/s^2], ";
    printString += " accelZ: ";
    printString += String(state.accelZ);
    printString += "[m/s^2]";
  
      // Used to Debug
    
    //printString += " magX: ";
    //printString += String(state.magX);
    //printString += "[uT], ";
    //printString += " magY: ";
    //printString += String(state.magY);
    //printString += "[uT], ";
    //printString += " magZ: ";
    //printString += String(state.magZ);
    //printString += "[uT]";
  
    return printString;
}



size_t OtherIMU::writeDataBytes(unsigned char * buffer, size_t idx) {
    float * data_slot = (float *) &buffer[idx];
    data_slot[0] = state.roll;
    data_slot[1] = state.pitch;
    data_slot[2] = state.heading;
    data_slot[3] = state.accelX;
    data_slot[4] = state.accelY;
    data_slot[5] = state.accelZ;
    data_slot[6] = state.gyroX;
    data_slot[7] = state.gyroY;
    data_slot[8] = state.gyroZ;
    return idx + 9*sizeof(float);
}


void OtherIMU::getOrientation(float& q0, float& q1, float& q2, float& q3,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float beta) {
    float const PI_F = 3.14159265F;
    float const samplefreq = 10.1F; // sample frequency is 10.1 Hz
    float const dt = 1.0f / samplefreq;
    // Copied from Adafruit_AHRS_Madgwick
    // Convert gyro from deg/s to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    float recipNorm, s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    // Normalize accelerometer
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Gradient descent step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Rate of change of quaternion
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate to yield new quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    state.q0 = q0;
    state.q1 = q1;
    state.q2 = q2;
    state.q3 = q3;


    float roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    float pitch = asinf(-2.0f * (q1*q3 - q0*q2));
    float yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    state.roll = roll * 180.0/PI_F;
    state.pitch = pitch * 180.0/PI_F;
    state.heading = yaw * 180.0/PI_F; // heading
}