#include "CombinedIMU2.h"
#include "Printer.h"

extern Printer printer;

CombinedIMU2::CombinedIMU2(void)
  : DataSource("rollCombined,pitchCombined,yawCombined,accelXCombined,accelYCombined,accelZCombined",
               "float,float,float,float,float,float") {
}

void CombinedIMU2::init(void) {
    Serial.print("Initializing Combined IMU... ");

    // Start by initializing Board IMU
    boardIMU.dev_i2c->begin();
  
    // Starting and Enabling Accelerometer and Magnetometer
    if ( boardIMU.Acc->begin()                    == LSM303AGR_ACC_STATUS_OK
        && boardIMU.Acc->Enable()                   == LSM303AGR_ACC_STATUS_OK 
        && boardIMU.Acc->EnableTemperatureSensor()  == LSM303AGR_ACC_STATUS_OK 
        && boardIMU.Mag->begin()                    == LSM303AGR_MAG_STATUS_OK
        && boardIMU.Mag->Enable()                   == LSM303AGR_MAG_STATUS_OK ){
    
        //Serial.println("done");
    }
    else {
        //Serial.println("failed!");
    }
    uint8_t address = 0b01101010;
    IMU_1.dev_i2c->begin();
    if (!IMU_1.IMU->begin_I2C(address, IMU_1.dev_i2c, 1)) {
        Serial.println("failed to initialize I2C on IMU 1");
    }
    uint8_t address = 0b01101011;
    IMU_2.dev_i2c->begin();
    if (!IMU_2.IMU->begin_I2C(address, IMU_2.dev_i2c, 2)) {
        Serial.println("failed to initialize I2C on IMU 2");
    }
}

// Return the median of three numbers (branch‑free)
static inline float median3(float a, float b, float c)
{
    return std::fmax(std::fmin(a, b), std::fmin(std::fmax(a, b), c));
}

inline float lowPassStep(float curr, float old, float alpha) {
    return alpha *curr + (1-alpha)*(old);
}

float CombinedIMU2::weightedMean(const std::array<float, 3>& raw, k = 3.0f) {

    // Get the median
    const float med = median3(x[0], x[1], x[2]);

    // Scaled Mean Absolute Deviation
    std::array<float, 3> d {
        std::fabs(x[0] - med),
        std::fabs(x[1] - med),
        std::fabs(x[2] - med)
    };

    const float mad = 1.4826f * median3(d[0], d[1], d[2]);   // → σ for Gaussian noise
    const float thresh = k * mad;

    // ----- weighted mean with outlier rejection -----
    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < 3; ++i) {
        if (mad == 0.0f || d[i] <= thresh) {
            num += accWeight[i] * x[i];
            den += accWeight[i];
        }
    }
    return den ? num / den : med;

}

void CombinedIMU2::read(void) {
    // Get our acceleration data with the weighted mean
    int32_t board_raw_acc[3]
    boardIMU.Acc->GetAxes(board_raw_acc);

    sensors_event_t accel1;
    sensors_event_t accel2;
    sensors_event_t gyro1;
    sensors_event_t gyro2;
    sensors_event_t temp; // unused
    IMU_1.IMU->getEvent(&accel1, &gyro1, &temp);
    IMU_2.IMU->getEvent(&accel2, &gyro2, &temp);
    std::array<float,3> ax = {((float)board_raw_acc[0] - accel_offsets[0])* 0.00980665, accel1.acceleration.x, accel2.acceleration.x};
    std::array<float,3> ay = {(-(float)board_raw_acc[1]- accel_offsets[1])* 0.00980665, accel1.acceleration.y, accel2.acceleration.y};
    std::array<float,3> az = {((float)board_raw_acc[2] - accel_offsets[2])* 0.00980665, accel1.acceleration.z, accel2.acceleration.z};

    if (!initialized) {
        state.accelX = weightedMean(ax);
        state.accelY = weightedMean(ay);
        state.accelZ = weightedMean(az);
    } else {
        state.accelX = lowPassStep(weightedMean(ax), state.accelX, alpha_a);
        state.accelY = lowPassStep(weightedMean(az), state.accelY, alpha_a);
        state.accelZ = lowPassStep(weightedMean(ay), state.accelZ, alpha_a);
    }
    

    // Get magnetometer data
    float mx = (float)raw_mag_data[0]; 
    float my = -(float)raw_mag_data[1]; // The IMU STILL doesn't use the RHR
    float mz = (float)raw_mag_data[2]; 

    // mGauss to uTesla
    mx = mx * .1;
    my = my * .1;
    mz = mz * .1;

    // Remove offsets from magnertometer measurements (base values in uTesla)
    mx = mx - mag_offsets[0];
    my = my - mag_offsets[1];
    mz = mz - mag_offsets[2];

    // Apply magnetometer soft iron error compensation
    mx = mx * mag_ironcomp[0][0] + my * mag_ironcomp[0][1] + mz * mag_ironcomp[0][2];
    my = mx * mag_ironcomp[1][0] + my * mag_ironcomp[1][1] + mz * mag_ironcomp[1][2];
    mz = mx * mag_ironcomp[2][0] + my * mag_ironcomp[2][1] + mz * mag_ironcomp[2][2];

    if (!initialized) {
        state.magX = mx;
        state.magY = my;
        state.magZ = mz;
    } else {
        state.magX = lowPassStep(mx, state.magX, alpha_a);
        state.magY = lowPassStep(my, state.magY, alpha_a);
        state.magZ = lowPassStep(mz, state.magZ, alpha_a);
    }

    // Get gyroscope data with simple mean
    float gx = (gyro1.gyro.x + gyro2.gyro.x) / 2;
    float gy = (gyro1.gyro.y + gyro2.gyro.y) / 2;
    float gz = (gyro1.gyro.z + gyro2.gyro.z) / 2;

    if (!initialized) {
        state.gyroX = gx;
        state.gyroY = gy;
        state.gyroZ = gz;
    } else {
        state.gyroX = lowPassStep(gx, state.gyroX, alpha_g);
        state.gyroY = lowPassStep(gy, state.gyroY, alpha_g);
        state.gyroZ = lowPassStep(gz, state.gyroZ, alpha_g);
    }
    initialized = true;
    // Populate the roll, pitch, yaw with Madgwick orientation
    getOrientation();
}

void CombinedIMU2::getOrientation() {
    /* -------- persistent filter state ------------------------------------- */
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion
    constexpr float beta = 0.1f;                               // 2·ζ·ω (tune if needed)

    /* -------- shorthand --------------------------------------------------- */
    const float gx = state.gyroX;               // rad s⁻¹
    const float gy = state.gyroY;
    const float gz = state.gyroZ;

    float ax = state.accelX, ay = state.accelY, az = state.accelZ;
    float mx = state.magX,   my = state.magY,   mz = state.magZ;

    /* -------- normalise accelerometer ------------------------------------- */
    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;                  // avoid NaN on bad data
    ax /= norm;  ay /= norm;  az /= norm;

    /* -------- normalise magnetometer -------------------------------------- */
    norm = std::sqrt(mx*mx + my*my + mz*mz);
    if (norm < 1e-6f) return;
    mx /= norm;  my /= norm;  mz /= norm;

    /* ======== Madgwick algorithm ======== */
    // Auxiliary variables to avoid repeated arithmetic
    const float _2q0mx = 2.0f*q0*mx, _2q0my = 2.0f*q0*my, _2q0mz = 2.0f*q0*mz;
    const float _2q1mx = 2.0f*q1*mx;
    const float _2q0   = 2.0f*q0,  _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    const float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

    // Reference direction of Earth’s magnetic field
    float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
    float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
    const float _2bx = std::sqrt(hx*hx + hy*hy);
    const float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;

    /* -------- gradient descent step (s) ----------------------------------- */
    float s0 = -_2q2*(2.0f*(q1q3 - q0q2) - ax)   + _2q1*(2.0f*(q0q1 + q2q3) - ay)
               - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
               + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
               + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    float s1 =  _2q3*(2.0f*(q1q3 - q0q2) - ax)   + _2q0*(2.0f*(q0q1 + q2q3) - ay)
               - 4.0f*q1*(1.0f - 2.0f*(q1q1 + q2q2) - az)
               + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
               + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
               + (_2bx*q3 - 4.0f*q1*_2bz)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    float s2 = -_2q0*(2.0f*(q1q3 - q0q2) - ax)   + _2q3*(2.0f*(q0q1 + q2q3) - ay)
               - 4.0f*q2*(1.0f - 2.0f*(q1q1 + q2q2) - az)
               + (-4.0f*q2*_2bx - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
               + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
               + (_2bx*q0 - 4.0f*q2*_2bz)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    float s3 =  _2q1*(2.0f*(q1q3 - q0q2) - ax)   + _2q2*(2.0f*(q0q1 + q2q3) - ay)
               + (-4.0f*q3*_2bx + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
               + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
               + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    // normalise step magnitude
    norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm != 0.0f) {
        s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;
    }

    /* -------- quaternion rate & integrate -------------------------------- */
    const float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
    const float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
    const float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
    const float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* -------- renormalise quaternion ------------------------------------- */
    norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    /* -------- convert to Euler degrees ----------------------------------- */
    state.roll  = std::atan2(2.0f*(q0*q1 + q2*q3),
                             1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.29578f;

    state.pitch = std::asin( std::clamp(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f) ) * 57.29578f;

    state.heading = std::atan2(2.0f*(q0*q3 + q1*q2),
                               1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.29578f;
}

String CombinedIMU2::printRollPitchHeading(void) {
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
    return printString; 
  }

String CombinedIMU2::printAccels(void) {
    String printString = "IMU:";
  
    printString += " accelX: ";
    printString += String(state.accelX);
    printString += "[mg], ";
    printString += " accelY: ";
    printString += String(state.accelY);
    printString += "[mg], ";
    printString += " accelZ: ";
    printString += String(state.accelZ);
    printString += "[mg]";
  
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

size_t CombinedIMU2::writeDataBytes(unsigned char * buffer, size_t idx) {
    float * data_slot = (float *) &buffer[idx];
    data_slot[0] = state.roll;
    data_slot[1] = state.pitch;
    data_slot[2] = state.heading;
    data_slot[3] = state.accelX;
    data_slot[4] = state.accelY;
    data_slot[5] = state.accelZ;
    return idx + 6*sizeof(float);
}
