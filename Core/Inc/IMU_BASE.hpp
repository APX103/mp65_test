//
// Created by PJLAB\lijialun on 10/18/24.
//

#ifndef IMU_BASE_H
#define IMU_BASE_H


#include "i2c.h"
#include <string>

struct AccelData {
    float accelX;
    float accelY;
    float accelZ;
};

struct GyroData {
    float gyroX;
    float gyroY;
    float gyroZ;
};

struct MagData {
    float magX;
    float magY;
    float magZ;
};

struct Quaternion {
    float qW;
    float qX;
    float qY;
    float qZ;
};

struct calData {
    bool valid;
    float accelBias[3];
    float gyroBias[3];
    float magBias[3];
    float magScale[3];
};

class IMUBase {
public:
    virtual ~IMUBase() = default;

    virtual int init(calData cal, uint8_t address) = 0;
    virtual void update() = 0;

    virtual void getAccel(AccelData* out) = 0;
    virtual void getGyro(GyroData* out) = 0;
    virtual void getMag(MagData* out) = 0;
    virtual void getQuat(Quaternion* out) = 0;
    virtual float getTemp() = 0;

    virtual int setGyroRange(int range) = 0;
    virtual int setAccelRange(int range) = 0;
    virtual int setIMUGeometry(int index) = 0;

    virtual void calibrateAccelGyro(calData* cal) = 0;
    virtual void calibrateMag(calData* cal) = 0;

    virtual bool hasMagnetometer() {
        return false;
    }
    virtual bool hasTemperature() {
        return false;
    }
    virtual bool hasQuatOutput() {
        return false;
    }

    virtual std::string IMUName(){
        return "Unknown";
    }
    virtual std::string IMUType(){
        return "Unknown";
    }
    virtual std::string IMUManufacturer(){
        return "Unknown";
    }
};

#endif //IMU_BASE_H
