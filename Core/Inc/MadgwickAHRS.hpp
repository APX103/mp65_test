//
// Created by PJLAB\lijialun on 10/11/24.
//
#ifndef MADGWICK_H
#define MADGWICK_H

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll{};
    float pitch{};
    float yaw{};
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick();
    void begin(const float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    float getQ0() {
        if (!anglesComputed) computeAngles();
        return q0;
    }
    float getQ1() {
        if (!anglesComputed) computeAngles();
        return q1;
    }
    float getQ2() {
        if (!anglesComputed) computeAngles();
        return q2;
    }
    float getQ3() {
        if (!anglesComputed) computeAngles();
        return q3;
    }
};

#endif //MADGWICK_H
