//
// Created by PJLAB\lijialun on 11/15/24.
//

#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include <array>

using namespace std;


struct QuaternionPos {
    double w, x, y, z;

    QuaternionPos(const double _w, const double _x, const double _y, const double _z) : w(_w), x(_x), y(_y), z(_z) {
    }

    [[nodiscard]] std::array<double, 3> xyz() const {
        return {x, y, z};
    }

    [[nodiscard]] std::array<double, 3> apply(const std::array<double, 3> &vec) const {
        const std::array<double, 3> q_xyz = xyz();

        const std::array<double, 3> t = {
            2 * (q_xyz[1] * vec[2] - q_xyz[2] * vec[1]),
            2 * (q_xyz[2] * vec[0] - q_xyz[0] * vec[2]),
            2 * (q_xyz[0] * vec[1] - q_xyz[1] * vec[0])
        };

        const std::array<double, 3> q_cross_t = {
            q_xyz[1] * t[2] - q_xyz[2] * t[1],
            q_xyz[2] * t[0] - q_xyz[0] * t[2],
            q_xyz[0] * t[1] - q_xyz[1] * t[0]
        };

        const std::array<double, 3> result = {
            vec[0] + w * t[0] + q_cross_t[0],
            vec[1] + w * t[1] + q_cross_t[1],
            vec[2] + w * t[2] + q_cross_t[2]
        };

        return result;
    }
};


struct Vector3D {
    double x, y, z;

    Vector3D() : x(0), y(0), z(0) {
    }

    Vector3D(const double x, const double y, const double z) : x(x), y(y), z(z) {
    }

    Vector3D operator+(const Vector3D &other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3D operator-(const Vector3D &other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector3D operator*(const double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3D operator/(const double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
};

class PositionEstimator {
private:
    Vector3D position;
    Vector3D velocity;
    Vector3D previousAcceleration;
    float dt;

public:
    PositionEstimator() : position(0, 0, 0), velocity(0, 0, 0), dt(0.02) {
    }

    [[nodiscard]] static Vector3D getWorldAcceleration(const float ax, const float ay, const float az, const float q0,
                                                       const float q1, const float q2, const float q3) {
        const QuaternionPos quaternion(q0, q1, q2, q3);
        const array<double, 3> vec{ax, ay, az};
        const auto out = quaternion.apply(vec);
        return Vector3D{out[0], out[1], out[2]};
    }

    void update(const float ax, const float ay, const float az, const float q0, const float q1, const float q2,
                const float q3) {
        if (dt <= 0) dt = 0.02;
        const auto cali_a = getWorldAcceleration(ax, ay, az, q0, q1, q2, q3);
        velocity = velocity + (cali_a + previousAcceleration) * 0.5 * dt;
        position = position + velocity * dt;
        previousAcceleration = cali_a;
    }

    [[nodiscard]] Vector3D getPosition() const {
        return position;
    }
};


#endif //POSITION_ESTIMATOR_H
