#pragma once
#include <array> 
#include <stdexcept>
#include <cmath>

#include "vectorFunctions.hpp"

class Quaternion
{
    private:
        double a, b, c, d;

    public:
        Quaternion(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}

        // gets the components of the quaternion as an array
        std::array<double, 4> getComponents() const;

        // multiplies two quaternions
        Quaternion operator*(const Quaternion &q) const;

        // get conjugate of quaternion
        Quaternion get_Conjugate() const;

        // get the norm of the quaternion
        double get_Norm() const;

        // normalise the quaternion
        void normalise();

        // get the inverse of the quaternion
        Quaternion get_Inverse() const;

        // multiplication by a scalar
        Quaternion operator*(const double &scalar) const;

        // division by a scalar
        Quaternion operator/(const double &scalar) const;

        // convert quaternion to axis angle
        AxisAngle conv_toAxisAngle() const;

};


class AxisAngle
{
    private:
        double theta;
        std::array<double, 3> axis;

    public:
        AxisAngle(double theta, std::array<double, 3> axis);

        // get the axis of rotation
        std::array<double, 3> get_Axis() const;

        // get the angle of rotation
        double get_Theta() const;

        // convert axis angle to quaternion
        Quaternion conv_toQuaternion() const;
};