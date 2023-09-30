#pragma once
#include <cmath>  // for std::sqrt
#include <array>  // for std::array
#include <stdexcept>


template<std::size_t N>
std::array<double, N> normalise(const std::array<double, N>& arr) {
    // Calculate the magnitude of the array
    double magnitude = 0.0;
    for (const auto& val : arr) {
        magnitude += val * val;
    }
    magnitude = std::sqrt(magnitude);

    // Handle the zero-vector case to avoid division by zero
    if (magnitude == 0.0) {
        return arr;
    }

    // Normalize the array
    std::array<double, N> normalized;
    for (std::size_t i = 0; i < N; ++i) {
        normalized[i] = arr[i] / magnitude;
    }

    return normalized;
}

class Vector3D 
{
    double x, y, z;

    public:
        Vector3D() : x(0.0), y(0.0), z(0.0) {}
        Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
        Vector3D(const Vector3D &vec) : x(vec.x), y(vec.y), z(vec.z) {}

        double get_x() const { return x; }
        double get_y() const { return y; }
        double get_z() const { return z; }

        // get the magnitude of the vector
        double get_Norm() const;

        // normalise the vector
        void normalise();

        // get the dot product of two vectors
        double dot(const Vector3D &vec) const;

        // get the cross product of two vectors 
        Vector3D cross(const Vector3D &vec) const;

        // addition of two vectors
        Vector3D operator+(const Vector3D &vec) const;

        // subtraction of two vectors
        Vector3D operator-(const Vector3D &vec) const;

        // multiplication of vector by a scalar
        Vector3D operator*(const double &scalar) const;

        // division of vector by a scalar
        Vector3D operator/(const double &scalar) const;

        // negation
        Vector3D operator-() const;

        void set_ToZero() { x = 0.0; y = 0.0; z = 0.0; }
};


Vector3D get_Normalised(const Vector3D& vec);

Vector3D operator*(const double& scalar, const Vector3D& vec);
