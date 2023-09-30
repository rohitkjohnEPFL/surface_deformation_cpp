#include "vectorTools.hpp"

// get the magnitude of the vector
double Vector3D::get_Norm() const
{
    return sqrt(x*x + y*y + z*z);
}

// normalise the vector
void Vector3D::normalise()
{
    double magnitude = get_Norm();
    if (magnitude == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
}

// get the dot product of two vectors
double Vector3D::dot(const Vector3D &vec) const
{
    return x*vec.x + y*vec.y + z*vec.z;
}

// get the cross product of two vectors
Vector3D Vector3D::cross(const Vector3D &vec) const
{
    return Vector3D(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
}

// addition of two vectors
Vector3D Vector3D::operator+(const Vector3D &vec) const
{
    return Vector3D(x + vec.x, y + vec.y, z + vec.z);
}

// subtraction of two vectors
Vector3D Vector3D::operator-(const Vector3D &vec) const
{
    return Vector3D(x - vec.x, y - vec.y, z - vec.z);
}

// multiplication of vector by a scalar
Vector3D Vector3D::operator*(const double &scalar) const
{
    return Vector3D(x*scalar, y*scalar, z*scalar);
}

// division of vector by a scalar
Vector3D Vector3D::operator/(const double &scalar) const
{
    if (scalar == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    return Vector3D(x/scalar, y/scalar, z/scalar);
}

Vector3D get_Normalised(const Vector3D& vec)
{
    double magnitude = vec.get_Norm();
    if (magnitude == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    return Vector3D(vec.get_x()/magnitude, vec.get_y()/magnitude, vec.get_z()/magnitude);
}

Vector3D operator*(const double& scalar, const Vector3D& vec)
{
    return Vector3D(scalar*vec.get_x(), scalar*vec.get_y(), scalar*vec.get_z());
}

Vector3D Vector3D::operator-() const {
    return Vector3D(-x, -y, -z);
}