#include "body.hpp"


// gets the components of the quaternion as an array
std::array<double, 4> Quaternion::getComponents() const
{
    return {a, b, c, d};
}

// multiplies two quaternions
Quaternion Quaternion::operator*(const Quaternion &q) const
{
    double a1 = a, b1 = b, c1 = c, d1 = d;
    double a2 = q.a, b2 = q.b, c2 = q.c, d2 = q.d;

    double a = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    double b = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    double c = a1*c2 - b1*d2 + c1*a2 + d1*b2;
    double d = a1*d2 + b1*c2 - c1*b2 + d1*a2;

    return Quaternion(a, b, c, d);
}

// get conjugate of quaternion
Quaternion Quaternion::get_Conjugate() const
{
    return Quaternion(a, -b, -c, -d);
}

// get the norm of the quaternion
double Quaternion::get_Norm() const
{
    return sqrt(a*a + b*b + c*c + d*d);
}

// normalise the quaternion while checking for division by zero
void Quaternion::normalise()
{
    double norm = get_Norm();
    if (norm == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    a /= norm;
    b /= norm;
    c /= norm;
    d /= norm;
}

// get the inverse of the quaternion
Quaternion Quaternion::get_Inverse() const 
{
    double norm = get_Norm();
    if (norm == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    double a = this->a / norm;
    double b = -this->b / norm;
    double c = -this->c / norm;
    double d = -this->d / norm;
    return Quaternion(a, b, c, d);
}

// multiplication by a scalar
Quaternion Quaternion::operator*(const double &scalar) const 
{
    return Quaternion(a*scalar, b*scalar, c*scalar, d*scalar);
}

// division by a scalar
Quaternion Quaternion::operator/(const double &scalar) const 
{
    if (scalar == 0)
    {
        throw std::runtime_error("Division by zero");
    }
    return Quaternion(a/scalar, b/scalar, c/scalar, d/scalar);
}

// convert quaternion to axis angle
AxisAngle Quaternion::conv_toAxisAngle() const
{
    double theta = 2 * acos(a);
    Vector3D axis = {b, c, d};
    return AxisAngle(theta, axis);
}


// get the axis of rotation
Vector3D AxisAngle::get_Axis() const 
{
    return axis;
}

// get the angle of rotation
double AxisAngle::get_Theta() const 
{
    return angle;
}

// convert axis angle to quaternion
Quaternion AxisAngle::conv_toQuaternion() const 
{
    double a = cos(angle/2);
    double b = axis.get_x() * sin(angle/2);
    double c = axis.get_y() * sin(angle/2);
    double d = axis.get_z() * sin(angle/2);
    return Quaternion(a, b, c, d);
}

void Body::reset_ForceTorque()
{
    force.set_ToZero();
    torque.set_ToZero();
}

// add force to body
void Body::add_Force(Vector3D force_new)
{
    force = force + force_new;
}