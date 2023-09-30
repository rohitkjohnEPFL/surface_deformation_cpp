#pragma once
#include "vectorTools.hpp"

// forward declaration of AxisAngle
class AxisAngle;

class Quaternion
{
    private:
        double a, b, c, d;

    public:
        // constructor
        Quaternion() : a(0), b(0), c(0), d(0) {}
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

    public:
        double angle;
        Vector3D axis;

        // constructor
        AxisAngle() : angle(0), axis(1, 0, 0) {}
        AxisAngle(double theta, Vector3D ax) : angle(theta) , axis(get_Normalised(ax)) {}

        // get the axis of rotation
        Vector3D get_Axis() const;

        // get the angle of rotation
        double get_Theta() const;

        // convert axis angle to quaternion
        Quaternion conv_toQuaternion() const;
};


class Body{

    private:
        int id;
        double density, radius;

    public:
        bool DynamicQ;
        double mass, inertia;
        Vector3D pos, vel, angVel;
        Quaternion ori;
        std::string BlockedDOFs;
        Vector3D force, torque;


        // Initialise with force and torque set to zero
        Body(
                int id, 
                double density, 
                double mass, 
                double radius, 
                double inertia, 
                bool DynamicQ, 
                Vector3D pos, 
                Vector3D vel, 
                Vector3D angVel, 
                Quaternion ori, 
                std::string BlockedDOFs
            ) : id(id), density(density), mass(mass), radius(radius), inertia(inertia), DynamicQ(DynamicQ), pos(pos), vel(vel), angVel(angVel), ori(ori), BlockedDOFs(BlockedDOFs), force({0, 0, 0}), torque({0, 0, 0}) {}  


        // Reset force and torque to zero
        void reset_ForceTorque();


        // Add force to the body
        void add_Force(Vector3D force);

        // Add torque to the body
        void add_Torque(Vector3D torque); 

        double get_Radius() const { return radius; }
        double get_Density() const { return density; }
        int get_ID() const { return id; }

};

