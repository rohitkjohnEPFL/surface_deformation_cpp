#pragma once
#include "body.hpp"

class Interaction
{
    // required attributes
    Body body1;
    Body body2;
    double dt;
    double young_mod;
    double poisson;
    double damping_coeff;

    // calculated attributes
    double shear_mod;
    double k_normal;
    double k_shear;
    double k_bending;
    double k_torsion;
    double edge_length;
    Quaternion ori1_init;
    Quaternion ori2_init;

    // calculated states
    Quaternion relative_ori;
    AxisAngle relative_ori_AA;
    Vector3D relative_pos;
    Vector3D orthonormal_axis;
    Vector3D twist_axis;
    Vector3D prev_normal;
    Vector3D curr_normal;
    Vector3D relativeVelocity;
    Vector3D shearInc;
    Vector3D contactPoint;

    // forces and moments
    Vector3D normal_force;
    Vector3D shear_force;
    Vector3D bending_moment;
    Vector3D torsion_moment;
    Vector3D damping_force;

    // deformations
    double normal_defo;
    double torsion_defo;
    Vector3D bending_defo;

    public:
        Interaction(Body b1, Body b2, double dt, double young, double poisson, double damping_coeff);

        // reset interaction
        void reset_ForceTorque();

        void calc_ForcesTorques();
        void calc_initForcesTorques();

        void update_currNormal();
        void update_relativePos();

        void calc_NormalForce();
        void calc_dampingForce();
        void calc_twistBending();
        void calc_torsionMoment();
        void calc_bendingMoment();

        void precompute_ForShear();
        void calc_IncidentVel();
        void rotate_shearForce();
        void calc_ShearForce();

        void apply_ForceTorque();

};