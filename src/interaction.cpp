#include "interaction.hpp"

#define PI 3.14159265358979323846

// intections constructor
//     Interaction(Body b1, Body b2, double dt, double young, double poisson, double damping_coeff); 
Interaction::Interaction(Body b1, Body b2, double dt, double young, double poisson, double damping_coeff) : 
body1(b1), 
body2(b2), 
dt(dt), 
young_mod(young), 
poisson(poisson), 
damping_coeff(damping_coeff), 
ori1_init(body1.ori), 
ori2_init(body2.ori)
{
    shear_mod = young_mod / (2 * (1 + poisson));

    // assign mass of the grid edge to the nodes
    edge_length    = (body1.pos - body2.pos).get_Norm();
    double rad     = body1.get_Radius();
    double halfVol = 0.5 * PI * rad * rad * edge_length;
    double density = body1.get_Density();
    double mass    = density * halfVol;
    double geomInert = 2.0 / 5.0 * mass * rad * rad;

    // assign edge length
    relative_pos = body2.pos - body1.pos;
    
    // assigning mass and moment of inertia to each node
    body1.mass    = body1.mass + mass;
    body2.mass    = body2.mass + mass;
    body1.inertia = body1.inertia + geomInert;
    body2.inertia = body2.inertia + geomInert;

    // calculating the normal vector 2 wrt 1
    curr_normal = relative_pos / edge_length;
    prev_normal = curr_normal;

    // calculating the contact point
    contactPoint = (body1.pos + body2.pos) / 2.0;


    // calculating the stiffnesses
    double area = PI * rad * rad;
    double torsionalAreaMoment = PI * rad * rad * rad * rad / 2.0;
    double bendingAreaMoment   = PI * rad * rad * rad * rad / 4.0;

    k_normal  = young_mod * area / edge_length;
    k_torsion = shear_mod * torsionalAreaMoment / edge_length;
    k_shear   = 12.0 * young_mod * bendingAreaMoment / (edge_length * edge_length * edge_length);
    k_bending = young_mod * bendingAreaMoment / edge_length;
}


    // def reset_ForceTorque(self) -> None:
    //     self.normal_force = np.array([0, 0, 0], dtype=F64)
    //     self.shear_force  = np.array([0, 0, 0], dtype=F64)
    //     self.bending_moment = np.array([0, 0, 0], dtype=F64)
    //     self.torsion_moment = np.array([0, 0, 0], dtype=F64)

void Interaction::reset_ForceTorque()
{
    normal_force.set_ToZero();
    shear_force.set_ToZero();
    bending_moment.set_ToZero();
    torsion_moment.set_ToZero();
    damping_force.set_ToZero();
}

    // def calc_ForcesTorques(self) -> None:
    //     # The force is calculated with respect to body 1.
    //     # The force on body 2 is the negative of this force
    //     self.update_currNormal()
    //     self.update_relativePos()
    //     self.calc_NormalForce()
    //     self.calc_dampingForce()
    //     self.calc_twistBending()
    //     self.calc_torsionMoment()
    //     self.calc_bendingMoment()
    //     self.calc_ShearForce()
    //     self.apply_ForceTorque()

void Interaction::calc_ForcesTorques()
{
    update_currNormal();
    update_relativePos();
    calc_NormalForce();
    calc_dampingForce();
    calc_twistBending();
    calc_torsionMoment();
    calc_bendingMoment();
    calc_ShearForce();
    apply_ForceTorque();
}

    // def calc_initForcesTorques(self) -> None:
    //     # This is force and torque in the 0th interation
    //     self.update_currNormal()
    //     self.update_relativePos()
    //     self.calc_NormalForce()
    //     self.calc_dampingForce()
    //     self.calc_twistBending()
    //     self.calc_torsionMoment()
    //     self.calc_bendingMoment()
    //     self.calc_ShearForce()
    //     self.apply_ForceTorque()

void Interaction::calc_initForcesTorques()
{
    update_currNormal();
    update_relativePos();
    calc_NormalForce();
    calc_dampingForce();
    calc_twistBending();
    calc_torsionMoment();
    calc_bendingMoment();
    calc_ShearForce();
    apply_ForceTorque();
}

void Interaction::update_currNormal()
{
    curr_normal = get_Normalised(body2.pos - body1.pos);
}


void Interaction::update_relativePos()
{
    relative_pos = body2.pos - body1.pos;
    Quaternion ori1 = body1.ori;
    Quaternion ori2 = body2.ori;
    Quaternion ori2_inv = ori2.get_Inverse();
    Quaternion ori1_init_inv = ori1_init.get_Inverse();

    relative_ori    = (ori1 * ori1_init_inv) * (ori2_init * ori2_inv);
    relative_ori_AA = relative_ori.conv_toAxisAngle();
    contactPoint    = (body1.pos + body2.pos) / 2.0;
}


void Interaction::calc_NormalForce()
{
    //  The force is calculated with respect to body 1.
    //  The force on body 2 is the negative of this force
    double defo = (body2.pos - body1.pos).get_Norm() - edge_length;
    normal_defo = defo;
    normal_force = k_normal * defo * curr_normal;
}


void Interaction::calc_dampingForce()
{
    damping_force = damping_coeff * (body2.vel - body1.vel);
}


void Interaction::calc_twistBending()
{
    AxisAngle axisAngle = relative_ori_AA;
    torsion_defo = axisAngle.angle * axisAngle.axis.dot(curr_normal);
    bending_defo = axisAngle.angle * axisAngle.axis - torsion_defo * curr_normal;
}


void Interaction::calc_torsionMoment()
{
    torsion_moment = k_torsion * torsion_defo * curr_normal;
}



void Interaction::calc_bendingMoment()
{
    bending_moment = k_bending * bending_defo;
}


void Interaction::precompute_ForShear()
{
    // '''To compute the shear increment, shear force is calculated using an incremental formulation
    // see: https://www.sciencedirect.com/science/article/pii/S0925857413001936?via%3Dihub

    // for the implementation
    // check the bool Law2_ScGeom6D_CohFrictPhys_CohesionMoment::go() function in
    // https://gitlab.com/yade-dev/trunk/-/blob/master/pkg/dem/CohesiveFrictionalContactLaw.cpp?ref_type=heads'''

    orthonormal_axis = prev_normal.cross(curr_normal);
    double angle     = dt * 0.5 * (body1.angVel + body2.angVel).dot(curr_normal);
    twist_axis       = angle * curr_normal;
    prev_normal      = curr_normal;
    relativeVelocity = relativeVelocity - relativeVelocity.dot(curr_normal) * curr_normal;
    shearInc = relativeVelocity * dt;
}


void Interaction::calc_IncidentVel()
{
    double rad = body1.get_Radius();
    double center2center_dist = (body2.pos - body1.pos).get_Norm();
    double penetrationDepth = 2.0 * rad - center2center_dist;

    // This alpha value is used to avoid granular ratcheting.
    // See the Vector3r ScGeom::getIncidentVel() function in
    // https://gitlab.com/yade-dev/trunk/-/blob/master/pkg/dem/ScGeom.cpp?ref_type=heads
    // around line 66
    double alpha = (rad + rad) / (rad + rad - penetrationDepth);

    Vector3D tangentialVel2 = body2.angVel.cross(- rad * curr_normal);
    Vector3D tangentialVel1 = body1.angVel.cross(  rad * curr_normal);
    relativeVelocity = (body2.vel - body1.vel) * alpha + tangentialVel2 - tangentialVel1;
}

void Interaction::rotate_shearForce()
{
    shear_force = shear_force - shear_force.cross(orthonormal_axis);
    shear_force = shear_force - shear_force.cross(twist_axis);
}


void Interaction::calc_ShearForce()
{
    precompute_ForShear();
    rotate_shearForce();
    shear_force = shear_force + k_shear * shearInc;
}


void Interaction::apply_ForceTorque()
{
    Vector3D totalForce1 =  normal_force + shear_force;
    Vector3D totalForce2 = -normal_force - shear_force;

    body1.force = body1.force + totalForce1 + damping_force;
    body2.force = body2.force + totalForce2 - damping_force;

    Vector3D contact_wrt_pos1 = contactPoint - body1.pos;
    Vector3D contact_wrt_pos2 = contactPoint - body2.pos;

    Vector3D body1_torqueDueToForce = contact_wrt_pos1.cross(totalForce1);
    Vector3D body2_torqueDueToForce = contact_wrt_pos2.cross(totalForce2);

    body1.torque = body1.torque + body1_torqueDueToForce - bending_moment - torsion_moment;
    body2.torque = body2.torque + body2_torqueDueToForce + bending_moment + torsion_moment;
}
