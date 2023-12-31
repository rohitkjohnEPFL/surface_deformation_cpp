#include <iostream>
#include "body.hpp"

int main(int argc, char* argv[])
{
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(2.0, 3.0, 4.0, 5.0);
    Quaternion q3 = q1 * q2;
    std::array<double, 4> components = q3.getComponents();
    for (int i = 0; i < 4; i++)
    {
        std::cout << components[i] << " ";
    }

    std::cout << std::endl;

    Quaternion q4 = q3.get_Inverse();

    for (int i = 0; i < 4; i++)
    {
        std::cout << q4.getComponents()[i] << " ";
    }

    Vector3D axis(1, 1, 0);

    AxisAngle axisAngle(1.0, axis);
    std::cout << std::endl;
    std::cout << axisAngle.get_Theta() << std::endl;

    Vector3D axis2 = axisAngle.get_Axis();
    std::cout << axis2.get_x() << " " << axis2.get_y() << " " << axis2.get_z();
}
