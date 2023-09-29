#include <iostream>
#include "body.hpp"

int main(int argc, char *argv[])
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

    AxisAngle axisAngle(1.0, {1.0, 2.0, 3.0});
    std::cout << std::endl;
    std::cout << axisAngle.get_Theta() << std::endl;

    std::array<double, 3> axis = axisAngle.get_Axis();
    for (int i = 0; i < 3; i++)
    {
        std::cout << axis[i] << " ";
    }    

}
