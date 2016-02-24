#include <omni_driver/omni.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_driver");
    ros::NodeHandle n;

    auto omni = std::make_shared<omni_ros::Omni>(n);
    ros::Duration(0.5).sleep();
    std::vector<double> joints(4);
    joints[0] = M_PI / 2;
    joints[1] = M_PI / 2;
    joints[2] = 3 * M_PI / 2;
    joints[3] = M_PI;
    omni->set_joint_positions(joints);

    ros::Duration(0.5).sleep();

    // omni->relax();
}