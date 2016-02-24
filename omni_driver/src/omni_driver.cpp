#include <omni_driver/omni.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_driver");
    ros::NodeHandle n;

    auto omni = std::make_shared<omni_ros::Omni>(n);

    omni->base_displace(0.1, 0);
    ros::Duration(2).sleep();
    omni->base_rotate(M_PI / 4);

    tf::Transform transform = omni->get_arm_frame();

    std::cout << "Current arm position (w.r.t. the base frame): " << transform.getOrigin().x() << "  " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl;

    ros::Duration(0.5).sleep();
    std::vector<double> joints(4);
    joints[0] = M_PI / 2;
    joints[1] = M_PI / 2;
    joints[2] = 3 * M_PI / 2;
    joints[3] = M_PI;
    omni->set_joint_positions(joints);

    transform = omni->get_arm_frame();
    std::cout << "Current arm position (w.r.t. the base frame): " << transform.getOrigin().x() << "  " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl;

    ros::Duration(0.5).sleep();

    // omni->relax();
}