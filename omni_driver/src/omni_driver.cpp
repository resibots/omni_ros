#include <omni_driver/omni.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_driver");
    ros::NodeHandle n;

    auto omni = std::make_shared<omni_ros::Omni>(n);

    std::cin.get();

    tf::Transform transform = omni->get_arm_frame();
    std::cout << "Current arm position (w.r.t. the initial base frame): " << transform.getOrigin().x() << "  " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl;

    omni->base_rotate(1);
    omni->base_displace(0.2, 0.3);
    std::vector<double> joints(4);
    joints[0] = 1.57;
    joints[1] = 1.57;
    joints[2] = 4.71;
    joints[3] = 3.14;

    omni->set_joint_positions(joints);
    std::cin.get();
    transform = omni->get_arm_frame();
    std::cout << "Current arm position (w.r.t. the initial base frame): " << transform.getOrigin().x() << "  " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl;
    std::cin.get();
    omni->base_return();
    std::cin.get();
    transform = omni->get_arm_frame();
    std::cout << "Current arm position (w.r.t. the initial base frame): " << transform.getOrigin().x() << "  " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl;
}
