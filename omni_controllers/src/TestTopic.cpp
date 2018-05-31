
#include <vector>
#include <string>

// ROS
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

//Local
#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/PolicyParams.h>
#include <omni_controllers/policies/NNpolicy.hpp>
#include <omni_controllers/policies/binary_matrix.hpp>

void getStates(const std_msgs::Float64MultiArray::ConstPtr& msgStates)
{
    ROS_INFO("Subscribing started to state values");
    //_flagStates = true;
    ros::spinOnce();
}

void getActions(const std_msgs::Float64MultiArray::ConstPtr& msgActions)
{
    ROS_INFO("Subscribing started to action values");
    //_flagActions = true;
    ros::spinOnce();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Publish_params");
    ros::NodeHandle nh;

    ros::Publisher my_msg_pub = nh.advertise<omni_controllers::PolicyParams>("/dynamixel_controllers/omni_policy_controller/policyParams", 1);

    Eigen::VectorXd params(115);
    Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/results/policy_params_1.bin", params);

    omni_controllers::PolicyParams msg;
    msg.params.clear();
    for (unsigned int i = 0; i < params.size(); i++) {
        msg.params.push_back(params(i));
    }

    msg.t = 4.0;
    msg.dT = 0.5;
    std::cout << "starting to publish" << std::endl;
    while (ros::ok()) {
        my_msg_pub.publish(msg);
        ros::spinOnce();
    }

    //Publish only once

    // _flagActions = false;
    // _flagStates = false;

    ros::Subscriber robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/States", 1, getStates);
    ros::Subscriber robot_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/Actions", 1, getActions);

    // double count = 0;
    // double limit = msg.t/msg.dT;
    //
    // while (ros::ok() && count<limit) {
    //     robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/States", 1, getStates);
    //     robot_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/Actions", 1, getActions);
    //     count++;
    //     ros::spinOnce();
    // }
}

//No. of params=(input+1).hidden neurons + (hidden+1)*output
//taking hidden as 1 for now, i=2, o=2, so total=7
//for 5 joints, total = (5+1)*10 + (11)*5 = 115
