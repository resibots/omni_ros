
#include <vector>
#include <string>
#include <iostream>

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

//http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
//multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]

namespace global {
    bool received_states = false, received_actions = false;
    std::vector<std::vector<double>> states, actions;
} // namespace global

void getStates(const std_msgs::Float64MultiArray::ConstPtr& msgStates)
{
    ROS_INFO("Subscribing started to record state values");

    for (unsigned int i = 0; i < msgStates->layout.dim[0].size; i++) {
        std::vector<double> temp;
        for (unsigned int j = 0; j < msgStates->layout.dim[1].size; j++)
            temp.push_back(msgStates->data[msgStates->layout.dim[0].stride * i + j]);

        global::states.push_back(temp);

    }

    global::received_states = true;
}

void getActions(const std_msgs::Float64MultiArray::ConstPtr& msgActions)
{
    ROS_INFO("Subscribing started to record action values");

    for (unsigned int i = 0; i < msgActions->layout.dim[0].size; i++) {
        std::vector<double> temp;
        for (unsigned int j = 0; j < msgActions->layout.dim[1].size; j++)
            temp.push_back(msgActions->data[msgActions->layout.dim[0].stride * i + j]);

        global::actions.push_back(temp);

    }

    // std::cout<<"Print to check"<<std::endl;
    // for (auto row: global::actions) {
    //   for (auto element:row)
    //     std::cout<<element<<" ";
    //   std::cout<<std::endl;
    // }
    global::received_actions = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Publish_params");
    ros::NodeHandle nh;

    ros::Publisher my_msg_pub = nh.advertise<omni_controllers::PolicyParams>("/dynamixel_controllers/omni_policy_controller/policyParams", 100, true);
    ros::Subscriber robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/States", 1, getStates);
    ros::Subscriber robot_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/Actions", 1, getActions);

    Eigen::VectorXd params(115);
    Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/results/policy_params_1.bin", params);

    omni_controllers::PolicyParams msg;
    msg.params.clear();
    for (unsigned int i = 0; i < params.size(); i++) {
        msg.params.push_back(params(i));
    }

    msg.t = 4.0;
    msg.dT = 0.5;

    my_msg_pub.publish(msg);
    while(!global::received_actions || !global::received_states)
      ros::spinOnce();

}

//No. of params=(input+1).hidden neurons + (hidden+1)*output
//taking hidden as 1 for now, i=2, o=2, so total=7
//for 5 joints, total = (5+1)*10 + (11)*5 = 115
