
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// ROS
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

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
    std::ofstream states;
    states.open("states.dat");

    if (!states) { // file couldn't be opened
        std::cerr << "Error: States file could not be opened" << std::endl;
        exit(1);
    }

    for (unsigned int i = 0; i < msgStates->layout.dim[0].size; i++) {
        std::vector<double> temp;
        for (unsigned int j = 0; j < msgStates->layout.dim[1].size; j++) {
            temp.push_back(msgStates->data[msgStates->layout.dim[0].stride * i + j]);
            states << (msgStates->data[msgStates->layout.dim[0].stride * i + j]) << " ";
        }

        global::states.push_back(temp);
        states << std::endl;
    }
    states.close();
    global::received_states = true;
}

void getActions(const std_msgs::Float64MultiArray::ConstPtr& msgActions)
{
    ROS_INFO("Subscribing started to record action values");
    std::ofstream actions;
    actions.open("actions.dat");

    if (!actions) { // file couldn't be opened
        std::cerr << "Error: States file could not be opened" << std::endl;
        exit(1);
    }

    for (unsigned int i = 0; i < msgActions->layout.dim[0].size; i++) {
        std::vector<double> temp;
        for (unsigned int j = 0; j < msgActions->layout.dim[1].size; j++){
          temp.push_back(msgActions->data[msgActions->layout.dim[0].stride * i + j]);
          actions << (msgActions->data[msgActions->layout.dim[0].stride * i + j]) << " ";
        }

        global::actions.push_back(temp);
        actions << std::endl;
    }

    actions.close();
    global::received_actions = true;
}

int main(int argc, char* argv[])
{
    // ros::init(argc, argv, "Publish_params");
    ros::init(argc, argv, "Publish_params_and_test_TF_listening");
    ros::NodeHandle nh;

    tf::TransformListener _listener;
    tf::StampedTransform _tfWorldToBase; //includes frame-id, child-id etc
    // tf::Transform _tfWorldToBase;

    // ros::Publisher my_msg_pub = nh.advertise<omni_controllers::PolicyParams>("/dynamixel_controllers/omni_policy_controller/policyParams", 100, true);
    // ros::Subscriber robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_policy_controller/States", 1, getStates);
    ros::Publisher my_msg_pub = nh.advertise<omni_controllers::PolicyParams>("/dynamixel_controllers/omni_arm_controller/policyParams", 100, true);
    ros::Subscriber robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_arm_controller/States", 1, getStates);
    ros::Subscriber robot_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_arm_controller/Actions", 1, getActions);

    Eigen::VectorXd params(175);
    Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/Results/results/policy_params_3.bin", params);

    // _listener.lookupTransform("/omnigrasper", "/world", ros::Time(0), _tfWorldToBase);
    // ROS_INFO("test x values:%f",_tfWorldToBase.getOrigin().x());

    omni_controllers::PolicyParams msg;
    msg.params.clear();
    for (unsigned int i = 0; i < params.size(); i++) {
        msg.params.push_back(params(i));
    }

    msg.t = 4.0;
    msg.dT = 0.1;

    my_msg_pub.publish(msg);
    while (!global::received_actions || !global::received_states)
        ros::spinOnce();

    //std_msgs::Float64 COM_val;

    try{
          // _listener.waitForTransform("/world", "/omnigrasper", ros::Time(0), ros::Duration(10.0));
          _listener.lookupTransform("/world", "/omnigrasper", ros::Time(0), _tfWorldToBase);
                //ROS_INFO("test (x,y,z) values:%f,%f,%f",_tfWorldToBase.getOrigin().x(),_tfWorldToBase.getOrigin().y(),_tfWorldToBase.getOrigin().z());

                //Creating a publisher that sends this information (the policy controller will subscribe to this)
                ROS_INFO("COM (x,y, theta_z) value:%f,%f,%f",_tfWorldToBase.getOrigin().x(),_tfWorldToBase.getOrigin().y(),tf::getYaw(_tfWorldToBase.getRotation()));

          }
           catch (tf::TransformException &ex) {
             ROS_ERROR("%s",ex.what());
             // ros::Duration(1.0).sleep();
             // continue;
    }
}

//No. of params=(input+1).hidden neurons + (hidden+1)*output
//taking hidden as 1 for now, i=2, o=2, so total=7
//for 5 joints, total = (5+1)*10 + (11)*5 = 115
// input states = 11, output states = 5, hidden = 10
// 12*10 + 11*5 = 120+55 = 175
