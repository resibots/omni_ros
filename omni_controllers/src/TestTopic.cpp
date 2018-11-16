
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// ROS
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
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
#include <omni_controllers/DoubleVector.h>
#include <omni_controllers/statesPub.h>
#include <omni_controllers/policies/NNpolicy.hpp>
#include <omni_controllers/policies/binary_matrix.hpp>

//http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
//multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]

namespace global {
    bool received_states = false, received_actions = false;
    std::vector<std::vector<double>> states, actions;
} // namespace global
//
// void getStates(const std_msgs::Float64MultiArray::ConstPtr& msgStates)
void getStates (const omni_controllers::statesPub::ConstPtr& msgStates)
{
    ROS_INFO("Subscribing started to record state values from TestTopic node::: Able to access callback!");
    // std::ofstream states;
    // states.open("states_24oct.dat");
    //
    // if (!states) { // file couldn't be opened
    //     std::cerr << "Error: States file could not be opened" << std::endl;
    //     exit(1);
    // }
    //
    // for (unsigned int i = 0; i < msgStates->layout.dim[0].size; i++) {
    //     std::vector<double> temp;
    //     for (unsigned int j = 0; j < msgStates->layout.dim[1].size; j++) {
    //         temp.push_back(msgStates->data[msgStates->layout.dim[0].stride * i + j]);
    //         states << (msgStates->data[msgStates->layout.dim[0].stride * i + j]) << " ";
    //     }
    //
    //     global::states.push_back(temp);
    //     states << std::endl;
    // }
    // states.close();
    // global::received_states = true;
}

// void getActions(const std_msgs::Float64MultiArray::ConstPtr& msgActions)
// {
//     ROS_INFO("Subscribing started to record action values");
//     std::ofstream actions;
//     actions.open("actions_25oct.dat");
//
//     if (!actions) { // file couldn't be opened
//         std::cerr << "Error: States file could not be opened" << std::endl;
//         exit(1);
//     }
//
//     for (unsigned int i = 0; i < msgActions->layout.dim[0].size; i++) {
//         std::vector<double> temp;
//         for (unsigned int j = 0; j < msgActions->layout.dim[1].size; j++){
//           temp.push_back(msgActions->data[msgActions->layout.dim[0].stride * i + j]);
//           actions << (msgActions->data[msgActions->layout.dim[0].stride * i + j]) << " ";
//         }
//
//         global::actions.push_back(temp);
//         actions << std::endl;
//     }
//
//     actions.close();
//     global::received_actions = true;
// }

int main(int argc, char* argv[])
{
    // ros::init(argc, argv, "Publish_params");
    ros::init(argc, argv, "Test_for_getStates");
    ros::NodeHandle nh;

    // tf::TransformListener _listener;
    // tf::StampedTransform _tfWorldToBase; //includes frame-id, child-id etc

    // ros::Publisher my_msg_pub = nh.advertise<omni_controllers::PolicyParams>("/dynamixel_controllers/omni_arm_controller/policyParams", 100, true);
    // ros::Subscriber robot_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_arm_controller/States", 1, getStates);

    ros::Subscriber robot_pos_sub = nh.subscribe<omni_controllers::statesPub>("/dynamixel_controllers/omni_arm_controller/states", 100, getStates);

    // ros::Subscriber robot_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/dynamixel_controllers/omni_arm_controller/Actions", 1, getActions);
    //
    // ros::Publisher COM_val_pub = nh.advertise<omni_controllers::DoubleVector>("/dynamixel_controllers/omni_arm_controller/YouBotBaseCOM", 100, true);

    //Eigen::VectorXd params(175);
    // Eigen::VectorXd params(175); //Change this to have reduced param list as now states include only the joint positions and not velocities anymore
    // Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/Results/results/policy_params_3.bin", params); //Use correct set of params for test, this one was with velocity

    // Eigen::VectorXd params(125); //see calculation at bottom of page
    // Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/dummy_results/policy_params_1.bin", params); //Generated with 5 output states, 6 input states

    // Eigen::VectorXd params(135); //see calculation at bottom of page
    // Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/dummy_results_omni/policy_params_1.bin", params); //Generated with 5 output states, 9 input states
    //
    // //
    // omni_controllers::PolicyParams msg;
    // msg.params.clear();
    // for (unsigned int i = 0; i < params.size(); i++) {
    //     msg.params.push_back(params(i));
    // }
    //
    // msg.t = 2.0;
    // msg.dT = 1; //for testing the size of states topic
    //
    // my_msg_pub.publish(msg);
    // while (!global::received_actions || !global::received_states)
    //      ros::spinOnce();

    // ros::spin();

    // Record the states and actions to verify what the blackdrops algorithm is using

  //   omni_controllers::DoubleVector COM; //if the variable is declared only once then the vector keeps growing. clear it at start of every lookuptransform
  //
    ros::Rate r(100);
    while (ros::ok())
    {
      ros::spin();
       r.sleep();
    }
  //           _listener.waitForTransform("/omnigrasper", "/world", ros::Time(0), ros::Duration(0.5));
  //           _listener.lookupTransform("/omnigrasper", "/world", ros::Time(0), _tfWorldToBase);
  //           COM.val.clear();
  //
  //           //Creating a publisher that sends this information (the policy controller will subscribe to this)
  //           //ROS_INFO("COM (x,y, theta_z) value:%f,%f,%f",_tfWorldToBase.getOrigin().x(),_tfWorldToBase.getOrigin().y(),tf::getYaw(_tfWorldToBase.getRotation()));
  //
  //           // Only base (x, y) value
  //           COM.val.push_back(_tfWorldToBase.getOrigin().x());
  //           COM.val.push_back(_tfWorldToBase.getOrigin().y());
  //           // COM.val.push_back(tf::getYaw(_tfWorldToBase.getRotation()));
  //
  //           COM_val_pub.publish(COM);
  //
  //       ros::spinOnce();
  //       r.sleep();
  // }
}

//No. of params=(input+1).hidden neurons + (hidden+1)*output
//taking hidden as 1 for now, i=2, o=2, so total=7
//for 5 joints, total = (5+1)*10 + (11)*5 = 115
// input states = 11, output states = 5, hidden = 10
// 12*10 + 11*5 = 120+55 = 175

// input states = 6, output states = 5, hidden = 10
// 7*10 + 11*5 = 70+55 = 125

// input states = 8 (5 joints, 3 com base and 1 time), output states = 5 (only 5 joints), hidden = 10
// 10*10 + 11*5 = 80+55 = 135
