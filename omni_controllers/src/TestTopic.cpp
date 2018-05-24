
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
#include <omni_controllers/PubMsg.h>
#include <omni_controllers/SubParamMsg.h>
#include <omni_controllers/SubPolMsg.h>
#include <omni_controllers/policies/NNpolicy.hpp>
#include <omni_controllers/policies/binary_matrix.hpp>

// void setParams(const omni_controllers::SubParamMsg::ConstPtr& msg)
//  {
//    ROS_INFO("Subscribing started, time t=: [%f]", msg->t);
//    ros::spin();
//  }

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Publish_params");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");
  //ros::Rate loop_rate(10);

  ros::Publisher my_msg_pub = nh.advertise<omni_controllers::SubParamMsg>("/dynamixel_controllers/omni_policy_controller/policyParams", 1);
  // ros::Subscriber sub_params= nh.subscribe<omni_controllers::SubParamMsg>("policyParams",1,setParams);

  omni_controllers::SubParamMsg msg;

  ros::Rate rate(1);

  Eigen::VectorXd params(115);

  Eigen::read_binary<Eigen::VectorXd>("/home/deba/Code/limbo/results/policy_params_1.bin", params);

  //std::cout<<"params"<<params.transpose()<<std::endl;

  std::cout<<"starting to publish"<<std::endl;
  while(ros::ok())
  {
    //No. of params=(input+1).hidden neurons + (hidden+1)*output
    //taking hidden as 1 for now, i=2, o=2, so total=7
    //for 5 joints, total = (5+1)*10 + (11)*5 = 115

    msg.params.clear();
    for(unsigned int i=0; i< params.size(); i++){
      msg.params.push_back(params(i));
    }

    //msg.params={0.5,0.25,0.5,-0.5,0.5,0.25,0.5};
    msg.t=4.0;
    msg.dT=1.0;
    my_msg_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }



return 0;
}
