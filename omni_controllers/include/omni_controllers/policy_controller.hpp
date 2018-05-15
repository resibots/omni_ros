/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  Copyright (c) 2018, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef POLICY_CONTROLLER_H
#define POLICY_CONTROLLER_H

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

namespace arm_speed_safe_controller {

    /**
     * FIXME: \brief Speed command controller for a robotic arm, with safety constraints.
     *
     * This class forwards the command signal down to a set of joints, if they
     * do not infringe some user-defined safety constraints.
     *
     * \tparam T class implementing the safety constraints
     *
     * \section ROS interface
     *
     * \param type hardware interface type.
     * \param joints Names of the joints to control.
     *
     * Subscribes to:
     * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
     */
    template <class SafetyConstraint = NoSafetyConstraints>
    class PolicyController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        PolicyController() {}
        ~PolicyController() { _sub_command.shutdown(); }

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
        {
            // List of controlled joints
            std::string param_name = "joints";
            if (!nh.getParam(param_name, joint_names)) {
                ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
                return false;
            }
            n_joints = joint_names.size();

            if (n_joints == 0) {
                ROS_ERROR_STREAM("List of joint names is empty.");
                return false;
            }
            for (unsigned int i = 0; i < n_joints; i++) {
                try {
                    joints.push_back(
                        std::make_shared<hardware_interface::JointHandle>(
                            hw->getHandle(joint_names[i])));
                    ROS_DEBUG_STREAM("Joint number " << i << ": " << joint_names[i]);
                }
                catch (const hardware_interface::HardwareInterfaceException& e) {
                    ROS_ERROR_STREAM("Exception thrown: " << e.what());
                    return false;
                }
            }

            // Safety Constraint
            if (!_constraint.init(joints, nh)) {
                ROS_ERROR_STREAM("Initialisation of the safety contraint failed");
                return false;
            }

            commands_buffer.writeFromNonRT(std::vector<double>(n_joints, 0.0));

            flag = false;
            publish_flag = false;
            count = 0;
            max_iterations = (int)T/dT;
            commands << 0.0, 0.0, 0.0, 0.0, 0.0;

            //double M_PI = 3.14;
            double boundary = 1;
            int state_dim = 5;
            int hidden_neurons = 10;
            int action_dim = 5;
            Eigen::VectorXd limits;
            //limits << M_PI, M_PI/4., M_PI/2., M_PI, M_PI/2.;
            limits << 3.14, 0.78, 1.57, 3.14, 1.57;
            Eigen::VectorXd max_u;
            max_u << 1.0, 1.0, 1.0, 1.0, 1.0;

            pol = make_shared<blackdrops::policy::NNPolicy<Params>>(boundary, state_dim, hidden_neurons, action_dim, limits, max_u);

            _sub_command = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &PolicyController::commandCB, this);
            _sub_params = nh.subscribe<omni_controllers::SubParamMsg>("policy_params",1,&PolicyController::setParams, this);
            realtime_pub.reset(new realtime_tools::RealtimePublisher<omni_controllers::PubMsg>(nh, "joint_states", 1));
            // realtime_pub.reset(new realtime_tools::RealtimePublisher<omni_controllers::PubMsg>(nh, "joint_states", 1));

            ros::spin();
            return true;
        }

        void starting(const ros::Time& time)
        {
            // Start controller with 0.0 velocities
            commands_buffer.readFromRT()->assign(n_joints, 0.0);
        }

        std::vector<std::string> joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
        unsigned int n_joints;


        //Note that these are not vector of vectors, but vector of doubles, so that finally a long vector of all commands is send and extracted by knowing the number of joints
        std::vector<double> jointList;
        std::vector<double> commandList;
        Eigen::VectorXd commands;

        double columns,rows,T,dT;
        int max_iterations,count;
        bool publish_flag,flag;

    private:
        SafetyConstraint _constraint;
        ros::Subscriber _sub_command;
        ros::Subscriber _sub_params;

        //realtime_tools::RealtimePublisher<omni_controllers::PubMsg> realtime_pub_;

        std::shared_ptr<blackdrops::policy::NNPolicy<Params>> pol;
        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::PubMsg>> realtime_pub;

        //pol = make_shared<blackdrops::policy::NNPolicy<Params>>(values, vlaues, alvuse);
        void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
        {
            if (msg->data.size() != n_joints) {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints << ")! Not executing!");
                return;
            }
            commands_buffer.writeFromNonRT(msg->data);
        }

       void setParams(const omni_controllers::SubParamMsg::ConstPtr& msg)
        {
          Eigen::VectorXd params; //copy the parameters in a local public array, save time information

          //std::vector<double>::iterator index;

          // for (index = msg->params.begin(); index!=msg->params.end(); index++)
          // params.push_back(*index);
          //
          // pol.set_params(params);   //set the policy parameters
          // flag = true;

          for (int i=0; i<msg->params.size(); i++)
          params=msg->params(i);

          pol.set_params(params);   //set the policy parameters
          flag = true;

          //save the time and Duration
          T = msg->t;
          dT = msg->dT;

          //Hence rows can be set now (correspond to number of runs in an episode)
          max_iterations = (int)T/dT;
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {
           if (flag) //blackdrops parameters to be implemented
           {
             if (count<max_iterations) //during the episode
             {
               commands = pol.next(joints);

               for(int i=0;i<commands.size();i++)
                commandList.push_back(commands(i)); 
               //commandList(count)=commands;

               //_constraint.enforce(commands, period);
               for (unsigned int j = 0; j < n_joints; j++) {
                  jointList.push_back(joints[j]->getPosition());
                  joints[j]->setCommand(commands[j]);
                 }

               count++;
             }
             else //episode is over
              {
                //record the last set of joint states
                for (unsigned int j = 0; j < n_joints; j++)
                   jointList.push_back(joints[j]->getPosition());

                //send zero velocities
                commands.setZero(commands.size());
                //_constraint.enforce(commands, period);
                for (unsigned int j = 0; j < n_joints; j++)
                   joints[j]->setCommand(commands[j]);

                //reset/set flags and counter
                flag = false;
                publish_flag = true;
                count = 0;
              }

            if(publish_flag)
            {
                //publishing
                if (realtime_pub->trylock()){
                for (unsigned i=0; i<jointList.size(); i++)
                {
                  realtime_pub->msg_.jointList.push_back(jointList[i]);
                  realtime_pub->msg_.commandList.push_back(commandList[i]);
                }
                realtime_pub->msg_.rows = max_iterations;
                realtime_pub->msg_.columns = n_joints;
                realtime_pub->unlockAndPublish();
                }

                //clear storage vectors after publishing is over for the last episode
                std::fill(jointList.begin(), jointList.end(), 0);
                std::fill(commandList.begin(), commandList.end(), 0);

                publish_flag = false;
             }
           }
             else
             {
               //still send zero velocities
               commands.setZero(n_joints);

               //_constraint.enforce(commands, period);
               for (unsigned int j = 0; j < n_joints; j++)
                joints[j]->setCommand(commands[j]);
             }

        } //end of update method
  };

  /** \cond HIDDEN_SYMBOLS */
  struct NoSafetyConstraints {
      bool init(const std::vector<std::shared_ptr<hardware_interface::JointHandle>>& joints,
          ros::NodeHandle& nh);
      bool enforce(std::vector<double>& commands, const ros::Duration& period);
  };
  /** \endcond */

} // namespace arm_speed_safe_controller

#endif

    // ros::Rate loop_rate(10);
    // std_msgs::Float64MultiArray AllJoints;
    // // using MyMatrix = Eigen::Matrix <double, 40, 5>;
    // // Mymatrix AllJoints;
    // //http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
    // // fill out message:
    // AllJoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // AllJoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // AllJoints.layout.dim[0].label = "height";
    // AllJoints.layout.dim[1].label = "width";
    // AllJoints.layout.dim[0].size = H;
    // AllJoints.layout.dim[1].size = W;
    // AllJoints.layout.dim[0].stride = H*W;
    // AllJoints.layout.dim[1].stride = W;
    // AllJoints.layout.data_offset = 0;
    //
    // std::vector<int> vec(W*H, 0);
    //
    // void update(const ros::Time& /*time*/, const ros::Duration& period)
    // {
    //     std::vector<double>& commands = *commands_buffer.readFromRT();
    //
    //     _constraint.enforce(commands, period);
    //
    //     n_joints = 5; // should take from blackdrops - using a 5-dof
    //     num_runs = 40; //should take from blackdrops - one episode lasts 40 runs usually
    //
    //     for (unsigned int i = 0; i < num_runs; i++) {
    //        Eigen::VectorXd commands = pol.next(joints);   //compute actions and run them
    //        for (unsigned int j = 0; j < n_joints; j++) {
    //              //AllJoints.push_back(joints[i]->getPosition());
    //              vec[i*W + j] = joints[j]->getPosition();
    //              joints[j]->setCommand(commands[j]);
    //              }
    //       }
    //     AllJoints.data = vec;
    //
    // //     while (ros::ok())
    // //        {
    // //            pub.publish(AllJoints); //wrong, use the real time publisher formats
    // //            loop_rate.sleep();
    // //          }
    // //
    // // }

// bool setVel(omni_controllers::BdpService::Request &req, omni_controllers::BdpService::Response &res)
// {
//   Eigen::VectorXd params = req.params;
//   ros::Time t = req.time;
//   ros::Duration dt = req.dt;
//
//   //how to make this only once -- and param list?
//   blackdrops::policy::NNPolicy<Params> pol(boundary, state_dim, hidden_neurons, action_dim, limits, max_u);
//   pol.set_params(params);
//
//   //compute actions
//   Eigen::VectorXd commands = pol.next(joints);
//
//   //start loop to execute the commands
//   return true;
// }
//
// //Client code
// ros::ServiceClient client = nh.serviceClient<omni_controllers::BdpService>("test_robot");
// omni_controllers::BdpService srv; //should this be in an srv folder at same level as src
//
// do
// {
//   if (client.call(srv))
//   {
//     ROS_INFO_STREAM("State-action pairs: " << srv.response); //maybe incorrect, should be srv.response.robotdata
//   }
//   else
//     {
//       ROS_ERROR("Failed to call service");
//       return 1;
//     }
//  }
//  while (ros::ok());

//
// limit rate of publishing
// if (publish_rate_ > 0.0  && time - last_publish_time_ < ros::Duration(1.0/publish_rate_) ) //publish rates?
// {
// ros::Subscriber sub_params = nh.subscribe<PACKAGE_NAME::MESSAGE_NAME>("TOPIC_policy_params",1,&ArmSpeedSafeController::setVel, this);
// //or should it be services?
//
// ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1); //sends the joint states

//Server code should have this? should use parameter?
// ros::spin();
