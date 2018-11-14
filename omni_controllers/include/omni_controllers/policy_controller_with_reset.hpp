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

#ifndef POLICY_CONTROLLER_WITH_RESET_H
#define POLICY_CONTROLLER_WITH_RESET_H

#include <string>
#include <vector>

// ROS
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

//Local
#include <omni_controllers/PolicyParams.h>
#include <omni_controllers/DoubleVector.h>

#include <omni_controllers/statesPub.h>
#include <omni_controllers/commandsPub.h>
#include <omni_controllers/MpcAction.h>
#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/policies/NNpolicy.hpp>

/*
Policy controller to run with blackdrops
In its current form, it does the following:

- Using ros_control, sends velocity commands to the arm and reads back the arm positions
- Through the update loop, sends commands on the \cmd_vel topic to move the Base

Along with the controller, the following must be run:

0. Blackdrops from limbo (./../build/exp/blackdrops/src/robot/omni_robot -m 10000 -r 4 -n 10 -b 1 -e 1 -u) Set CMAES parameters as needed
1. TestTopic to launch the node that publishes to the YouBotBaseCOM topic (rosrun omni_contrlers TestTopic)
2. For (1), first the motion capture has to be launched (roslaunch resibots_launch vrpn_optitrack.launch) Note that the IP in the dropdown is set to 152.81.10.239 in the streaming pc
3. From the omnigrasper pc, launch the drivers for the base (roslaunch youbot_driver_ros_interface  youbot_driver.launch)

At end of every episode, following is done:
4. Have the teleop launched, so that the base can be moved back to starting position (this is the location given in init_state for st vector of the omni.cpp) at end of every episode (roslaunch teleop_youbot teleop_omnigrasper.launch
)
5. Call the service (rosservice call /dynamixel_controllers/omni_arm_controller/manualReset) to reset the arm to default configuration

Reading of policy parameters
Sending back states (joint positions of arm, and YouBotBaseCOM from the custom topic = total 7 states) and commands (5 for joints + 2 for base = 7 actions)

*/

namespace arm_speed_safe_controller {

    template <class SafetyConstraint = NoSafetyConstraints>
    class PolicyControllerWithReset : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        PolicyControllerWithReset() {}
        ~PolicyControllerWithReset() { _sub_command.shutdown(); }

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

            _mpc_flag = false;

            // Bdp_eps_flag = false;
            publish_flag = false;
            // reset_flag = false;
            manual_reset_flag = false;
            _episode_iterations = 0;

            // std::vector<double> limits_dummy;
            // std::vector<double> max_u_dummy;
            //
            // int state_dim, action_dim, hidden_neurons;
            // double boundary;
            //
            // if (!nh.getParam("policy_params/state_dim", state_dim)
            //     || !nh.getParam("policy_params/action_dim", action_dim)
            //     || !nh.getParam("policy_params/hidden_neurons", hidden_neurons)
            //     || !nh.getParam("policy_params/limits", limits_dummy)
            //     || !nh.getParam("policy_params/max_u", max_u_dummy)) {
            //     ROS_ERROR_STREAM("Some parameters not received!");
            //     return false;
            // }
            //
            // Eigen::VectorXd limits(state_dim);
            // Eigen::VectorXd max_u(action_dim);
            //
            // //Convert to Eigen vectors
            // for (unsigned int i = 0; i < state_dim; i++) {
            //     limits(i) = limits_dummy[i];
            // }
            //
            // for (unsigned int i = 0; i < action_dim; i++) {
            //     max_u(i) = max_u_dummy[i];
            // }
            //
            // _policy = std::make_shared<blackdrops::policy::NNPolicy>(
            //     boundary, state_dim, hidden_neurons, action_dim, limits, max_u);

            // _sub_params = nh.subscribe<omni_controllers::PolicyParams>("policyParams", 1, &PolicyControllerWithReset::setParams, this); //Receives from blackdrops

            _sub_mpc = nh.subscribe<omni_controllers::MpcAction>("MpcActions", 1, &PolicyControllerWithReset::SetMpcActions, this);
            _serv_reset = nh.advertiseService("manualReset", &PolicyControllerWithReset::manualReset, this); //To bring back to default configuration in between episodes
            // _sub_COM_base = nh.subscribe<omni_controllers::DoubleVector>("YouBotBaseCOM", 1, &PolicyControllerWithReset::getCOM, this); //To read current COM (x, y) of base
            // _realtime_pub_twist = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, "/cmd_vel", 1); // For publishing twist messages to base
            _realtime_pub_margin = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh, "margin", 4);
            // _realtime_pub_joints.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "States", 1));

            _realtime_pub_joints.reset(new realtime_tools::RealtimePublisher<omni_controllers::statesPub>(nh, "States", 1));
            _realtime_pub_commands.reset(new realtime_tools::RealtimePublisher<omni_controllers::commandsPub>(nh, "Actions", 1));

            // _realtime_pub_joints->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            // _realtime_pub_joints->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            // _realtime_pub_commands.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "Actions", 1));
            // _realtime_pub_commands->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            // _realtime_pub_commands->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());

            _defaultConfig = {0.0, 0.0, 0.0, 0.0, 0.0}; //For the arm 5 joints
            // _num_states_COM = 2;  // Only (x, y) of the base COM
            //TO DO : Take from blackdrops

            return true;
        }

        void starting(const ros::Time& time)
        {
            // Start controller with 0.0 velocities
            commands_buffer.readFromRT()->assign(n_joints, 0.0);
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {

          // if (Bdp_eps_flag) // Blackdrops parameters to be implemented
          if (_mpc_flag) // Blackdrops parameters to be implemented
            {
                ros::Time curr_time = ros::Time::now();
                // if ((_episode_iterations < 2) && (curr_time.toSec() - _prev_time.toSec() >= dT)) //During the episode, when mpc commands can be sent
                if (_episode_iterations < 2) //During the episode, when mpc commands can be sent
                {
                    // _commands = _policy->next(states_to_eigen());

                    _commands = _mpc_commands;

                    // _commands already set in the callback and does not change

                    for (unsigned int j = 0; j < n_joints; j++) {
                        _commandList.push_back(_commands(j));
                        _jointList.push_back(joints[j]->getPosition());

                        joints[j]->setCommand(_commands(j));
                    }

                    //Add base positions
                    // for (unsigned int k = 0; k < _num_states_COM; k++) {
                    //     _jointList.push_back(_baseCOM[k]);
                    // }

                    // Add base velocities
                    // for (unsigned int k = n_joints; k < n_joints + _num_states_COM; k++) {
                    //     _commandList.push_back(_commands(k));
                    // }

                    // Extract the last 2 elements of the commands vector to create twist message :: TO-DO make it generic
                    // if (_realtime_pub_twist->trylock()) {
                    //
                    //     _realtime_pub_twist->msg_.linear.x = _commands(5);
                    //     _realtime_pub_twist->msg_.linear.y = _commands(6);
                    //     _realtime_pub_twist->msg_.linear.z = 0.0;
                    //
                    //     _realtime_pub_twist->msg_.angular.x = 0.0;
                    //     _realtime_pub_twist->msg_.angular.y = 0.0;
                    //     // _realtime_pub_twist->msg_.angular.z = _commands(7);
                    //     _realtime_pub_twist->msg_.angular.z = 0.0;
                    //
                    //     _realtime_pub_twist->unlockAndPublish();
                    // }

                    // _prev_time = ros::Time::now();
                    _episode_iterations++;

                    if (_realtime_pub_margin->trylock()) {
                        _realtime_pub_margin->msg_.data = _constraint.consult(period);
                        _realtime_pub_margin->unlockAndPublish();
                    }
                }
                //
                // else if ((_episode_iterations < max_iterations) && (curr_time.toSec() - _prev_time.toSec()) < dT) //Wait period during an ongoing episode
                // {
                //     for (unsigned int j = 0; j < n_joints; j++) {
                //         joints[j]->setCommand(_commands(j)); //Sending the earlier set of commands
                //     }
                // }

                else //Episode is over
                {
                    for (unsigned int j = 0; j < n_joints; j++) {
                        _jointList.push_back(joints[j]->getPosition()); //Record the last set of joint states
                        joints[j]->setCommand(0); //Send zero velocities
                    }

                    //Add base positions
                    // for (unsigned int k = 0; k < _num_states_COM; k++) {
                    //     _jointList.push_back(_baseCOM[k]);
                    // }

                    // Send zero velocities on \cmd_vel
                    // if (_realtime_pub_twist->trylock()) {
                    //     // _realtime_pub_twist->msg_.data = _twist_msg;
                    //
                    //     _realtime_pub_twist->msg_.linear.x = 0.0;
                    //     _realtime_pub_twist->msg_.linear.y = 0.0;
                    //     _realtime_pub_twist->msg_.linear.z = 0.0;
                    //
                    //     _realtime_pub_twist->msg_.angular.x = 0.0;
                    //     _realtime_pub_twist->msg_.angular.y = 0.0;
                    //     _realtime_pub_twist->msg_.angular.z = 0.0;
                    //
                    //     _realtime_pub_twist->unlockAndPublish();
                    // }

                    //Reset/set flags and _episode_iterations
                    _mpc_flag = false;
                    publish_flag = true;
                    // reset_flag = true;
                    _episode_iterations = 0;
                }
            } //End of blackdrops mode

            else if (manual_reset_flag) { //Return to default configuration (only for the arm for now)

                //Make base stationery first

                // Send zero velocities on \cmd_vel
                //
                // if (_realtime_pub_twist->trylock()) {
                //     // _realtime_pub_twist->msg_.data = _twist_msg;
                //
                //     _realtime_pub_twist->msg_.linear.x = 0.0;
                //     _realtime_pub_twist->msg_.linear.y = 0.0;
                //     _realtime_pub_twist->msg_.linear.z = 0.0;
                //
                //     _realtime_pub_twist->msg_.angular.x = 0.0;
                //     _realtime_pub_twist->msg_.angular.y = 0.0;
                //     _realtime_pub_twist->msg_.angular.z = 0.0;
                //
                //     _realtime_pub_twist->unlockAndPublish();
                // }

                std::vector<double> q;
                Eigen::VectorXd velocities(n_joints); //TO DO : This should be changed to action_dim but kept at 5 as we only want to send vel to arm now

                //This next part is only for the arm manual reset

                double time_step = 0.05;
                double threshold = 1e-3;
                double gain = 0.2 / (M_PI * time_step);

                // Current angles
                for (unsigned int i = 0; i < n_joints; i++)
                    q.push_back(joints[i]->getPosition());

                // Error : difference between target and current angle
                std::vector<double> q_err(n_joints, 0.0);
                for (unsigned i = 0; i < n_joints; i++)
                    q_err.at(i) = _defaultConfig.at(i) - q.at(i);

                // Highest error among all joints
                double derr = -std::numeric_limits<double>::max();
                for (unsigned i = 0; i < n_joints; i++) {
                    if (std::abs(q_err.at(i)) > derr)
                        derr = std::abs(q_err.at(i));
                }

                if (derr > threshold) {

                    //Compute velocities to be sent
                    for (unsigned i = 0; i < n_joints; i++) {
                        if (std::abs(q_err.at(i)) > threshold) {
                            velocities(i) = q_err.at(i) * gain;

                            if (velocities(i) > 1.0)
                                velocities(i) = 1.0;
                            if (velocities(i) < -1.0)
                                velocities(i) = -1.0;
                        }
                        else
                            velocities(i) = 0.0;
                    }

                    // Send velocity commands
                    for (unsigned int j = 0; j < n_joints; j++)
                        joints[j]->setCommand(velocities(j));
                }
                else { //Default configuration already reached
                    // reset_flag = false;
                    manual_reset_flag = false;
                }

            } //End of reset mode

            else {
                // Outside of an episode and when already at default configuration, send zero velocities
                for (unsigned int j = 0; j < n_joints; j++) {
                    joints[j]->setCommand(0);
                }
                // _constraint.enforce(period);

            } //End of if-block related to sending correct velocities depending on: blackdrops/reset/zero modes

            // Publishing the data gathered during the episode
            if (publish_flag) {
              if (_realtime_pub_joints->trylock()) {

                    //check details at http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
                    //multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
                    // fill out message:
                    // _realtime_pub_joints->msg_.layout.dim[0].label = "Iterations";
                    // _realtime_pub_joints->msg_.layout.dim[1].label = "JointStates";
                    // _realtime_pub_joints->msg_.layout.dim[0].size = max_iterations; //H
                    // // _realtime_pub_joints->msg_.layout.dim[1].size = n_joints + _num_states_COM; // W for joints + 2 val of COM (time as state is added later in blackdrops hpp)
                    // _realtime_pub_joints->msg_.layout.dim[1].size = n_joints; // W for joints + 2 val of COM (time as state is added later in blackdrops hpp)
                    //
                    // // _realtime_pub_joints->msg_.layout.dim[0].stride = n_joints + _num_states_COM; // For joints + 2 val of COM
                    // _realtime_pub_joints->msg_.layout.dim[0].stride = n_joints;
                    // _realtime_pub_joints->msg_.layout.dim[1].stride = 1;
                    // _realtime_pub_joints->msg_.layout.data_offset = 0;

                    // _realtime_pub_joints->msg_.data = _jointList;
                    // _realtime_pub_joints->unlockAndPublish();
                    //
                    // if (_realtime_pub_joints->trylock()) {
                    //     _realtime_pub_joints->msg_.data.clear();
                    //     _realtime_pub_joints->unlock();
                    // }

                    // _realtime_pub_joints->msg_.data = _jointList;

                    // for(int i=0; i< _jointList.size(); i++){
                    //   _realtime_pub_joints->msg_.val[i]=_jointList(i);
                    //   // _realtime_pub_joints->msg_.val.push_back(_jointList(i));
                    // }

                    double tmpval;
                    for(int i=0; i< _jointList.size(); i++){
                       // _realtime_pub_commands->msg_.val[i]= _commandList(i);

                        tmpval = _jointList[i];
                       _realtime_pub_joints->msg_.val.push_back(tmpval);

                      // _realtime_pub_commands->msg_.val.push_back(_commandList(i));
                    }

                    _realtime_pub_joints->unlockAndPublish();

                    if (_realtime_pub_joints->trylock()) {
                      //
                      // for(int i=0; i< _jointList.size(); i++){
                      //   _realtime_pub_joints->msg_.val[i]= 0;
                      // }

                        _realtime_pub_joints->msg_.val.clear();
                        _realtime_pub_joints->unlock();
                    }

                    _jointList.clear();
                }

                if (_realtime_pub_commands->trylock()) {
                    // _realtime_pub_commands->msg_.layout.dim[0].label = "Iterations";
                    // _realtime_pub_commands->msg_.layout.dim[1].label = "Actions";
                    // _realtime_pub_commands->msg_.layout.dim[0].size = max_iterations; //H
                    // // _realtime_pub_commands->msg_.layout.dim[1].size = n_joints + _num_states_COM; //W (with 2 values for the base velocities)
                    // // _realtime_pub_commands->msg_.layout.dim[0].stride = n_joints + _num_states_COM;
                    //
                    // _realtime_pub_commands->msg_.layout.dim[1].size = n_joints; //W (with 2 values for the base velocities)
                    // _realtime_pub_commands->msg_.layout.dim[0].stride = n_joints;
                    //
                    // _realtime_pub_commands->msg_.layout.dim[1].stride = 1;
                    // _realtime_pub_commands->msg_.layout.data_offset = 0;
                    //
                    // _realtime_pub_commands->msg_.data = _commandList;
                    //
                    // _realtime_pub_commands->unlockAndPublish();
                    //
                    // if (_realtime_pub_commands->trylock()) {
                    //     _realtime_pub_commands->msg_.data.clear();
                    //     _realtime_pub_commands->unlock();
                    // }

                    double tmpval;
                    for(int i=0; i< _commandList.size(); i++){
                       // _realtime_pub_commands->msg_.val[i]= _commandList(i);

                        tmpval = _commandList[i];
                       _realtime_pub_commands->msg_.val.push_back(tmpval);

                      // _realtime_pub_commands->msg_.val.push_back(_commandList(i));
                    }

                    _realtime_pub_commands->unlockAndPublish();

                    if (_realtime_pub_commands->trylock()) {

                      // for(int i=0; i< _commandList.size(); i++){
                      // _realtime_pub_commands->msg_.val[i]=0;
                      // }

                       _realtime_pub_commands->msg_.val.clear();
                      _realtime_pub_commands->unlock();
                    }

                  _commandList.clear();
                }

                if (_realtime_pub_margin->trylock()) {
                    _realtime_pub_margin->msg_.data = _constraint.consult(period);
                    _realtime_pub_margin->unlockAndPublish();
                }

                publish_flag = false;
            } //end of publishing
            _constraint.enforce(period);
        } //end of update method

        std::vector<std::string>joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
        unsigned int n_joints;

    private:
        SafetyConstraint _constraint;
        ros::Subscriber _sub_command;
        // ros::Subscriber _sub_params;
        ros::Subscriber _sub_mpc;
        ros::ServiceServer _serv_reset;
        // ros::Publisher _pub_twist;
        // ros::Subscriber _sub_COM_base;

        // double T, dT; //_rows to help in the publish matrix
        int max_iterations, _episode_iterations;
        bool publish_flag, Bdp_eps_flag, reset_flag, manual_reset_flag, _mpc_flag;
        // int _num_states_COM;

        // Temporary vectors that store all values during the whole episode
        std::vector<double> _jointList;
        std::vector<double> _commandList;

        // std::array<double, 2> _baseCOM; // TO DO Make generic
        // std::vector<double> _baseCOM;

        //Default joint angle values for reset purposes
        std::vector<double> _defaultConfig;

        Eigen::VectorXd _commands;
        Eigen::VectorXd _mpc_commands;
        // ros::Time _prev_time;

        // std::shared_ptr<blackdrops::policy::NNPolicy> _policy;
        // std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> _realtime_pub_joints;
        // std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> _realtime_pub_commands;

        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::statesPub>> _realtime_pub_joints;
        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::commandsPub>> _realtime_pub_commands;

        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> _realtime_pub_margin;
        // std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> _realtime_pub_twist;

        // void setParams(const omni_controllers::PolicyParams::ConstPtr& msg)
        // {
        //     Eigen::VectorXd params(msg->params.size()); //copy the parameters in a local public array, save time information
        //
        //     for (int i = 0; i < msg->params.size(); i++)
        //         params(i) = msg->params[i];
        //
        //     _policy->set_params(params); //set the policy parameters
        //     Bdp_eps_flag = true;
        //
        //     dT = msg->dT;
        //
        //     //Hence rows can be set now (correspond to number of runs in an episode)
        //     max_iterations = std::ceil(msg->t / msg->dT) + 1;
        //     _prev_time = ros::Time::now() - ros::Duration(2 * dT);
        // }

        // void getCOM(const omni_controllers::DoubleVector::ConstPtr& COMmsg)
        // {
        //   for (int i = 0; i < COMmsg->val.size(); i++) {
        //         _baseCOM[i] = COMmsg->val[i];
        //     }
        // }

        void SetMpcActions(const omni_controllers::MpcAction::ConstPtr& msg)
        {
          _mpc_flag = true;
          for (int i = 0; i < msg->val.size(); i++) {
                _mpc_commands(i) = msg->val[i];
            }
            max_iterations = 1; //change this to the number of steps in an episode when using episodic mode
        }

        bool manualReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
        {
            manual_reset_flag = true;
            return true;
        }

        inline Eigen::VectorXd states_to_eigen()
        {
            // Eigen::VectorXd res(joints.size() + _num_states_COM); // TO DO
            Eigen::VectorXd res(joints.size());

            for (size_t i = 0; i < joints.size(); ++i) //Arm
                res[i] = joints[i]->getPosition();

            // for (size_t i = 0; i < _num_states_COM; ++i) //Base
            //     res[joints.size() + i] = _baseCOM[i];

            // res[joints.size() + 3] = _episode_iterations * dT; //Time
            return res;
        }
    }; // policy_controller
} // namespace arm_speed_safe_controller

#endif
