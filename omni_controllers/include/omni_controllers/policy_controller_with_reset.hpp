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
#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/policies/NNpolicy.hpp>

namespace arm_speed_safe_controller {

    /**
    * FIXME: \Controller to implement Black-drops policies on a robotic arm, without safety constraints.
    *
    * This class forwards the policy action output in the form of velocity command signals
    down to a set of joints
    *
    * \section ROS interface
    *
    * \param type hardware interface type.
    * \param joints Names of the joints to control.
    *
    * Subscribes to custom msg :
    omni_controllers::PolicyParams - to accept parameter list of the policy for every episode
    */

    // Making following changes to the original policy controller written for the arm
    //
    // States include only positions of the 6 arm joints, (x, y, theta) for the base and time (use mocap) - total 10 (remove the velocities)
    // Actions include 6 arm velocities, (xdot, ydot, thetadot) for the \cmd_vel topic of the youbot - total 9

    // For testing purposes:
    // Step 1: Do not run with blackdrops or change any of how the jointVelList works etc. Simply playback a known policy and send very very low velocity to base to see if this works -- WORKS OK
    // Notes : Maybe reset for the base wheels is not needed since out of update loop, there is just nothing to publish on the cmd_vel. (Will it be needed to read the states for next episode? Check later)
    // Step 2: If step 1 works, then test if the motion capture data can be read back
    // Step 3: Change the jointvelList to include the base portions

    // Update on 15 oct
    // STEP 1 : Sending commands on cmd_vel works OK. Need to create deceleration module to bring wheels to stop ---- TO DO
    // STEP 2 : Reading state of base from tf transform works OK. Created a topic for that, to be launched from TestTopic and can subscribe from within the controller
    // STEP 3 : Remove the velocity recording part and try using only positions for the states, for learning. Remember to change the state action space accordingly, as well as size of params for policy

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

            Bdp_eps_flag = false;
            publish_flag = false;
            reset_flag = false;
            manual_reset_flag = false;
            _episode_iterations = 0;

            std::vector<double> limits_dummy;
            std::vector<double> max_u_dummy;

            int state_dim, action_dim, hidden_neurons;
            double boundary;

            if (!nh.getParam("policy_params/state_dim", state_dim)
                || !nh.getParam("policy_params/action_dim", action_dim)
                || !nh.getParam("policy_params/hidden_neurons", hidden_neurons)
                || !nh.getParam("policy_params/limits", limits_dummy)
                || !nh.getParam("policy_params/max_u", max_u_dummy)) {
                ROS_ERROR_STREAM("Some parameters not received!");
                return false;
            }

            Eigen::VectorXd limits(state_dim);
            Eigen::VectorXd max_u(action_dim);

            //Convert to Eigen vectors
            for (unsigned int i = 0; i < state_dim; i++) {
                limits(i) = limits_dummy[i];
            }

            for (unsigned int i = 0; i < action_dim; i++) {
                max_u(i) = max_u_dummy[i];
            }

            _policy = std::make_shared<blackdrops::policy::NNPolicy>(
                boundary, state_dim, hidden_neurons, action_dim, limits, max_u);

            _sub_params = nh.subscribe<omni_controllers::PolicyParams>("policyParams", 1, &PolicyControllerWithReset::setParams, this);
            _serv_reset = nh.advertiseService("manualReset", &PolicyControllerWithReset::manualReset, this);

            // _test_pub_COM = nh.advertise<std::vector<std::vector<double>>>

            ROS_INFO("Subscribing from ros init");
            _sub_COM_base1 = nh.subscribe<omni_controllers::DoubleVector>("YouBotBaseCOM", 1, &PolicyControllerWithReset::getCOM1, this);
            ROS_INFO("Subscribing from ros init is ok");

            _realtime_pub_margin = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh, "margin", 4);

            _realtime_pub_joints.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "States", 1));
            _realtime_pub_joints->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            _realtime_pub_joints->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            _realtime_pub_commands.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "Actions", 1));
            _realtime_pub_commands->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
            _realtime_pub_commands->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());

            _defaultConfig = {0.0, 0.0, 0.0, 0.0, 0.0};

            _pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // For publishing twist messages to base

            return true;
        }

        void starting(const ros::Time& time)
        {
            // Start controller with 0.0 velocities
            commands_buffer.readFromRT()->assign(n_joints, 0.0);
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {

            if (Bdp_eps_flag) // blackdrops parameters to be implemented
            {
                ros::Time curr_time = ros::Time::now();
                if ((_episode_iterations < max_iterations) && (curr_time.toSec() - _prev_time.toSec() >= dT)) //during the episode, when blackdrops commands can be sent
                {
                    _commands = _policy->next(states_to_eigen());

                    // ROS_INFO("Update: Commands received after policy update : OK");
                    for (unsigned int j = 0; j < n_joints; j++) {
                        _commandList.push_back(_commands(j));
                        _jointList.push_back(joints[j]->getPosition());

                        joints[j]->setCommand(_commands(j));
                    }

                    //Add base positions
                    for (unsigned int k = 0; k < 3; k++) {
                        _jointList.push_back(_baseCOM[k]);
                        ROS_INFO("Update: Adding COM to publisher message");
                        //Send the arm velocities here
                    }

                    // Remove arm velocity
                    // for (unsigned int j = 0; j < n_joints; j++) {
                    //     _jointList.push_back(joints[j]->getVelocity());
                    // }

                    //_baseCOMall.push_back(_baseCOM);
                    std::cout << "Added current COM value" << std::endl;

                    // Send low velocities to test
                    _twist_msg.linear.y = 0.0;
                    _twist_msg.linear.x = 0.05;
                    _twist_msg.linear.z = 0.0;
                    _twist_msg.angular.z = 0.0;
                    _twist_msg.angular.y = 0.0;
                    _twist_msg.angular.x = 0.0;

                    _pub_twist.publish(_twist_msg);

                    _prev_time = ros::Time::now();
                    _episode_iterations++;

                    if (_realtime_pub_margin->trylock()) {
                        _realtime_pub_margin->msg_.data = _constraint.consult(period);
                        _realtime_pub_margin->unlockAndPublish();
                    }
                }

                else if ((_episode_iterations < max_iterations) && (curr_time.toSec() - _prev_time.toSec()) < dT) //wait period during an ongoing episode
                {
                    for (unsigned int j = 0; j < n_joints; j++) {
                        joints[j]->setCommand(_commands(j)); //Sending the earlier set of commands
                    }
                }
                else //episode is over
                {
                    for (unsigned int j = 0; j < n_joints; j++) {
                        //record the last set of joint states
                        _jointList.push_back(joints[j]->getPosition());

                        //send zero velocities
                        joints[j]->setCommand(0);
                        // _constraint.enforce(period);
                    }

                    //Add base positions
                    for (unsigned int k = 0; k < 3; k++) {
                        _jointList.push_back(_baseCOM[k]);
                        //Send the arm velocities here
                    }

                    // Remove arm velocity
                    // for (unsigned int j = 0; j < n_joints; j++) {
                    //     _jointList.push_back(joints[j]->getVelocity());
                    // }

                    // Send zero velocities on \cmd_vel
                    _twist_msg.linear.y = 0.0;
                    _twist_msg.linear.x = 0.0;
                    _twist_msg.linear.z = 0.0;
                    _twist_msg.angular.z = 0.0;
                    _twist_msg.angular.y = 0.0;
                    _twist_msg.angular.x = 0.0;

                    _pub_twist.publish(_twist_msg);

                    //reset/set flags and _episode_iterations
                    Bdp_eps_flag = false;
                    publish_flag = true;
                    reset_flag = true;
                    _episode_iterations = 0;
                }
            } //End of blackdrops mode

            else if (manual_reset_flag) { //Return to default configuration

                std::vector<double> q;
                Eigen::VectorXd velocities(5); //this should be changed to action_dim

                double time_step = 0.05;
                double threshold = 1e-3;
                double gain = 0.2 / (M_PI * time_step);

                // Current angles
                for (unsigned int i = 0; i < n_joints; i++)
                    q.push_back(joints[i]->getPosition());

                // Move joint angles in the 0 - 2π range ??

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
                    reset_flag = false;
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
                // std::cout << "publishing is starting" << std::endl;omni_controllers::ResetManual
                if (_realtime_pub_joints->trylock()) {

                    //check details at http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
                    //multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
                    // fill out message:
                    _realtime_pub_joints->msg_.layout.dim[0].label = "Iterations";
                    //_realtime_pub_joints->msg_.layout.dim[1].label = "JointAndVelStates";
                    _realtime_pub_joints->msg_.layout.dim[1].label = "JointStates";
                    _realtime_pub_joints->msg_.layout.dim[0].size = max_iterations; //H
                    //_realtime_pub_joints->msg_.layout.dim[1].size = n_joints * 2; //W
                    // _realtime_pub_joints->msg_.layout.dim[1].size = n_joints; // W for joints only

                    _realtime_pub_joints->msg_.layout.dim[1].size = n_joints+3; // W for joints+3val of COM only
                    //_realtime_pub_joints->msg_.layout.dim[0].stride = n_joints * 2;

                    // _realtime_pub_joints->msg_.layout.dim[0].stride = n_joints; //For joints only
                    _realtime_pub_joints->msg_.layout.dim[0].stride = n_joints+3; //For joints+3val of COM
                    _realtime_pub_joints->msg_.layout.dim[1].stride = 1;
                    _realtime_pub_joints->msg_.layout.data_offset = 0;

                    _realtime_pub_joints->msg_.data = _jointList;

                    _realtime_pub_joints->unlockAndPublish();

                    if (_realtime_pub_joints->trylock()) {
                        _realtime_pub_joints->msg_.data.clear();
                        _realtime_pub_joints->unlock();
                    }

                    _jointList.clear();
                }
                if (_realtime_pub_commands->trylock()) {
                    _realtime_pub_commands->msg_.layout.dim[0].label = "Iterations";
                    _realtime_pub_commands->msg_.layout.dim[1].label = "Actions";
                    _realtime_pub_commands->msg_.layout.dim[0].size = max_iterations; //H
                    _realtime_pub_commands->msg_.layout.dim[1].size = n_joints; //W (remember to add values for the twist velocities)
                    _realtime_pub_commands->msg_.layout.dim[0].stride = n_joints;
                    _realtime_pub_commands->msg_.layout.dim[1].stride = 1;
                    _realtime_pub_commands->msg_.layout.data_offset = 0;

                    _realtime_pub_commands->msg_.data = _commandList;

                    _realtime_pub_commands->unlockAndPublish();

                    if (_realtime_pub_commands->trylock()) {
                        _realtime_pub_commands->msg_.data.clear();
                        _realtime_pub_commands->unlock();
                    }

                    _commandList.clear();
                }

                if (_realtime_pub_margin->trylock()) {
                    _realtime_pub_margin->msg_.data = _constraint.consult(period);
                    _realtime_pub_margin->unlockAndPublish();
                }

                // for (int i = 0; i < _baseCOMall.size(); i++) {
                //     for (int j = 0; j < _baseCOMall[i].size(); j++) {
                //         std::cout << _baseCOMall[i][j];
                //     }
                // }
                //
                // _baseCOM.clear();
                // _baseCOMall.clear();

                publish_flag = false;
            } //end of publishing
            _constraint.enforce(period);
        } //end of update method

        std::vector<std::string>
            joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
        unsigned int n_joints;

    private:
        SafetyConstraint _constraint;
        ros::Subscriber _sub_command;
        ros::Subscriber _sub_params;
        ros::ServiceServer _serv_reset;
        ros::Publisher _pub_twist;
        ros::Subscriber _sub_COM_base1;
        ros::Subscriber _sub_COM_base2;
        ros::NodeHandle _n;

        geometry_msgs::Twist _twist_msg;
        ros::Publisher _test_pub_COM;

        double T, dT; //_rows to help in the publish matrix
        int max_iterations, _episode_iterations;
        bool publish_flag, Bdp_eps_flag, reset_flag, manual_reset_flag;

        // Temporary vectors that store all values during the whole episode
        std::vector<double> _jointList;
        std::vector<double> _commandList;

        //std::vector<double> _baseCOM;
        std::array<double, 3> _baseCOM;
        std::vector<std::vector<double>> _baseCOMall;

        //Default joint angle values for reset purposes
        std::vector<double> _defaultConfig;

        Eigen::VectorXd _commands;
        ros::Time _prev_time;

        //Eigen::VectorXd _mocap_baseCOM;

        std::shared_ptr<blackdrops::policy::NNPolicy> _policy;
        // std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::PublishMatrix>> _realtime_pub;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> _realtime_pub_joints;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> _realtime_pub_commands;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> _realtime_pub_margin;

        void setParams(const omni_controllers::PolicyParams::ConstPtr& msg)
        {
            // std::cout << "starting callback function, receiving blackdrops params" << std::endl;
            Eigen::VectorXd params(msg->params.size()); //copy the parameters in a local public array, save time information

            for (int i = 0; i < msg->params.size(); i++)
                params(i) = msg->params[i];

            _policy->set_params(params); //set the policy parameters
            Bdp_eps_flag = true;

            dT = msg->dT;

            //Hence rows can be set now (correspond to number of runs in an episode)
            max_iterations = std::ceil(msg->t / msg->dT) + 1;

            _prev_time = ros::Time::now() - ros::Duration(2 * dT);
        }

        void getCOM1(const omni_controllers::DoubleVector::ConstPtr& COMmsg)
        {
            //_baseCOM.clear();
            //Eigen::VectorXd _mocap_baseCOM(COMmsg->val.size());
            std::cout << "entered callback for COM (from ros init)" << std::endl;
            std::cout << "printing COM subscribed to: \n"
                      << std::endl;
            for (int i = 0; i < COMmsg->val.size(); i++) {
                //_baseCOM.push_back(COMmsg->val[i]);
                _baseCOM[i] = COMmsg->val[i];
                // std::cout << COMmsg->val[i] << "," << std::endl;
                std::cout << _baseCOM[i] << "," << std::endl;
            }
        }

        // void getCOM2(const omni_controllers::DoubleVector::ConstPtr& COMmsg)
        // {
        //   std::cout << "entered callback for COM (from update)" << std::endl;
        //   std::cout << "printing COM subscribed to: \n" << std::endl;
        //   for (int i = 0; i < COMmsg->val.size(); i++)
        //       std::cout << COMmsg->val[i] << "," << std::endl;
        //   std::cout << "\n" << std::endl;
        // }

        bool manualReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
        {
            manual_reset_flag = true;
            return true;
        }

        inline Eigen::VectorXd states_to_eigen()
        {
            // Eigen::VectorXd res(joints.size() * 2 + 1); //Removing velocity, only keeping arm positions and time
            Eigen::VectorXd res(joints.size() + 1);

            for (size_t i = 0; i < joints.size(); ++i) {
                res[i] = joints[i]->getPosition();
                //res[5 + i] = joints[i]->getVelocity();
            }
            // for (size_t i = joints.size(); i < joints.size()*2; ++i)
            //res[joints.size() * 2] = _episode_iterations * dT; //to add a state for time (in steps of dT)
            res[joints.size()] = _episode_iterations * dT;
            return res;
        }
    }; // policy_controller
} // namespace arm_speed_safe_controller

#endif
