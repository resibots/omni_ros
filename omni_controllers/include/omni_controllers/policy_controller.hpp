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

#include <string>
#include <vector>

// ROS
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>

//Local
#include <omni_controllers/PublishData.h>
#include <omni_controllers/PublishMatrix.h>
#include <omni_controllers/PolicyParams.h>
#include <omni_controllers/PolicyInit.h>
#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/policies/NNpolicy.hpp>

namespace arm_speed_safe_controller {

    /**
     * FIXME: \Controller to implement Black-drops policies on a robotic arm, with safety constraints.
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
        // PolicyController() {}
        PolicyController() : flag(false), publish_flag(false), _episode_iterations(0) {}
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

            // flag = false;
            // publish_flag = false;
            // _episode_iterations = 0;

            // Initialisation of the policy
            // TODO: get these values from ros parameters

            // double boundary = 1;
            // int state_dim = 2;
            // int action_dim = 2;
            // int hidden_neurons = 1;
            // Eigen::VectorXd limits;
            // limits << 3.14, 0.78;
            // Eigen::VectorXd max_u;
            // max_u << 1.0, 1.0;

            int state_dim, action_dim, hidden_neurons;
            double boundary;
            Eigen::VectorXd limits;
            std::vector<double> limits_dummy;
            Eigen::VectorXd max_u;
            std::vector<double> max_u_dummy;

            nh.getParam("state_dim", state_dim);
            nh.getParam("action_dim", action_dim);
            nh.getParam("hidden_neurons", hidden_neurons);
            nh.getParam("boundary", boundary);
            nh.getParam("limits", limits_dummy);
            nh.getParam("max_u", max_u_dummy);

            //Convert to Eigen vectors
            for(unsigned int i = 0; i < state_dim; i++)
            {
              limits(i) = limits_dummy[i];
              max_u(i) = max_u_dummy[i];
            }

            _policy = std::make_shared<blackdrops::policy::NNPolicy>(
                boundary, state_dim, hidden_neurons, action_dim, limits, max_u);

            // _sub_command = nh.subscribe<std_msgs::Float64MultiArray>("commands", 1, &PolicyController::commandCB, this);
            _sub_params = nh.subscribe<omni_controllers::PolicyParams>("policyParams", 1, &PolicyController::setParams, this);
            _realtime_pub.reset(new realtime_tools::RealtimePublisher<omni_controllers::PublishMatrix>(nh, "jointStates", 1));

            return true;
        }

        void starting(const ros::Time& time)
        {
            // Start controller with 0.0 velocities
            commands_buffer.readFromRT()->assign(n_joints, 0.0);
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {
            if (flag) // blackdrops parameters to be implemented
            {
                if (_episode_iterations < max_iterations) //during the episode
                {
                    commands = _policy->next(joints_to_eigen());

                    for (unsigned int j = 0; j < n_joints; j++) {
                        JointValues.data.push_back(joints[j]->getPosition());
                        CommandValues.data.push_back(commands(j));
                        joints[j]->setCommand(commands(j));
                    }

                    // _constraint.enforce(commands, period);

                    _episode_iterations++;
                }
                else //episode is over
                {
                    commands.setZero(commands.size());
                    for (unsigned int j = 0; j < n_joints; j++){
                        //record the last set of joint states
                        JointValues.data.push_back(joints[j]->getPosition()); //Doubt -- CommandValues size to differ?
                        //send zero velocities
                        joints[j]->setCommand(commands[j]);
                  }

                    //_constraint.enforce(commands, period);

                    //reset/set flags and _episode_iterations
                    flag = false;
                    publish_flag = true;
                    _episode_iterations = 0;
                }
            }
            else { // outside of an episode, send zero velocities
                for (unsigned int j = 0; j < n_joints; j++)
                    joints[j]->setCommand(0);

                //_constraint.enforce(commands, period);
            }

            // Publishing the data gathered during the episode
            if (publish_flag && _realtime_pub->trylock()) {

                _realtime_pub->msg_.JointPos.push_back(JointValues);
                _realtime_pub->msg_.CommandVel.push_back(CommandValues);
                _realtime_pub->unlockAndPublish();

                //clear storage vectors after publishing is over for the last episode
                std::fill(JointValues.data.begin(), JointValues.data.end(), 0);
                std::fill(CommandValues.data.begin(), CommandValues.data.end(), 0);

                publish_flag = false;
            }

        } //end of update method

        std::vector<std::string> joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
        unsigned int n_joints;

        //Storing of data for every episode
        omni_controllers::PublishData JointValues;
        omni_controllers::PublishData CommandValues;
        Eigen::VectorXd commands;

    private:
        SafetyConstraint _constraint;
        ros::Subscriber _sub_command;
        ros::Subscriber _sub_params;

        double columns, rows, T, dT;
        int max_iterations, _episode_iterations;
        bool publish_flag, flag;

        std::shared_ptr<blackdrops::policy::NNPolicy> _policy;
        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::PublishMatrix>> _realtime_pub;

        void setParams(const omni_controllers::PolicyParams::ConstPtr& msg)
        {
            Eigen::VectorXd params(msg->params.size()); //copy the parameters in a local public array, save time information

            for (int i = 0; i < msg->params.size(); i++)
                params(i) = msg->params[i];

            _policy->set_params(params); //set the policy parameters
            flag = true;

            dT = msg->dT;

            //Hence rows can be set now (correspond to number of runs in an episode)
            max_iterations = (int)msg->t / msg->dT;
        }

        inline Eigen::VectorXd joints_to_eigen()
        {
            Eigen::VectorXd res(joints.size());
            for (size_t i = 0; i < joints.size(); ++i)
                res[i] = joints[i]->getPosition();
            return res;
        }
    };
} // namespace arm_speed_safe_controller

#endif
