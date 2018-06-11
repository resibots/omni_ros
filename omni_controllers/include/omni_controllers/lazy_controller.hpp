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

#ifndef OMNI_CONTROLLERS_LAZY_H
#define OMNI_CONTROLLERS_LAZY_H

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#include <omni_controllers/arm_speed_safe_controller.hpp>

namespace lazy_controller {

    struct NoSafetyConstraints;

    /**
     * \brief Not a controller, but a debugging tool for safety limits in
     *      Omnigrasper
     *
     * This class does nothing but publish the distance to the closest safety
     * limit area. It is used for debugging purposes only.
     *
     * \tparam T class implementing the safety constraints
     *
     * \section ROS interface
     *
     * \param type hardware interface type.
     * \param joints Names of the joints to control.
     *
     * Publishes to:
     * - \b min_height (std_msgs::Float64) : how far we are from the closest
     *   safety limit.
     */
    template <class SafetyConstraint = NoSafetyConstraints>
    class LazyController : public controller_interface::
                               Controller<hardware_interface::VelocityJointInterface> {
    public:
        LazyController() {}
        ~LazyController() {}

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
        {
            // Get the list of controlled joints
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
            previousAngles.resize(n_joints);

            // Retrieve handles to the joints
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

            // Init the safety constraint
            if (!_constraint.init(joints, nh)) {
                ROS_ERROR_STREAM("Initialisation of the safety contraint failed");
                return false;
            }

            _realtime_pub_margin
                = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
                    nh, "margin", 4);

            return true;
        }

        void starting(const ros::Time& time) {}

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {
            for (auto joint : joints) {
                joint->setCommand(0);
            }
            if (_realtime_pub_margin->trylock()) {
                _realtime_pub_margin->msg_.data = _constraint.consult(period);
                _realtime_pub_margin->unlockAndPublish();
            }
        }

        std::vector<std::string> joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
            _realtime_pub_margin;
        unsigned int n_joints;

    private:
        SafetyConstraint _constraint;
        std::vector<double> previousAngles;
    };

} // namespace lazy_controller

#endif
