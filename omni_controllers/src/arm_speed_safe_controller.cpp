/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/lazy_controller.hpp>
#include <omni_controllers/policy_controller.hpp>
#include <omni_controllers/policy_controller_with_reset.hpp>
#include <pluginlib/class_list_macros.h>

namespace arm_speed_safe_controller {
    typedef arm_speed_safe_controller::ArmSpeedSafeController<NoSafetyConstraints> ArmSpeedUnsafeController;
    typedef arm_speed_safe_controller::ArmSpeedSafeController<OmnigrasperHeightConstraint> OmnigrasperSpeedSafeController;
    typedef arm_speed_safe_controller::ArmSpeedSafeController<OmnigrasperHeightSmooth> OmnigrasperSpeedSmoothedSafeController;

    typedef arm_speed_safe_controller::PolicyController<> OmnigrasperUnsafePolicyController;
    typedef arm_speed_safe_controller::PolicyController<OmnigrasperHeightConstraint> OmnigrasperConstraintPolicyController;
    typedef arm_speed_safe_controller::PolicyController<OmnigrasperHeightSmooth> OmnigrasperSmoothConstraintPolicyController;

    typedef arm_speed_safe_controller::PolicyControllerWithReset<> OmnigrasperUnsafePolicyControllerWithReset;
    typedef arm_speed_safe_controller::PolicyControllerWithReset<OmnigrasperHeightConstraint> OmnigrasperConstraintPolicyControllerWithReset;
    typedef arm_speed_safe_controller::PolicyControllerWithReset<OmnigrasperHeightSmooth> OmnigrasperSmoothConstraintPolicyControllerWithReset;

} // namespace arm_speed_safe_controller

PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::ArmSpeedUnsafeController,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperSpeedSafeController,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperSpeedSmoothedSafeController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperUnsafePolicyController,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperConstraintPolicyController,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperSmoothConstraintPolicyController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperUnsafePolicyControllerWithReset,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperConstraintPolicyControllerWithReset,
    controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arm_speed_safe_controller::OmnigrasperSmoothConstraintPolicyControllerWithReset,
    controller_interface::ControllerBase)

// Fake controller that does nothing and only reports the distance to the closest
// height limit of the cartesian safety

namespace lazy_controller {
    typedef lazy_controller::LazyController<
        arm_speed_safe_controller::OmnigrasperHeightSmooth>
        OmnigrasperSmoothLazyController;
}
PLUGINLIB_EXPORT_CLASS(lazy_controller::OmnigrasperSmoothLazyController,
    controller_interface::ControllerBase)
