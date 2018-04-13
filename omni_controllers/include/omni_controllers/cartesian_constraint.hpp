#ifndef OMNI_CONTROLLERS_CARTESIAN_CONSTRAINT
#define OMNI_CONTROLLERS_CARTESIAN_CONSTRAINT

#include <ros/node_handle.h>

#include <unordered_set>
#include <algorithm> // std::equal
#include <cmath> // for sin/cos, computing direct kinematic model
#include <Eigen/Core>

namespace arm_speed_safe_controller {

    class OmnigrasperHeightConstraint {
    public:
        /** Initialising method. It must be called before using the enforce method.

            soft_min_height is used to allow the arm to recover. If it is
            between soft_min_height and hard_min_height, speeds that would make
            the arm raise are allowed. If it falls below the hard limit, the arm
            is stopped.

            @param joints JointHandles we the controller is managing
            @param n
        **/
        bool init(const std::vector<std::shared_ptr<hardware_interface::JointHandle>>& joints,
            ros::NodeHandle& nh)
        {
            if (!nh.getParam("hard_min_height", _hard_min_height)) {
                ROS_ERROR_STREAM("Failed to getParam 'hard_min_height' (namespace: "
                    << nh.getNamespace() << ").");
                return false;
            }
            else if (!nh.getParam("soft_min_height", _soft_min_height)) {
                ROS_ERROR_STREAM("Failed to getParam 'soft_min_height' (namespace: "
                    << nh.getNamespace() << ").");
                return false;
            }
            // soft height limit is not allowed to be lower than the hard one
            else if (_soft_min_height < _hard_min_height) {
                ROS_ERROR_STREAM("The soft min height ("
                    << _soft_min_height << ") should not be lower than "
                    << "the hard min height (" << _hard_min_height << ")");
                return false;
            }

            _joints = joints;
            if (N_joints != _joints.size()) {
                ROS_ERROR_STREAM("The number of joints allocated to the "
                    << "controller's cartesian constraint ("
                    << _joints.size() << ") differs from "
                    << "the required number of joints (" << N_joints << ")");
                return false;
            }

            // Check that the joints are all there and in the right order
            std::vector<std::string> joint_names(_joints.size());
            for (auto joint : _joints) {
                joint_names.push_back(joint->getName());
            }
            std::vector<std::string> expected_joint_names
                = {"arm_0_1", "arm_1_2", "arm_2_3", "arm_3_4", "arm_4_5"};
            if (!std::equal(joint_names.begin(), joint_names.end(), expected_joint_names.begin())) {
                ROS_ERROR_STREAM("The joint names are either not as expected.");
                return false;
            }

            return true;
        }

        /**
            The main method of this class. Stops all actuators if any one is beyond
            set joint limits.
            @return true if and only if the joints are outside the limits
        **/
        bool enforce(std::vector<double>& commands, const ros::Duration& period)
        {
            std::vector<double> angles(N_joints), future_angles(N_joints);
            for (auto joint : _joints) {
                angles.push_back(joint->getPosition());
                future_angles.push_back(joint->getPosition() + joint->getVelocity() * period.toSec());
            }

            double current_height = height_reached(angles),
                   future_height = height_reached(future_angles);

            // Stopping condition : being below the hard limit or
            //                      being below  soft limit and still going down
            if (current_height <= _hard_min_height
                || (current_height <= _soft_min_height && future_height < current_height)) {
                commands.assign(N_joints, 0);
                // tell client software that we entered the safety mode
                return true;
            }
            else {
                return false;
            }
        }

    protected:
        /** This method is specific to our robotic arm based on Dynamixel Pros.
            @param angles current joint angles for each actuator
            @return lowest height between the center of the last joint and the end effector
        **/
        inline double height_reached(const std::vector<double>& angles)
        {
            Eigen::Matrix4d T_plate_0;
            T_plate_0 << 1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.141,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_0_1;
            T_0_1 << std::cos(angles[0]), -std::sin(angles[0]), 0., 0.,
                std::sin(angles[0]), std::cos(angles[0]), 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_1_2;
            T_1_2 << -std::sin(angles[1]), -std::cos(angles[1]), 0., 0.,
                0., 0., -1., 0.,
                std::cos(angles[1]), -std::sin(angles[1]), 0., 0.,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_2_3;
            T_2_3 << std::sin(angles[2]), std::cos(angles[2]), 0., 0.264,
                -std::cos(angles[2]), std::sin(angles[2]), 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_3_4;
            T_3_4 << std::cos(angles[3]), -std::sin(angles[3]), 0., 0.,
                0., 0., 1., 0.252,
                -std::sin(angles[3]), -std::cos(angles[3]), 0., 0.,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_4_5;
            T_4_5 << std::cos(angles[4]), std::sin(angles[4]), 0., 0.,
                0., 0., 1., 0.,
                std::sin(angles[4]), -std::cos(angles[4]), 0., 0.,
                0., 0., 0., 1.;
            Eigen::Matrix4d T_5_hook;
            T_5_hook << 1., 0., 0., -0.06,
                0., 0., -1., -0.17,
                0., 1., 0., 0.,
                0., 0., 0., 1.;

            // Position of the last joint
            Eigen::Matrix4d last_joint_trans = T_plate_0 * T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5;
            Eigen::Vector3d last_joint_pos = last_joint_trans.col(3).head(3);

            // Position of the end effecotr's tip
            Eigen::Matrix4d full_trans = last_joint_trans * T_5_hook;
            Eigen::Vector3d end_pos = full_trans.col(3).head(3);

            return std::min(last_joint_pos(2), end_pos(2));
        }

        double _hard_min_height, _soft_min_height;
        std::vector<std::string> _joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> _joints;
        static constexpr int N_joints = 5;
    };
} // namespace arm_speed_safe_controller

#endif
