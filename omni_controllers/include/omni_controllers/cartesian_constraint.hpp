#ifndef OMNI_CONTROLLERS_CARTESIAN_CONSTRAINT
#define OMNI_CONTROLLERS_CARTESIAN_CONSTRAINT

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h> // defines JointHandle
#include <XmlRpcException.h>

#include <sstream> // for debug messages
#include <unordered_set>
#include <algorithm> // std::equal
#include <cmath> // for sin/cos, computing direct kinematic model
#include <Eigen/Core>

namespace arm_speed_safe_controller {

    /** Description of a zone where some height limit applies **/
    struct Zone {
        double x_min, x_max, y_min, y_max;
        double hard_min_height, soft_min_height;
    };

    class OmnigrasperHeightConstraint {
    public:
        OmnigrasperHeightConstraint()
            : _global_zone_set(false) {}

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
            _joints = joints;
            if (N_joints != _joints.size()) {
                ROS_ERROR_STREAM("The number of joints allocated to the "
                    << "controller's cartesian constraint ("
                    << _joints.size() << ") differs from "
                    << "the required number of joints (" << N_joints << ")");
                return false;
            }

            // Check that the joints are all there and in the right order
            std::vector<std::string> joint_names;
            for (auto joint : _joints) {
                joint_names.push_back(joint->getName());
            }

            std::vector<std::string> expected_joint_names
                = {"arm_0_1", "arm_1_2", "arm_2_3", "arm_3_4", "arm_4_5"};
            if (!std::equal(joint_names.begin(), joint_names.end(), expected_joint_names.begin())) {
                std::stringstream names, expected_names;
                for (auto name : joint_names)
                    names << name << " ";
                for (auto name : expected_joint_names)
                    expected_names << name << " ";
                ROS_ERROR_STREAM("The joint names are not as expected.\n"
                    << "\tWe got " << names.str()
                    << " (" << joint_names.size() << ")\n"
                    << "\tWe expected " << expected_names.str()
                    << " (" << expected_joint_names.size() << ")");
                return false;
            }

            // retrieve and parse one or several height limit zones
            if (!_parse_zones(nh))
                return false;

            return true;
        }

        /**
            The main method of this class. Stops all actuators if any one is beyond
            set joint limits.
            @return true if and only if the joints are outside the limits
        **/
        bool enforce(std::vector<double>& commands, const ros::Duration& period)
        {
            std::vector<double> angles, future_angles;
            for (size_t i = 0; i < _joints.size(); i++) {
                auto joint = _joints[i];
                angles.push_back(joint->getPosition());
                future_angles.push_back(joint->getPosition() + commands[i] * period.toSec());
            }

            if (_zone_triggered(angles, future_angles)) {
                commands.assign(N_joints, 0);
                // tell client software that we entered the safety mode
                return true;
            }
            else
                return false;
        }

    protected:
        bool _parse_zones(ros::NodeHandle& nh)
        {
            XmlRpc::XmlRpcValue heights_param; // temporary map, from parameter server
            if (nh.getParam("min_heights", heights_param)) {
                ROS_ASSERT(heights_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
                try {
                    // for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = heights_param.begin(); it != heights_param.end(); ++it) {
                    for (int i = 0; i < heights_param.size(); ++i) {
                        ROS_ASSERT(heights_param[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                        XmlRpc::XmlRpcValue& entry = heights_param[i];
                        if (entry.hasMember("hard_min_height")
                            && entry.hasMember("soft_min_height")) {

                            // soft height limit is not allowed to be lower than the hard one
                            if ((double)entry["soft_min_height"] < (double)entry["hard_min_height"]) {
                                ROS_ERROR_STREAM("The soft min height ("
                                    << (double)entry["soft_min_height"] << ") should not be lower than "
                                    << "the hard min height (" << (double)entry["hard_min_height"] << ")");
                                return false;
                            }

                            if (entry.hasMember("x_min")
                                && entry.hasMember("x_max")
                                && entry.hasMember("y_min")
                                && entry.hasMember("y_max")) {
                                // Boundaries defined, meaning that this
                                // constraint only applies within
                                Zone new_zone;
                                new_zone.x_min = (double)entry["x_min"];
                                new_zone.x_max = (double)entry["x_max"];
                                new_zone.y_min = (double)entry["y_min"];
                                new_zone.y_max = (double)entry["y_max"];
                                new_zone.hard_min_height = (double)entry["hard_min_height"];
                                new_zone.soft_min_height = (double)entry["soft_min_height"];
                                _zones.push_back(new_zone);
                            }
                            else {
                                // No boundaries defined, means this is the
                                // default constraint
                                if (!_global_zone_set) {
                                    _global_zone.hard_min_height = (double)entry["hard_min_height"];
                                    _global_zone.soft_min_height = (double)entry["soft_min_height"];
                                    _global_zone_set = true;
                                }
                                else {
                                    ROS_ERROR_STREAM("Only one global zone can "
                                        << "be defined (a zone without min and "
                                        << "max values along X and Y axes).");
                                    return false;
                                }
                            }
                        }
                        else {
                            ROS_ERROR_STREAM("The minimum height parameters are "
                                             "not properly defined. There needs "
                                             "to at least be 'hard_min_height' "
                                             "and 'soft_min_height' and "
                                             "optionally  'x_min', 'x_max', "
                                             "'y_min' and 'y_max'.");

                            continue;
                        }
                    }
                }
                catch (XmlRpc::XmlRpcException& e) {
                    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                        << "configuration: " << e.getMessage() << ".\n"
                        << "Please check the configuration against the "
                           "documentation (if we made one).");
                    return false;
                }
            }
            else {
                ROS_FATAL_STREAM("Could not get the parameter 'min_heights' "
                    << "(namespace " << nh.getNamespace() << ")");
                return false;
            }
            return true;
        }

        /** Check among all zones whether any is triggered
         * This means that we check whether any of the watched joints is below
         * one of the limits we defined, a.k.a. zones.
         * 
         * @return true if at least one of the height constraints is not respected
         */
        bool _zone_triggered(std::vector<double>& angles, std::vector<double>& future_angles)
        {
            Eigen::Vector3d current_lowest = _lowest_joint(angles),
                            future_lowest = _lowest_joint(future_angles);

            bool triggered = false;
            if (_global_zone_set) {
                triggered = _check_zone(_global_zone, current_lowest, future_lowest, true);
            }

            for (auto zone : _zones) {
                triggered = triggered || _check_zone(zone, current_lowest, future_lowest);
            }

            return triggered;
        }

        /** This method is specific to our robotic arm based on Dynamixel Pros.
            @param angles current joint angles for each actuator
            @return coordinates of the lowest between the center of the last
                joint and the end effector
        **/
        inline Eigen::Vector3d _lowest_joint(const std::vector<double>& angles)
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

            // Position of the end effector's tip
            Eigen::Matrix4d full_trans = last_joint_trans * T_5_hook;
            Eigen::Vector3d end_pos = full_trans.col(3).head(3);

            return (last_joint_pos(2) < end_pos(2)) ? last_joint_pos : end_pos;
        }

        /** Now we see for a given zone if it's triggered
         */
        bool _check_zone(const Zone& zone, Eigen::Vector3d current_lowest,
            Eigen::Vector3d future_lowest, bool global_zone = false)
        {
            // Zoning condition: are the relevant joints in this zone
            if ((current_lowest(0) >= zone.x_min && current_lowest(0) <= zone.x_max
                    && current_lowest(1) >= zone.y_min && current_lowest(1) <= zone.y_max)
                || global_zone) {
                // Stopping condition: being below the hard limit or
                //                      being below  soft limit and still going down
                if (current_lowest(2) <= zone.hard_min_height
                    || (current_lowest(2) <= zone.soft_min_height && future_lowest(2) < current_lowest(2))) {
                    return true;
                }
            }
            return false;
        }

        std::vector<Zone> _zones;
        Zone _global_zone;
        bool _global_zone_set;
        std::vector<std::string> _joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> _joints;
        static constexpr int N_joints = 5;
    }; // namespace arm_speed_safe_controller
} // namespace arm_speed_safe_controller

#endif
