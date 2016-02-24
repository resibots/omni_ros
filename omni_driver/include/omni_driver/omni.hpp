#ifndef OMNI_DRIVER_OMNI_HPP
#define OMNI_DRIVER_OMNI_HPP

#include <vector>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/tf.h>

namespace omni_ros {

    class Omni {
    public:
        using trajectory_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        Omni(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers", double arm_timeout = 5);
        ~Omni();

        void init();
        void relax();
        void reset();
        void zero();
        void set_joint_positions(const std::vector<double>& joints);
        std::vector<double> get_joint_positions();

    protected:
        void _send_trajectory();
        // ROS node handle
        ros::NodeHandle _nh;
        // Store the values of parameters for this ROS node
        std::string  _namespace;
        // Seconds to wait for the completion of the arm trajectory
        double _arm_timeout;
        // Trajectory Action Lib Client
        std::shared_ptr<trajectory_client> _traj_client;
        trajectory_msgs::JointTrajectory _traj_msg;
    };
}

#endif