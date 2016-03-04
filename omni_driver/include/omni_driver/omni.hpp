#ifndef OMNI_DRIVER_OMNI_HPP
#define OMNI_DRIVER_OMNI_HPP

#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace omni_ros {

    class Omni {
    public:
        using trajectory_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        Omni(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers", double arm_timeout = 5);
        ~Omni();

        bool init();
        bool relax();
        bool reset();
        bool reset(const std::vector<double>& joints);
        bool zero();
        bool set_joint_positions(const std::vector<double>& joints);
        bool base_displace(double x, double y);
        bool base_rotate(double theta);
        void base_velocity(double x, double y, double theta);
        bool base_return(double Kp=0.9, double Ki=0.3);

        tf::Transform get_arm_frame();

    protected:
        void _current_position_update();
        void _initial_position_update();
        bool _send_arm_trajectory();

        // ROS node handle
        ros::NodeHandle _nh;
        // Store the values of parameters for this ROS node
        std::string _world_frame, _base_link_frame, _arm_frame, _namespace;
        // Seconds to wait for the completion of the arm trajectory
        double _arm_timeout;
        // Trajectory Action Lib Client
        std::shared_ptr<trajectory_client> _traj_client;
        trajectory_msgs::JointTrajectory _traj_msg;

        // to send velocity commands to YouBot base of Omnipointer
        ros::Publisher _velocity_command;

        // TF listener to get the frames for the arm and the base
        tf::TransformListener _listener;
        // TF position
        tf::StampedTransform _arm_pos, _base_pos, _base_init_pos;
    };
}

#endif
