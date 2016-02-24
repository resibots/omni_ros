#include <omni_driver/omni.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

using namespace omni_ros;

Omni::Omni(ros::NodeHandle nh, std::string ns, double arm_timeout) : _nh(nh), _namespace(ns), _arm_timeout(arm_timeout)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();
}

Omni::~Omni()
{
    ROS_INFO_STREAM("Relaxing...");
    relax();
}

void Omni::init()
{
    std::string traj_topic = _namespace + "/omni_arm_controller/follow_joint_trajectory";
    _traj_client = std::make_shared<trajectory_client>(traj_topic, true);
    // TO-DO: blocking duration in params?
    if (!_traj_client->waitForServer(ros::Duration(1.0)))
        ROS_ERROR_STREAM("Actionlib server could not be found at: " << traj_topic);

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();

    msg.joint_names.push_back("arm_joint_1");
    msg.joint_names.push_back("arm_joint_2");
    msg.joint_names.push_back("arm_joint_3");
    msg.joint_names.push_back("arm_joint_4");

    _traj_msg = msg;
    ROS_INFO_STREAM("Trajectory actionlib controller initialized!");

    // Reset
    ROS_INFO_STREAM("Reset...");
    reset();
}

void Omni::relax()
{
     _traj_msg.points.clear();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    point.positions.push_back(3.19395);
    point.positions.push_back(1.37881);
    point.positions.push_back(3.08923);
    point.positions.push_back(1.78024);

    point.time_from_start = ros::Duration(_arm_timeout);

    _traj_msg.points.push_back(point);

    _send_trajectory();
}

void Omni::reset()
{
    zero();
}

void Omni::zero()
{
    // Clear message points
     _traj_msg.points.clear();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    point.positions.push_back(M_PI);
    point.positions.push_back(M_PI);
    point.positions.push_back(M_PI);
    point.positions.push_back(M_PI);

    point.time_from_start = ros::Duration(_arm_timeout);

    _traj_msg.points.push_back(point);

    _send_trajectory();
}

void Omni::set_joint_positions(const std::vector<double>& joints)
{
    _traj_msg.points.clear();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    for (size_t i = 0; i < joints.size(); i++)
        point.positions.push_back(joints[i]);

    point.time_from_start = ros::Duration(_arm_timeout);

    _traj_msg.points.push_back(point);
    _send_trajectory();
}

std::vector<double> Omni::get_joint_positions()
{
    std::vector<double> result;
    //auto state = ac.getState();
    return result;
}

void Omni::_send_trajectory()
{
    _traj_msg.header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = _traj_msg;
    _traj_client->sendGoal(goal);

    _traj_client->waitForResult(ros::Duration(_arm_timeout + 0.5));
    if (_traj_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN_STREAM("Trajectory execution failed with status '" << _traj_client->getState().toString() << "'");
}