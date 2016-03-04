#include <chrono>

#include <geometry_msgs/Twist.h>

#include <youbot_driver_ros_interface/BaseDisplace.h>
#include <youbot_driver_ros_interface/BaseRotate.h>

#include <omni_driver/omni.hpp>

using namespace omni_ros;

Omni::Omni(ros::NodeHandle nh, std::string ns, double arm_timeout) : _nh(nh), _namespace(ns), _arm_timeout(arm_timeout)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();
}

Omni::~Omni()
{
    relax();
}

bool Omni::init()
{
    ROS_DEBUG_STREAM("Init...");
    // Private node handle
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("World", _world_frame, std::string("world"));
    n_p.param("BaseLink", _base_link_frame, std::string("optitrack_base/base_link"));
    n_p.param("Arm", _arm_frame, std::string("optitrack_arm/base_link"));

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
    ROS_DEBUG_STREAM("Trajectory actionlib controller initialized!");

    // Initialise the publisher to send velocity command
    int message_queue_size = 1;
    _velocity_command = _nh.advertise<geometry_msgs::Twist>("/cmd_vel",
      message_queue_size);
    if (!_velocity_command)
      ROS_WARN_STREAM("could not create the publisher");

    // Reset
    return reset();
}

bool Omni::relax()
{
    ROS_DEBUG_STREAM("Relaxing...");
    _traj_msg.points.clear();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    point.positions.push_back(3.19395);
    point.positions.push_back(1.37881);
    point.positions.push_back(3.08923);
    point.positions.push_back(1.78024);

    point.time_from_start = ros::Duration(_arm_timeout);

    _traj_msg.points.push_back(point);

    return _send_arm_trajectory();
}

bool Omni::reset()
{
    ROS_DEBUG_STREAM("Reset...");
    _initial_position_update();
    return zero();
}

bool Omni::reset(const std::vector<double>& joints)
{
    ROS_DEBUG_STREAM("Reset...");
    _initial_position_update();
    return set_joint_positions(joints);
}

bool Omni::zero()
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

    return _send_arm_trajectory();
}

bool Omni::set_joint_positions(const std::vector<double>& joints)
{
    _traj_msg.points.clear();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    for (size_t i = 0; i < joints.size(); i++)
        point.positions.push_back(joints[i]);

    point.time_from_start = ros::Duration(_arm_timeout);

    _traj_msg.points.push_back(point);
    return _send_arm_trajectory();
}

bool Omni::base_displace(double x, double y)
{
    ros::ServiceClient client = _nh.serviceClient<youbot_driver_ros_interface::BaseDisplace>("/base/displace");
    youbot_driver_ros_interface::BaseDisplace srv;
    srv.request.longitudinal = x;
    srv.request.transversal = y;

    if (!client.call(srv)) {
        ROS_ERROR_STREAM("Failed to call service /base/displace");
        return false;
    }

    return true;
}

bool Omni::base_rotate(double theta)
{
    ros::ServiceClient client = _nh.serviceClient<youbot_driver_ros_interface::BaseRotate>("/base/rotate");
    youbot_driver_ros_interface::BaseRotate srv;
    srv.request.angle = theta;

    if (!client.call(srv)) {
        ROS_ERROR_STREAM("Failed to call service /base/rotate");
        return false;
    }

    return true;
}

void Omni::base_velocity(double x, double y, double theta)
{
    // Prepare the message
    geometry_msgs::Twist command;
    command.linear.x = x;
    command.linear.y = y;
    command.angular.z = theta;

    // Send the command message
    _velocity_command.publish(command);
}

bool Omni::base_return(double Kp, double Ki)
{
    auto last_time = std::chrono::steady_clock::now();
    bool reached = false;

    double ex1 = 0.0, ex0 = 0.0, ux = 0.0;
    double ey1 = 0.0, ey0 = 0.0, uy = 0.0;
    double et1 = 0.0, et0 = 0.0, ut = 0.0;

    last_time = std::chrono::steady_clock::now();
    reached = false;

    while (!reached && ros::ok()) {
        auto now = std::chrono::steady_clock::now();
        double delta = (double)std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now).count() / 1000.0);

        ex1 = ex0;
        ey1 = ey0;
        et1 = et0;

        _current_position_update();
        tf::Transform tmp = _base_pos.inverse() * _base_init_pos;

        ex0 = tmp.getOrigin().x();
        ux = Kp*ex0 + Ki*ex1;

        ey0 = tmp.getOrigin().y();
        uy = Kp*ey0 + Ki*ey1;

        double roll, pitch, yaw;
        tf::Matrix3x3(tmp.getRotation()).getRPY(roll, pitch, yaw);
        et0 = yaw;
        ut = Kp*et0 + Ki*et1;

        reached = (std::abs(ex0) + std::abs(ey0) + std::abs(et0) < 4e-3)
          && (std::abs(ux) + std::abs(uy) + std::abs(ut) < 1e-3);

        if(!reached) {
          if (ux > 0.5)
              ux = 0.5;
          if (ux < -0.5)
              ux = -0.5;

          if (uy > 0.5)
              uy = 0.5;
          if (uy < -0.5)
              uy = -0.5;

          if (ut > 0.5)
              ut = 0.5;
          if (ut < -0.5)
              ut = -0.5;

          base_velocity(ux, uy, ut);

          last_time = now;
        }
        else
          ROS_DEBUG_STREAM("The base is back to its original position");
    }
    base_velocity(0, 0, 0);
}

tf::Transform Omni::get_arm_frame()
{
    _current_position_update();
    return _base_init_pos.inverse() * _base_pos * _arm_pos;
}

void Omni::_initial_position_update()
{
    ros::Time start_t = ros::Time::now();
    while (_nh.ok()) {
        try {
            // Retrieve base frame in the world frame
            _listener.lookupTransform(_world_frame, _base_link_frame, ros::Time(0), _base_init_pos);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_WARN_STREAM("Failed to get transfromation from '" << _world_frame << "' to '" << _base_link_frame << "': " << ex.what());
        }
        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(2.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get transfromation from '" << _base_link_frame << "' to '" << _arm_frame);
            break;
        }
    }
}

void Omni::_current_position_update()
{
    ros::Time start_t = ros::Time::now();
    while (_nh.ok()) {
        try {
            // Retrieve base frame in the world frame
            _listener.lookupTransform(_world_frame, _base_link_frame, ros::Time(0), _base_pos);
        }
        catch (tf::TransformException ex) {
            ROS_WARN_STREAM("Failed to get transfromation from '" << _world_frame << "' to '" << _base_link_frame << "': " << ex.what());
        }

        try {
            // Retrieve arm frame in the base frame
            _listener.lookupTransform(_base_link_frame, _arm_frame, ros::Time(0), _arm_pos);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_WARN_STREAM("Failed to get transfromation from '" << _base_link_frame << "' to '" << _arm_frame << "': " << ex.what());
        }

        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(2.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get transfromation from '" << _base_link_frame << "' to '" << _arm_frame);
            break;
        }
    }
}

bool Omni::_send_arm_trajectory()
{
    _traj_msg.header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = _traj_msg;
    _traj_client->sendGoal(goal);

    _traj_client->waitForResult(ros::Duration(_arm_timeout  * 2));
    if (_traj_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN_STREAM("Trajectory execution failed with status '" << _traj_client->getState().toString() << "'");
        return false;
    }

    return true;
}
