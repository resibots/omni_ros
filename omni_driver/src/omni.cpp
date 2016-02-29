#include <chrono>

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
    ROS_INFO_STREAM("Init...");
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
    ROS_INFO_STREAM("Trajectory actionlib controller initialized!");

    // Reset
    return reset();
}

bool Omni::relax()
{
    ROS_INFO_STREAM("Relaxing...");
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
    ROS_INFO_STREAM("Reset...");
    _initial_position_update();
    return zero();
}

bool Omni::reset(const std::vector<double>& joints)
{
    ROS_INFO_STREAM("Reset...");
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

bool Omni::base_return()
{
    double Kp = 1;
    double Kd = 0.0;
    double Ki = 0.2;

    double e2 = 0.0, e1 = 0.0, e0 = 0.0, u2 = 0.0, u1 = 0.0, u0 = 0.0;
    double N = 100;        // filter coefficients

    auto last_time = std::chrono::steady_clock::now();
    bool reached = false;

    while (!reached && ros::ok()) {
        auto now = std::chrono::steady_clock::now();
        double delta = (double)std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now).count() / 1000.0);

        double a0 = (1+N * delta);    
        double a1 = -(2 + N * delta);
        double a2 = 1;
        double b0 = Kp * (1+N * delta) + Ki * delta *(1 + N * delta) + Kd * N;
        double b1 = -(Kp * (2+N * delta) + Ki * delta + 2 * Kd * N);
        double b2 = Kp + Kd * N;
        double ku1 = a1 / a0;
        double ku2 = a2 / a0; 
        double ke0 = b0 / a0;
        double ke1 = b1 / a0; 
        double ke2 = b2 / a0;

        e2 = e1; e1 = e0; u2 = u1; u1 = u0;

        _current_position_update();
        tf::Transform tmp = _base_init_pos.inverse() * _base_pos;
        double roll, pitch, yaw;
        tf::Matrix3x3(tmp.getRotation()).getRPY(roll, pitch, yaw);

        e0 = -yaw;
        u0 = -ku1 * u1 - ku2 * u2 + ke0 * e0 + ke1 * e1 + ke2 * e2;

        std::cout << "signal before saturation: " << u0 << std::endl;
        if (u0 > M_PI)
            u0 = M_PI;
        if (u0 < -M_PI)
            u0 = -M_PI;

        std::cout << "signal after saturation: " << u0 << std::endl;

        base_rotate(u0);

        last_time = now;
        reached = std::abs(e0) < 1e-2;
    }

    double e2_2 = 0.0, e1_2 = 0.0, e0_2 = 0.0, u2_2 = 0.0, u1_2 = 0.0, u0_2 = 0.0; 

    last_time = std::chrono::steady_clock::now();
    reached = false;

    while (!reached && ros::ok()) {
        auto now = std::chrono::steady_clock::now();
        double delta = (double)std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now).count() / 1000.0);

        double a0 = (1+N * delta);    
        double a1 = -(2 + N * delta);
        double a2 = 1;
        double b0 = Kp * (1+N * delta) + Ki * delta *(1 + N * delta) + Kd * N;
        double b1 = -(Kp * (2+N * delta) + Ki * delta + 2 * Kd * N);
        double b2 = Kp + Kd * N;
        double ku1 = a1 / a0;
        double ku2 = a2 / a0; 
        double ke0 = b0 / a0;
        double ke1 = b1 / a0; 
        double ke2 = b2 / a0;

        e2 = e1; e1 = e0; u2 = u1; u1 = u0;
        e2_2 = e1_2; e1_2 = e0_2; u2_2 = u1_2; u1_2 = u0_2;

        _current_position_update();
        tf::Transform tmp = _base_init_pos.inverse() * _base_pos;

        e0 = -(tmp.getOrigin().x());
        u0 = -ku1 * u1 - ku2 * u2 + ke0 * e0 + ke1 * e1 + ke2 * e2;

        e0_2 = -(tmp.getOrigin().y());
        u0_2 = -ku1 * u1_2 - ku2 * u2_2 + ke0 * e0_2 + ke1 * e1_2 + ke2 * e2_2;

        std::cout << "signal before saturation: " << u0 << std::endl;
        std::cout << "signal_2 before saturation: " << u0_2 << std::endl;
        if (u0 > 0.5)
            u0 = 0.5;
        if (u0 < -0.5)
            u0 = -0.5;

        if (u0_2 > 0.5)
            u0_2 = 0.5;
        if (u0_2 < -0.5)
            u0_2 = -0.5;

        std::cout << "signal after saturation: " << u0 << std::endl;
        std::cout << "signal after saturation: " << u0_2 << std::endl;

        base_displace(u0, u0_2);

        last_time = now;
        reached = std::abs(e0) + std::abs(e0_2) < 1e-3;
    }
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

    _traj_client->waitForResult(ros::Duration(_arm_timeout + 0.5));
    if (_traj_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN_STREAM("Trajectory execution failed with status '" << _traj_client->getState().toString() << "'");
        return false;
    }

    return true;
}
