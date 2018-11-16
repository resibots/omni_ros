#ifndef POLICY_CONTROLLER_WITH_RESET_H
#define POLICY_CONTROLLER_WITH_RESET_H

#include <string>
#include <vector>

// ROS
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

//Local
#include <omni_controllers/PolicyParams.h>
#include <omni_controllers/DoubleVector.h>

#include <omni_controllers/statesPub.h>
#include <omni_controllers/commandsPub.h>
#include <omni_controllers/MpcAction.h>
#include <omni_controllers/arm_speed_safe_controller.hpp>
#include <omni_controllers/cartesian_constraint.hpp>
#include <omni_controllers/policies/NNpolicy.hpp>

/*

REDUNDANT : FOR BLACKDROPS ONLY -- REWRITE FOR MPC LATER
Policy controller to run with blackdrops
In its current form, it does the following:

- Using ros_control, sends velocity commands to the arm and reads back the arm positions
- Through the update loop, sends commands on the \cmd_vel topic to move the Base

Along with the controller, the following must be run:

0. Blackdrops from limbo (./../build/exp/blackdrops/src/robot/omni_robot -m 10000 -r 4 -n 10 -b 1 -e 1 -u) Set CMAES parameters as needed
1. TestTopic to launch the node that publishes to the YouBotBaseCOM topic (rosrun omni_contrlers TestTopic)
2. For (1), first the motion capture has to be launched (roslaunch resibots_launch vrpn_optitrack.launch) Note that the IP in the dropdown is set to 152.81.10.239 in the streaming pc
3. From the omnigrasper pc, launch the drivers for the base (roslaunch youbot_driver_ros_interface  youbot_driver.launch)

At end of every episode, following is done:
4. Have the teleop launched, so that the base can be moved back to starting position (this is the location given in init_state for st vector of the omni.cpp) at end of every episode (roslaunch teleop_youbot teleop_omnigrasper.launch
)
5. Call the service (rosservice call /dynamixel_controllers/omni_arm_controller/manualReset) to reset the arm to default configuration

Reading of policy parameters
Sending back states (joint positions of arm, and YouBotBaseCOM from the custom topic = total 7 states) and commands (5 for joints + 2 for base = 7 actions)

*/

namespace arm_speed_safe_controller {

    template <class SafetyConstraint = NoSafetyConstraints>
    class PolicyControllerWithReset : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        PolicyControllerWithReset() {}
        ~PolicyControllerWithReset() { _sub_command.shutdown(); }

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

            _mpc_flag = false;
            publish_flag = false;
            manual_reset_flag = false;
            _episode_iterations = 1;

            _sub_mpc = nh.subscribe<omni_controllers::MpcAction>("mpcActions", 1, &PolicyControllerWithReset::SetMpcActions, this);
            _serv_reset = nh.advertiseService("manualReset", &PolicyControllerWithReset::manualReset, this); //To bring back to default configuration in between episodes
            _realtime_pub_margin = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh, "margin", 4);
            _realtime_pub_joints = std::make_shared<realtime_tools::RealtimePublisher<omni_controllers::statesPub>>(nh, "states", 1);
          // _realtime_pub_joints = std::make_shared<realtime_tools::RealtimePublisher<omni_controllers::statesPub>>(nh, "/dynamixel_controllers/omni_arm_controller/States", 1);


            // _realtime_pub_joints.reset(new realtime_tools::RealtimePublisher<omni_controllers::statesPub>(nh, "States", 1));
            _realtime_pub_commands.reset(new realtime_tools::RealtimePublisher<omni_controllers::commandsPub>(nh, "actions", 1));
            _defaultConfig = {0.0, 0.0, 0.0, 0.0, 0.0}; //For the arm 5 joints
            return true;
        }

        void starting(const ros::Time& time)
        {
            // Start controller with 0.0 velocities
            commands_buffer.readFromRT()->assign(n_joints, 0.0);
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {
          ros::Time curr_time = ros::Time::now();

          if (_mpc_flag) // Blackdrops parameters to be implemented
            {
               // ROS_INFO("Inside UPDATE : Starting mpc flag=true related actions");
               if (_episode_iterations < 2) //During the episode (here it is set to only one step episodes), when mpc commands can be sent
               // if ((_episode_iterations < 2) && (curr_time.toSec() - _prev_time.toSec() >= 0.1)) //during the episode, when blackdrops commands can be sent
                {
                    _commands = Eigen::VectorXd::Map(_mpc_commands.data(), _mpc_commands.size());
                    ROS_INFO("Executing action for one step..");
                    for (unsigned int j = 0; j < n_joints; j++) {
                        _commandList.push_back(_commands(j));
                        //_jointList.push_back(joints[j]->getPosition()); // We need the joint positions only after having sent the commands, i.e after the episode

                        joints[j]->setCommand(_commands(j));
                    }
                     _prev_time = ros::Time::now();
                    _episode_iterations++;

                    if (_realtime_pub_margin->trylock()) {
                        _realtime_pub_margin->msg_.data = _constraint.consult(period);
                        _realtime_pub_margin->unlockAndPublish();
                    }
                    // ROS_INFO("Finished executing action for one step..");
                }

                // else if ((_episode_iterations < 2) && (curr_time.toSec() - _prev_time.toSec()) < 0.1) //wait period during an ongoing episode
                // {
                //     for (unsigned int j = 0; j < n_joints; j++) {
                //         joints[j]->setCommand(_commands(j)); //Sending the earlier set of commands
                //     }
                // }

                else //Episode is over
                {
                    ROS_INFO("One step episode is over, recording joint positions..");
                    for (unsigned int j = 0; j < n_joints; j++) {
                        _jointList.push_back(joints[j]->getPosition()); //Record the last set of joint states
                        joints[j]->setCommand(0); //Send zero velocities
                    }

                   _mpc_flag = false;
                    publish_flag = true;
                    //_episode_iterations =;
                }

                // ROS_INFO("End of mpc=true related actions");
            } //End of mpc mode

            else if (manual_reset_flag) { //Return to default configuration (only for the arm for now) -- called by a service thus not related to mpc/blackdrops

                // ROS_INFO("Starting p-control to reset back to default configuration..");
                std::vector<double> q;
                Eigen::VectorXd velocities(n_joints); //TO DO : This should be changed to action_dim but kept at 5 as we only want to send vel to arm now

                //This next part is only for the arm manual reset

                double time_step = 0.05;
                double threshold = 1e-3;
                double gain = 0.2 / (M_PI * time_step);

                // Current angles
                for (unsigned int i = 0; i < n_joints; i++)
                    q.push_back(joints[i]->getPosition());

                // Error : difference between target and current angle
                std::vector<double> q_err(n_joints, 0.0);
                for (unsigned i = 0; i < n_joints; i++)
                    q_err.at(i) = _defaultConfig.at(i) - q.at(i);

                // Highest error among all joints
                double derr = -std::numeric_limits<double>::max();
                for (unsigned i = 0; i < n_joints; i++) {
                    if (std::abs(q_err.at(i)) > derr)
                        derr = std::abs(q_err.at(i));
                }

                if (derr > threshold) {

                    //Compute velocities to be sent
                    for (unsigned i = 0; i < n_joints; i++) {
                        if (std::abs(q_err.at(i)) > threshold) {
                            velocities(i) = q_err.at(i) * gain;

                            if (velocities(i) > 1.0)
                                velocities(i) = 1.0;
                            if (velocities(i) < -1.0)
                                velocities(i) = -1.0;
                        }
                        else
                            velocities(i) = 0.0;
                    }

                    // Send velocity commands
                    for (unsigned int j = 0; j < n_joints; j++)
                        joints[j]->setCommand(velocities(j));
                }
                else { //Default configuration already reached
                  manual_reset_flag = false;
                }

            } //End of reset mode

            else {
                // Outside of an episode and when already at default configuration, send zero velocities
                // ROS_INFO("Limbo state (outside of episode or bringing to reset), sending zero velocities...");
                for (unsigned int j = 0; j < n_joints; j++) {
                    joints[j]->setCommand(0);
                }
                // _constraint.enforce(period);

            }
            //End of if-else if-else block related to sending correct velocities depending on: mpc/reset/zero modes

            // Publishing the data gathered during the episode
            if (publish_flag) {
              ROS_INFO("Starting the realtime publishing of states");
              if (_realtime_pub_joints->trylock()) {
                    _realtime_pub_joints->msg_.val.clear();
                    // ROS_INFO("cleared the message in realtime joint pub");
                    double tmpval;
                    for(int i=0; i< _jointList.size(); i++){
                        tmpval = _jointList[i];
                       _realtime_pub_joints->msg_.val.push_back(tmpval);

                    }

                    _realtime_pub_joints->unlockAndPublish();

                    if (_realtime_pub_joints->trylock()) {
                        _realtime_pub_joints->msg_.val.clear();
                        // ROS_INFO("cleared the message in realtime joint pub");
                        _realtime_pub_joints->unlock();
                    }

                    _jointList.clear();
                }

                if (_realtime_pub_commands->trylock()) {

                    _realtime_pub_commands->msg_.val.clear();
                    // ROS_INFO("cleared the message in realtime command pub");
                    double tmpval;
                    for(int i=0; i< _commandList.size(); i++){
                        tmpval = _commandList[i];
                       _realtime_pub_commands->msg_.val.push_back(tmpval);
                    }

                    _realtime_pub_commands->unlockAndPublish();

                    if (_realtime_pub_commands->trylock()) {
                      _realtime_pub_commands->msg_.val.clear();
                      // ROS_INFO("cleared the message in realtime command pub");
                      _realtime_pub_commands->unlock();
                    }

                  _commandList.clear();
                }

                if (_realtime_pub_margin->trylock()) {
                    _realtime_pub_margin->msg_.data = _constraint.consult(period);
                    _realtime_pub_margin->unlockAndPublish();
                }

                publish_flag = false;
            } //end of publishing
            _constraint.enforce(period);
        } //end of update method

        std::vector<std::string>joint_names;
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
        unsigned int n_joints;

    private:
        SafetyConstraint _constraint;
        ros::Subscriber _sub_command;
        ros::Subscriber _sub_mpc;
        ros::ServiceServer _serv_reset;

        int _episode_iterations;
        bool publish_flag, manual_reset_flag, _mpc_flag;

        // Temporary vectors that store all values during the whole episode
        std::vector<double> _jointList;
        std::vector<double> _commandList;

        //Default joint angle values for reset purposes
        std::vector<double> _defaultConfig;
        ros::Time _prev_time;

        Eigen::VectorXd _commands;
        // Eigen::VectorXd _mpc_commands;
        std::vector<double> _mpc_commands;

        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::statesPub>> _realtime_pub_joints;
        std::shared_ptr<realtime_tools::RealtimePublisher<omni_controllers::commandsPub>> _realtime_pub_commands;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> _realtime_pub_margin;

        void SetMpcActions(const omni_controllers::MpcAction::ConstPtr& msg)
        {
          // ROS_INFO("Receiving Mpc actions on MpcAction topic");
          //Maybe have to clear _mpc_commands first
          _mpc_commands.clear();
          _mpc_flag = true;
          publish_flag = false;
          _episode_iterations = 1;
          double tmpval;
          for (int i = 0; i < msg->val.size(); i++) {
              tmpval = msg->val[i];
              // ROS_INFO("received value:", tmpval);
              // std::cout <<"received command:"<< tmpval << std::endl;
              //_mpc_commands[i] = tmpval;
              _mpc_commands.push_back(tmpval);
            }
          ROS_INFO("Commands received!");
          _prev_time = ros::Time::now() - ros::Duration(2*0.1);
          // ROS_INFO("Commands received were:", _mpc_commands.transpose());
        }

        bool manualReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
        {
            manual_reset_flag = true;
            return true;
        }

        // inline Eigen::VectorXd states_to_eigen()
        // {
        //     Eigen::VectorXd res(joints.size());
        //
        //     for (size_t i = 0; i < joints.size(); ++i) //Arm
        //         res[i] = joints[i]->getPosition();
        //
        //     return res;
        // }
    }; // policy_controller
} // namespace arm_speed_safe_controller

#endif
