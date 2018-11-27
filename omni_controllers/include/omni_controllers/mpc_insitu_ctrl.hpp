#ifndef MPC_INSITU_CTRL_H
#define MPC_INSITU_CTRL_H

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

//Blackdrops related
// #include <limbo/limbo.hpp>
// #include <tbb/parallel_for.h>
// #include <blackdrops/blackdrops.hpp>
// #include <blackdrops/gp_model.hpp>
// #include <blackdrops/model/gp/kernel_lf_opt.hpp>
// #include <blackdrops/model/multi_gp.hpp>
// #include <blackdrops/model/multi_gp/multi_gp_parallel_opt.hpp>
// #include <blackdrops/system/dart_system.hpp>
// #include <blackdrops/system/omni_robot.hpp>

// #include "dart/dart.hpp"
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>

// struct Params {
//     struct blackdrops : public ::blackdrops::defaults::blackdrops { //TODO: Remove
//         BO_PARAM(size_t, action_dim, 5);
//         BO_PARAM(size_t, model_input_dim, 5);
//         BO_PARAM(size_t, model_pred_dim, 5);
//         BO_PARAM(double, dt, 0.1);
//         BO_PARAM(double, T, 4.0);
//         BO_DYN_PARAM(double, boundary);
//         BO_DYN_PARAM(bool, verbose);
//         BO_DYN_PARAM(bool, stochastic);
//     };
//
//     struct dart_system {
//         BO_PARAM(double, sim_step, 0.001);
//     };
//
//     struct dart_policy_control {
//         BO_PARAM(dart::dynamics::Joint::ActuatorType, joint_type, dart::dynamics::Joint::VELOCITY);
//     };
//
//     struct gp_model {
//         BO_PARAM(double, noise, 0.01);
//     };
//
//     struct mean_constant {
//         BO_PARAM(double, constant, 0.0);
//     };
//
//     struct kernel : public limbo::defaults::kernel {
//         BO_PARAM(double, noise, gp_model::noise());
//         BO_PARAM(bool, optimize_noise, true);
//     };
//
//     struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
//     };
//
// };


namespace arm_speed_safe_controller {

    template <class SafetyConstraint = NoSafetyConstraints>
    class MpcInSituController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        MpcInSituController() {}
        ~MpcInSituController() { _sub_command.shutdown(); }

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

            _sub_mpc = nh.subscribe<omni_controllers::MpcAction>("mpcActions", 1, &MpcInSituController::SetMpcActions, this);
            _serv_reset = nh.advertiseService("manualReset", &MpcInSituController::manualReset, this); //To bring back to default configuration in between episodes
            _realtime_pub_margin = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh, "margin", 4);
            _realtime_pub_joints = std::make_shared<realtime_tools::RealtimePublisher<omni_controllers::statesPub>>(nh, "states", 1);
          // _realtime_pub_joints = std::make_shared<realtime_tools::RealtimePublisher<omni_controllers::statesPub>>(nh, "/dynamixel_controllers/omni_arm_controller/States", 1);


            // _realtime_pub_joints.reset(new realtime_tools::RealtimePublisher<omni_controllers::statesPub>(nh, "States", 1));
            _realtime_pub_commands.reset(new realtime_tools::RealtimePublisher<omni_controllers::commandsPub>(nh, "actions", 1));
            _defaultConfig = {0.0, 0.0, 0.0, 0.0, 0.0}; //For the arm 5 joints
            return true;

          //Blackdrops related

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
                        // joints[j]->setCommand(0); //Send zero velocities
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
                    // joints[j]->setCommand(0);
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
