#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "serial_comm_helper.h"

class MyRobotHW : public hardware_interface::RobotHW
{
public:
    explicit MyRobotHW(ros::NodeHandle& nh);
    // ~MyRobotHW() = default;
    // virtual ~MyRobotHW() = default;
    virtual ~MyRobotHW();  // ✅ correct declaration for the .cpp file


    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

    // Getters for current state
    const std::vector<double>& getJointPositions() const { return joint_position_; }
    double getGripperPosition() const { return gripper_position_; }

    // Setters for sending commands
    void setJointCommands(const std::vector<double>& positions);
    void setGripperCommand(double gripper_deg);

private:
    ros::NodeHandle nh_;
    int num_joints_;  // Declare num_joints_ here
    SerialCommHelper serial_helper_;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // Joint configuration
    static constexpr int NUM_JOINTS = 6;  // Declare num_joints_
    std::vector<std::string> joint_names_ = {
        "Joint01", "Joint02", "Joint03", 
        "Joint04", "Joint05", "Joint06"
    };

    // State vectors
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Command vectors
    std::vector<double> joint_position_command_;
    double gripper_position_ = 0.0;
    double gripper_position_command_ = 0.0;


    // Zero velocity and effort for initialization
    double zero_velocity_ = 0.0;
    double zero_effort_ = 0.0;

};
