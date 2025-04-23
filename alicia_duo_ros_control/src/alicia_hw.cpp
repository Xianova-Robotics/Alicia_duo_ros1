#include "alicia_hw.h"



MyRobotHW::MyRobotHW(ros::NodeHandle& nh)
    : nh_(nh), serial_helper_(nh) {}


// Corrected read method signature to match the base class
void MyRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
    auto [joints, gripper_deg] = serial_helper_.readJointAndGripper();
    if (joints.size() == 6)
    {
        joint_position_ = joints;
        gripper_position_ = gripper_deg;
    }
}

// Corrected write method signature to match the base class
void MyRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
    serial_helper_.writeServoCommand(joint_position_command_, gripper_position_command_);
}

// Setters for sending joint commands and gripper command
void MyRobotHW::setJointCommands(const std::vector<double>& positions)
{
    if (positions.size() == joint_position_command_.size())
    {
        joint_position_command_ = positions;
    }
}

void MyRobotHW::setGripperCommand(double gripper_deg)
{
    gripper_position_command_ = gripper_deg;
}

MyRobotHW::~MyRobotHW() {}


bool MyRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    ROS_INFO("MyRobotHW::init() called, interfaces registered.");

    num_joints_ = 6;
    joint_names_ = {"Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06"};

    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    joint_position_command_.resize(num_joints_, 0.0);

    for (int i = 0; i < num_joints_; ++i)
    {
        joint_state_interface_.registerHandle(
            hardware_interface::JointStateHandle(joint_names_[i],
                                                 &joint_position_[i],
                                                 &joint_velocity_[i],
                                                 &joint_effort_[i]));

        position_joint_interface_.registerHandle(
            hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                            &joint_position_command_[i]));
    }

    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle("Grip_control", &gripper_position_, &zero_velocity_, &zero_effort_));

    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle("Grip_control"),
                                        &gripper_position_command_));

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    return true;
}
