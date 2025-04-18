#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

class ManipulatorJointListener
{
public:
    ManipulatorJointListener()
    {
        joint_sub_ = nh_.subscribe("/joint_states", 10, &ManipulatorJointListener::jointCallback, this);
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        ROS_INFO_STREAM("Received JointState:");
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string& joint_name = msg->name[i];
            double position = (i < msg->position.size()) ? msg->position[i] : 0.0;

            ROS_INFO_STREAM("  " << joint_name << ": " << position);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leader_joint_listener");
    ManipulatorJointListener listener;
    ros::spin();
    return 0;
}
