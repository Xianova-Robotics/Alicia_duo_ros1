#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <cmath>

class ManipulatorJointPublisher
{
public:
    ManipulatorJointPublisher()
        : rate_(50.0), angle_(0.0)
    {
        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // Must match your URDF exactly (case-sensitive!)
        joint_names_ = {
            "Joint01", "Joint02", "Joint03",
            "Joint04", "Joint05", "Joint06"
        };
    }

    void run()
    {
        while (ros::ok())
        {
            sensor_msgs::JointState js;
            js.header.stamp = ros::Time::now();
            js.name = joint_names_;

            // Simulate one moving joint and fixed values for others
            js.position = {
                std::sin(angle_),
                0.5,
                -0.3,
                0.2,
                -0.1,
                0.0
            };

            joint_pub_.publish(js);

            angle_ += 0.05;
            rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Rate rate_;
    double angle_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulator_joint_publisher");
    ManipulatorJointPublisher node;
    node.run();
    return 0;
}
