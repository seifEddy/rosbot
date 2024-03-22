#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class RosBot : public hardware_interface::RobotHW
{
  public:
    RosBot(ros::NodeHandle& nh);
    ~RosBot();
    bool init();
    void write();
    void read();
  private:

    ros::NodeHandle _nh;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;
    ros::Subscriber right_wheel_sub;
    ros::Subscriber left_wheel_sub;
    ros::Publisher right_wheel_pub;
    ros::Publisher left_wheel_pub;
    void right_wheel_callback(const std_msgs::Int32::ConstPtr& msg);
    void left_wheel_callback(const std_msgs::Int32::ConstPtr& msg);
    std_msgs::Float64 right_wheel_cmd, left_wheel_cmd;

    double right_wheel_pos, left_wheel_pos, right_wheel_vel, left_wheel_vel;
    double cmd[2];

    // Data member arrays to store the state of the robot's resources (joints, sensors)
    double pos[2], prev_pos[2];
    double vel[2];
    double eff[2];
    const unsigned int ticks_per_rev = 980;
    const float wheel_radius = 0.03;
    ros::Time t = ros::Time::now();
    double dt;
};