#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

class RosBot : public hardware_interface::RobotHW
{
public:
  RosBot(ros::NodeHandle& nh) : _nh(nh)
  { 
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them*
    init();
    /** SETUP ROS PUBLISHERS AND SUBSCRIBERS **/
    // Setup the subscribers we need
    right_wheel_sub = _nh.subscribe("right_encoder", 1000, &RosBot::right_wheel_callback, this);
    left_wheel_sub = _nh.subscribe("left_encoder", 1000, &RosBot::left_wheel_callback, this);
    // Setup the publishers we need
    right_wheel_pub = _nh.advertise<std_msgs::Float64>("right_motor_cmd", 1000);
    left_wheel_pub = _nh.advertise<std_msgs::Float64>("left_motor_cmd", 1000);

  }
  ~RosBot(){}
  bool init()
  {
    hardware_interface::JointStateHandle state_handle_a("right_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);
    hardware_interface::JointStateHandle state_handle_b("left_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);
    
    hardware_interface::JointHandle eff_handle_a(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[0]);
    jnt_eff_interface.registerHandle(eff_handle_a);
    hardware_interface::JointHandle eff_handle_b(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[1]);
    jnt_eff_interface.registerHandle(eff_handle_b);

    registerInterface(&jnt_eff_interface);

    return true;
  }
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

void RosBot::read()
{
  dt = (ros::Time::now() - t).toSec();
  // Update time
  t = ros::Time::now();

  pos[0] = right_wheel_pos;
  pos[1] = left_wheel_pos;

  // Determine the velocity m/sec
  // if (dt < 0.0001)
  // {
    // vel[0] = 0.0;
    // vel[1] = 0.0;
  // }
  // else
  // {
    vel[0] = (pos[0] - prev_pos[0]) / dt;
    vel[1] = (pos[1] - prev_pos[1]) / dt;
  // }
  

  // ROS_INFO("Right wheel velocity: %f m/s |---| %f sec", vel[0], dt);
  // Update the previous positions
  prev_pos[0] = pos[0];
  prev_pos[1] = pos[1];
}

void RosBot::write()
{
  right_wheel_cmd.data = cmd[0];
  left_wheel_cmd.data = cmd[1];
  right_wheel_pub.publish(right_wheel_cmd);
  left_wheel_pub.publish(left_wheel_cmd);
}

void RosBot::right_wheel_callback(const std_msgs::Int32::ConstPtr& msg)
{
  right_wheel_pos = (double)msg->data / ticks_per_rev * wheel_radius * 2 * M_PI; 
}

void RosBot::left_wheel_callback(const std_msgs::Int32::ConstPtr& msg)
{
  left_wheel_pos = (double)msg->data / ticks_per_rev * wheel_radius * 2 * M_PI;  
}