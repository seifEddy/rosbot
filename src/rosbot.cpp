#include <rosbot/rosbot.h>

RosBot::RosBot(ros::NodeHandle& nh) : _nh(nh)
{ 
    init();
    /** SETUP ROS PUBLISHERS AND SUBSCRIBERS **/
    // Setup the subscribers we need
    right_wheel_sub = _nh.subscribe("right_encoder", 1000, &RosBot::right_wheel_callback, this);
    left_wheel_sub = _nh.subscribe("left_encoder", 1000, &RosBot::left_wheel_callback, this);
    // Setup the publishers we need
    right_wheel_pub = _nh.advertise<std_msgs::Float64>("right_motor_cmd", 1000);
    left_wheel_pub = _nh.advertise<std_msgs::Float64>("left_motor_cmd", 1000);

}

RosBot::~RosBot()
{
    
}

bool RosBot::init()
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

void RosBot::read()
{
    dt = (ros::Time::now() - t).toSec();
    // Update time
    t = ros::Time::now();
    
    pos[0] = right_wheel_pos;
    pos[1] = left_wheel_pos;
    
    vel[0] = (pos[0] - prev_pos[0]) / dt;
    vel[1] = (pos[1] - prev_pos[1]) / dt;
    
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