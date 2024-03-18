#include <ros/ros.h>
#include <rosbot/rosbot.h>
#include <controller_manager/controller_manager.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbot");
    ros::NodeHandle nh;

    RosBot rosbot(nh);

    controller_manager::ControllerManager cm(&rosbot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(20.0); // 10 Hz rate
    
    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        rosbot.read();

        cm.update(time, period);
        
        rosbot.write();
        
        rate.sleep();
    }
    return 0;
}