#include <ros/ros.h>

#include "nightmare/engine.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "nightmare_engine");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}