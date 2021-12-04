#include <ros/ros.h>

#include "hardware/handler.h"
#include "types.h"

using namespace nightmare_types;

int main (int argc, char **argv)
{
    // node initialization
    ros::init(argc, argv, "nightmare_engine");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // handler initialization
    handler h;

    // read parameters from parameter server
    ros::param::get("hardware/servo_port", h.servo_port);
    ros::param::get("hardware/comm_port", h.comm_port);
    ros::param::get("hardware/sensor_port", h.sensor_port);

    std::vector<std::vector<int>> vect;
    ros::param::get("hardware/leg_configuration", vect);
    for (int leg = 0; leg < 6; leg++)
        for (int servo = 0; servo < 3; servo++)
            h.legs[leg].servos[servo].id = vect[leg][servo];

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}