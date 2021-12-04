#ifndef NIGHTMARE_HANDLER_H
#define NIGHTMARE_HANDLER_H

#include <ros/ros.h>

#include "LX16A/lx16a.h"
#include "types.h"

namespace nightmare_type{

typedef struct servo {
    uint8_t id;
    float hardware_angle_raw;
    int16_t hardware_angle;
    float commanded_angle_raw;
    int16_t commanded_angle;
    bool enabled;
} servo;

typedef struct leg {
    uint8_t id;
    servo servos[3];
    float force = 0;
} legs;

typedef struct handler {
    leg legs[6];
    std::string servo_port;
    std::string sensor_port;
    std::string comm_port;
} handler;

void update(void);

};

#endif