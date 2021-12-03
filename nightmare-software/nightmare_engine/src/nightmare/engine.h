#ifndef NIGHTMARE_ENGINE_H
#define NIGHTMARE_ENGINE_H

#include <Eigen/Core>

#include "types.h"

using namespace N;

typedef struct leg
{
    uint8_t id;
    Eigen::Vector3d tip;
    double cmd_cx;
    double cmd_fm;
    double cmd_tb;
} leg;

typedef struct engine
{

} engine;

#endif