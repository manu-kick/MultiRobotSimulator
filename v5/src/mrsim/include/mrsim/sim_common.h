#pragma once
#include <string>
#include "world.h"
#include "simple_geometry.h"

struct RobotHandle {
    WorldItem* ptr;              // non-owning; World keeps it alive
    std::string id;
    std::string type;
    Pose init;
    int arm_index{-1};           // -1 => robot control; >=0 => controlling link i
};
