#pragma once

#include <string>

struct Pose {
    float x;
    float y;
    float yaw;
    float v;
};

struct Sign {
    std::string id;
    float distance;
};

struct Light {
    std::string id;
    float distance;
};