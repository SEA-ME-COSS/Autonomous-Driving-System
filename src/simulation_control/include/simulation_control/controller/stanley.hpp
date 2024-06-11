#pragma once

#include "utils/car_struct.h"
#include "utils/msg_structs.h"

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <unistd.h>

class Stanley {

public:
    Stanley(double k, double ks);

    Stanley();
    ~Stanley();

    void stanley_control(std::vector<Path> refPoses, double target_v, double x, double y, double yaw, double v);

    double getThrottle();
    double getDelta();

private:
    // Struct
    Car car;

    // Vehicle State    
    double x;
    double y;
    double yaw;
    double v;
    double acceleration;

    double front_x;
    double front_y;

    // PID 
    // double pid_integral;
    // double previous_error;
    double kp;
    // double ki;
    // double kd;

    // Vehicle Config
    double WB;

    // System Config
    double dt;

    // Stanley Parameters
    double k;
    double ks;

    // Path
    std::vector<Path> refPoses;

    // Target Status
    double target_speed;
    int target_node;
    int previous_node;
    double delta;

    // etc
    double node_mindistance;

    void update_state(double x, double y, double yaw, double v);
    double pid_speed_control();
    void update_target_node();
    double stanley_steer_calc();
    double pi_2_pi(double angle);
    bool isNearby(std::vector<double> cpoint, std::vector<double> target_point); 

    void stanley_throttle_calc();
};