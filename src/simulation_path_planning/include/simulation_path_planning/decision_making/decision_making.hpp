#pragma once

#include "data_structure/ros2_msg_struct.h"
#include "data_structure/decision_making_struct.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>

class DecisionMaking {
public:
    DecisionMaking(VehicleState current_state, float normal_throttle,
                   std::vector<Sign> *signs, Pose *pose);
    DecisionMaking();
    ~DecisionMaking();

    void decide();
    float getThrottle();
    int getState();

private:
    std::vector<Sign> *signs;
    Pose *pose;

    float throttle;

    VehicleState current_state;
    float normal_throttle;

    bool crosswalknow_timecheck;
    std::chrono::steady_clock::time_point crosswalknow_stoptime;

    bool crosswalkworthy_timecheck;
    std::chrono::steady_clock::time_point crosswalkworthy_time;

    bool rotarynow_timecheck;
    std::chrono::steady_clock::time_point rotarynow_stoptime;

    bool rotaryworthy_timecheck;
    std::chrono::steady_clock::time_point rotaryworthy_time;

    bool trafficredworthy_timecheck;
    std::chrono::steady_clock::time_point trafficredworthy_time;

    bool trafficyellowworthy_timecheck;
    std::chrono::steady_clock::time_point trafficyellowworthy_time;

    bool trafficgreenworthy_timecheck;
    std::chrono::steady_clock::time_point trafficgreenworthy_time;

    float crosswalksign_mindistance;
    float crosswalksign_ignore;

    float rotarysign_mindistance;
    float rotarysign_ignore;

    float trafficsign_mindistance;
    
    float status_break_time;

    bool trafficlight_status;

    bool previous_greentraffic;

    void StatusDecision();

    void DefaultState();
    void TrafficLightRedBeforeState();
    void TrafficLightRedNowState();
    void TrafficLightYellowBeforeState();
    void TrafficLightYellowNowState();
    void TrafficLightGreenBeforeState();
    void TrafficLightGreenNowState();

    void CrosswalkBeforeState();
    void CrosswalkNowState();
    void RoundAboutBeforeState();
    void RoundAboutNowState();

    void RacingState();

    bool isSignWorthy(const std::string state, bool& sign_check, std::chrono::steady_clock::time_point& sign_miss_time);
};
