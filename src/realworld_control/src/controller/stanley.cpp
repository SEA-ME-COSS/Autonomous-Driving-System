#include "controller/stanley.hpp"

Stanley::Stanley(double k, double ks) {
    // Vehicle Status
    this->acceleration = 0.0;

    // PID Config
    // this->pid_integral = 0.0;
    // this->previous_error = 0.0;
    this->kp = this->car.kp;
    // this->ki = this->car.ki;
    // this->kd = this->car.kd;

    // Vehicle Config
    this->WB = this->car.WB;

    // System Config
    this->dt = 0.1;

    // Stanley Parameters
    this->k = k;
    this->ks = ks;

    // Target
    this->target_node = 0;
    this->previous_node = 0;

    // etc
    this->node_mindistance = 5.0;
}

Stanley::Stanley() {}
Stanley::~Stanley() {}

void Stanley::stanley_control(std::vector<Path> refPoses, double target_v, double x, double y, double yaw, double v) {
    this->target_speed = target_v;
    this->refPoses = refPoses;
    this->update_state(x, y, yaw, v);
    std::cout << "Current Speed : " << this->v << " Current accerleration : " << this->acceleration << " Target Speed : " << target_v << std::endl;

    // this->acceleration = this->pid_speed_control();
    this->update_target_node();
    this->delta = this->stanley_steer_calc();
    this->stanley_throttle_calc();
    std::cout << "After Speed : " << this->v << " After accerleration : " << this->acceleration << std::endl;
}

void Stanley::update_state(double x, double y, double yaw, double v) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->v = v;

    this->front_x = x + ((WB / 2.0) * cos(yaw));
    this->front_y = y + ((WB / 2.0) * sin(yaw));
}

double Stanley::pid_speed_control() {
    // double error = target_speed - v;
    // pid_integral += error * dt;
    // double derivative = (error - previous_error)/dt;
    // double acceleration = kp * error + ki * pid_integral + kd * derivative;
    // previous_error = error;

    double acceleration = kp * (target_speed - v);

    return acceleration;
}

void Stanley::update_target_node() {
    int closest_node = -1;
    int second_closest_node = -1;
    double min_distance = std::numeric_limits<double>::max();
    double second_min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < this->refPoses.size(); ++i) {
        double dx = this->front_x - this->refPoses[i].x;
        double dy = this->front_y - this->refPoses[i].y;
        double distance = std::pow(dx, 2) + std::pow(dy, 2);

        if (distance < min_distance) {
            second_min_distance = min_distance;
            second_closest_node = closest_node;
            min_distance = distance;
            closest_node = i;
        } 
        else if (distance < second_min_distance) {
            second_min_distance = distance;
            second_closest_node = i;
        }
    }

    this->target_node = std::max(closest_node, second_closest_node);
}


double Stanley::stanley_steer_calc() {
    double dx = this->front_x - this->refPoses[this->target_node].x;
    double dy = this->front_y - this->refPoses[this->target_node].y;

    double front_axle_vec_rot_90_x = cos(this->yaw - M_PI / 2.0);
    double front_axle_vec_rot_90_y = sin(this->yaw - M_PI / 2.0);

    double e = dx * front_axle_vec_rot_90_x + dy * front_axle_vec_rot_90_y;

    double theta_e = pi_2_pi(this->refPoses[target_node].yaw - this->yaw);

    double delta = theta_e + atan2(this->k * e, v + ks);

    return delta;
}

double Stanley::pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void Stanley::stanley_throttle_calc() {
    // this->v += this->acceleration * this->dt;
    this->v = this->target_speed;
} 

bool Stanley::isNearby(std::vector<double> cpoint, std::vector<double> target_point) {
    double distance = std::pow(cpoint[0]-target_point[0], 2) + std::pow(cpoint[1]-target_point[1], 2);

    return distance < this->node_mindistance;
}

double Stanley::getThrottle() {
    return v;
}

double Stanley::getDelta() {
    return delta;
}