#include "planner/planner.hpp"

Planner::Planner(Map* map, double resolution, std::vector<std::array<int, 3>> waypoints) {
    this->map = map;
    this->resolution = resolution;

    double vehicle_width = car.W * resolution;
    double vehicle_length = car.H * resolution;

    this->footprint.push_back( {-vehicle_width/2, -vehicle_length/2} );
    this->footprint.push_back( {vehicle_width/2, -vehicle_length/2} );
    this->footprint.push_back( {vehicle_width/2, vehicle_length/2});
    this->footprint.push_back( {-vehicle_width/2, vehicle_length/2} );

	this->info.change_penalty = 3.0;
	this->info.non_straight_penalty = 2.0;
	this->info.reverse_penalty = 2.0;
	this->info.minimum_turning_radius = 120.0;

	this->theta_resolution = 5;

	this->max_iterations = 1000;
	this->tolerance = 10.0;
    this->it_on_approach = 10;

    this->smoother = new nav2_smac_planner::Smoother<Map>();
    this->optimizer_params.debug = false;
    this->smoother->initialize(this->optimizer_params);
    this->smoother_params.max_curvature = 1.0f / this->info.minimum_turning_radius;
	this->smoother_params.curvature_weight = 30.0;
	this->smoother_params.distance_weight = 0.0;
	this->smoother_params.smooth_weight = 100.0;
	this->smoother_params.costmap_weight = 0.025;
	this->smoother_params.max_time = 0.1;

    this->waypoints = waypoints;
}

Planner::Planner() {
}

Planner::~Planner() {
}

void Planner::plan_route() {
    std::array<int, 3> startPose;
    std::array<int, 3> goalPose;
    int num_it = 0;
    std::vector<int> tempState = {0,0,0};

    // Connect target node with other node
    for(int k = 0; (k + 1) < static_cast<int>(waypoints.size()); ++k) {
        startPose = waypoints[k];
        goalPose = waypoints[k+1];

        // TODO: Exception Handling (About Block)
        if (!map->isValid(startPose[0], startPose[1])) {std::cout<<"Start Error"<<std::endl;}
        if (!map->isValid(goalPose[0], goalPose[1])) {std::cout<<"Goal Error"<<std::endl;}

        nav2_smac_planner::AStarAlgorithm<Map, nav2_smac_planner::GridCollisionChecker<Map, Point>> a_star(nav2_smac_planner::MotionModel::REEDS_SHEPP, info);

        a_star.initialize(false, max_iterations, it_on_approach);
        a_star.setFootprint(footprint, false);
        a_star.createGraph(map->getSizeInCellsX(), map->getSizeInCellsY(), 360 / theta_resolution, map);

        startPose[2] /= theta_resolution;
        goalPose[2] /= theta_resolution;

        a_star.setStart(startPose[0], startPose[1], startPose[2]);
        a_star.setGoal(goalPose[0], goalPose[1], goalPose[2]);

        nav2_smac_planner::NodeSE2::CoordinateVector plan;
	
        bool found = a_star.createPath(plan, num_it, tolerance);
        if (!found) {std::cout<<"NO WAY"<<std::endl; return;}

        for (size_t i = plan.size()-1; 0 < i; --i) {
            waypoints_route.push_back({plan[i].x, plan[i].y, plan[i].theta * M_PI / 180 * theta_resolution});
        }
    }

    std::vector<Eigen::Vector2d> smooth_path;
    for (const auto& node : waypoints_route) {
        smooth_path.push_back(Eigen::Vector2d(node[0], node[1]));
    }

    if (!smoother->smooth(smooth_path, map, smoother_params)) {
        std::cout << "Smoothing Fail" << std::endl;
    }
    else {
        for (size_t i = 0; i < smooth_path.size(); ++i) {
            waypoints_route[i][0] = smooth_path[i].x();
            waypoints_route[i][1] = smooth_path[i].y();
        }
    }
}

std::vector<std::vector<double>> Planner::get_route() {
    // std::cout << "Waypoints route:" << std::endl;
    // for (const auto& waypoint : waypoints_route) {
    //     std::cout << "x: " << waypoint[0] << ", y: " << waypoint[1] << ", theta: " << waypoint[2] << std::endl;
    // }
    return waypoints_route;
}

double Planner::pi_2_degree(double angle) {
    int pose_heading = static_cast<int>(angle * 180 / M_PI);
    while (pose_heading < 0) {pose_heading += 360;}
    while (360 <= pose_heading) {pose_heading -= 360;}

    return pose_heading;
}