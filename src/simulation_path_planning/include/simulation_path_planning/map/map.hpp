#pragma once

#include "utils/loader.hpp"

#include "data_structure/ros2_msg_struct.h"

#include <vector>
#include <iostream>
#include <cmath>

class Map {
public:
    Map(const std::string& map_path, double resolution, std::vector<std::array<int, 3>> waypoints_set);
	Map();
    ~Map();

    std::vector<std::vector<unsigned int>> get_cost_map();
    std::vector<double> get_map_info();

	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my);
	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
	double worldToMap(double coordinate);
	double mapToWorld(double coordinate);
	double getCost(int mx, int my);
	double getCost(int idx);
    bool isValid(unsigned int mx, unsigned int my);

	unsigned int getSizeInCellsX();
	unsigned int getSizeInCellsY();

	static constexpr double UNKNOWN = 255;
	static constexpr double OCCUPIED = 254;
	static constexpr double INSCRIBED = 253;
	static constexpr double FREE = 0;

	static constexpr double DRIVEWAY = 0;
	static constexpr double SOLIDLINE = 255;
	static constexpr double PARKINGLOT = 0;
	static constexpr double DOTTEDLINE = 254;
	static constexpr double PASSAVAILABLE = 253;
	static constexpr double STOPLINE = 0;
	static constexpr double CROSSWALK = 0;
	static constexpr double ROUNDABOUT = 255;
	static constexpr double OBSTACLE = 255;
	static constexpr double ISOCCUPIED = 200;

private:
	double resolution;
	double origin_x;
	double origin_y;
	double size_x;
	double size_y;

	std::vector<std::array<int, 3>> waypoints_set;

    std::vector<std::vector<unsigned int>> cost_map;
    void set_cost_map(std::vector<std::vector<unsigned int>> *map_data);
};
