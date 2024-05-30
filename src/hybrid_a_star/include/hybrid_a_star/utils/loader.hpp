#pragma once

#include <string>
#include <cmath>
#include <vector>
#include <array>
#include <map>
#include <fstream>
#include <sstream>
#include <stdexcept>

inline std::vector<std::vector<unsigned int>> load_map_data(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::vector<std::vector<unsigned int>> map_data;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<unsigned int> row;
        int num;
        while (iss >> num) {
            row.push_back(num);
        }
        map_data.push_back(row);
    }

    file.close();

    return map_data;
}

inline std::map<int, std::vector<int>> load_nodes(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::map<int, std::vector<int>> nodes;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<int> nodeData;
        std::string value;
        while (std::getline(ss, value, ',')) {
            nodeData.push_back(stoi(value));
        }
        while (nodeData[3] < 0) {nodeData[3] += 360;}
        while (360 <= nodeData[3]) {nodeData[3] -= 360;}
        nodes[nodeData[0]] = {nodeData[1], nodeData[2], nodeData[3]};
    }

    file.close();

    return nodes;
}

inline std::vector<int> load_points(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::vector<int> points;
    std::string line;

    while (std::getline(file, line)) {
        points.push_back(std::stoi(line));
    }

    file.close();

    return points;
}

inline std::vector<std::array<int, 3>> load_waypoints(const std::string& nodes_file_path, const std::string& points_file_path) {
    std::map<int, std::vector<int>> nodes = load_nodes(nodes_file_path);
    std::vector<int> points = load_points(points_file_path);

    // make waypoints to actual coordinate
    int node_x;
    int node_y;
    int node_angle;
    std::vector<std::array<int, 3>> waypoints;
    for (int& point_id : points) {
        node_x = nodes[point_id][0];
        node_y = nodes[point_id][1];
        node_angle = nodes[point_id][2]; // degree
        waypoints.push_back({node_x, node_y, node_angle});
    }

    return waypoints;
}
