#include "map/map.hpp"

Map::Map(const std::string& map_path, double resolution, std::vector<std::array<int, 3>> waypoints_set) { 
    std::vector<std::vector<unsigned int>> map_data = load_map_data(map_path);
	
	this->origin_x = 0.0;
	this->origin_y = 0.0;
    this->size_x = static_cast<double>(map_data[0].size());
    this->size_y = static_cast<double>(map_data.size());

    this->resolution = resolution;

    this->waypoints_set = waypoints_set;

    this->set_cost_map(&map_data);
}

Map::Map() {
}

Map::~Map() {
}

void Map::set_cost_map(std::vector<std::vector<unsigned int>> *map_data) {
	cost_map.resize(size_y, std::vector<unsigned int>(size_x, 0));
    std::cout << "Map Size: " << cost_map[0].size() << " x " << cost_map.size() << std::endl;

    for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
			int index = static_cast<int>((*map_data)[y][x]);
            switch (index) {
                case 0:
                    cost_map[y][x] = DRIVEWAY;
                    break;
                case 1:
                    cost_map[y][x] = SOLIDLINE;
                    break;
                case 2:
                    cost_map[y][x] = PARKINGLOT;
                    break;
                case 3:
                    cost_map[y][x] = DOTTEDLINE;
                    break;
                case 4:
                    cost_map[y][x] = STOPLINE;
                    break;
                case 5:
                    cost_map[y][x] = CROSSWALK;
                    break;
                case 6:
                    cost_map[y][x] = ROUNDABOUT;
                    break;
                default:
                    std::cout << "Map Invalid Error" << std::endl;
                    break;
            }
        }
    }
}

bool Map::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) {
	if (wx < origin_x || wy < origin_y)
		return false;
		
	mx = static_cast<unsigned int>((wx - origin_x) / resolution);
	my = static_cast<unsigned int>((wy - origin_y) / resolution);

	return isValid(mx, my);
}

void Map::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const {
	wx = origin_x + (mx + 0.5) * resolution;
	wy = origin_y + (my + 0.5) * resolution;
}

double Map::worldToMap(double coordinate) {
    return coordinate / resolution;
}

double Map::mapToWorld(double coordinate) {
    return coordinate * resolution;
}

double Map::getCost(int mx, int my) {
	if (!isValid(mx, my)) {return UNKNOWN;}
	return cost_map[my][mx];
}

double Map::getCost(int idx) {
	int mx = idx % getSizeInCellsX();
	int my = idx / getSizeInCellsY();
	
	return getCost(mx, my);
}

bool Map::isValid(unsigned int mx, unsigned int my) {
	if (mx < size_x && my < size_y) {return true;}
	else {return false;}
}

unsigned int Map::getSizeInCellsX() {
	return size_x;
}

unsigned int Map::getSizeInCellsY() {
	return size_y;
}

std::vector<std::vector<unsigned int>> Map::get_cost_map() {
    return cost_map;
}

std::vector<double> Map::get_map_info() {
    return {size_x, size_y};
}











