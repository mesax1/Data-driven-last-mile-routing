#pragma once

#include "../model/solution.h"
#include "../model/instance.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <map>


struct TSPZoneMatrixSolution {
    std::vector<std::vector<float>> zones_time_matrix;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > zones_time_matrix_df;
    std::shared_ptr<TSPSolution> zone_solution;

};
TSPZoneMatrixSolution create_zones_matrix(TSPInstance& instance);