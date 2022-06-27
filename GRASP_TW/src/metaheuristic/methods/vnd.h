#pragma once
#include "./neighborhood.h"
#include "../model/solution.h"
#include "../methods/create_zone_matrix.h"
#include <unordered_map>

class LocalOptimaOutputs {
public:
    std::vector<std::vector<std::string>> all_stored_outputs;
    std::ostringstream& write_csv_output(std::vector<std::string> lines);
};

std::shared_ptr <TSPSolution> run_prob_tsp_vnd(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, std::vector<TSPNeighborhood*>& neighborhoods, bool by_zones);

std::shared_ptr <TSPSolution> run_zones_vnd(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, std::vector<TSPNeighborhood*>& neighborhoods, bool by_zones);

std::shared_ptr <TSPSolution> run_vnd_generator(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, std::vector<TSPNeighborhood*>& neighborhoods, std::string local_optima_direction, int& local_optima_counter, bool by_zones);

std::vector<std::string> store_output_info(TSPInstance& instance, TSPSolution* current_solution);

//std::ostringstream& write_csv_output(std::vector<std::string> lines);
//std::vector<std::vector<std::string>> all_stored_outputs;


