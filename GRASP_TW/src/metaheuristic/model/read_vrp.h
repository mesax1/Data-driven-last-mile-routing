#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <map>

struct TSPInstanceValues{
    int n_cust;
    int depot_pos;
    float Q;
    std::vector<std::vector<float>> time_matrix;
    std::vector<std::vector<float>> time_windows;
    std::vector<float> service_time;
    std::vector<float> demands;
    std::vector<std::vector<float>> location;
    std::vector<std::vector<unsigned int>> sorted_time_matrix;
    //std::vector<std::vector<std::vector<float>>> insertion_tsp_matrix;
    std::unordered_map<int, std::string> customers_map;
    std::vector<std::string> zone_id;
    std::unordered_map<std::string, int> zones_map;
    std::map<std::string, int> ordered_zones_map;
    //std::string quality_score;
};

TSPInstanceValues read_input_tsptw(const std::string& filename, const int& in_test_phase);

struct TSPHistoricValues{
    std::vector<std::vector<float>> probabilities_matrix;
    std::vector<std::vector<int>> n_routes_matrix;
    std::vector<std::string> historic_zones_id;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > probabilities_df;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > angles_df;
    std::unordered_map< std::string, std::unordered_map<std::string, int> > n_routes_df;
    int max_n_routes;
};

TSPHistoricValues read_historic_files(const std::string& route_fid, const int& number_of_zones, const int& number_of_customers, const int& in_test_phase);