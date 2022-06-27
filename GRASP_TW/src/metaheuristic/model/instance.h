#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <map>

class TSPInstance {
    public:
    int n_cust;
    int depot_pos;
    float Q;

    float k_1;
    float k_2;
    float k_3;
    float k_4;
    float k_5;



    bool normalize ;
    bool in_test_phase ;
    float worst_initial_cost;
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
    std::vector<std::vector<float>> probabilities_matrix;
    std::vector<std::vector<int>> n_routes_matrix;
    std::vector<std::string> historic_zones_id;
    int max_n_routes ;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > probabilities_df;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > angles_df;
    std::unordered_map< std::string, std::unordered_map<std::string, int> > n_routes_df;



    //std::string quality_score;
    std::unordered_map< std::string, std::unordered_map<std::string, float> > zones_time_matrix_df;


    TSPInstance();
    TSPInstance(const std::string& tsp_file, const std::string& route_fid, const int& in_test_phase);
    ~TSPInstance();

};