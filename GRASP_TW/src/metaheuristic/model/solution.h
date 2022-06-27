#pragma once
#include <vector>
#include <string> 
#include <sstream>
#include "./instance.h"
#include <memory>


class TSPCustomer {
public:
    int id;
    std::vector<float> time_window;
    float demand;
    int vehicle_route;
    bool isRouted;
    float service_time;
    float visited_time;
    std::string zone_id;

    TSPCustomer();
    TSPCustomer(std::shared_ptr <TSPCustomer> other_customer);
    TSPCustomer(int id, std::vector<float> time_window, float service_time, float demand, std::string zone_id);
    ~TSPCustomer();
};

class TSPVehicle{
public:
    int Q;
    std::vector<std::shared_ptr <TSPCustomer>> customers;
    int load;
    int currentTime;
    int currentCustomer;
    float cost;
    std::vector<float> arrivalTime;

    TSPVehicle(int n_cust, int Q);
    TSPVehicle(std::shared_ptr <TSPVehicle> other_vehicle);
    ~TSPVehicle();

    void add_customer(std::shared_ptr <TSPCustomer> customer, const TSPInstance& instance);
    void insert_customer(std::shared_ptr <TSPCustomer> customer, int position, const TSPInstance& instance);
    void remove_customer(int position, const TSPInstance& instance);
};

class TSPSolution {
public:
    std::vector<std::shared_ptr <TSPVehicle>>vehicles;
    std::vector<std::shared_ptr <TSPCustomer>> customers;
    float cost;
    int n_vehicles;

    float time_cost;
    float heading_penalization_cost;
    float angle_penalization_cost;
    float time_window_cost;
    float zone_time_penalization_cost;





    TSPSolution();
    TSPSolution(TSPInstance& instance, int n_vehicles);
    TSPSolution(std::shared_ptr<TSPSolution> otherSolution);
    ~TSPSolution();

    float compute_cost(const TSPInstance& instance);
    float compute_prob_cost(const TSPInstance& instance);
    float compute_zones_cost(const TSPInstance& instance);
    std::ostringstream TSP_output_number_solutions() const;
    std::ostringstream TSP_output_string_solutions(const TSPInstance& instance) const;
    std::ostringstream TSP_output_string_local_optimas(const TSPInstance& instance) const;
    std::ostringstream TSP_output_string_zones(const TSPInstance& instance) const;


};

