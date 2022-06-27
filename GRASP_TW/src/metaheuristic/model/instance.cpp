#include "read_vrp.h"
#include "instance.h"

using namespace std;

TSPInstance::TSPInstance(){}

TSPInstance::TSPInstance(const string& tsp_file, const string& route_fid, const int& in_test_phase){
    TSPInstanceValues values = read_input_tsptw(tsp_file, in_test_phase);
    TSPHistoricValues historic_values = read_historic_files(route_fid, values.zones_map.size(), values.n_cust, in_test_phase);
    this->n_cust = values.n_cust;
    this->depot_pos = values.depot_pos;
    this->Q = values.Q;
    this->normalize = true;
    this->in_test_phase = true;
    this->worst_initial_cost = 0;
    this->demands = values.demands;
    this->time_matrix = values.time_matrix;
    this->time_windows = values.time_windows;
    this->service_time = values.service_time;
    this->sorted_time_matrix = values.sorted_time_matrix;
    this->location = values.location;
    //this->insertion_tsp_matrix = values.insertion_tsp_matrix;
    this->customers_map = values.customers_map;
    this->zone_id = values.zone_id;
    this->zones_map = values.zones_map;
    this->ordered_zones_map = values.ordered_zones_map;
    this->probabilities_matrix = historic_values.probabilities_matrix;
    this->n_routes_matrix = historic_values.n_routes_matrix;

    this->historic_zones_id = historic_values.historic_zones_id;
    this->max_n_routes = historic_values.max_n_routes;
    this->probabilities_df = historic_values.probabilities_df;
    this->angles_df = historic_values.angles_df;
    this->n_routes_df = historic_values.n_routes_df;


    //this->quality_score = values.quality_score;
    this->worst_initial_cost = 0;

    this->k_1 = 1;
    this->k_2 = 1;
    this->k_3 = 1;
    this->k_4 = 1;
    this->k_5 = 1;

}

TSPInstance::~TSPInstance(){}