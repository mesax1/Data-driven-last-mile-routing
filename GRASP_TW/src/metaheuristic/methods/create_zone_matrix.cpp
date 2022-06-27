//
// Created by mesar on 3/04/2021.
//

#include "create_zone_matrix.h"
#include <vector>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <boost/algorithm/string.hpp>


using namespace std;


TSPZoneMatrixSolution create_zones_matrix(TSPInstance& instance){
    //CREATE TEMPORAL SOLUTIONS, WITH JUST ONE VEHICLE
    //THESE SOLUTIONS WILL HOLD INFORMATION ABOUT THE ZONES IN THE ROUTES
    shared_ptr <TSPSolution> solution = make_shared <TSPSolution>(instance, 1);
    shared_ptr <TSPSolution> zone_solution = make_shared <TSPSolution>(instance, 1);

    shared_ptr<TSPCustomer> a;
    shared_ptr<TSPCustomer> b;
    map<string, int> ordered_zones_map (instance.ordered_zones_map);

    vector<shared_ptr<TSPCustomer>> complete_vehicle;
    //TSPVehicle& vehicle = *zone_solution->vehicles[0];
    complete_vehicle.reserve(ordered_zones_map.size());

    zone_solution->customers.clear();

    //CREATE MATRICES THAT WILL HOLD INFORMATION ABOUT AVERAGE TRAVEL TIME  BETWEEN ZONES
    vector<vector<float>> zones_time_matrix(ordered_zones_map.size(), vector<float>(ordered_zones_map.size(), 0));
    unordered_map< string, unordered_map<string, float> > zones_time_matrix_df;

    map<string, int>::iterator zone_itr_1;
    map<string, int>::iterator zone_itr_2;

    zone_itr_1 = ordered_zones_map.begin();
    zone_itr_2 = ordered_zones_map.begin();

    //THESE PARAMETERS WILL NORMALIZE THE DISTANCES AND TIMES INSIDE THE MATRICES OF ZONE_TIME AND ZONE_DISTANCE
    //THEY START AT 1 TO AVOID THE ERROR OF DIVIDING BY 0 WHEN THERE'S NO INFORMATION AVAILABLE
    //EACH OF THESE VALUES WILL CHANGE LATER IN THE CODE, ACCORDING TO THE MAXIMUM VALUES IN THE MATRICES

    float max_time_distance = 1.0;
    float max_nan_time_distance = 1.0;

    vector<float> time_window = {-1, -1};
    float service_time = 0;
    float demand = 0;



    for (int h=0; h < ordered_zones_map.size(); h++) {
        for (int k=0; k < ordered_zones_map.size(); k++) {
            float total_time_rows = 0;
            float total_distance_rows = 0;
            int customers_of_zone = 0;
            //ADD ALL DISTANCES AND TIMES FROM ZONE A TO ZONE B, WITH A AND B REPRESENTING EACH DIFFERENT ZONE IN THE ROUTE
            for (int i=0; i < solution->customers.size(); i++){
                if (solution->customers[i]->zone_id == zone_itr_1->first) {
                    a = solution->customers[i];
                    for (int j = 0; j < solution->customers.size(); j++) {
                        if (solution->customers[j]->zone_id == zone_itr_2->first){
                            b = solution->customers[j];
                            total_time_rows += instance.time_matrix[a->id][b->id];
                            customers_of_zone += 1;
                        }
                    }
                }
            }
            //DISTANCE OR TIME FROM ZONE A TO ZONE A EQUALS TO 0
            if (zone_itr_1->first == zone_itr_2->first){
                zones_time_matrix[h][k] = 0;
            }else {
                //AVERAGE THE SUMS OF TIMES  ACCORDING TO THE NUMBER OF CUSTOMERS IN EACH ZONE
                zones_time_matrix[h][k] = total_time_rows / customers_of_zone;


                if (zones_time_matrix[h][k] >= max_time_distance){
                    if ((zone_itr_1->first != "nan")&&(zone_itr_2->first != "nan")) {
                        max_time_distance = zones_time_matrix[h][k];
                    }
                }
                if (zones_time_matrix[h][k] >= max_nan_time_distance){
                    max_nan_time_distance = zones_time_matrix[h][k];
                }
            }
            zone_itr_2++;
        }

        shared_ptr<TSPCustomer> c = std::make_shared<TSPCustomer>(h, time_window, service_time, demand, zone_itr_1->first );
        complete_vehicle.push_back(c);
        zone_itr_1++;
        zone_itr_2 = ordered_zones_map.begin();
        zone_solution->customers.push_back(c);
        if (c->zone_id == "nan"){
            zone_solution->vehicles[0]->customers.insert(zone_solution->vehicles[0]->customers.begin(), c);
        }
        else {
            zone_solution->vehicles[0]->customers.push_back(c);
        }
    }
    zone_solution->vehicles[0]->customers.push_back(zone_solution->vehicles[0]->customers[0]);



    //NORMALIZE THE VALUES INSIDE THE ZONE_TIME AND DISTANCE_TIME MATRICES WITH RESPECT TO THEIR MAXIMUM POSSIBLE VALUES
    //THIS NORMALIZATION IS DIFFERENT FOR DISTANCES/TIMES BETWEEN ZONES, AND THE DISTANCE/TIME FROM ZONES TO THE DEPOT

    zone_itr_1 = ordered_zones_map.begin();
    zone_itr_2 = ordered_zones_map.begin();
    for (int h=0; h < ordered_zones_map.size(); h++) {
        for (int k=0; k < ordered_zones_map.size(); k++) {
            if ((zone_itr_1->first != "nan")&&(zone_itr_2->first != "nan")) {
                zones_time_matrix_df[zone_itr_1->first][zone_itr_2->first] = zones_time_matrix[h][k] / max_time_distance;
                instance.zones_time_matrix_df[zone_itr_1->first][zone_itr_2->first] = zones_time_matrix[h][k] / max_time_distance;
            }
            else{
                zones_time_matrix_df[zone_itr_1->first][zone_itr_2->first] = zones_time_matrix[h][k] / max_nan_time_distance;
                instance.zones_time_matrix_df[zone_itr_1->first][zone_itr_2->first] = zones_time_matrix[h][k] / max_nan_time_distance;
            }
            zone_itr_2++;
        }
        zone_itr_1++;
        zone_itr_2 = ordered_zones_map.begin();
    }

    return {zones_time_matrix, zones_time_matrix_df, zone_solution};
}
