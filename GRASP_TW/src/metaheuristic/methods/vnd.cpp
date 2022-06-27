#include "./vnd.h"
#include <iostream>
#include <fstream>
#include <iterator>
#include <set>
#include <boost/format.hpp>

using namespace std;
template < typename Type > std::string to_str (const Type & t)
{
    std::ostringstream os;
    os << t;
    return os.str ();
}

vector<string> store_output_info(TSPInstance& instance, TSPSolution* current_solution){
    vector<string> lines = {};
    string chain;
    string chain_2;
    vector<string> customers_ids;
    vector<string> zones_ids;
    set <string> zones_ids_set;

        customers_ids = vector<string>();
        zones_ids = vector<string>();
        for(auto& customer : current_solution->vehicles[0]->customers) {

            customers_ids.push_back(instance.customers_map.at(customer->id));
            auto it = zones_ids_set.find(customer->zone_id);
            if (it != zones_ids_set.end())
            {
                // Do something with it, no more lookup needed.
            }
            else
            {
                zones_ids_set.insert(customer->zone_id);
                zones_ids.push_back(customer->zone_id);
            }
        }
        int cont = 0;
        int cont_2 = 0;
        for(auto& id : customers_ids) {
            if(cont == customers_ids.size() - 1) {
                chain += id;
            } else {
                chain += (boost::format("%1%, ") % id).str();
            }
            cont++;
        }
        lines.push_back(chain);
        for(auto& id : zones_ids) {
            if(cont_2 == zones_ids.size() - 1) {
                chain_2 += id;
            } else {
                chain_2 += (boost::format("%1%, ") % id).str();
            }
            cont_2++;
        }
        lines.push_back(chain_2);
    lines.push_back(to_str(current_solution->cost));
    lines.push_back(to_str(current_solution->time_cost));
    lines.push_back(to_str(current_solution->heading_penalization_cost));
    lines.push_back(to_str(current_solution->angle_penalization_cost));
    lines.push_back(to_str(current_solution->time_window_cost));
    lines.push_back(to_str(current_solution->zone_time_penalization_cost));


    return lines;
}

ostringstream& LocalOptimaOutputs::write_csv_output(vector<string> lines)
{
    ostringstream* vts = new ostringstream();

    // Convert all but the last element to avoid a trailing ","
    copy(lines.begin(), lines.end() - 1, ostream_iterator<string>(*vts, "\n"));
    // Now add the last element with no delimiter
    *vts << lines.back();
    return *vts;
}

shared_ptr <TSPSolution> run_vnd_generator(shared_ptr <TSPSolution> initial_solution,
                              TSPInstance& instance,
                              vector<TSPNeighborhood*>& neighborhoods,
                              string local_optima_direction,
                              int& local_optima_counter,
                              bool by_zones
)
{
    shared_ptr <TSPSolution> best_solution = make_shared<TSPSolution>(initial_solution);
    float previous_cost = best_solution->compute_prob_cost(instance);
    int neighborhood = 0;
    float current_cost = 0;
    while(neighborhood < neighborhoods.size()) {
        shared_ptr <TSPSolution> local_optima = neighborhoods[neighborhood]->search_neighbors(best_solution, instance, by_zones);
        current_cost = local_optima->cost;

        if(current_cost < previous_cost) {
            best_solution = local_optima;
            previous_cost = current_cost;
            neighborhood = 0;
        } else {
            neighborhood++;
            string complete_local_optima_direction = local_optima_direction + +"_it_" + to_str(local_optima_counter);
            ofstream local_optima_output(complete_local_optima_direction);
            local_optima_output << local_optima->TSP_output_string_local_optimas(instance).str()  << endl;
            local_optima_counter++;

        }
    }
    return best_solution;
}

shared_ptr <TSPSolution> run_zones_vnd(shared_ptr <TSPSolution> initial_solution,
                  TSPInstance& instance,
                  vector<TSPNeighborhood*>& neighborhoods,
                  bool by_zones
                  )
{
    shared_ptr <TSPSolution> best_solution = make_shared<TSPSolution>(initial_solution);
    float previous_cost = best_solution->compute_zones_cost(instance);
    int neighborhood = 0;
    float current_cost = 0;
    while(neighborhood < neighborhoods.size()) {
        shared_ptr <TSPSolution> local_optima = neighborhoods[neighborhood]->search_neighbors(best_solution, instance, by_zones);
        current_cost = local_optima->cost;
        if(current_cost < previous_cost) {
            best_solution = local_optima;
            previous_cost = current_cost;
            neighborhood = 0;
        } else {
            neighborhood++;
        }
    }
    return best_solution;
}

shared_ptr <TSPSolution> run_prob_tsp_vnd(shared_ptr <TSPSolution> initial_solution,
                                          TSPInstance& instance,
                                          vector<TSPNeighborhood*>& neighborhoods,
                                          bool by_zones
)
{
    shared_ptr <TSPSolution> best_solution = make_shared<TSPSolution>(initial_solution);
    float previous_cost = best_solution->compute_prob_cost(instance);
    int neighborhood = 0;
    float current_cost = 0;
    while(neighborhood < neighborhoods.size()) {
        shared_ptr <TSPSolution> local_optima = neighborhoods[neighborhood]->search_neighbors(best_solution, instance, by_zones);
        current_cost = local_optima->cost;
        if(current_cost < previous_cost) {
            best_solution = local_optima;
            previous_cost = current_cost;
            neighborhood = 0;
        } else {
            neighborhood++;
        }
    }
    return best_solution;
}


