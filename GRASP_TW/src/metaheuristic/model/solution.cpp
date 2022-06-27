#include "./solution.h"
#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <set>
#include <unordered_map>
#include <memory>
#include <boost/algorithm/string.hpp>

using namespace std;


//TSP INSTANCES

TSPVehicle::TSPVehicle(const shared_ptr<TSPVehicle>  other_vehicle)
        :Q(other_vehicle->Q)
        ,customers(other_vehicle->customers)
        , load(other_vehicle->load)
        , currentTime(other_vehicle->currentTime)
        , currentCustomer(other_vehicle->currentCustomer)
        , cost(other_vehicle->cost)
        , arrivalTime(other_vehicle->arrivalTime)
{
}

TSPVehicle::TSPVehicle(int n_cust, int Q)
{
    /**
     *  Vehicle object that carries the goods
     */

    this->Q = Q;
    this->customers = {};
    this->load = 0;
    this->currentTime = 0;
    this->currentCustomer = 0; // starting location is depot
    this->cost = 0;
    this->arrivalTime = {};


}

TSPVehicle::~TSPVehicle()
{
}

void TSPVehicle::add_customer(shared_ptr <TSPCustomer> customer, const TSPInstance& instance)
{
    /**
     * Add customer to vehicle route
     * @param customer Object of class Node
     * @param instance Object of class TSPInstance
     */

    this->customers.push_back(customer);
    this->currentTime += instance.time_matrix[this->currentCustomer][customer->id];
    this->currentCustomer = customer->id;
    this->arrivalTime.push_back(currentTime);
    customer->visited_time = this->currentTime;
    if (customer->id == instance.depot_pos){
        customer->visited_time = 0;
    }
    this->currentTime += customer->service_time;
    this->load += customer->demand;
}

void TSPVehicle::insert_customer(shared_ptr <TSPCustomer> customer, int position, const TSPInstance& instance)
{
    this->customers.insert(this->customers.begin() + position, customer); // Insert customer between customerBefore and customerAfter
    this->load += customer->demand;
}

void TSPVehicle::remove_customer(int position, const TSPInstance& instance)
{
    this->load -= this->customers[position]->demand;
    this->customers.erase(this->customers.begin() + position);
}

TSPCustomer::TSPCustomer()
{
}

TSPCustomer::TSPCustomer(shared_ptr <TSPCustomer> other_customer)
        : id(other_customer->id)
        , time_window(other_customer->time_window)
        , demand(other_customer->demand)
        , vehicle_route(other_customer->vehicle_route)
        , isRouted(other_customer->isRouted)
        , service_time(other_customer->service_time)
        , visited_time(other_customer->visited_time)
        , zone_id(other_customer->zone_id)


{
}

TSPCustomer::TSPCustomer(int id,  vector<float> time_window, float service_time, float demand, string zone_id)
{
    this->id = id;
    this->time_window = time_window;
    this->service_time = service_time;
    this->visited_time = -1;
    this->vehicle_route = -1;
    this->isRouted = false;
    this->demand = demand;
    this->zone_id = zone_id;

}

TSPCustomer::~TSPCustomer()
{
}

TSPSolution::TSPSolution()
{
}

TSPSolution::TSPSolution(TSPInstance& instance, int n_vehicles)
{
    this->vehicles = vector<shared_ptr<TSPVehicle>>();
    for(int n = 0; n < n_vehicles; n++) {
        this->vehicles.push_back(std::make_shared<TSPVehicle>(instance.n_cust, instance.Q));
    }
    this->customers = vector<shared_ptr<TSPCustomer>>();
    //Assign zone_id to stops with zone_id=nan, assigning it's nearest neighbor zone_id
    for(int n = 0; n < instance.n_cust; n++) {
        if (instance.zone_id[n] == "nan") {
            if (n != instance.depot_pos) {
                int nearest_customer = 0;
                string n_zone = "nan";
                while (n_zone == "nan") {
                    n_zone = instance.zone_id[instance.sorted_time_matrix[n][nearest_customer]];
                    nearest_customer++;
                }
                cout << "Previous zone_id: " << instance.zone_id[n] << endl;
                instance.zones_map.at(instance.zone_id[n])--;
                instance.zone_id[n] = n_zone;
                cout << "New zone_id: " << instance.zone_id[n] << endl;
                instance.zones_map.at(instance.zone_id[n])++;
            }
        }
        this->customers.push_back(std::make_shared<TSPCustomer>(n, instance.time_windows[n], instance.service_time[n], instance.demands[n], instance.zone_id[n]));
    }

    this->cost = 0;
    this->n_vehicles = n_vehicles;
    this->time_cost = 0;
    this->heading_penalization_cost = 0;
    this->angle_penalization_cost = 0;
    this->time_window_cost = 0;
    this->zone_time_penalization_cost = 0;

}

TSPSolution::TSPSolution(shared_ptr<TSPSolution> otherSolution)
        : vehicles(otherSolution->vehicles)
        , customers(otherSolution->customers)
        , cost(otherSolution->cost)
        , n_vehicles(otherSolution->n_vehicles)
        , time_cost(otherSolution->time_cost)
        , heading_penalization_cost(otherSolution->heading_penalization_cost)
        , angle_penalization_cost(otherSolution->angle_penalization_cost)
        , time_window_cost(otherSolution->time_window_cost)
        , zone_time_penalization_cost(otherSolution->zone_time_penalization_cost)


{
}


TSPSolution::~TSPSolution()
{
}


float TSPSolution::compute_cost(const TSPInstance& instance)
{
    this->cost = 0;
    int prev = -1;
    int curr = -1;
    bool first_it;
    for(auto& vehicle : this->vehicles) {
        prev = -1;
        curr = -1;
        first_it = true;
        for(int i = 0; i < vehicle->customers.size(); i++) {
            curr = vehicle->customers[i]->id;
            this->cost += instance.service_time[curr];
            if(prev != -1) {
                this->cost += instance.time_matrix[prev][curr];
            }
            prev = curr;
        }
    }
    return this->cost;
}

float TSPSolution::compute_prob_cost(const TSPInstance& instance)
{
    std::unordered_map<std::string, int> * solution_zones_order = new std::unordered_map<std::string, int> ();
    float m_time_cost = 0;
    float m_time_window_cost = 0;
    float m_heading_penalization_cost = 0;
    float m_angle_penalization_cost = 0;
    float m_zone_time_penalization_cost = 0;
    float m_zone_distance_penalization_cost = 0;
    float m_normalized_time_cost = 0;
    float m_heading_antecedence_cost = 0;
    float m_heading_immediate_precedence_cost = 0;
    float m_heading_immediate_antecedence_cost = 0;
    float m_cluster_zones_cost = 0;
    float m_super_cluster_zone_cost = 0;

    this->cost = 0;

    int prev = -1;
    int curr = -1;
    string current_zone;

    int zone_order_counter = 0;
    bool first_it;
    for(auto& vehicle : this->vehicles) {
        prev = -1;
        curr = -1;

        first_it = true;
        for(int i = 0; i < vehicle->customers.size(); i++) {
            curr = vehicle->customers[i]->id;
            current_zone = vehicle->customers[i]->zone_id;


            m_time_cost += instance.service_time[curr];
            if(prev != -1) {
                m_time_cost +=  instance.time_matrix[prev][curr];
                unordered_map<string, int>::iterator it = solution_zones_order->find(current_zone);
                // key already present in the map
                if (it != solution_zones_order->end()) {
                }
                else {  // key not found
                    solution_zones_order->insert(make_pair(current_zone ,zone_order_counter));
                    zone_order_counter++;
                }
                auto current_customer = vehicle->customers[i];
                if (current_customer->time_window[0] > -1) {
                    if ((m_time_cost < current_customer->time_window[0]) ||
                        (m_time_cost > current_customer->time_window[1])) {
                        m_time_window_cost += 1;
                    }
                }
            }

            prev = curr;
        }
    }
    int x_ij = 0;
    int immediate_x_ij = 0;
    float p_ij = 0.0;
    float antecedence_ij = 0.0;
    float angle_ij = 0.0;
    float zone_distance_ij = 0.0;
    float zone_time_ij = 0.0;

    string zone_1;
    string zone_2;




    m_normalized_time_cost = (m_time_cost) / instance.worst_initial_cost;
    m_time_window_cost = m_time_window_cost / instance.n_cust;


    this->time_cost = m_normalized_time_cost;
    this->time_window_cost = m_time_window_cost;


    this->cost =  instance.k_1* m_normalized_time_cost + instance.k_2* this->heading_penalization_cost + instance.k_3* this->angle_penalization_cost
                  +  instance.k_4* m_time_window_cost + instance.k_5* this->zone_time_penalization_cost;



    delete solution_zones_order;
    return this->cost;
}

float TSPSolution::compute_zones_cost(const TSPInstance& instance)
{
    std::unordered_map<std::string, int> * solution_zones_order = new std::unordered_map<std::string, int> ();

    float m_heading_penalization_cost = 0;
    float m_angle_penalization_cost = 0;
    float m_zone_time_penalization_cost = 0;
    float m_zone_distance_penalization_cost = 0;
    float m_normalized_time_cost = 0;
    float m_heading_antecedence_cost = 0;
    float m_heading_immediate_precedence_cost = 0;
    float m_heading_immediate_antecedence_cost = 0;
    float m_cluster_zones_cost = 0;
    float m_super_cluster_zone_cost = 0;
    float m_time_window_cost = 0;


    this->cost = 0;

    int prev = -1;
    int curr = -1;
    string current_zone;

    int zone_order_counter = 0;
    bool first_it;

    for(auto& vehicle : this->vehicles) {
        prev = -1;
        curr = -1;

        first_it = true;
        for(int i = 0; i < vehicle->customers.size(); i++) {

            current_zone = vehicle->customers[i]->zone_id;

            unordered_map<string, int>::iterator it = solution_zones_order->find(current_zone);
            // key already present in the map
            if (it != solution_zones_order->end()) {
            }
            else {  // key not found
                solution_zones_order->insert(make_pair(current_zone ,zone_order_counter));
                zone_order_counter++;
            }

            //prev = curr;
        }
    }
    int x_ij = 0;
    int immediate_x_ij = 0;
    float p_ij = 0.0;
    float antecedence_ij = 0.0;
    float angle_ij = 0.0;

    float zone_time_ij = 0.0;

    string zone_1;
    string zone_2;


    for (int i=0; i < instance.historic_zones_id.size(); i++){
        zone_1 = instance.historic_zones_id[i];
        for (int j=0; j < instance.historic_zones_id.size(); j++){
            zone_2 = instance.historic_zones_id[j];
            if (solution_zones_order->at(zone_1) <= solution_zones_order->at(zone_2)){
                x_ij = 1;
            }
            else{
                x_ij = 0;
            }
            if (solution_zones_order->at(zone_1) - solution_zones_order->at(zone_2) == -1){
                immediate_x_ij = 1;
            }
            else{
                immediate_x_ij = 0;
            }
            p_ij = instance.probabilities_df.at(zone_1).at(zone_2);
            antecedence_ij = instance.probabilities_df.at(zone_2).at(zone_1);
            angle_ij = instance.angles_df.at(zone_1).at(zone_2);
            zone_time_ij = instance.zones_time_matrix_df.at(zone_1).at(zone_2);


            m_heading_antecedence_cost += ( ((x_ij * (antecedence_ij))  ))/ instance.n_cust;
            m_heading_immediate_precedence_cost +=  ((immediate_x_ij * (1-p_ij))  / (instance.historic_zones_id.size()+1));
            m_heading_immediate_antecedence_cost += ((immediate_x_ij * (antecedence_ij))  / (instance.historic_zones_id.size()+1));

            m_heading_penalization_cost += ( ((x_ij * (1-p_ij))  ))/ instance.n_cust;
            m_angle_penalization_cost +=   ((immediate_x_ij * (angle_ij))  / (instance.historic_zones_id.size()+1));
            m_zone_time_penalization_cost += ((immediate_x_ij * (zone_time_ij))  / (instance.historic_zones_id.size()+1));

        }
    }


    zone_2 = "nan";
    auto it = solution_zones_order->find(zone_2);
    if(it != solution_zones_order->end())
        it->second = zone_order_counter;

    for (int j=0; j < instance.historic_zones_id.size(); j++){
        zone_1 = instance.historic_zones_id[j];
        if (solution_zones_order->at(zone_1) <= solution_zones_order->at(zone_2)){
            x_ij = 1;
        }
        else{
            x_ij = 0;
        }
        if (solution_zones_order->at(zone_1) - solution_zones_order->at(zone_2) == -1){
            immediate_x_ij = 1;
        }
        else{
            immediate_x_ij = 0;
        }
        p_ij = instance.probabilities_df.at(zone_1).at(zone_2);
        antecedence_ij = instance.probabilities_df.at(zone_2).at(zone_1);
        angle_ij = instance.angles_df.at(zone_1).at(zone_2);
        zone_time_ij = instance.zones_time_matrix_df.at(zone_1).at(zone_2);


        m_heading_antecedence_cost += ( ((x_ij * (antecedence_ij))  ))/ instance.n_cust;
        m_heading_immediate_precedence_cost +=  ((immediate_x_ij * (1-p_ij))  / (instance.historic_zones_id.size()+1));
        m_heading_immediate_antecedence_cost += ((immediate_x_ij * (antecedence_ij))  / (instance.historic_zones_id.size()+1));

        m_heading_penalization_cost += ( ((x_ij * (1-p_ij))  ))/ instance.n_cust;
        m_angle_penalization_cost +=   ((immediate_x_ij * (angle_ij))  / (instance.historic_zones_id.size()+1));
        m_zone_time_penalization_cost += ((immediate_x_ij * (zone_time_ij))  / (instance.historic_zones_id.size()+1));
        
    }



    this->time_cost = m_normalized_time_cost;
    this->heading_penalization_cost = m_heading_penalization_cost;
    this->angle_penalization_cost = m_angle_penalization_cost;
    this->time_window_cost = 0;
    this->zone_time_penalization_cost = m_zone_time_penalization_cost;




    this->cost =  instance.k_1* m_normalized_time_cost + instance.k_2* this->heading_penalization_cost + instance.k_3* this->angle_penalization_cost
                  +  instance.k_4* m_time_window_cost + instance.k_5* this->zone_time_penalization_cost;

    delete solution_zones_order;
    return this->cost;
}


ostringstream TSPSolution::TSP_output_number_solutions() const
{

    vector<string> lines = {};

    string chain;
    vector<int> customers_ids;
    for(auto& vehicle : this->vehicles) {
        customers_ids = vector<int>();
        for(auto& customer : vehicle->customers) {
            // if(customer->id != 99)
            customers_ids.push_back(customer->id);
        }
        chain = ""; // revisar este metodo
        int cont = 0;
        for(auto& id : customers_ids) {
            if(cont == customers_ids.size() - 1) {
                chain += to_string(id);
            } else {
                chain += (boost::format("%1% ") % id).str();
            }
            cont++;
        }
        lines.push_back(chain);
    }

    ostringstream vts = ostringstream();

    // Convert all but the last element to avoid a trailing ","
    copy(lines.begin(), lines.end() - 1, ostream_iterator<string>( vts, "\n"));
    // Now add the last element with no delimiter
    vts << lines.back();

    return vts;
}

template < typename Type > std::string to_str (const Type & t)
{
    std::ostringstream os;
    os << t;
    return os.str ();
}

ostringstream TSPSolution::TSP_output_string_solutions(const TSPInstance& instance) const
{
    vector<string> lines = {};

    string chain;
    vector<string> customers_ids;
    for(auto& vehicle : this->vehicles) {
        customers_ids = vector<string>();
        for(auto& customer : vehicle->customers) {
            customers_ids.push_back(instance.customers_map.at(customer->id));
        }
        int cont = 0;
        for(auto& id : customers_ids) {
            if(cont == customers_ids.size() - 1) {
                chain += id;
            } else {
                chain += (boost::format("%1%, ") % id).str();
            }
            cont++;
        }
        lines.push_back(chain);

    }

    ostringstream vts = ostringstream();

    // Convert all but the last element to avoid a trailing ","
    copy(lines.begin(), lines.end() - 1, ostream_iterator<string>(vts, "\n"));
    // Now add the last element with no delimiter
    vts << lines.back();

    return vts;
}

ostringstream TSPSolution::TSP_output_string_local_optimas(const TSPInstance& instance) const
{

    vector<string> lines = {};
    string chain;
    string chain_2;
    vector<string> customers_ids;
    vector<string> zones_ids;
    set <string> zones_ids_set;
    for(auto& vehicle : this->vehicles) {
        customers_ids = vector<string>();
        zones_ids = vector<string>();
        for(auto& customer : vehicle->customers) {

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
    }
    lines.push_back(to_str(this->cost));
    lines.push_back(to_str(this->time_cost));
    lines.push_back(to_str(this->heading_penalization_cost));
    lines.push_back(to_str(this->angle_penalization_cost));
    lines.push_back(to_str(this->time_window_cost));
    lines.push_back(to_str(this->zone_time_penalization_cost));

    ostringstream vts = ostringstream();

    // Convert all but the last element to avoid a trailing ","
    copy(lines.begin(), lines.end() - 1, ostream_iterator<string>(vts, "\n"));
    // Now add the last element with no delimiter
    vts << lines.back();

    return vts;
}
/*
vector<string> store_output_info(const TSPInstance& instance, TSPSolution* current_solution){
    vector<string> lines = {};
    string chain;
    string chain_2;
    vector<string> customers_ids;
    vector<string> zones_ids;
    set <string> zones_ids_set;
    for(auto& vehicle : current_solution->vehicles) {
        customers_ids = vector<string>();
        zones_ids = vector<string>();
        for(auto& customer : vehicle->customers) {

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
    }
    lines.push_back(to_str(current_solution->cost));
    lines.push_back(to_str(current_solution->time_cost));
    lines.push_back(to_str(current_solution->heading_penalization_cost));
    lines.push_back(to_str(current_solution->angle_penalization_cost));
    lines.push_back(to_str(current_solution->time_window_cost));
    lines.push_back(to_str(current_solution->zone_time_penalization_cost));
    lines.push_back(to_str(current_solution->zone_distance_penalization_cost));

    return lines;
}
*/
ostringstream TSPSolution::TSP_output_string_zones(const TSPInstance& instance) const
{

    vector<string> lines = {};
    string chain;
    vector<string> zones_ids;
    set <string> zones_ids_set;
    for(auto& vehicle : this->vehicles) {
        zones_ids = vector<string>();
        for(auto& customer : vehicle->customers) {
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
        for(auto& id : zones_ids) {
            if(cont == zones_ids.size() - 1) {
                chain += id;
            } else {
                chain += (boost::format("%1%, ") % id).str();
            }
            cont++;
        }
        lines.push_back(chain);
    }

    ostringstream vts = ostringstream();

    // Convert all but the last element to avoid a trailing ","
    copy(lines.begin(), lines.end() - 1, ostream_iterator<string>(vts, "\n"));
    // Now add the last element with no delimiter
    vts << lines.back();

    return vts;
}
