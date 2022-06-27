#include "./neighborhood.h"
#include <algorithm>
#include <queue>
#include <unordered_map>

using namespace std;

TSPNeighborhood::TSPNeighborhood(bool first_improvement /*= false*/, const string& id /*= ""*/)
{
    this->first_improvement = first_improvement;
    this->id = id;
}

TSPNeighborhood::~TSPNeighborhood()
{
}

std::shared_ptr <TSPSolution> TSPNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
}


TSPSwapNeighborhood::TSPSwapNeighborhood()
{
}

TSPSwapNeighborhood::TSPSwapNeighborhood(bool first_improvement /*= false*/)
        : TSPNeighborhood(first_improvement, "A")
{
}

TSPSwapNeighborhood::~TSPSwapNeighborhood()
{
}

std::shared_ptr <TSPSolution> TSPSwapNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
    int n_vehicles = initial_solution->vehicles.size();

    float min_delta = 0.0;

    vector<int> best_move;
    best_move.clear();
    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    TSPVehicle vehicle_two(instance.n_cust, instance.Q);
    int initial_value_for;
    int min_initial_value_for;
    int min_final_value_for;
    int max_value_for;
    int final_value_for;

    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;
    TSPCustomer d;

    TSPCustomer f;
    TSPCustomer g;
    TSPCustomer h;
    // demands
    int route1_new_demand;
    int route2_new_demand;

    float delta = 0.0;


    for(int vehicle_one_id = 0; vehicle_one_id < n_vehicles; vehicle_one_id++) {
        vehicle_one = *initial_solution->vehicles[vehicle_one_id];
        for (int customer_one_id = 1; customer_one_id < vehicle_one.customers.size() - 1; customer_one_id++) {
            for (int vehicle_two_id = vehicle_one_id; vehicle_two_id < n_vehicles; vehicle_two_id++) {
                vehicle_two = *initial_solution->vehicles[vehicle_two_id];
                if (vehicle_one_id != vehicle_two_id) {
                    initial_value_for = 1;
                    continue; // With these line Swap is now only intraroute
                } else {
                    initial_value_for = customer_one_id + 1;


                    a = *vehicle_one.customers[customer_one_id - 1];
                    b = *vehicle_one.customers[customer_one_id];
                    c = *vehicle_one.customers[customer_one_id + 1];

                    initial_value_for = customer_one_id - instance.zones_map.at(b.zone_id) - 1;
                    min_initial_value_for = max(initial_value_for, 1);
                    final_value_for = customer_one_id + instance.zones_map.at(b.zone_id)  + 1;
                    max_value_for =  vehicle_two.customers.size() - 1;
                    min_final_value_for = min(final_value_for, max_value_for);

                    for (int customer_two_id = min_initial_value_for; customer_two_id < min_final_value_for; customer_two_id++) {
                            /*
                            route1 =  [A B C D]
                            route2 =  [E F G H]
                            Swap(B, G) leaving us with:
                            new_route1 = [A G C D]
                            new_route2 = [E F B H]
                            */

                            f = *vehicle_two.customers[customer_two_id - 1];
                            g = *vehicle_two.customers[customer_two_id];
                            h = *vehicle_two.customers[customer_two_id + 1];



                            if ((b.id == g.id) ) {
                                continue;
                            }

                        if ((by_zones == true) && (b.zone_id != g.zone_id) ){
                            continue;
                        }

                            if ((vehicle_one_id == vehicle_two_id) && (customer_two_id - customer_one_id  == 1)) {
                                d = *vehicle_one.customers[customer_one_id + 2];
                                delta = +instance.time_matrix[a.id][g.id] + instance.time_matrix[b.id][d.id] +instance.time_matrix[g.id][b.id]
                                        -instance.time_matrix[b.id][g.id] - instance.time_matrix[a.id][b.id] - instance.time_matrix[g.id][d.id];
                            }
                            else if (abs(customer_two_id - customer_one_id)  > 1){
                                delta = +instance.time_matrix[a.id][g.id] + instance.time_matrix[g.id][c.id] + instance.time_matrix[f.id][b.id] + instance.time_matrix[b.id][h.id]
                                        - instance.time_matrix[a.id][b.id] - instance.time_matrix[b.id][c.id] - instance.time_matrix[f.id][g.id] - instance.time_matrix[g.id][h.id];
                            }

                            // Store the current best possible move
                            if (delta < min_delta) {
                                min_delta = delta;
                                best_move.clear();
                                best_move.push_back(vehicle_one_id);
                                best_move.push_back(vehicle_two_id);
                                best_move.push_back(customer_one_id);
                                best_move.push_back(customer_two_id);
                                if (this->first_improvement) {
                                    initial_solution = apply_best_move(initial_solution, best_move, instance);
                                    initial_solution->cost = initial_solution->compute_prob_cost(instance);
                                    return initial_solution;
                                }
                            }
                        }
                    }
                }
            }
        }

    if(min_delta < 0) {

        initial_solution = apply_best_move(initial_solution, best_move, instance);
        initial_solution->cost = initial_solution->compute_prob_cost(instance);
    }
    return initial_solution;

}

/**
 * Apply the Swap (exchange) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_two_id}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
std::shared_ptr <TSPSolution> TSPSwapNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{
    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int vehicle_one_id = best_move[0];
        int vehicle_two_id = best_move[1];
        int customer_one_id = best_move[2];
        int customer_two_id = best_move[3];

        if(vehicle_one_id == vehicle_two_id) {
            shared_ptr<TSPCustomer> customer_one = local_optima->vehicles[vehicle_one_id]->customers[customer_one_id];
            shared_ptr<TSPCustomer> customer_two = local_optima->vehicles[vehicle_two_id]->customers[customer_two_id];
            if(customer_two_id > customer_one_id) {
                if(customer_two_id == customer_one_id + 1) {
                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);
                } else {
                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);

                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id + 1, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);
                }
            } else {
                if(customer_one_id == customer_two_id + 1) {
                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);
                } else {
                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);

                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id + 1, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);
                }
            }

        }
        return local_optima;
    } else {
        return initial_solution;
    }
}


TSPRelocateNeighborhood::TSPRelocateNeighborhood()
{
}

TSPRelocateNeighborhood::TSPRelocateNeighborhood(bool first_improvement /*= false*/)
        : TSPNeighborhood(first_improvement, "B")
{
}

TSPRelocateNeighborhood::~TSPRelocateNeighborhood()
{
}

std::shared_ptr <TSPSolution> TSPRelocateNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
    int n_vehicles = initial_solution->vehicles.size();

    float min_delta = 0.0;
    vector<int> best_move;
    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    TSPVehicle vehicle_two(instance.n_cust, instance.Q);
    // customers
    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;

    TSPCustomer f;
    TSPCustomer g;
    // demands
    int route1_new_demand;
    int route2_new_demand;

    float delta = 0.0;

    for(int vehicle_one_id = 0; vehicle_one_id < n_vehicles; vehicle_one_id++) {
        vehicle_one = *initial_solution->vehicles[vehicle_one_id];
        for(int customer_one_id = 1; customer_one_id < vehicle_one.customers.size() - 1; customer_one_id++) {
            for(int vehicle_two_id = 0; vehicle_two_id < n_vehicles; vehicle_two_id++) {
                if(vehicle_two_id == vehicle_one_id){
                    vehicle_two = *initial_solution->vehicles[vehicle_two_id];
                    for(int customer_new_pos = 1; customer_new_pos < vehicle_two.customers.size() - 1; customer_new_pos++) {
                        /*
                            route1 =  [A B C D]
                            route2 =  [E F G H]
                            Relocate(B, pos(G)) leaving us with:
                            new_route1 = [A C D]
                            new_route2 = [E F B G H]
                        */

                        a = *vehicle_one.customers[customer_one_id - 1];
                        b = *vehicle_one.customers[customer_one_id];
                        c = *vehicle_one.customers[customer_one_id + 1];

                        f = *vehicle_two.customers[customer_new_pos - 1];
                        g = *vehicle_two.customers[customer_new_pos];

                        if(((vehicle_one_id == vehicle_two_id) && (c.id == g.id)) ||
                           ((vehicle_one_id == vehicle_two_id) && (b.id == g.id)) ) {
                            continue;
                        }

                        if ((by_zones == true) && (b.zone_id != g.zone_id) && (b.zone_id != f.zone_id) ){
                            continue;
                        }

                        delta = instance.time_matrix[a.id][c.id] + instance.time_matrix[f.id][b.id] + instance.time_matrix[b.id][g.id] \
                            - instance.time_matrix[a.id][b.id] - instance.time_matrix[b.id][c.id] - instance.time_matrix[f.id][g.id];

                        // Store the current best possible move
                        if(delta < min_delta) {
                            min_delta = delta;
                            best_move.clear();
                            best_move = { vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos };
                            if(this->first_improvement) {
                                initial_solution = apply_best_move(initial_solution, best_move, instance);
                                initial_solution->cost = initial_solution->compute_prob_cost(instance);
                                return initial_solution;
                            }
                        }
                    }
                }
            }
        }
    }
    if(min_delta < 0) {
        initial_solution = apply_best_move(initial_solution, best_move, instance);
        initial_solution->cost = initial_solution->compute_prob_cost(instance);
    }
    return initial_solution;
}

/**
 * Apply the relocate (insertion) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
std::shared_ptr <TSPSolution> TSPRelocateNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{
    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int vehicle_one_id = best_move[0];
        int vehicle_two_id = best_move[1];
        int customer_one_id = best_move[2];
        int customer_new_pos = best_move[3];

        shared_ptr<TSPCustomer> customer_one = local_optima->vehicles[vehicle_one_id]->customers[customer_one_id];

        if(vehicle_one_id == vehicle_two_id) {
            if(customer_new_pos > customer_one_id) {
                customer_new_pos--;
            }
        }
        local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id, instance);
        local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_new_pos, instance);
        local_optima->vehicles[vehicle_two_id]->customers[customer_new_pos]->vehicle_route = vehicle_two_id;

        return local_optima;
    } else {
        return initial_solution;
    }
}


TSP_ProbZoneSwapNeighborhood::TSP_ProbZoneSwapNeighborhood()
{
}

TSP_ProbZoneSwapNeighborhood::TSP_ProbZoneSwapNeighborhood(bool first_improvement /*= false*/)
        : TSPNeighborhood(first_improvement, "B")
{
}

TSP_ProbZoneSwapNeighborhood::~TSP_ProbZoneSwapNeighborhood()
{
}

std::shared_ptr <TSPSolution> TSP_ProbZoneSwapNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{

    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    TSPVehicle vehicle_two(instance.n_cust, instance.Q);

    int initial_value_for;
    int min_initial_value_for;
    int min_final_value_for;
    int max_value_for;
    int final_value_for;

    int n_vehicles = initial_solution->vehicles.size();
    std::shared_ptr <TSPVehicle> temporal_vehicle = make_shared <TSPVehicle> (instance.n_cust, instance.Q);
    std::shared_ptr <TSPSolution> temporal_solution = make_shared <TSPSolution> (instance, n_vehicles);
    float min_delta = 0.0;
    vector<int> best_move;
    vector<int> move;

    // customers
    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;
    TSPCustomer d;

    TSPCustomer e;
    TSPCustomer f;
    TSPCustomer g;
    TSPCustomer h;

    TSPCustomer previous_c;
    TSPCustomer current_c;
    TSPCustomer next_c;

    unordered_map<string, int> zones_map (instance.zones_map);

    vector<vector<float>> dist = instance.time_matrix;
    // delta
    float delta = 0.0;
    int vehicle_one_id = 0;
    int zone_1_start = 0;
    int zone_1_end = 0;
    int zone_2_start = 0;
    int zone_2_end = 0;

    for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
        temporal_vehicle->customers.push_back(initial_solution->vehicles[0]->customers[i]);
    }
    temporal_solution->cost = initial_solution->cost;
    temporal_solution->vehicles[0] = temporal_vehicle;
    float best_cost = initial_solution->cost;
    vehicle_one = *initial_solution->vehicles[vehicle_one_id];

    for(int vehicle_one_id = 0; vehicle_one_id < n_vehicles; vehicle_one_id++) {
        vehicle_one = *initial_solution->vehicles[vehicle_one_id];
        for (int customer_one_id = 1; customer_one_id < vehicle_one.customers.size() - 1; customer_one_id++) {
            for (int vehicle_two_id = vehicle_one_id; vehicle_two_id < n_vehicles; vehicle_two_id++) {
                vehicle_two = *initial_solution->vehicles[vehicle_two_id];
                for (int customer_two_id = 1; customer_two_id < vehicle_two.customers.size() - 1; customer_two_id++) {
                    initial_value_for = customer_one_id + 1;


                    a = *vehicle_one.customers[customer_one_id - 1];
                    b = *vehicle_one.customers[customer_one_id];
                    c = *vehicle_one.customers[customer_one_id + 1];
                        /*
                        route1 =  [A B C D]
                        route2 =  [E F G H]
                        Swap(B, G) leaving us with:
                        new_route1 = [A G C D]
                        new_route2 = [E F B H]
                        */

                        f = *vehicle_two.customers[customer_two_id - 1];
                        g = *vehicle_two.customers[customer_two_id];
                        h = *vehicle_two.customers[customer_two_id + 1];



                        if ((b.id == g.id) ) {
                            continue;
                        }


                        move.clear();
                        move = {vehicle_one_id, vehicle_two_id, customer_one_id, customer_two_id};
                        temporal_solution = apply_best_move(temporal_solution, move, instance);

                        //temporal_solution->cost = temporal_solution->compute_prob_cost(instance);
                        temporal_solution->cost = temporal_solution->compute_zones_cost(instance);
                        // Store the current best possible move
                        if (temporal_solution->cost < best_cost){
                            best_cost = temporal_solution->cost;
                            best_move.clear();
                            best_move = {vehicle_one_id, vehicle_two_id, customer_one_id, customer_two_id};
                            if (this->first_improvement == true){
                                /*
                                initial_solution->vehicles[0]->customers.clear();

                                for (int i=0; i < temporal_solution->vehicles[0]->customers.size(); i++){
                                    initial_solution->vehicles[0]->customers.push_back(temporal_solution->vehicles[0]->customers[i]);
                                }
                                initial_solution->cost = best_cost;
                                return initial_solution;
                                 */
                                return temporal_solution;
                            }
                        }
                        temporal_solution->vehicles[0]->customers.clear();
                        temporal_solution->vehicles[0]->customers.reserve(initial_solution->vehicles[0]->customers.size());
                        for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
                            temporal_solution->vehicles[0]->customers.push_back(initial_solution->vehicles[0]->customers[i]);
                        }
                }
            }
        }
    }

    if (this->first_improvement == false) {
        if (best_cost < initial_solution->cost) {
            initial_solution = apply_best_move(initial_solution, best_move, instance);
            //initial_solution->cost = initial_solution->compute_prob_cost(instance);
            initial_solution->cost = initial_solution->compute_zones_cost(instance);
        }
    }
    return initial_solution;
}


/**
 * Apply the Swap (exchange) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_two_id}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
std::shared_ptr <TSPSolution> TSP_ProbZoneSwapNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{
    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int vehicle_one_id = best_move[0];
        int vehicle_two_id = best_move[1];
        int customer_one_id = best_move[2];
        int customer_two_id = best_move[3];

        if(vehicle_one_id == vehicle_two_id) {
            shared_ptr<TSPCustomer> customer_one = local_optima->vehicles[vehicle_one_id]->customers[customer_one_id];
            shared_ptr<TSPCustomer> customer_two = local_optima->vehicles[vehicle_two_id]->customers[customer_two_id];
            if(customer_two_id > customer_one_id) {
                if(customer_two_id == customer_one_id + 1) {
                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);
                } else {
                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);

                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id + 1, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);
                }
            } else {
                if(customer_one_id == customer_two_id + 1) {
                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);
                } else {
                    local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_two_id, instance);
                    local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id + 1, instance);

                    local_optima->vehicles[vehicle_one_id]->insert_customer(customer_two, customer_one_id + 1, instance);
                    local_optima->vehicles[vehicle_two_id]->remove_customer(customer_two_id + 1, instance);
                }
            }

        }
        return local_optima;
    } else {
        return initial_solution;
    }
}
/*
std::shared_ptr <TSPSolution> TSP_ProbZoneSwapNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
    int n_vehicles = initial_solution->vehicles.size();
    std::shared_ptr <TSPVehicle> temporal_vehicle = make_shared <TSPVehicle> (instance.n_cust, instance.Q);
    std::shared_ptr <TSPSolution> temporal_solution = make_shared <TSPSolution> (instance, n_vehicles);
    float min_delta = 0.0;
    vector<int> best_move;
    vector<int> move;
    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    // customers
    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;
    TSPCustomer d;

    TSPCustomer e;
    TSPCustomer f;
    TSPCustomer g;
    TSPCustomer h;

    TSPCustomer previous_c;
    TSPCustomer current_c;
    TSPCustomer next_c;

    unordered_map<string, int> zones_map (instance.zones_map);

    vector<vector<float>> dist = instance.time_matrix;
    // delta
    float delta = 0.0;
    int vehicle_one_id = 0;
    int zone_1_start = 0;
    int zone_1_end = 0;
    int zone_2_start = 0;
    int zone_2_end = 0;

    for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
        temporal_vehicle->customers.push_back(initial_solution->vehicles[0]->customers[i]);
    }
    temporal_solution->cost = initial_solution->cost;
    temporal_solution->vehicles[0] = temporal_vehicle;
    float best_cost = initial_solution->cost;
    vehicle_one = *initial_solution->vehicles[vehicle_one_id];

    for (auto zone_1 : zones_map){
        if (zone_1.first == "nan") {
            continue;
        }
        for(int customer_one_start = 1; customer_one_start < vehicle_one.customers.size() - 1; customer_one_start++) {
            if (vehicle_one.customers[customer_one_start]->zone_id == zone_1.first) {
                a = *vehicle_one.customers[customer_one_start - 1];
                b = *vehicle_one.customers[customer_one_start];
                zone_1_start = customer_one_start;
                break;
            }
        }
        for (int customer_one_end = zone_1_start; customer_one_end < vehicle_one.customers.size() - 1; customer_one_end++) {
            current_c = *vehicle_one.customers[customer_one_end];
            next_c = *vehicle_one.customers[customer_one_end + 1];
            if (current_c.zone_id != next_c.zone_id) {
                c = *vehicle_one.customers[customer_one_end];
                d = *vehicle_one.customers[customer_one_end + 1];
                zone_1_end = customer_one_end;
                break;
            }
        }

        for (auto zone_2 : zones_map) {

            if ((zone_2.first == zone_1.first) || (zone_2.first == "nan")) {
                continue;
            }
            for (int customer_one_start = zone_1_end + 1;
                 customer_one_start < vehicle_one.customers.size() - 1; customer_one_start++) {
                if (vehicle_one.customers[customer_one_start]->zone_id == zone_2.first) {

                    e = *vehicle_one.customers[customer_one_start - 1];
                    f = *vehicle_one.customers[customer_one_start];
                    zone_2_start = customer_one_start;

                    break;
                }
            }
            for (int customer_one_end = zone_2_start;
                 customer_one_end < vehicle_one.customers.size() - 1; customer_one_end++) {
                current_c = *vehicle_one.customers[customer_one_end];
                next_c = *vehicle_one.customers[customer_one_end + 1];
                if (current_c.zone_id != next_c.zone_id) {

                    g = *vehicle_one.customers[customer_one_end];
                    h = *vehicle_one.customers[customer_one_end + 1];
                    zone_2_end = customer_one_end;

                    break;
                }
            }

            if (zone_1_start < zone_2_start) {
                    move.clear();
                    move = {zone_1_start, zone_1_end, zone_2_start, zone_2_end, zone_1.second, zone_2.second};
                temporal_solution = apply_best_move(temporal_solution, move, instance);

                //temporal_solution->cost = temporal_solution->compute_prob_cost(instance);
                temporal_solution->cost = temporal_solution->compute_zones_cost(instance);

                if (temporal_solution->cost < best_cost){
                    best_cost = temporal_solution->cost;
                    best_move.clear();

                    best_move = {zone_1_start, zone_1_end, zone_2_start, zone_2_end, zone_1.second, zone_2.second};
                    if (this->first_improvement == true){
                        initial_solution->vehicles[0]->customers.clear();

                        for (int i=0; i < temporal_solution->vehicles[0]->customers.size(); i++){
                            initial_solution->vehicles[0]->customers.push_back(temporal_solution->vehicles[0]->customers[i]);
                        }
                        initial_solution->cost = best_cost;
                        return initial_solution;
                    }
                }
                temporal_solution->vehicles[0]->customers.clear();
                temporal_solution->vehicles[0]->customers.reserve(initial_solution->vehicles[0]->customers.size());
                for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
                    temporal_solution->vehicles[0]->customers.push_back(initial_solution->vehicles[0]->customers[i]);
                }

            }
        }
    }


if (this->first_improvement == false) {
    if (best_cost < initial_solution->cost) {
        initial_solution = apply_best_move(initial_solution, best_move, instance);
        //initial_solution->cost = initial_solution->compute_prob_cost(instance);
        initial_solution->cost = initial_solution->compute_zones_cost(instance);
    }
}

    return initial_solution;
}
*/
/**
 * Apply the relocate (insertion) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
/*
std::shared_ptr <TSPSolution> TSP_ProbZoneSwapNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{

    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int zone_1_start = best_move[0];
        int zone_1_end = best_move[1];
        int zone_2_start = best_move[2];
        int zone_2_end = best_move[3];

        TSPVehicle& vehicle = *local_optima->vehicles[0];
        vector<shared_ptr<TSPCustomer>> aux_vector_1 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_2 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_3 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_4 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_5 = {};
        vector<shared_ptr<TSPCustomer>> complete_vehicle = {};
        complete_vehicle.reserve(vehicle.customers.size());

        if (zone_1_start < zone_2_start) {
            aux_vector_1.reserve(zone_1_start);
            aux_vector_2.reserve(zone_1_end - zone_1_start);
            aux_vector_3.reserve(zone_2_start - zone_1_end);
            aux_vector_4.reserve(zone_2_end - zone_2_start);
            aux_vector_5.reserve(vehicle.customers.size() - zone_2_end);

            for(int h = 0; h < zone_1_start; h++)
                aux_vector_1.push_back(vehicle.customers[h]);
            for(int h = zone_1_start; h < zone_1_end+1; h++)
                aux_vector_2.push_back(vehicle.customers[h]);
            for(int h = zone_1_end+1; h < zone_2_start; h++)
                aux_vector_3.push_back(vehicle.customers[h]);
            for(int h = zone_2_start; h < zone_2_end+1; h++)
                aux_vector_4.push_back(vehicle.customers[h]);
            for(int h = zone_2_end+1; h < vehicle.customers.size(); h++)
                aux_vector_5.push_back(vehicle.customers[h]);

            for (int h=0; h < aux_vector_1.size(); h++)
                complete_vehicle.push_back(aux_vector_1[h]);
            for (int h=0; h < aux_vector_4.size(); h++)
                complete_vehicle.push_back(aux_vector_4[h]);
            for (int h=0; h < aux_vector_3.size(); h++)
                complete_vehicle.push_back(aux_vector_3[h]);
            for (int h=0; h < aux_vector_2.size(); h++)
                complete_vehicle.push_back(aux_vector_2[h]);
            for (int h=0; h < aux_vector_5.size(); h++)
                complete_vehicle.push_back(aux_vector_5[h]);
        }

        vehicle.customers = complete_vehicle;

        return local_optima;
    } else {
        return initial_solution;
    }
}
 */

TSP_ProbZoneRelocateNeighborhood::TSP_ProbZoneRelocateNeighborhood()
{
}

TSP_ProbZoneRelocateNeighborhood::TSP_ProbZoneRelocateNeighborhood(bool first_improvement /*= false*/)
        : TSPNeighborhood(first_improvement, "B")
{
}

TSP_ProbZoneRelocateNeighborhood::~TSP_ProbZoneRelocateNeighborhood()
{
}

std::shared_ptr <TSPSolution> TSP_ProbZoneRelocateNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    TSPVehicle vehicle_two(instance.n_cust, instance.Q);
    // customers
    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;

    TSPCustomer f;
    TSPCustomer g;

    float delta = 0.0;


    int n_vehicles = initial_solution->vehicles.size();
    std::shared_ptr <TSPVehicle> temporal_vehicle = make_shared <TSPVehicle> (instance.n_cust, instance.Q);
    std::shared_ptr <TSPSolution> temporal_solution = make_shared <TSPSolution> (instance, n_vehicles);
    float min_delta = 0.0;
    vector<int> best_move;
    vector<int> move;

    for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
        temporal_vehicle->customers.push_back(initial_solution->vehicles[0]->customers[i]);
    }
    temporal_solution->cost = initial_solution->cost;
    temporal_solution->vehicles[0] = temporal_vehicle;
    float best_cost = initial_solution->cost;


    for(int vehicle_one_id = 0; vehicle_one_id < n_vehicles; vehicle_one_id++) {
        vehicle_one = *initial_solution->vehicles[vehicle_one_id];
        for(int customer_one_id = 1; customer_one_id < vehicle_one.customers.size()-1; customer_one_id++) {
            for(int vehicle_two_id = 0; vehicle_two_id < n_vehicles; vehicle_two_id++) {
                if(vehicle_two_id == vehicle_one_id){
                    vehicle_two = *initial_solution->vehicles[vehicle_two_id];
                    for(int customer_new_pos = 1; customer_new_pos < vehicle_two.customers.size() - 1; customer_new_pos++) {
                        /*
                            route1 =  [A B C D]
                            route2 =  [E F G H]
                            Relocate(B, pos(G)) leaving us with:
                            new_route1 = [A C D]
                            new_route2 = [E F B G H]
                        */

                        a = *vehicle_one.customers[customer_one_id - 1];
                        b = *vehicle_one.customers[customer_one_id];
                        c = *vehicle_one.customers[customer_one_id + 1];

                        f = *vehicle_two.customers[customer_new_pos - 1];
                        g = *vehicle_two.customers[customer_new_pos];

                        if(((vehicle_one_id == vehicle_two_id) && (c.id == g.id)) ||
                           ((vehicle_one_id == vehicle_two_id) && (b.id == g.id)) ) {
                            continue;
                        }

                            move.clear();
                            move = {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos};

                            temporal_solution = apply_best_move(temporal_solution, move, instance);

                            //temporal_solution->cost = temporal_solution->compute_prob_cost(instance);
                            temporal_solution->cost = temporal_solution->compute_zones_cost(instance);

                            // Store the current best possible move
                            if (temporal_solution->cost < best_cost){

                                best_move.clear();
                                best_move = {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos};
                                best_cost = temporal_solution->cost;
                                if (this->first_improvement == true){
                                    /*
                                    initial_solution->vehicles[0]->customers.clear();

                                    for (int i=0; i < temporal_solution->vehicles[0]->customers.size(); i++){
                                        initial_solution->vehicles[0]->customers.push_back(temporal_solution->vehicles[0]->customers[i]);
                                    }
                                    initial_solution->cost = best_cost;
                                    return initial_solution;
                                     */
                                    return temporal_solution;
                                }

                            }
                            temporal_solution->vehicles[0]->customers.clear();
                            temporal_solution->vehicles[0]->customers.reserve(initial_solution->vehicles[0]->customers.size());
                            for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
                                temporal_solution->vehicles[0]->customers.push_back(initial_solution->vehicles[0]->customers[i]);
                            }
                    }
                }
            }
        }
    }
    if (this->first_improvement == false) {
        if (best_cost < initial_solution->cost) {
            initial_solution = apply_best_move(initial_solution, best_move, instance);
            //initial_solution->cost = initial_solution->compute_prob_cost(instance);
            initial_solution->cost = initial_solution->compute_zones_cost(instance);
        }
    }
    return initial_solution;
}
/**
 * Apply the relocate (insertion) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
std::shared_ptr <TSPSolution> TSP_ProbZoneRelocateNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{
    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int vehicle_one_id = best_move[0];
        int vehicle_two_id = best_move[1];
        int customer_one_id = best_move[2];
        int customer_new_pos = best_move[3];

        shared_ptr<TSPCustomer> customer_one = local_optima->vehicles[vehicle_one_id]->customers[customer_one_id];

        if(vehicle_one_id == vehicle_two_id) {
            if(customer_new_pos > customer_one_id) {
                customer_new_pos--;
            }
        }
        local_optima->vehicles[vehicle_one_id]->remove_customer(customer_one_id, instance);
        local_optima->vehicles[vehicle_two_id]->insert_customer(customer_one, customer_new_pos, instance);
        local_optima->vehicles[vehicle_two_id]->customers[customer_new_pos]->vehicle_route = vehicle_two_id;

        return local_optima;
    } else {
        return initial_solution;
    }
}

/*
std::shared_ptr <TSPSolution> TSP_ProbZoneRelocateNeighborhood::search_neighbors(std::shared_ptr <TSPSolution> initial_solution,TSPInstance& instance, bool  by_zones)
{
    int n_vehicles = initial_solution->vehicles.size();
    std::shared_ptr <TSPVehicle> temporal_vehicle = make_shared <TSPVehicle> (instance.n_cust, instance.Q);
    std::shared_ptr <TSPSolution> temporal_solution = make_shared <TSPSolution> (instance, n_vehicles);
    float min_delta = 0.0;
    vector<int> best_move;
    vector<int> move;
    TSPVehicle vehicle_one(instance.n_cust, instance.Q);
    // customers
    TSPCustomer a;
    TSPCustomer b;
    TSPCustomer c;
    TSPCustomer d;

    TSPCustomer e;
    TSPCustomer f;
    TSPCustomer g;
    TSPCustomer h;

    TSPCustomer previous_c;
    TSPCustomer current_c;
    TSPCustomer next_c;

    unordered_map<string, int> zones_map (instance.zones_map);

    vector<vector<float>> dist = instance.time_matrix;

    float delta = 0.0;
    int vehicle_one_id = 0;
    int zone_1_start = 0;
    int zone_1_end = 0;
    int zone_2_start = 0;
    int zone_2_end = 0;

    for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
        temporal_vehicle->customers.push_back(initial_solution->vehicles[0]->customers[i]);
    }
    temporal_solution->cost = initial_solution->cost;
    temporal_solution->vehicles[0] = temporal_vehicle;
    float best_cost = initial_solution->cost;

    vehicle_one = *initial_solution->vehicles[vehicle_one_id];
    for (auto zone_1 : zones_map){
        if (zone_1.first == "nan"){
            continue;
        }
        for(int customer_one_start = 1; customer_one_start < vehicle_one.customers.size() - 1; customer_one_start++) {
            if (vehicle_one.customers[customer_one_start]->zone_id == zone_1.first) {
                a = *vehicle_one.customers[customer_one_start - 1];
                b = *vehicle_one.customers[customer_one_start];
                zone_1_start = customer_one_start;
                break;
            }
        }
        for (int customer_one_end = zone_1_start; customer_one_end < vehicle_one.customers.size() - 1; customer_one_end++) {
            current_c = *vehicle_one.customers[customer_one_end];
            next_c = *vehicle_one.customers[customer_one_end + 1];
            if (current_c.zone_id != next_c.zone_id) {
                c = *vehicle_one.customers[customer_one_end];
                d = *vehicle_one.customers[customer_one_end + 1];
                zone_1_end = customer_one_end;
                break;
            }
        }
        for (auto zone_2 : zones_map) {
            if ((zone_2.first == zone_1.first) || (zone_2.first == "nan")) {
                continue;
            }
            for (int customer_one_start = zone_1_end + 1;
                 customer_one_start < vehicle_one.customers.size() - 1; customer_one_start++) {
                if (vehicle_one.customers[customer_one_start]->zone_id == zone_2.first) {
                    e = *vehicle_one.customers[customer_one_start - 1];
                    f = *vehicle_one.customers[customer_one_start];
                    zone_2_start = customer_one_start;
                    break;
                }
            }
            for (int customer_one_end = zone_2_start;
                 customer_one_end < vehicle_one.customers.size() - 1; customer_one_end++) {
                current_c = *vehicle_one.customers[customer_one_end];
                next_c = *vehicle_one.customers[customer_one_end + 1];
                if (current_c.zone_id != next_c.zone_id) {
                    g = *vehicle_one.customers[customer_one_end];
                    h = *vehicle_one.customers[customer_one_end + 1];
                    zone_2_end = customer_one_end;
                    break;
                }
            }

            if (zone_1_start < zone_2_start) {

                    move.clear();
                    move = {zone_1_start, zone_1_end, zone_2_start, zone_2_end, zone_1.second, zone_2.second};

                temporal_solution = apply_best_move(temporal_solution, move, instance);

                //temporal_solution->cost = temporal_solution->compute_prob_cost(instance);
                temporal_solution->cost = temporal_solution->compute_zones_cost(instance);

                if (temporal_solution->cost < best_cost){

                    best_move.clear();
                    best_move = {zone_1_start, zone_1_end, zone_2_start, zone_2_end, zone_1.second, zone_2.second};
                    best_cost = temporal_solution->cost;
                    if (this->first_improvement == true){
                        initial_solution->vehicles[0]->customers.clear();

                        for (int i=0; i < temporal_solution->vehicles[0]->customers.size(); i++){
                            initial_solution->vehicles[0]->customers.push_back(temporal_solution->vehicles[0]->customers[i]);
                        }
                        initial_solution->cost = best_cost;
                        return initial_solution;
                    }

                }
                temporal_solution->vehicles[0]->customers.clear();
                temporal_solution->vehicles[0]->customers.reserve(initial_solution->vehicles[0]->customers.size());
                for (int i=0; i < initial_solution->vehicles[0]->customers.size(); i++){
                    temporal_solution->vehicles[0]->customers.push_back(initial_solution->vehicles[0]->customers[i]);
                }

            }
        }
    }

if (this->first_improvement == false) {
    if (best_cost < initial_solution->cost) {
        initial_solution = apply_best_move(initial_solution, best_move, instance);
        //initial_solution->cost = initial_solution->compute_prob_cost(instance);
        initial_solution->cost = initial_solution->compute_zones_cost(instance);
    }
}
    return initial_solution;
}
*/
/**
 * Apply the relocate (insertion) to the best possible move in the neighborhood
 * @param initial_solution Current solution of the VRP
 * @param best_move {vehicle_one_id, vehicle_two_id, customer_one_id, customer_new_pos}
 *
 * @return local_optima: New VRP solution corresponding to the local optima of this neighborhood
 *          if there's no local optima, returns initial_solution instead
 */
 /*
std::shared_ptr <TSPSolution> TSP_ProbZoneRelocateNeighborhood::apply_best_move(std::shared_ptr <TSPSolution> initial_solution, vector<int>& best_move,TSPInstance& instance)
{

    if(best_move.size() != 0) {
        std::shared_ptr <TSPSolution> local_optima = initial_solution;
        int zone_1_start = best_move[0];
        int zone_1_end = best_move[1];
        int zone_2_start = best_move[2];
        int zone_2_end = best_move[3];

        TSPVehicle& vehicle = *local_optima->vehicles[0];
        vector<shared_ptr<TSPCustomer>> aux_vector_1 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_2 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_3 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_4 = {};
        vector<shared_ptr<TSPCustomer>> aux_vector_5 = {};
        vector<shared_ptr<TSPCustomer>> complete_vehicle = {};
        complete_vehicle.reserve(vehicle.customers.size());


        if (zone_1_start < zone_2_start) {
            aux_vector_1.reserve(zone_1_start);
            aux_vector_2.reserve(zone_1_end - zone_1_start);
            aux_vector_3.reserve(zone_2_start - zone_1_end);
            aux_vector_4.reserve(zone_2_end - zone_2_start);
            aux_vector_5.reserve(vehicle.customers.size() - zone_2_end);

            for(int h = 0; h < zone_1_start; h++)
                aux_vector_1.push_back(vehicle.customers[h]);
            for(int h = zone_1_start; h < zone_1_end+1; h++)
                aux_vector_2.push_back(vehicle.customers[h]);
            for(int h = zone_1_end+1; h < zone_2_start; h++)
                aux_vector_3.push_back(vehicle.customers[h]);
            for(int h = zone_2_start; h < zone_2_end+1; h++)
                aux_vector_4.push_back(vehicle.customers[h]);
            for(int h = zone_2_end+1; h < vehicle.customers.size(); h++)
                aux_vector_5.push_back(vehicle.customers[h]);

            for (int h=0; h < aux_vector_1.size(); h++)
                complete_vehicle.push_back(aux_vector_1[h]);
            for (int h=0; h < aux_vector_3.size(); h++)
                complete_vehicle.push_back(aux_vector_3[h]);
            for (int h=0; h < aux_vector_2.size(); h++)
                complete_vehicle.push_back(aux_vector_2[h]);
            for (int h=0; h < aux_vector_4.size(); h++)
                complete_vehicle.push_back(aux_vector_4[h]);
            for (int h=0; h < aux_vector_5.size(); h++)
                complete_vehicle.push_back(aux_vector_5[h]);
        }
        vehicle.customers = complete_vehicle;

        return local_optima;
    } else {
        return initial_solution;
    }
}
  */