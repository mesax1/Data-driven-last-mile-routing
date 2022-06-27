#include "./giant_tour_creation.h"
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <string>

using namespace std;

//STARTS TSP with TW methods

TSPGiantTour::TSPGiantTour()
        : giant_tour_solution(make_shared <TSPSolution>())
{
}

TSPGiantTour::TSPGiantTour(int seed)
        : giant_tour_solution(make_shared <TSPSolution>())
{
    srand(seed);
}


TSPGiantTour::~TSPGiantTour()
{
}

std::shared_ptr<TSPSolution> TSPGiantTour::run_fixed_zones(TSPInstance& instance,  std::shared_ptr<TSPSolution> zones_solution, int randomization)
{
}

std::shared_ptr<TSPSolution> TSPGiantTour::run(TSPInstance& instance, int randomization /*= 3*/)
{
}




TSP_GiantTour_FN::TSP_GiantTour_FN()
{
}

TSP_GiantTour_FN::TSP_GiantTour_FN(int seed)
        : TSPGiantTour(seed)
{
}

TSP_GiantTour_FN::~TSP_GiantTour_FN()
{
}

std::shared_ptr<TSPSolution> TSP_GiantTour_FN::run_fixed_zones(TSPInstance& instance,  std::shared_ptr<TSPSolution> zones_solution, int randomization)
{
}

std::shared_ptr<TSPSolution> TSP_GiantTour_FN::run(TSPInstance& instance, int randomization /*= 3*/)
{

    int n_vehicles = 1;

    std::shared_ptr<TSPSolution> solution = make_shared <TSPSolution>(instance, n_vehicles);

    this->giant_tour_solution = solution;

    int vehicle_id = 0;
    int custId;

    //START ROUTE AT THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);

    solution->customers[instance.depot_pos]->isRouted = true;

    int customers_without_route = instance.n_cust - 1;

    //THE INITIALIZATION VALUE OF BIG_NUMBER IS JUST USED TO AVOID ACCESSING ILLEGAL POSITION IN THE VECTOR, IN THE NEXT WHILE LOOP
    //IT CAN BE CHANGED TO ANY VALUE GREATER THAN THE NUMBER OF CUSTOMERS THAT NEED TO BE ROUTED
    unsigned int big_number = instance.n_cust * (instance.zones_time_matrix_df.size()-1);

    //WHILE THERE ARE STILL CUSTOMERS THAT ARE NOT ROUTED
    while(customers_without_route > 0) {
        //DETERMINE THE AMOUNT OF N FURTHEST NEIGHBORS, WITH N= MINIMUM VALUE BETWEEN RANDOMIZATION_PARAMETER AND NUMBER OF STILL UNROUTED CUSTOMERS FROM THE SAME ZONE_ID
        int number_of_nn = min(randomization, customers_without_route);
        shared_ptr <TSPCustomer> current_c = solution->vehicles[vehicle_id]->customers.back();
        vector<unsigned int> neighbors = instance.sorted_time_matrix[current_c->id];

        vector<unsigned int> near_neighbors(number_of_nn, big_number);
        int index = neighbors.size() - 1;
        int neighbors_index = 0;
        //ONCE N IS DETERMINED, FIND THE N FURTHEST NEIGHBORS TO THE LAST-ROUTED-CUSTOMER WITHIN THE SAME ZONE_ID
        while((near_neighbors[number_of_nn - 1] == big_number) && (index >=  0)) {
            if((!(solution->customers[neighbors[index]]->isRouted)) && (solution->customers[neighbors[index]]->id != current_c->id))  {
                near_neighbors[neighbors_index] = neighbors[index];
                neighbors_index++;
            }
            index--;

        }
            // PICK A RANDOM CUSTOMER FROM THE N POSSIBLE NEIGHBORS, AND ADD IT TO THE TAIL OF THE CURRENT SOLUTION. MARK CUSTOMER AS ROUTED.
            int randomNext = rand() % near_neighbors.size();
            custId = near_neighbors[randomNext];
            solution->vehicles[vehicle_id]->add_customer(solution->customers[custId], instance);
            solution->customers[custId]->isRouted = true;
            solution->customers[custId]->vehicle_route = vehicle_id;
            customers_without_route--;
    }
    // WHEN ALL CUSTOMERS ARE ROUTED, GO BACK TO THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);

    return solution;
}


TSPGiantTour_Prob_RNN::TSPGiantTour_Prob_RNN()
{
}

TSPGiantTour_Prob_RNN::TSPGiantTour_Prob_RNN(int seed)
        : TSPGiantTour(seed)
{
}

TSPGiantTour_Prob_RNN::~TSPGiantTour_Prob_RNN()
{
}

std::shared_ptr<TSPSolution> TSPGiantTour_Prob_RNN::run_fixed_zones(TSPInstance& instance,  std::shared_ptr<TSPSolution> zones_solution, int randomization)
{
}


std::shared_ptr<TSPSolution> TSPGiantTour_Prob_RNN::run(TSPInstance& instance, int randomization)
{
    //TSP Solution = 1 vehicle
    int n_vehicles = 1;

    unordered_map<string, int> unrouted_zones_map (instance.zones_map);

    std::shared_ptr<TSPSolution> solution = make_shared <TSPSolution>(instance, n_vehicles);

    this->giant_tour_solution = solution;

    int vehicle_id = 0;
    int custId;

    //START ROUTE AT THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);
    solution->customers[instance.depot_pos]->isRouted = true;
    unrouted_zones_map[solution->customers[instance.depot_pos]->zone_id]--;

    unsigned int big_number = instance.n_cust * (instance.zones_time_matrix_df.size()-1);
    int customers_without_route = instance.n_cust - 1;

    //WHILE THERE ARE STILL CUSTOMERS THAT ARE NOT ROUTED
    while(customers_without_route > 0) {

        shared_ptr <TSPCustomer> current_c = solution->vehicles[vehicle_id]->customers.back();

        //DETERMINE THE AMOUNT OF N NEAREST NEIGHBORS, WITH N= MINIMUM VALUE BETWEEN RANDOMIZATION_PARAMETER AND NUMBER OF STILL UNROUTED CUSTOMERS FROM THE SAME ZONE_ID
        int number_of_nn = 0;
        if ((unrouted_zones_map[current_c->zone_id] > 0) && (current_c->zone_id != "nan")) {
            number_of_nn = min(min(randomization, customers_without_route), unrouted_zones_map[current_c->zone_id]);
        }
        else {
            number_of_nn = min(randomization, customers_without_route);
        }


        vector<unsigned int> neighbors = instance.sorted_time_matrix[current_c->id];
        vector<unsigned int> near_neighbors(number_of_nn, big_number);
        //THE INITIALIZATION VALUE OF BIG_NUMBER IS JUST USED TO AVOID ACCESSING ILLEGAL POSITION IN THE VECTOR, IN THE NEXT WHILE LOOP
        //IT CAN BE CHANGED TO ANY VALUE GREATER THAN THE NUMBER OF CUSTOMERS THAT NEED TO BE ROUTED

        int index = 0;
        int neighbors_index = 0;

        //ONCE N IS DETERMINED, FIND THE N NEAREST NEIGHBORS TO THE LAST-ROUTED-CUSTOMER WITHIN THE SAME ZONE_ID
        while ((near_neighbors[number_of_nn - 1] == big_number) && (index <  neighbors.size())) {
            if ((unrouted_zones_map[current_c->zone_id] > 0) && (current_c->zone_id != "nan")) {
                if ((!solution->customers[neighbors[index]]->isRouted) && (solution->customers[neighbors[index]]->id != current_c->id) &&
                    ((solution->customers[neighbors[index]]->zone_id == current_c->zone_id) || (solution->customers[neighbors[index]]->zone_id == "nan"))) {
                    near_neighbors[neighbors_index] = neighbors[index];
                    neighbors_index++;
                }
            }
            //ROUTE FIRST CUSTOMERS OF SAME ZONE_ID, NOT INCLUDING CUSTOMER WITH NAN ZONE_ID
            else {
                if ((!solution->customers[neighbors[index]]->isRouted) && (solution->customers[neighbors[index]]->id != current_c->id)){
                    near_neighbors[neighbors_index] = neighbors[index];
                    neighbors_index++;
                }
            }
            index++;

        }
        // PICK A RANDOM CUSTOMER FROM THE N POSSIBLE NEIGHBORS, AND ADD IT TO THE TAIL OF THE CURRENT SOLUTION. MARK CUSTOMER AS ROUTED.
        int randomNext = rand() % near_neighbors.size();
        custId = near_neighbors[randomNext];
        solution->vehicles[vehicle_id]->add_customer(solution->customers[custId], instance);
        solution->customers[custId]->isRouted = true;
        solution->customers[custId]->vehicle_route = vehicle_id;
        customers_without_route--;
        unrouted_zones_map[solution->customers[custId]->zone_id]--;

    }
    // WHEN ALL CUSTOMERS ARE ROUTED, GO BACK TO THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);


    return solution;
}


TSPGiantTour_fixed_zones_RNN::TSPGiantTour_fixed_zones_RNN()
{
}

TSPGiantTour_fixed_zones_RNN::TSPGiantTour_fixed_zones_RNN(int seed)
        : TSPGiantTour(seed)
{
}

TSPGiantTour_fixed_zones_RNN::~TSPGiantTour_fixed_zones_RNN()
{
}

std::shared_ptr<TSPSolution> TSPGiantTour_fixed_zones_RNN::run_fixed_zones(TSPInstance& instance,  std::shared_ptr<TSPSolution> zones_solution, int randomization)
{
    //TSP Solution = 1 vehicle
    int n_vehicles = 1;

    unordered_map<string, int> unrouted_zones_map (instance.zones_map);

    std::shared_ptr<TSPSolution> solution = make_shared <TSPSolution>(instance, n_vehicles);

    this->giant_tour_solution = solution;

    int vehicle_id = 0;
    int custId;

    //START ROUTE AT THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);
    solution->customers[instance.depot_pos]->isRouted = true;
    unrouted_zones_map[solution->customers[instance.depot_pos]->zone_id]--;

    unsigned int big_number = instance.n_cust * (instance.zones_time_matrix_df.size()-1);
    int customers_without_route = instance.n_cust - 1;

    int zone_counter = 1;
    string current_zone = zones_solution->vehicles[0]->customers[zone_counter]->zone_id;

    //WHILE THERE ARE STILL CUSTOMERS THAT ARE NOT ROUTED
    while(customers_without_route > 0) {

        shared_ptr <TSPCustomer> current_c = solution->vehicles[vehicle_id]->customers.back();

        //DETERMINE THE AMOUNT OF N NEAREST NEIGHBORS, WITH N= MINIMUM VALUE BETWEEN RANDOMIZATION_PARAMETER AND NUMBER OF STILL UNROUTED CUSTOMERS FROM THE SAME ZONE_ID
        int number_of_nn = 0;
        if (unrouted_zones_map[current_zone] == 0){
            zone_counter++;
            current_zone = zones_solution->vehicles[0]->customers[zone_counter]->zone_id;
        }
        if ((unrouted_zones_map[current_zone] > 0) && (current_zone != "nan")) {
            number_of_nn = min(min(randomization, customers_without_route), unrouted_zones_map[current_zone]);
        }
        else {
            number_of_nn = min(randomization, customers_without_route);
        }


        vector<unsigned int> neighbors = instance.sorted_time_matrix[current_c->id];
        vector<unsigned int> near_neighbors(number_of_nn, big_number);
        //THE INITIALIZATION VALUE OF BIG_NUMBER IS JUST USED TO AVOID ACCESSING ILLEGAL POSITION IN THE VECTOR, IN THE NEXT WHILE LOOP
        //IT CAN BE CHANGED TO ANY VALUE GREATER THAN THE NUMBER OF CUSTOMERS THAT NEED TO BE ROUTED

        int index = 0;
        int neighbors_index = 0;

        //ONCE N IS DETERMINED, FIND THE N NEAREST NEIGHBORS TO THE LAST-ROUTED-CUSTOMER WITHIN THE SAME ZONE_ID
        while ((near_neighbors[number_of_nn - 1] == big_number) && (index <  neighbors.size())) {
            if ((unrouted_zones_map[current_zone] > 0) && (current_zone != "nan")) {
                if ((!solution->customers[neighbors[index]]->isRouted) && (solution->customers[neighbors[index]]->id != current_c->id) &&
                    ((solution->customers[neighbors[index]]->zone_id == current_zone) || (solution->customers[neighbors[index]]->zone_id == "nan"))) {
                    near_neighbors[neighbors_index] = neighbors[index];
                    neighbors_index++;
                }
            }
                //ROUTE FIRST CUSTOMERS OF SAME ZONE_ID, NOT INCLUDING CUSTOMER WITH NAN ZONE_ID
            else {
                if ((!solution->customers[neighbors[index]]->isRouted) && (solution->customers[neighbors[index]]->id != current_c->id)){
                    near_neighbors[neighbors_index] = neighbors[index];
                    neighbors_index++;
                }
            }
            index++;

        }
        // PICK A RANDOM CUSTOMER FROM THE N POSSIBLE NEIGHBORS, AND ADD IT TO THE TAIL OF THE CURRENT SOLUTION. MARK CUSTOMER AS ROUTED.
        int randomNext = rand() % near_neighbors.size();
        custId = near_neighbors[randomNext];
        solution->vehicles[vehicle_id]->add_customer(solution->customers[custId], instance);
        solution->customers[custId]->isRouted = true;
        solution->customers[custId]->vehicle_route = vehicle_id;
        customers_without_route--;
        unrouted_zones_map[solution->customers[custId]->zone_id]--;

    }
    // WHEN ALL CUSTOMERS ARE ROUTED, GO BACK TO THE DEPOT
    solution->vehicles[vehicle_id]->add_customer(solution->customers[instance.depot_pos], instance);


    return solution;
}

