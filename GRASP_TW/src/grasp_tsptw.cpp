#include "metaheuristic/methods/giant_tour_creation.h"
#include "metaheuristic/methods/create_zone_matrix.h"
#include "metaheuristic/methods/neighborhood.h"
#include "metaheuristic/methods/vnd.h"
#include "metaheuristic/model/instance.h"
#include "metaheuristic/model/solution.h"
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <omp.h>
#include <sstream>
#include <cmath>
#include <random>



using namespace std;

template < typename Type > std::string to_str (const Type & t)
{
    std::ostringstream os;
    os << t;
    return os.str ();
}


/*
 * RUN SEARCH IS THE MAIN ALGORITHM FUNCTION
 * INPUTS:
 * randomization --> int Indicate randomization value for construction of initial solutions with random_nearest_neighbor
 * seed --> int Random number generator state
 * instance_name --> string  "route_"+route_number+".txt";
 * in_test_phase --> 0 if currently Train phase, 1 if currently apply/test phase
 * neighborhood_combinations_sliced --> vector of vectors containing the Neighborhoods for VND
 * first_improvement --> Boolean indicating if first_improvement strategy or best_improvement strategy is used in the VND (Default=false, use best_improvement)
 * normalize --> Indicate if normalize costs of the objective function. Default=True
 */

void run_search(int randomization, int seed,  const string& instance_name, int in_test_phase,  vector<vector<TSPNeighborhood*>>neighborhood_combinations_sliced, vector<TSPNeighborhood*> zones_neighborhoods, int k_1, int k_2, int k_3, int k_4, int k_5) {
    string direc_input = "";
    string direc_output = "";
    string timing_direc = "";
    string local_optima_direc = "";

    //WRITE OUTPUT CSV'S TO DIFFERENT FOLDERS, DEPENDING ON TRAINING PHASE OR APPLY PHASE
    if (in_test_phase == 0) {
        direc_input = "../data/model_build_outputs/parsed_files/";
        timing_direc = "../data/model_build_outputs/timing/";
        direc_output = "../data/model_build_outputs/grasp_routes_prob_random_"+ to_str(randomization)+"/";
        local_optima_direc = "../data/model_build_outputs/local_optima_analisis/";
    }
    else if (in_test_phase == 1) {
        direc_input = "../data/model_apply_outputs/parsed_files_val/";
        timing_direc = "../data/model_apply_outputs/timing/";
        direc_output = "../data/model_apply_outputs/grasp_routes_prob_random_"+ to_str(randomization)+"/";
        local_optima_direc = "../data/model_apply_outputs/local_optima_analisis/";

    }

    time_t grasp_start, grasp_end;

    time(&grasp_start);

    vector<string> splitting_result;
    boost::split(splitting_result, instance_name, boost::is_any_of("."));


    string global_results = direc_output +"solution_" + splitting_result[0];


    vector<string> route_and_number;
    boost::split(route_and_number, splitting_result[0], boost::is_any_of("_"));
    TSPInstance instance = TSPInstance(direc_input + instance_name, route_and_number[1], in_test_phase);
    cout << "Parsing instance done" << endl;


    //ASSIGN RANDOM_SEED_STATE TO THE METHOD FOR CREATING RANDOM FURTHEST NEIGHBOR SOLUTION
    vector<TSPGiantTour *> giant_tour_heuristics;
    vector<TSPGiantTour *> furthest_heuristics = {new TSP_GiantTour_FN(seed)};  //Furthest neighbor for max initial_solution time

    // CREATE STRUCTURES FOR STORING INITIAL SOLUTIONS - CURRENT BEST SOLUTION - FINAL BEST SOLUTION
    //TSPSolution *giant_tour_solution;
    shared_ptr <TSPSolution> initial_zones_solution;
    shared_ptr <TSPSolution> initial_furthest_solution;
    shared_ptr <TSPSolution> best_solution = make_shared<TSPSolution>();
    shared_ptr <TSPSolution> best_initial_solution = make_shared<TSPSolution>();
    shared_ptr <TSPSolution> train_best_initial_solution = make_shared<TSPSolution>();
    shared_ptr <TSPSolution> train_best_solution = make_shared<TSPSolution>();
    //shared_ptr <TSPSolution> giant_tour_solution;

    bool is_train_best_solution_null = true;
    float train_best_solution_cost = std::numeric_limits<float>::max();
    LocalOptimaOutputs local_optima_storage;

    bool normalize = true;
    instance.normalize = normalize;

    if (in_test_phase == 1){
        instance.in_test_phase = true;
    }
    else
    {
        instance.in_test_phase = false;
    }

    //ASSIGN PONDERATIONS FOR COST FUNCTION
    instance.k_1 = k_1;
    instance.k_2 = k_2;
    instance.k_3 = k_3;
    instance.k_4 = k_4;
    instance.k_5 = k_5;


    // CREATE FURTHEST SOLUTION USING RANDOM FURTHEST NEIGHBOR HEURISTIC
    cout << "Computing initial solution" << endl;
    initial_furthest_solution = furthest_heuristics[0]->run(instance, randomization);
    float initial_furthest_cost = initial_furthest_solution->compute_cost(instance);
    cout << "Furthest cost: " << initial_furthest_cost << endl;
    // THIS COST OF THIS FURTHEST SOLUTION WILL BE USED TO NORMALIZE THE ROUTE-TIME COST OF THE OTHER SOLUTIONS GENERATED IN THIS GRASP
    instance.worst_initial_cost = initial_furthest_cost;

    //CREATE ZONE_TIME AND ZONE_DISTANCE PENALIZATION MATRICES
    TSPZoneMatrixSolution zones_info = create_zones_matrix(instance);

    initial_zones_solution = zones_info.zone_solution;

    initial_zones_solution->compute_zones_cost(instance);
    cout << "initial_zones_solution: " << initial_zones_solution->cost << " Time Cost: " << initial_zones_solution->time_cost <<" Heading Cost: " << initial_zones_solution->heading_penalization_cost << " Angle cost: " << initial_zones_solution->angle_penalization_cost  <<endl;
    cout << " Zone_time cost: " <<  initial_zones_solution->zone_time_penalization_cost  <<  endl;






    //DETERMINE NUMBER OF GRASP ITERATIONS THAT WILL BE PERFORMED, ACCORDING TO NUMBER OF CUSTOMERS AND NUMBER OF ZONES IN THE ROUTE
    //int calc_iterations = instance.n_cust * log(instance.n_cust/(instance.zones_time_matrix_df.size()-1))/ sqrt(instance.zones_time_matrix_df.size()-1) ;
    //int number_of_zones = instance.zones_time_matrix_df.size()-1;
    //int total_iterations = max(calc_iterations,  number_of_zones);

    int total_iterations = omp_get_max_threads();

    omp_set_num_threads(1);
    total_iterations = 25;
    cout << "number_of_grasp_iteration: " << total_iterations << endl;

    string local_optima_direction = local_optima_direc + "solution_" + splitting_result[0];
    int local_optima_counter = 0;

    //std::random_device rd;
    std::mt19937 rng(seed);


    vector<shared_ptr <TSPSolution>> initial_zones_solutions_vector;
    initial_zones_solutions_vector.reserve(total_iterations);
    for (int it = 0; it < total_iterations; it++) {
        shared_ptr <TSPSolution> initial_local_zones = make_shared<TSPSolution>();
        int n= 0;
        initial_local_zones->vehicles.push_back(std::make_shared<TSPVehicle>(instance.n_cust, instance.Q));
        for (auto customer: initial_zones_solution->vehicles[0]->customers){
            shared_ptr<TSPCustomer> c = std::make_shared<TSPCustomer>(std::make_shared<TSPCustomer>(n, customer->time_window, customer->service_time, customer->demand, customer->zone_id));
            initial_local_zones->vehicles[0]->customers.push_back(c);
            n++;
        }
        initial_zones_solutions_vector.push_back(initial_local_zones);
        std::shuffle ( initial_zones_solutions_vector[it]->vehicles[0]->customers.begin()+1, initial_zones_solutions_vector[it]->vehicles[0]->customers.end()-1, rng );
    }

    bool is_best_solution_null = true;
    float best_solution_cost = std::numeric_limits<float>::max();

#pragma omp parallel
    {
        int seeds = omp_get_thread_num();

        //ASSIGN RANDOM_SEED_STATE TO EACH THREAD IN THE METHOD TO CREATE INITIAL SOLUTIONS WITH RANDOM NEAREST NEIGHBOR
        //giant_tour_heuristics = {new TSPGiantTour_Prob_RNN(seeds)};
        giant_tour_heuristics = {new TSPGiantTour_fixed_zones_RNN(seeds)};


        //START GRASP ITERATIONS
#pragma omp for
        for (int it = 0; it < total_iterations; it++) {
            shared_ptr <TSPSolution> local_optima_zones;
            local_optima_zones = run_zones_vnd(initial_zones_solutions_vector[it], instance, zones_neighborhoods, true);


            shared_ptr <TSPSolution> giant_tour_solution;

            //CREATE INITIAL SOLUTION WITH CLUSTERED RANDOM NEAREST NEIGHBOR
            //giant_tour_solution = giant_tour_heuristics[0]->run(instance, randomization);
            giant_tour_solution = giant_tour_heuristics[0]->run_fixed_zones(instance, local_optima_zones, randomization);

            giant_tour_solution->heading_penalization_cost = local_optima_zones->heading_penalization_cost;
            giant_tour_solution->angle_penalization_cost = local_optima_zones->angle_penalization_cost;
            giant_tour_solution->zone_time_penalization_cost = local_optima_zones->zone_time_penalization_cost;
            giant_tour_solution->cost = giant_tour_solution->compute_prob_cost(instance);



            float local_optima_cost;
            shared_ptr <TSPSolution> local_optima;
            local_optima = run_prob_tsp_vnd(giant_tour_solution, instance, neighborhood_combinations_sliced[0], true);
            //local_optima = giant_tour_solution;
            local_optima_cost = local_optima->cost;

            //cout << "local_optima_zones: " << local_optima_zones->cost << " Time Cost: " << local_optima_zones->time_cost <<" Heading Cost: " << local_optima_zones->heading_penalization_cost << " Angle cost: " << local_optima_zones->angle_penalization_cost  <<endl;
            //cout << " Zone_time cost: " <<  local_optima_zones->zone_time_penalization_cost << " Zone_distance cost: " <<  local_optima_zones->zone_distance_penalization_cost  <<  endl;


#pragma omp critical
            {
                //std::random_shuffle ( initial_local_zones->vehicles[0]->customers.begin(), initial_local_zones->vehicles[0]->customers.end() );
                //local_optima_zones = run_zones_vnd(initial_zones_solutions_vector[it], instance, zones_neighborhoods, true);


                if (is_best_solution_null || local_optima_cost < best_solution_cost) {

                    //best_tsp_solution = giant_tour_solution;
                    best_initial_solution = giant_tour_solution;
                    best_solution = local_optima;

                    best_solution_cost = local_optima_cost;
                    is_best_solution_null = false;


                }



                if (is_train_best_solution_null || best_solution_cost < train_best_solution_cost) {
                    train_best_solution = best_solution;
                    train_best_initial_solution = best_initial_solution;

                    train_best_solution_cost = best_solution_cost;
                    cout << train_best_solution_cost << endl;
                    is_train_best_solution_null = false;
                }
            }
        }

    }

    cout << "Final Cost: " << train_best_solution->cost << " Time Cost: " << instance.k_1 << "* " << train_best_solution->time_cost <<" Heading Cost: " <<instance.k_2 << "* " <<  train_best_solution->heading_penalization_cost << " Angle cost: " << instance.k_3 << "* " <<  train_best_solution->angle_penalization_cost  <<endl;
    cout << "Time_window cost: " << instance.k_4 << "* " <<   train_best_solution->time_window_cost << "Zone_time cost: " <<  instance.k_5 << "* " << train_best_solution->zone_time_penalization_cost  << endl;



    ofstream global_output(global_results);
    global_output << train_best_solution->TSP_output_string_solutions(instance).str() << endl;

    global_output.close();


    time(&grasp_end);
    cout << "Grasp time: " << grasp_end - grasp_start << endl;

    for (auto giant_tour: giant_tour_heuristics){
        delete giant_tour;
    }
    for (auto furthest_tour: furthest_heuristics){
        delete furthest_tour;
    }
}

int main(int argc, char* argv[])
{
    string route_number_str = argv[1];
    size_t pos;
    string n_randomization = argv[2];
    size_t pos3;
    int randomization = stoi(n_randomization, &pos3);
    int route = stoi(route_number_str, &pos);
    string n_in_test_phase = argv[3];
    size_t pos2;
    int in_test_phase = stoi(n_in_test_phase, &pos2);
    string n_k1 = argv[4];
    size_t pos4;
    int k_1 = stoi(n_k1, &pos4);
    string n_k2 = argv[5];
    size_t pos5;
    int k_2 = stoi(n_k2, &pos5);
    string n_k3 = argv[6];
    size_t pos6;
    int k_3 = stoi(n_k3, &pos6);
    string n_k4 = argv[7];
    size_t pos7;
    int k_4 = stoi(n_k4, &pos7);
    string n_k5 = argv[8];
    size_t pos8;
    int k_5 = stoi(n_k5, &pos8);


    int seed = 2;
    

    bool first_improvement = false;

    string instance_name = "route_"+to_string(route)+".txt";
    vector<TSPNeighborhood*> neighborhoods = {};

    int indexes[] = { 0, 1,  };
    vector<string> neighborhoods_names = { "r", "s", };


    neighborhoods = {new TSPRelocateNeighborhood(first_improvement),
                     new TSPSwapNeighborhood(first_improvement),
    };

    //vector<TSPNeighborhood*> zones_neighborhoods = {new TSP_ProbZoneRelocateNeighborhood(true), new TSP_ProbZoneSwapNeighborhood(true)};
	vector<TSPNeighborhood*> zones_neighborhoods = {new TSP_ProbZoneRelocateNeighborhood(true)};

    vector<vector<TSPNeighborhood*>> neighborhood_combinations;
    vector<vector<TSPNeighborhood*>> neighborhood_combinations_sliced;


    vector<vector<string>> neighborhood_names_combinations;
    vector<vector<string>> neighborhood_names_combinations_sliced;

    do {
        vector<TSPNeighborhood*> combination;
        vector<string> names_combination;
        for(unsigned int i = 0; i < neighborhoods_names.size(); i++) {
            combination.push_back(neighborhoods[indexes[i]]);
            names_combination.push_back(neighborhoods_names[indexes[i]]);
        }
        neighborhood_combinations.push_back(combination);
        neighborhood_names_combinations.push_back(names_combination);
    } while(std::next_permutation(begin(indexes), end(indexes)));

    for(unsigned int i = 0; i < neighborhood_combinations.size(); i += 1) {
        if (i == 0){
            neighborhood_combinations_sliced.push_back(neighborhood_combinations[i]);
            neighborhood_names_combinations_sliced.push_back(neighborhood_names_combinations[i]);
        }
    }

    cout << "In test phase type "<< in_test_phase << endl;

    run_search(randomization, seed, instance_name, in_test_phase,  neighborhood_combinations_sliced, zones_neighborhoods,
               k_1, k_2, k_3, k_4, k_5);

    for (auto neighborhood: neighborhoods){
        delete neighborhood;
    }
    for (auto neighborhood: zones_neighborhoods){
        delete neighborhood;
    }
    cout << "Sali de run_search" << endl;
}
