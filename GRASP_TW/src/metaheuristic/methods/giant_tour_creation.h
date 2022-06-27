#pragma once
#include "../model/solution.h"
#include "../model/instance.h"


//STARTS TSP with TW structures

class TSPGiantTour {
public:
    std::shared_ptr<TSPSolution> giant_tour_solution;
    TSPInstance instance;

    TSPGiantTour();
    TSPGiantTour(int seed);
    ~TSPGiantTour();

    virtual std::shared_ptr<TSPSolution> run(TSPInstance& instance, int alpha = 3);
    virtual std::shared_ptr<TSPSolution> run_fixed_zones(TSPInstance& instance, std::shared_ptr<TSPSolution> zones_solution,int alpha = 3);

};


class TSPGiantTour_Prob_RNN:public TSPGiantTour {
public:

    TSPGiantTour_Prob_RNN();
    TSPGiantTour_Prob_RNN(int seed);
    ~TSPGiantTour_Prob_RNN();

    std::shared_ptr<TSPSolution> run(TSPInstance& instance, int alpha = 3);
    std::shared_ptr<TSPSolution> run_fixed_zones(TSPInstance& instance, std::shared_ptr<TSPSolution> zones_solution,int alpha = 3);
};

class TSP_GiantTour_FN:public TSPGiantTour {
public:

    TSP_GiantTour_FN();
    TSP_GiantTour_FN(int seed);
    ~TSP_GiantTour_FN();

    std::shared_ptr<TSPSolution> run(TSPInstance& instance, int alpha = 3);
    std::shared_ptr<TSPSolution> run_fixed_zones(TSPInstance& instance, std::shared_ptr<TSPSolution> zones_solution,int alpha = 3);
};

class TSPGiantTour_fixed_zones_RNN:public TSPGiantTour {
public:

    TSPGiantTour_fixed_zones_RNN();
    TSPGiantTour_fixed_zones_RNN(int seed);
    ~TSPGiantTour_fixed_zones_RNN();

    std::shared_ptr<TSPSolution> run_fixed_zones(TSPInstance& instance, std::shared_ptr<TSPSolution> zones_solution, int alpha = 3);

};
