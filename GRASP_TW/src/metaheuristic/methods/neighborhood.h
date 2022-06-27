#pragma once
#include "../model/solution.h"

class TSPNeighborhood{
public:
    bool first_improvement;
    std::string id;

    TSPNeighborhood(bool first_improvement = false, const std::string& id = "");
    ~TSPNeighborhood();

    virtual std::shared_ptr <TSPSolution> search_neighbors(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, bool by_zones);
};

class TSPSwapNeighborhood: public TSPNeighborhood{
public:
    TSPSwapNeighborhood();

    TSPSwapNeighborhood(bool first_improvement = false);

    ~TSPSwapNeighborhood();

    std::shared_ptr <TSPSolution> search_neighbors(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, bool  by_zones);
    std::shared_ptr <TSPSolution> apply_best_move(std::shared_ptr <TSPSolution> initial_solution, std::vector<int>& best_move, TSPInstance& instance);
};

class TSPRelocateNeighborhood: public TSPNeighborhood{
public:
    TSPRelocateNeighborhood();

    TSPRelocateNeighborhood(bool first_improvement = false);

    ~TSPRelocateNeighborhood();

    std::shared_ptr <TSPSolution> search_neighbors(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, bool  by_zones);
    std::shared_ptr <TSPSolution> apply_best_move(std::shared_ptr <TSPSolution> initial_solution, std::vector<int>& best_move, TSPInstance& instance);
};

class TSP_ProbZoneSwapNeighborhood: public TSPNeighborhood{
public:
    TSP_ProbZoneSwapNeighborhood();

    TSP_ProbZoneSwapNeighborhood(bool first_improvement = false);

    ~TSP_ProbZoneSwapNeighborhood();

    std::shared_ptr <TSPSolution> search_neighbors(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, bool  by_zones);
    std::shared_ptr <TSPSolution> apply_best_move(std::shared_ptr <TSPSolution> initial_solution, std::vector<int>& best_move, TSPInstance& instance);
};

class TSP_ProbZoneRelocateNeighborhood: public TSPNeighborhood{
public:
    TSP_ProbZoneRelocateNeighborhood();

    TSP_ProbZoneRelocateNeighborhood(bool first_improvement = false);

    ~TSP_ProbZoneRelocateNeighborhood();

    std::shared_ptr <TSPSolution> search_neighbors(std::shared_ptr <TSPSolution> initial_solution, TSPInstance& instance, bool  by_zones);
    std::shared_ptr <TSPSolution> apply_best_move(std::shared_ptr <TSPSolution> initial_solution, std::vector<int>& best_move, TSPInstance& instance);
};