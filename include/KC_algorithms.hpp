#pragma once

#include "KC_structs.hpp"
#include "KC_searching.hpp"
#include "KC_search_params.hpp"

// Metric utility functions
double metric_length(vector<int> path, ControlSet *control_set);
double metric_AOL(vector<int> path, ControlSet *control_set);

// Algorithm Identifiers
#define ALGO_LATTICE_ASTAR 0
#define ALGO_MESH_ASTAR 1

struct SearchAlgorithm {
    /*
     A high-level wrapper class that orchestrates the path planning process.
     It handles initialization, execution of the selected algorithm (Lattice A* or MeshA*),
     and results aggregation.
    */

    string name;             // Name of the experiment/algorithm instance.

    Map *map;                // The environment map.
    ControlSet *control_set; // The Motion Primitive Library.
    MeshInfo *mesh_info;     // Precomputed Mesh topology (Psi configurations). 
                             // Required only for MeshA*.

    // Search Parameters
    float w;    // Heuristic weight (for Weighted A*).
    
    // Goal Definition:
    // A path is valid if it ends within distance R and angle tolerance A from the goal.
    float R;    
    int A;      // Heading tolerance in discrete steps (e.g., A=1 allows error of +/- 1 index).
    
    bool lazy;  // Enables Lazy A* (postponed collision checking).
    int algo;   // Selected algorithm: ALGO_LATTICE_ASTAR or ALGO_MESH_ASTAR.
    
    // Specific Search Contexts (Strategy Pattern)
    // Only one of these is active during a solve() call, depending on 'algo'.
    MeshGraphParams *mesh_params;     
    StateLatticeParams *lattice_params;
    
    // Problem Instance (Start/Finish)
    // Stored as discrete states (i, j, theta).
    int si, sj, st; 
    int fi, fj, ft;

    // Performance Statistics
    double search_time;             // Time spent in the pure A* search loop.
    double trajectory_reconstruction_time; // Time spent reconstructing the primitive sequence.
    double total_runtime;           // Total execution time.
    
    double cost_path;               // Cost of the found solution.
    int steps;                      // Total number of A* expansions.
    bool is_solved;                 // Success flag.
    int checked_cells;              // Total number of grid cells checked for collision.
    
    // The result trajectory as a sequence of primitive IDs.
    vector<int> path;  

    SearchAlgorithm(string name, Map *map, ControlSet *control_set, MeshInfo *mesh_info,
                    bool lazy, float w, float R, int A,
                    int algo);

    // Solves the problem between start and finish states.
    // start/finish are pointers to discrete Lattice states (x, y, theta).
    void solve(Vertex *start, Vertex *finish);
    
    // Exports results to a log file.
    void dump_result_of_search(string filename);

    ~SearchAlgorithm();
};
