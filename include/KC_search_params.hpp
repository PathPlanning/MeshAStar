#pragma once

#include "KC_structs.hpp"
#include "KC_heap.hpp"
#include "KC_searching.hpp"

struct StateLatticeParams {
    /*
     Encapsulates the context and parameters for Lattice-based A* search.
     This structure implements the "Policy" pattern, providing the search algorithm 
     with domain-specific logic (successors, heuristics, goal checks).
    */

    Map *task_map;             // The environment map.
    Vertex *start, *finish;    // Start and Goal states (discrete coordinates + heading).
    
    // Goal Definition Parameters:
    // A state is considered a goal if it is within distance R and angle tolerance A
    // (distance in descrete headings) from the exact finish state.
    float R; 
    int A;  
    
    float w;                   // Heuristic weight (for Weighted A*).

    SearchTree *ast;           // Pointer to the search tree (OPEN/CLOSED lists).
    ControlSet *control_set;   // The Motion Primitive Library.
    
    int checked_cells = 0;     // Counter for collision checks (performance metric).

    StateLatticeParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, 
                       float w = 1.0, float R = 3.0, int A = 1);

    ptrVertex get_start_vertex();
    bool is_goal(ptrVertex v);
    
    // Generates successors for a given node.
    // logical 'lazy' flag enables Lazy A* evaluation strategy if needed.
    void get_successors(ptrSearchNode v, vector<ptrSearchNode> &list, bool lazy = false);
    
    float heuristic(ptrSearchNode v);
    bool lazy_check_collision(ptrSearchNode v);
    
    ~StateLatticeParams();
};

struct MeshGraphParams {
    /*
     Encapsulates the context and parameters for MeshA* search.
     Operates on the Mesh Graph where nodes are Extended Cells (x, y, Psi).
    */

    Map *task_map; 
    Vertex *start, *finish;
    
    float R; 
    int A;
    float w;

    SearchTree *ast;  
    MeshInfo *mesh_info;       // Topology of the Mesh Graph (Configs/Psi transitions).
    ControlSet *control_set;   // Required for trajectory reconstruction and exact costs.
    
    int checked_cells = 0;
    
    MeshGraphParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, MeshInfo *mesh_info,
                     float w = 1.0, float R = 3.0, int A = 1);

    ptrVertex get_start_vertex();
    bool is_goal(ptrVertex v);
    
    // Successor generation uses the precomputed transition table in MeshInfo.
    void get_successors(ptrSearchNode v, vector<ptrSearchNode> &list, bool lazy);
    
    float heuristic(ptrSearchNode v);
    
    // Checks if the transition to this node is valid (collision-free).
    bool lazy_check_collision(ptrSearchNode v);
    
    ~MeshGraphParams();
};
