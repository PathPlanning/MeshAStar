#pragma once

#include <vector>
#include <string>
#include <tuple>
#include <map>

#include "KC_heap.hpp"

using namespace std;

// Forward declarations
struct Primitive;
struct ControlSet;
struct MeshInfo; 

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

struct Vertex {
    /*
     Represents a node in the search graph.
     In this framework, a vertex can represent either:
      1. A discrete state (i, j, theta) in the Lattice Graph.
      2. An extended cell (i, j, Psi) in the Mesh Graph.
     
     This structure is designed to handle both cases efficiently.

     Memory Optimization Note:
     Fields exclusive to either Lattice or Mesh representations are stored in UNIONS.
     Unions allow different data members to occupy the same memory location.
     For example, 'theta' and 'config_id' share the same 4 bytes.
     - Benefit: Reduces memory footprint significantly (critical for large-scale search).
     - Constraint: Only one field from the union can be valid at any given time.
    */

    int i, j;  // Grid coordinates of the cell (common to both Lattice and Mesh).

    union {
        int theta;      // Discrete heading (theta). Used only for Lattice states.
        int config_id;  // Configuration ID (Psi). Used only for Mesh Extended Cells.
    };

    // Constructors
    Vertex(int i, int j, int theta);
    
    // Constructor for MeshA* nodes.
    // 'is_mesh_node' is a flag to distinguish signature, asserted to be true in implementation.
    Vertex(int i, int j, int config_id, bool is_mesh_node); 
};

struct Primitive {
    /*
     Represents a motion primitive - a precomputed, kinodynamically feasible trajectory segment.
     Primitives are defined in a local coordinate frame starting at (0, 0) with a specific start heading.
    */

    int start_theta;               // The discrete heading from which this primitive starts.
    ptrVertex goal;                // The relative goal state reached by this primitive.
    
    // The sequence of grid cells swept by the agent's body while executing this primitive.
    // Used for collision checking (collision traces).
    vector<int> swept_cells_i; 
    vector<int> swept_cells_j;
    
    double length;                 // Arc length of the trajectory.
    double accumulated_heading_change; // Total change in heading (radians), used for AOL metric.
    
    int id;                        // Unique identifier of the primitive.

    Primitive();
    void add_collision(int i, int j);
    ~Primitive();
};

struct ControlSet {
    /*
     The Motion Primitive Library (Control Set).
     Stores the set of available actions for the agent.
    */

    string filename;
    
    // Maps a start heading (theta) to a list of applicable primitive IDs.
    vector<vector<int>> prims_by_heading; 
    
    // Lookup vector to access a primitive object by its ID.
    vector<Primitive*> list_all_prims;  

    // Reverse lookup:
    // Maps a specific transition tuple (start_theta, delta_x, delta_y, end_theta)
    // to the corresponding primitive ID. 
    map<tuple<int, int, int, int>, int> prim_between_states; 

    ControlSet();
    void load_primitives(string file);
    vector<int>& get_prims_by_heading(int heading);
    float cost(int id);
    
    // Returns the ID of the primitive connecting the defined states.
    int get_prim_between_states(int st, int di, int dj, int ft);
    
    ~ControlSet();
};

struct MeshInfo {
    /*
     Stores precomputed topological information about Configurations (Psi) for MeshA*.
     This structure effectively defines the edges of the Mesh Graph.
    */

    string filename;
    
    // -------------------------------------------------------------------------
    // Successor Transition Table
    // -------------------------------------------------------------------------
    // For each configuration ID (Psi), stores a list of possible successors.
    // Each successor is a tuple of 4 values: (di, dj, next_config, prim_id).
    //
    // 1) di, dj: Relative grid shift of the successor's projection.
    //            Since successors are adjacent, these values are always in {-1, 0, 1}.
    // 2) next_config: The configuration ID (Psi') of the successor extended cell.
    // 3) prim_id: The ID of the primitive connecting the current cell to this successor.
    //    IMPORTANT: This ID is uniquely defined IF AND ONLY IF the successor is an 
    //    Initial Extended Cell (see Paper/Theorem). For non-initial successors, the 
    //    primitive is ambiguous (multiple primitives might pass through), so prim_id is -1.
    vector<vector<tuple<int, int, int, int>>> successors;  

    // -------------------------------------------------------------------------
    // Initial Configuration Mapping
    // -------------------------------------------------------------------------
    // Maps a discrete heading (0 to NUM_HEADINGS-1) to the Initial Configuration ID (Psi_theta)
    // associated with the bundle of primitives starting at that heading.
    vector<int> initial_config_by_theta;  

    // Reverse map: Returns the discrete heading (theta) for a given Initial Configuration ID.
    // Returns -1 if the configuration is not Initial.
    vector<int> theta_by_initial_config;

    // -------------------------------------------------------------------------
    // Internal Primitive Structure (Finals)
    // -------------------------------------------------------------------------
    // For each configuration ID, stores the list of primitives contained within it.
    // Used for Heuristic computation and Pruning (checking if all endpoints are closed).
    //
    // Each entry is a tuple of 5 values: (ft, di, dj, k, id)
    // 1) ft: Final discrete heading where the primitive ends.
    // 2) di, dj: Relative coordinate shift from the current cell to the primitive's endpoint.
    // 3) k: The index of the current cell in the primitive's collision trace (swept cells).
    //       (i.e., the tuple (prim, k) is an element of the configuration set Psi).
    // 4) id: The unique ID of this primitive.
    vector<vector<tuple<int, int, int, int, int>>> primitives_in_config;

    MeshInfo();
    void load(string file);
};
