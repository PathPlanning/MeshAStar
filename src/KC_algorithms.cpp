#include <iostream>
#include <fstream>
#include <algorithm>
#include <ctime>   // For clock()

#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include "rassert.hpp"
#include "common.hpp"
#include "KC_heap.hpp"

// External global memory pool
extern MyHEAP *HEAP;

// -----------------------------------------------------------------------------
// Metric Functions
// -----------------------------------------------------------------------------

double metric_length(vector<int> path, ControlSet *control_set) {
    /*
     Calculates the total physical length of the trajectory.
    */
    if (path.empty()) return 0.0;

    double result = 0.0;
    for (int id : path) {
        result += control_set->list_all_prims[id]->length;
    }
    return result;
}

double metric_AOL(vector<int> path, ControlSet *control_set) {
    /*
     Calculates the AOL (Angle-Over-Length) metric.
     This serves as a proxy for path "smoothness" or "wiggliness".
     Higher AOL indicates more aggressive turning per unit of distance.
    */
    if (path.empty()) return 0.0;

    double total_heading_change = 0.0;
    for (int id : path) {
        total_heading_change += control_set->list_all_prims[id]->accumulated_heading_change;
    }
    
    // Normalize by total length
    return total_heading_change / metric_length(path, control_set);
}

// -----------------------------------------------------------------------------
// Search Algorithm Implementation
// -----------------------------------------------------------------------------

SearchAlgorithm::SearchAlgorithm(string name, Map *map, ControlSet *control_set, MeshInfo *mesh_info,
                                 bool lazy, float w, float R, int A,
                                 int algo) 
{
    this->name = name;
    this->map = map;
    this->control_set = control_set;
    this->mesh_info = mesh_info;

    this->w = w;
    this->R = R;
    this->A = A;
    this->lazy = lazy;
    this->algo = algo;

    // Initialize pointers to nullptr to ensure safe state
    this->lattice_params = nullptr;
    this->mesh_params = nullptr;
    
    this->is_solved = false;
    this->steps = 0;
    this->cost_path = 0.0;
}

static inline void validate_primitive_transition(ptrSearchNode u, ptrSearchNode v, Primitive *prim) {
    /*
     Debug helper: Validates that the primitive 'prim' actually connects state u to state v.
    */
    
    // Suppress unused variable warnings in Release builds (when rassert is no-op)
    (void)u; (void)v; (void)prim;

    rassert(prim != nullptr &&
            (v->vertex->i - u->vertex->i) == prim->goal->i &&
            (v->vertex->j - u->vertex->j) == prim->goal->j &&
            v->vertex->theta == prim->goal->theta &&
            u->vertex->theta == prim->start_theta,
            "Invalid primitive transition detected during reconstruction!");
}

static inline void reconstruct_path_lattice(ResultSearch *result, SearchAlgorithm *search) {
    /*
     Reconstructs the trajectory for Lattice-based A*.
    */

    if (result->path_found) {
        ptrSearchNode current = result->final_node;

        while (!(current->parent == NULL_Node)) {
            ptrSearchNode parent = current->parent;

            // Instead of storing parent_prim_id in every node (waste of memory),
            // we look up which primitive connects 'parent' to 'current'.
            // Assumption (from the Problem Statement section): At most one primitive exists between two lattice states.
            
            int st = parent->vertex->theta;
            int ft = current->vertex->theta;
            int di = current->vertex->i - parent->vertex->i;
            int dj = current->vertex->j - parent->vertex->j;

            int id = search->control_set->get_prim_between_states(st, di, dj, ft);
            rassert(id >= 0, "Error: No primitive connects these lattice states!");

            search->path.push_back(id);
            current = parent;
        }
        
        reverse(search->path.begin(), search->path.end());

        search->cost_path = result->final_node->g;
        search->is_solved = true;
        
        // The final_node was returned by StepAstar but never added to the CLOSED list
        // (search stops immediately). Thus, it is not managed by SearchTree.
        // We must delete it manually to avoid a small memory leak.
        HEAP->delete_SearchNode(result->final_node); 
    } else {
        search->cost_path = -1;
        search->is_solved = false;
    }
    search->steps = result->steps;
}

static inline void reconstruct_path_mesh(ResultSearch *result, SearchAlgorithm *search) {
    /*
     Reconstructs the trajectory for MeshA*.
     The path in the search tree consists of Extended Cells (Initial Configurations).
     We must reconstruct the actual motion primitives connecting these cells.

     PAPER LOGIC NOTE:
     As described in the article, MeshA* path reconstruction relies ONLY on
     Initial Extended Cells. Non-initial (intermediate) cells are transient
     and are not stored in memory after expansion (pruned from CLOSED).
     
     This function backtracks from Initial to Initial cell and reconstructs
     the specific primitive connecting them.
    */

    static vector<ptrSearchNode> initials_in_path; 

    initials_in_path.clear();
    search->path.clear();
    search->steps = result->steps;

    if (!result->path_found) {
        search->cost_path = -1;
        search->is_solved = false;
        return;
    }

    // Backtrack through Initial Extended Cells
    ptrSearchNode current = result->final_node;
    while (!(current == NULL_Node)) { 
        initials_in_path.push_back(current);
        current = current->parent; 
    }
    reverse(initials_in_path.begin(), initials_in_path.end());

    // Convert sequence of Extended Cells into sequence of Primitives
    // (Algorithm 3: Trajectory Reconstruction from the paper)
    for (size_t t = 0; t + 1 < initials_in_path.size(); t++) {
        ptrVertex v = initials_in_path[t]->vertex;     // Current Initial Cell
        ptrVertex u = initials_in_path[t+1]->vertex;   // Next Initial Cell

        // Retrieve the heading associated with the Initial Configuration of these cells
        int st = search->mesh_info->theta_by_initial_config[v->config_id];
        int ft = search->mesh_info->theta_by_initial_config[u->config_id];
        
        int di = u->i - v->i;
        int dj = u->j - v->j;

        // Solve the local Boundary Value Problem (lookup)
        int prim_id = search->control_set->get_prim_between_states(st, di, dj, ft);
        
        rassert(prim_id != -1, "Mesh reconstruction failed: No primitive connects these states!");
        search->path.push_back(prim_id);
    }
    
    search->cost_path = result->final_node->g;
    search->is_solved = true;
    
    HEAP->delete_SearchNode(result->final_node);
}

void SearchAlgorithm::solve(Vertex *start, Vertex *finish) {
    /*
     Main execution method.
     Instantiates the appropriate search parameters and runs A*.
    */
    
    // Ensure we are in a clean state (mutual exclusivity check)
    rassert(lattice_params == nullptr && mesh_params == nullptr, "Search context already active!");

    double t0;
    
    // Log the current task
    si = start->i; sj = start->j; st = start->theta;
    fi = finish->i; fj = finish->j; ft = finish->theta;
    
    ResultSearch result;  

    if (algo == ALGO_LATTICE_ASTAR) {
        // --- State Lattice A* ---
        lattice_params = new StateLatticeParams(start, finish, map, control_set, w, R, A);

        t0 = (double)clock();
        result = AstarSearch(lattice_params, lazy);
        search_time = ((double)clock() - t0) / CLOCKS_PER_SEC;

        t0 = (double)clock();
        reconstruct_path_lattice(&result, this);
        trajectory_reconstruction_time = ((double)clock() - t0) / CLOCKS_PER_SEC;

    } else if (algo == ALGO_MESH_ASTAR) {
        // --- MeshA* ---
        mesh_params = new MeshGraphParams(start, finish, map, control_set, mesh_info, w, R, A);

        t0 = (double)clock();
        result = AstarSearch(mesh_params, false); // Note: Lazy check is handled inside MeshGraphParams logic if needed
        search_time = ((double)clock() - t0) / CLOCKS_PER_SEC;

        t0 = (double)clock();
        reconstruct_path_mesh(&result, this);
        trajectory_reconstruction_time = ((double)clock() - t0) / CLOCKS_PER_SEC;

    } else {
        rassert(0, "Unknown algorithm identifier!");
        return;
    }

    total_runtime = search_time + trajectory_reconstruction_time;

    // Cleanup and statistics collection
    checked_cells = 0;
    if (lattice_params != nullptr) {
        checked_cells += lattice_params->checked_cells; // Typo fixed: cheked -> checked (assuming fix in header)
        delete lattice_params;
        lattice_params = nullptr;
    }
    if (mesh_params != nullptr) {
        checked_cells += mesh_params->checked_cells;
        delete mesh_params;
        mesh_params = nullptr;
    }
}

void SearchAlgorithm::dump_result_of_search(string filename) {
    /*
     Writes the search results to a log file.
    */

    ofstream file(filename);

    if (algo == ALGO_LATTICE_ASTAR)
        file << "=== Results: Lattice-Based A* ===" << endl;
    else if (algo == ALGO_MESH_ASTAR)
        file << "=== Results: MeshA* ===" << endl;
    
    file << "Algorithm Instance: " << name << endl;
    file << "Parameters:" << endl;
    file << "  Control Set: " << control_set->filename << endl;
    file << "  Map: " << map->filename << endl;
    file << "  Goal Tolerance: R = " << R << ", A = " << A << " (discrete headings)" << endl;
    file << "  Heuristic Weight (w): " << w << endl;
    file << "  Lazy Evaluation: " << (lazy ? "Enabled" : "Disabled") << endl;
    
    file << "Problem Instance:" << endl;
    file << "  Start:  (" << si << ", " << sj << ", " << st << ")" << endl;
    file << "  Finish: (" << fi << ", " << fj << ", " << ft << ")" << endl;
    
    file << "Metrics:" << endl;
    file << "  Solved: " << (is_solved ? "Yes" : "No") << endl;
    file << "  Expansions (Steps): " << steps << endl;
    file << "  Collision Checks: " << checked_cells << endl;
    file << "  Search Time (s): " << search_time << endl;
    file << "  Reconstruction Time (s): " << trajectory_reconstruction_time << endl;
    file << "  Total Time (s): " << total_runtime << endl;
    file << "  Path Cost: " << cost_path << endl;
    file << "  Path Length (m): " << metric_length(path, control_set) << endl;
    file << "  AOL Metric: " << metric_AOL(path, control_set) << endl;
    
    file << "--------------------" << endl;
    file << "Primitive Sequence (IDs):" << endl;
    for (int id : path) {
        file << id << " ";
    }
    file << endl;
    
    file.close();
}

SearchAlgorithm::~SearchAlgorithm() {
    if (lattice_params != nullptr) delete lattice_params;
    if (mesh_params != nullptr) delete mesh_params;
}
