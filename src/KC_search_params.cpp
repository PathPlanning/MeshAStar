#include <cmath>
#include <iostream>
#include <algorithm>
#include <tuple>

#include "KC_search_params.hpp"
#include "rassert.hpp"
#include "common.hpp"

extern MyHEAP *HEAP;
static const float EPS = 0.01f;

// -----------------------------------------------------------------------------
// Geometric Helper Functions
// -----------------------------------------------------------------------------

static inline int euclid_dist_sq(int i1, int j1, int i2, int j2) {
    return (i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2);
}

static inline int angle_dist(int theta1, int theta2) {
    /*
     Computes the shortest distance between two headings in discrete steps,
     accounting for the circularity of angles (modulo arithmetic).
    */
    rassert(0 <= theta1 && theta1 < NUM_HEADINGS && 0 <= theta2 && theta2 < NUM_HEADINGS, "Invalid heading index!");
    int d = abs(theta1 - theta2);
    return min(d, NUM_HEADINGS - d);
}

// -----------------------------------------------------------------------------
// Collision Checking Helpers
// -----------------------------------------------------------------------------

static inline bool check_cell(Map *task_map, int i, int j, int *stats_cnt) {
    /*
     Checks if a grid cell (i, j) is traversable.
     
     Performance Note:
     Includes a pointer to an integer counter 'stats_cnt'.
     - If stats_cnt is NOT NULL, it implies this check is critical for validity
       (e.g., edge validation), and we increment the collision check counter.
     - If stats_cnt IS NULL, this is a speculative check (e.g., heuristic lookahead
       or pruning), and we do not count it towards the official metrics to avoid
       skewing the comparison with standard baselines.
    */

    if (stats_cnt != nullptr) {
        (*stats_cnt)++;
        // Note: Simulated sensor delay can be inserted here for benchmarks.
    }
    
    return task_map->in_bounds(i, j) && task_map->traversable(i, j);
}

static inline bool check_primitive(Map *task_map, int i, int j, int start_idx, Primitive *prim, int &stats_cnt) {
    /*
     Checks if a motion primitive 'prim', when applied at (i, j), is collision-free.
     'start_idx' allows skipping the first few cells of the trace (optimization).
    */

    for (size_t k = start_idx; k < prim->swept_cells_i.size(); k++) {
        // Compute absolute coordinates of the k-th cell in the collision trace
        int i_coll = i + prim->swept_cells_i[k];
        int j_coll = j + prim->swept_cells_j[k];
        
        if (!check_cell(task_map, i_coll, j_coll, &stats_cnt))
            return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// StateLatticeParams Implementation
// -----------------------------------------------------------------------------

StateLatticeParams::StateLatticeParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, 
                                        float w, float R, int A) {
    this->task_map = map;
    this->start = start;
    this->finish = finish;
    this->R = R + EPS; // Add epsilon to handle floating point inaccuracies
    this->A = A;
    this->w = w;
    
    rassert(R >= 0.0 && A >= 0 && w >= 0.0 && A < NUM_HEADINGS, "Invalid search parameters!");

    this->control_set = control_set;
    this->ast = new SearchTree();
}

ptrVertex StateLatticeParams::get_start_vertex() {
    return HEAP->new_Vertex(start->i, start->j, start->theta);
}

bool StateLatticeParams::is_goal(ptrVertex v) {
    /*
     Goal Condition:
     A state is a goal if it is within Euclidean distance R and angular tolerance A
     from the exact target state.
    */
    float dist_sq = (float)euclid_dist_sq(v->i, v->j, finish->i, finish->j);
    int a_dist = angle_dist(v->theta, finish->theta);
    return (dist_sq <= R * R) && (a_dist <= A);    
}

void StateLatticeParams::get_successors(ptrSearchNode current, vector<ptrSearchNode> &list, bool lazy) {
    /*
     Generates successors for a state (x, y, theta) in the State Lattice graph.
     Iterates over all motion primitives available for the current heading 'theta'.
    */

    ptrVertex v = current->vertex;
    
    for (int prim_id : control_set->get_prims_by_heading(v->theta)) {
        Primitive *prim = control_set->list_all_prims[prim_id];
        
        // Target state coordinates
        int next_i = v->i + prim->goal->i;
        int next_j = v->j + prim->goal->j;
        int next_theta = prim->goal->theta;

        // Validity Checks:
        // 1. Map bounds (target state must be inside).
        // 2. Collision-free motion (unless Lazy evaluation is active).
        // 3. Not previously expanded (Duplicate detection).
        
        bool collision_free = lazy || check_primitive(task_map, v->i, v->j, 0, prim, checked_cells);
        
        if (collision_free && 
            task_map->in_bounds(next_i, next_j) &&
            !ast->was_expanded(next_i, next_j, next_theta)) 
        {
            ptrVertex u = HEAP->new_Vertex(next_i, next_j, next_theta);
            ptrSearchNode new_node = HEAP->new_SearchNode(u);
            new_node->parent = current;
            new_node->g = current->g + (float)control_set->cost(prim_id);
            new_node->f = new_node->g + heuristic(new_node);

            list.push_back(new_node);
        }
    }
}

float StateLatticeParams::heuristic(ptrSearchNode node) {
    /*
     Euclidean distance heuristic.
     Admissible and consistent for 2D geometric costs.
    */
    return w * sqrtf((float)euclid_dist_sq(node->vertex->i, node->vertex->j, finish->i, finish->j));
}

bool StateLatticeParams::lazy_check_collision(ptrSearchNode node) {
    /*
     Lazy Validation:
     Verifies the edge connecting 'parent' to 'node' is collision-free.
     Called when extracting 'node' from OPEN.
    */

    ptrSearchNode parent = node->parent;
    if (parent == NULL_Node) return true; // Start node is always valid
    
    // --- RECONSTRUCT PRIMITIVE ID ---
    int st = parent->vertex->theta;
    int ft = node->vertex->theta;
    int di = node->vertex->i - parent->vertex->i;
    int dj = node->vertex->j - parent->vertex->j;

    // Lookup the primitive that corresponds to this motion
    int prim_id = control_set->get_prim_between_states(st, di, dj, ft);
    
    // This should technically never fail if the graph logic is correct, 
    // but rassert guards against logic errors.
    rassert(prim_id >= 0, "Lazy check error: No primitive connects these states!");
    
    Primitive *prim = control_set->list_all_prims[prim_id];
    
    return check_primitive(task_map, parent->vertex->i, parent->vertex->j, 0, prim, checked_cells);
}

StateLatticeParams::~StateLatticeParams() {
    delete ast;
}

// -----------------------------------------------------------------------------
// MeshGraphParams Implementation
// -----------------------------------------------------------------------------

MeshGraphParams::MeshGraphParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, MeshInfo *mesh_info,
                                   float w, float R, int A) {
    this->task_map = map;
    this->start = start;
    this->finish = finish;
    this->R = R + EPS;
    this->A = A;
    this->w = w;
    
    rassert(R >= 0.0 && A >= 0 && w >= 0.0 && A < NUM_HEADINGS, "Invalid Mesh parameters!");

    this->control_set = control_set;
    this->mesh_info = mesh_info;
    this->ast = new SearchTree();
}

ptrVertex MeshGraphParams::get_start_vertex() {
    /*
     According to the Main Theorem (Equivalence), if the search on the lattice starts at s=(i, j, theta),
     the search on the mesh graph must start at u = (i, j, Psi_theta), where Psi_theta is the 
     Initial Configuration associated with heading theta.
    */

    int start_config = mesh_info->initial_config_by_theta[start->theta];
    // Create the initial extended cell corresponding to the start configuration.
    ptrVertex v = HEAP->new_Vertex(start->i, start->j, start_config, true); 
    return v;
}

bool MeshGraphParams::is_goal(ptrVertex v) {
    /*
     Checks if extended cell 'v' satisfies the goal condition.
     Per the theorem, only Initial Extended Cells (corresponding to discrete lattice states) can be goals.
     
     1. Recover theta from the configuration Psi (if Psi is an Initial Configuration).
     2. If successful, check geometric bounds (R, A) against the target state.
    */

    int theta = mesh_info->theta_by_initial_config[v->config_id];
    
    if (theta < 0) return false; // Not an Initial Extended Cell -> Cannot be a goal state.

    // Proceed with standard goal check using the recovered theta
    float dist_sq = (float)euclid_dist_sq(v->i, v->j, finish->i, finish->j);
    int a_dist = angle_dist(theta, finish->theta);
    
    return (dist_sq <= R * R) && (a_dist <= A);    
}

static inline void mesh_set_parent(ptrSearchNode prev, ptrSearchNode next, MeshInfo *mesh_info) {
    /*
     Sets the parent pointer for a new node.
     Optimization: To save memory and simplify reconstruction, we only need to store pointers 
     to the previous *Initial Extended Cell* in the path.
     
     If 'prev' is Initial, it becomes the direct parent.
     If 'prev' is NOT Initial (intermediate node), its parent (which must be Initial) is inherited.
    */

    if (mesh_info->theta_by_initial_config[prev->vertex->config_id] >= 0) {
        // Prev is Initial
        next->parent = prev;
    } else {
        // Prev is Intermediate -> Skip it, link to its parent
        next->parent = prev->parent;
    }

    // Memory Optimization: Non-initial cells are not needed for reconstruction once processed.
    if (mesh_info->theta_by_initial_config[next->vertex->config_id] < 0) {
        next->mem_after_closed = false;
    }
}

static inline bool mesh_should_prune(Map* task_map, int i, int j, int config_id, MeshGraphParams *params) {
    /*
     Advanced Pruning Logic (The "Finals" check).
     
     We check if the current extended cell u=(i, j, Psi) is redundant.
     According to the paper, from a non-initial cell u, we can reach a set of Initial Extended Cells:
        Finals(u) = { (u_k, c_k) }
     
     If ALL reachable initial cells u_k in Finals(u) are either:
       1. Blocked by obstacles, OR
       2. Already Expanded (in CLOSED)
     Then there is no need to expand u, because any optimal path through u would eventually 
     pass through one of u_k, and we have already found a path to them (or they are blocked).
    */
    
    // 1. Basic check: If u itself is already expanded, prune.
    if (params->ast->was_expanded(i, j, config_id)) return true;

    // 2. If u is Initial, we generally must expand it (unless already closed above).
    if (params->mesh_info->theta_by_initial_config[config_id] >= 0) return false;

    // 3. For Non-Initial cells, check reachability of Finals(u).
    for (auto p : params->mesh_info->primitives_in_config[config_id]) {
        // Unpack properties of the primitive endpoint (potential u_k)
        int next_theta = get<0>(p);
        int di = get<1>(p);
        int dj = get<2>(p);
        
        int next_i = i + di;
        int next_j = j + dj;
        int next_config = params->mesh_info->initial_config_by_theta[next_theta];
        
        // If this endpoint u_k is VALID (free) and NOT EXPANDED, then u is potentially useful.
        // We perform a "speculative" check (stats_cnt = NULL).
        if (check_cell(task_map, next_i, next_j, nullptr) && 
            (!params->ast->was_expanded(next_i, next_j, next_config))) {
            return false; // Found at least one useful successor -> Do NOT prune.
        }
    }
    
    // All endpoints are either blocked or closed -> Prune this cell.
    return true;
}

void MeshGraphParams::get_successors(ptrSearchNode current, vector<ptrSearchNode> &list, bool lazy) {
    /*
     Generates successors in the Mesh Graph.
     Note: Lazy evaluation is NOT used in MeshA* due to the complexity of extended cells.
    */
    (void)lazy;
    rassert(lazy == false, "Lazy evaluation not supported in MeshA*!");

    ptrVertex v = current->vertex;
    
    // --- Optimization: Direct Jump to Unique Successor ---
    // If a non-initial cell has exactly ONE valid reachable Initial Cell (u_k in Finals(u)),
    // we can skip intermediate steps and jump directly to it.
    if (!current->mem_after_closed) { // "mem_after_closed=false" implies non-initial
        int valid_endpoints_count = 0;
        tuple<int, int, int, int, int> unique_succ_data; 
        
        // Iterate over Finals(v)
        for (auto p : mesh_info->primitives_in_config[v->config_id]) {
            int next_theta = get<0>(p);
            int di = get<1>(p);
            int dj = get<2>(p);
            int k_trace = get<3>(p); // index in collision trace
            int prim_id = get<4>(p);

            int next_i = v->i + di;
            int next_j = v->j + dj;
            int next_config = mesh_info->initial_config_by_theta[next_theta];
            
            if (check_cell(task_map, next_i, next_j, nullptr) && 
                (!ast->was_expanded(next_i, next_j, next_config))) {
                
                valid_endpoints_count++;
                unique_succ_data = make_tuple(next_i, next_j, next_config, prim_id, k_trace);
            }
            if (valid_endpoints_count > 1) break;
        }

        if (valid_endpoints_count == 0) return; // Dead end -> No successors.
        
        if (valid_endpoints_count == 1) {
            // Optimization: Generate the single Initial Successor directly.
            int i = get<0>(unique_succ_data);
            int j = get<1>(unique_succ_data);
            int next_config = get<2>(unique_succ_data);
            int prim_id = get<3>(unique_succ_data);
            int k_current = get<4>(unique_succ_data);
            
            Primitive *prim = control_set->list_all_prims[prim_id];
            
            // Validate the specific primitive segment connecting v to this endpoint
            // (Relative shift from v to endpoint must be validated)
            int rel_di = i - prim->goal->i;
            int rel_dj = j - prim->goal->j;

            if (check_primitive(task_map, rel_di, rel_dj, k_current, prim, checked_cells)) {
                ptrVertex u = HEAP->new_Vertex(i, j, next_config, true);
                ptrSearchNode new_node = HEAP->new_SearchNode(u);
                
                mesh_set_parent(current, new_node, mesh_info);
                
                new_node->g = current->g + (float)control_set->cost(prim_id); // Full primitive cost
                new_node->f = new_node->g + heuristic(new_node);
                
                list.push_back(new_node);
            }
            return; // Done.
        }
    }
    
    // --- Standard Successor Generation (Algorithm 2) ---
    // Used when the optimization above doesn't apply.
    for (auto triple : mesh_info->successors[v->config_id]) {
        int di = get<0>(triple);
        int dj = get<1>(triple);
        int next_config = get<2>(triple);
        int connecting_prim_id = get<3>(triple); // -1 if ambiguous
        
        int i = v->i + di;
        int j = v->j + dj;

        // Determine transition cost
        // If connecting_prim_id >= 0, it means we are transitioning into an Initial Cell,
        // so the transition corresponds to completing a specific primitive.
        float cost = (connecting_prim_id >= 0) ? (float)control_set->cost(connecting_prim_id) : 0.0f;

        if (check_cell(task_map, i, j, &checked_cells) &&
            !mesh_should_prune(task_map, i, j, next_config, this)) {
            
            ptrVertex u = HEAP->new_Vertex(i, j, next_config, true);
            ptrSearchNode new_node = HEAP->new_SearchNode(u);
            
            mesh_set_parent(current, new_node, mesh_info);
            
            new_node->g = current->g + cost;
            new_node->f = new_node->g + heuristic(new_node);

            list.push_back(new_node);
        }
    }
}

static inline float base_heuristic(ptrSearchNode node, MeshGraphParams *params) {
    /*
     Heuristic Function for MeshA* (Definition from paper).
     
     Case 1: u is Initial (Psi = Psi_theta).
        h(u) = h_lattice(i, j, theta).
        
     Case 2: u is Non-Initial.
        h(u) = min { h(u_k) + c_k } over all (u_k, c_k) in Finals(u).
        This essentially looks ahead to all possible ways the current partial motions can complete.
    */

    // Case 1: Initial Extended Cell
    if (params->mesh_info->theta_by_initial_config[node->vertex->config_id] >= 0) {
        return sqrtf((float)euclid_dist_sq(node->vertex->i, node->vertex->j, params->finish->i, params->finish->j));
    }
    
    // Case 2: Non-Initial Extended Cell
    // We iterate over Finals(u) (precomputed in primitives_in_config)
    float min_h = 1e12; // Infinity

    for (auto p : params->mesh_info->primitives_in_config[node->vertex->config_id]) {
        int next_theta = get<0>(p);
        int di = get<1>(p);
        int dj = get<2>(p);
        int prim_id = get<4>(p);

        int i = node->vertex->i + di;
        int j = node->vertex->j + dj;
        int next_config = params->mesh_info->initial_config_by_theta[next_theta];

        // Note on Admissibility:
        // We use the FULL primitive cost here, even if 'node' is an intermediate step.
        // This is correct because in our mesh graph construction, edge weights for internal 
        // transitions are 0. Thus, 'g(node)' equals 'g(start_of_prim)'.
        // To ensure f(node) >= f(start), the heuristic h(node) must include the 
        // full remaining cost (which equals the full primitive cost in this cost model).
        float cost_to_endpoint = (float)params->control_set->cost(prim_id);
        
        float endpoint_h = 1e9; // Infinity

        // Filter valid endpoints (blocked or closed endpoints are not candidates for the optimal path)
        if (check_cell(params->task_map, i, j, nullptr) && 
            (!params->ast->was_expanded(i, j, next_config))) {
            
            // h(u_k) = Euclidean distance from endpoint to goal
            endpoint_h = sqrtf((float)euclid_dist_sq(i, j, params->finish->i, params->finish->j));
        }

        // Update min: h(u) = min( h(u_k) + cost_k )
        if (cost_to_endpoint + endpoint_h < min_h) {
            min_h = cost_to_endpoint + endpoint_h;
        }
    }
    
    return min_h;
}

float MeshGraphParams::heuristic(ptrSearchNode node) {
    return w * base_heuristic(node, this); 
}

bool MeshGraphParams::lazy_check_collision(ptrSearchNode node) {
    (void)node;
    return true; // Not used
}

MeshGraphParams::~MeshGraphParams() {
    delete ast;
}
