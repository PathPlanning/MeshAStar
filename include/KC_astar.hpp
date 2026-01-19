#pragma once

#include "KC_heap.hpp"

// Global pointer to the memory pool (must be defined in the main source file)
extern MyHEAP* HEAP;

struct ResultSearch {
    /*
     Encapsulates the result of the search algorithm.
    */

    bool path_found;           // True if a valid path to the goal was found.
    int steps;                 // Number of iterations (expansions) performed.
    ptrSearchNode final_node;  // The goal node (start of the back-pointers chain for reconstruction).
    
    ResultSearch() : path_found(false), steps(0), final_node(NULL_Node) {}

    ResultSearch(bool path_found, int steps, ptrSearchNode final_node) 
        : path_found(path_found), steps(steps), final_node(final_node) {}
};

template <typename T>
static inline void add_start_node_to_open(T *params) {
    /*
     Initializes the search by creating the start node and pushing it to the OPEN list.
     
     Template Parameter T:
        A struct (e.g., StateLatticeParams, MeshGraphParams) that provides:
        - get_start_vertex()
        - heuristic(node)
        - ast (pointer to SearchTree)
    */

    ptrSearchNode start_node = HEAP->new_SearchNode(params->get_start_vertex());
    start_node->g = 0.0f; 
    start_node->f = params->heuristic(start_node);
    
    params->ast->add_to_open(start_node);
}

template <typename T>
static inline ptrSearchNode StepAstar(T *params, bool lazy = false) {
    /*
     Performs a single iteration (step) of the A* algorithm:
     1. Pops the best node from OPEN.
     2. Checks validity (if Lazy evaluation is enabled).
     3. Checks if Goal is reached.
     4. Expands the node (generates successors and adds to OPEN).

     Returns:
        - The goal node if the target is reached.
        - NULL_Node otherwise (search continues).
    */

    // THREAD-SAFETY WARNING:
    // 'static' is used here for performance optimization (avoiding vector reallocation).
    // This makes the function NON-THREAD-SAFE. 
    // Moreover, the global instance 'extern MyHEAP *HEAP' creates a shared state across the application.
    static vector<ptrSearchNode> succ_list; // Static buffer to avoid reallocation overhead.

    ptrSearchNode current;
    
    while (true) {
        current = params->ast->get_best_node_from_open();

        // If OPEN is empty, the search failed.
        if (current == NULL_Node) {
            return NULL_Node; 
        }
        
        // Lazy Node Evaluation:
        // If 'lazy' is true, collision checking was deferred during successor generation.
        // We must check validity now, effectively "lazily" validating the edge.
        if (!lazy || params->lazy_check_collision(current)) {
            break; // Node is valid.
        } else {
            // Node is invalid (collision detected upon extraction). Discard and retry.
            HEAP->delete_SearchNode(current); 
        }
    }

    ptrVertex v = current->vertex;
    
    // Check if the current node satisfies the goal condition.
    if (params->is_goal(v)) {
        return current;
    }

    // Expansion Phase
    succ_list.clear();
    params->get_successors(current, succ_list, lazy);
    
    for (ptrSearchNode new_node : succ_list) {
        params->ast->add_to_open(new_node);
    }
    
    // Mark current node as Expanded (add to CLOSED).
    params->ast->add_to_closed(current);
    
    return NULL_Node; // Goal not yet reached in this step.
}

template <typename T>
ResultSearch AstarSearch(T *params, bool lazy = false) {
    /*
     Executes the complete A* search loop.
     
     Args:
        params: Pointer to the search parameters (context).
        lazy: Flag to enable Lazy A* (deferred collision checking).
    */

    add_start_node_to_open(params);
    
    int step_count = 0; 
    
    while (!params->ast->open_is_empty()) {
        step_count++;
        
        ptrSearchNode result_node = StepAstar(params, lazy);
        
        if (!(result_node == NULL_Node)) {
            // Path found.
            return ResultSearch(true, step_count, result_node);
        }
    }

    // OPEN list exhausted, no path found.
    return ResultSearch(false, step_count, NULL_Node);
}
