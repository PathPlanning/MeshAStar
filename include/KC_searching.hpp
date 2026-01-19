#pragma once

#include <vector>
#include <queue>
#include <cstdint> // For uint8_t

#include "KC_heap.hpp"
#include "KC_structs.hpp"

using namespace std;

// -----------------------------------------------------------------------------
// Map Representation
// -----------------------------------------------------------------------------

struct Map {
    /*
     Represents the discrete workspace (grid map).
    */

    string filename;
    
    // 2D grid: cells[x][y] is true if occupied (obstacle), false if free.
    vector<vector<bool>> cells; 
    
    int width, height;

    Map();
    void read_file_to_cells(string file_map, bool obs=true);
    void empty_map(int w, int h);
    bool in_bounds(int i, int j);
    bool traversable(int i, int j);
};

// -----------------------------------------------------------------------------
// Search Structures (A* Components)
// -----------------------------------------------------------------------------

struct SearchNode {
    /*
     A wrapper around a graph Vertex containing A* specific metadata:
     g-value, f-value, and parent pointer.
    */

    ptrVertex vertex;        // The underlying state/cell in the graph.
    float g, f;              // Cost-to-come and total estimated cost.
    ptrSearchNode parent;    // Pointer to the parent node (for path reconstruction).
    
    // Flag indicating if this node must be preserved in memory after being closed.
    // In Lattice planning, all nodes are kept.
    // In MeshA*, primarily Initial Extended Cells are required for reconstruction,
    // allowing for memory optimization (pruning regular successors).
    bool mem_after_closed;  

    SearchNode(ptrVertex v);
};

struct NodeCompare {
    /*
     Comparator for the priority queue (OPEN list).
     Orders nodes such that the node with the lowest f-value is at the top.
    */
    bool operator() (ptrSearchNode const (&n1), ptrSearchNode const (&n2));
};

struct SearchTree {
    /*
     Manages the core data structures for A*: the OPEN list (priority queue)
     and the CLOSED list.
    */

    // The OPEN list using the custom smart pointers.
    priority_queue<ptrSearchNode, vector<ptrSearchNode>, NodeCompare> open;

    // The CLOSED list implemented as a bit-array for O(1) access.
    // It maps a state tuple (x, y, layer) to a visited bit.
    // 'layer' corresponds to 'theta' in Lattice or 'config_id' in MeshA*.
    vector<uint8_t> fast_closed; 

    // Keeps track of expanded nodes to facilitate memory cleanup and path reconstruction.
    vector<ptrSearchNode> expanded_nodes;

    SearchTree();
    
    bool open_is_empty();
    void add_to_open(ptrSearchNode item);
    void add_to_closed(ptrSearchNode item);
    
    // Checks if a state (x, y, layer_index) has already been expanded.
    // layer_index: theta (Lattice) OR config_id (MeshA*).
    bool was_expanded(int i, int j, int layer_index);
    
    ptrSearchNode get_best_node_from_open();
    
    ~SearchTree();
};
