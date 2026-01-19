#include <fstream>
#include <sstream>
#include <cstring> // For memset
#include <iostream>
#include <algorithm>

#include "KC_searching.hpp"
#include "KC_heap.hpp"
#include "common.hpp"
#include "rassert.hpp"

// Global memory pool
extern MyHEAP* HEAP;

// -----------------------------------------------------------------------------
// Map Implementation
// -----------------------------------------------------------------------------

Map::Map() {
    width = 0;
    height = 0;
}

void Map::read_file_to_cells(string _file, bool obs) {
    /*
     Parses a grid map from a file.
     Supports standard MovingAI benchmark format (.map).
     
     Format structure:
       type octile
       height Y
       width X
       map
       ... (grid rows) ...
     
     Args:
       _file: Path to the map file.
       obs: If true, obstacles are marked as blocked (1). If false, map is treated as empty.
    */

    filename = _file + (obs ? " (with obstacles)" : " (empty)");
    ifstream file(_file);
    rassert(file.is_open(), "Error: Cannot open map file: " + _file);
    
    string line;
    
    // Skip header lines (MovingAI format usually has 4 header lines)
    // We look for the "map" keyword or simply skip fixed lines if standard is guaranteed.
    // Robust parsing strategy:
    while (getline(file, line)) {
        if (line.find("height") != string::npos || 
            line.find("width") != string::npos || 
            line.find("type") != string::npos) {
            continue;
        }
        if (line.find("map") != string::npos) {
            break; // Header ends, data follows
        }
    }

    // Read grid data
    while (getline(file, line)) {
        if (line.empty()) continue;

        vector<bool> cells_row;
        // Pre-allocate to avoid reallocations
        cells_row.reserve(line.length());

        for (char c : line) {
            bool is_blocked = false;
            
            // MovingAI standard characters:
            // . = free
            // G = free (swamp/grass, treated as free here)
            // @ = obstacle (out of bounds)
            // O = obstacle (out of bounds)
            // T = tree (obstacle)
            // S = swamp (obstacle)
            // W = water (obstacle)
            
            // Simplified check based on common formats:
            if (c == '.' || c == 'G' || c == 'S') {
                is_blocked = false;
            } else if (c == '@' || c == 'O' || c == 'T' || c == 'W' || c == '#') {
                is_blocked = true;
            } else if (c == '\r' || c == '\n') {
                continue; // Ignore line endings
            } else {
                // Unknown char, treat as obstacle or throw error? 
                // For safety, assume obstacle if obs=true, else warning.
                 rassert(false, string("Unknown map symbol: ") + c);
            }

            if (!obs) is_blocked = false; // Force empty map mode
            
            cells_row.push_back(is_blocked);
        }

        if (!cells.empty()) {
             // Ensure rectangular map
             rassert(cells.back().size() == cells_row.size(), "Map is not rectangular!");
        }

        cells.push_back(cells_row);
    }
    file.close();

    height = (int)cells.size();
    if (height > 0) width = (int)cells[0].size();

    rassert(height <= MAX_MAP_HEIGHT && width <= MAX_MAP_WIDTH,
            "Map dimensions exceed MAX_MAP_HEIGHT/WIDTH! Increase constants in common.hpp.");
}

void Map::empty_map(int w, int h) {
    /*
     Generates an empty (obstacle-free) map of specified dimensions.
    */
    filename = "<empty map " + to_string(h) + "x" + to_string(w) + ">";
    
    rassert(h <= MAX_MAP_HEIGHT && w <= MAX_MAP_WIDTH, "Map dimensions too large!");
    
    cells.assign(h, vector<bool>(w, false)); // 0 = free
    width = w;
    height = h;
}

bool Map::in_bounds(int i, int j) {
    return (i >= 0 && i < height) && (j >= 0 && j < width);
}

bool Map::traversable(int i, int j) {
    // cells[i][j] == 1 means blocked, so traversable if == 0 (false)
    return !cells[i][j];
}

// -----------------------------------------------------------------------------
// Search Structures Implementation
// -----------------------------------------------------------------------------

SearchNode::SearchNode(ptrVertex v) {
    vertex = v;
    parent = NULL_Node;
    
    // Default: Keep nodes in memory (State Lattice behavior).
    // MeshA* overrides this for intermediate nodes.
    mem_after_closed = true; 
}

bool NodeCompare::operator() (ptrSearchNode const (&n1), ptrSearchNode const (&n2)) {
    /*
     Comparator for min-priority queue.
     Returns true if n1 is "less desirable" than n2 (has higher f-value).
     This places the node with the LOWEST f-value at the top().
    */
    return n1->f > n2->f;  
}

// -----------------------------------------------------------------------------
// SearchTree (OPEN/CLOSED) Implementation
// -----------------------------------------------------------------------------

SearchTree::SearchTree() {
    /*
     Initializes the Search Tree.
     We allocate a bitset large enough to cover the entire state space:
     State Space Size = Width * Height * Max_Layers
     Layer = Theta (Lattice) or Configuration (Mesh).
     
     Since MAX_CONFIGS >= NUM_HEADINGS, using MAX_CONFIGS ensures coverage for both.
    */
    
    // Calculate total bits needed
    uint64_t total_states = (uint64_t)MAX_MAP_HEIGHT * MAX_MAP_WIDTH * MAX_CONFIGS;
    
    // Allocate bytes (1 byte = 8 bits) + padding
    fast_closed.assign((total_states / 8) + 1, 0); 
}

static inline size_t index_in_closed(int i, int j, int layer_index) {
    /*
     Computes a unique linear index for a state (x, y, layer).
     This index maps directly to a specific bit in the 'fast_closed' bitset.
     
     Key Insight:
     - For Lattice A*, 'layer_index' is 'theta' (heading).
     - For MeshA*, 'layer_index' is 'config_id' (Configuration Psi).
     
     Since Vertex uses a union for theta/config_id, they occupy the same memory slot
     and can be treated uniformly here.
    */

    rassert(i >= 0 && i < MAX_MAP_HEIGHT && 
            j >= 0 && j < MAX_MAP_WIDTH && 
            layer_index >= 0 && layer_index < MAX_CONFIGS, 
            "State index out of bounds!"); 

    // Linear mapping:  layer * (Area) + row * (Width) + col
    uint64_t map_area = (uint64_t)MAX_MAP_HEIGHT * MAX_MAP_WIDTH;
    uint64_t width    = (uint64_t)MAX_MAP_WIDTH;
    
    // Using uint64_t to prevent overflow during calculation
    size_t num = (size_t)(layer_index * map_area + i * width + j);
    return num;
}

bool SearchTree::open_is_empty() {
    return open.empty();
}

void SearchTree::add_to_open(ptrSearchNode item) {
    open.push(item);
}

void SearchTree::add_to_closed(ptrSearchNode item) {
    /*
     Marks the node as Expanded (Visited) in the CLOSED set.
     
     1. Set the corresponding bit in 'fast_closed'.
     2. Manage memory lifecycle:
        - If 'mem_after_closed' is true, keep the node for path reconstruction.
        - If false (MeshA* intermediate node optimization), delete it immediately to free pool slots.
    */

    ptrVertex v = item->vertex;
    
    // Access the union field generically (using 'config_id' which aliases 'theta')
    // This works because we unified the types in KC_structs.hpp
    int layer_index = v->config_id; 

    size_t num = index_in_closed(v->i, v->j, layer_index);
    
    // Set the bit (OR operation)
    fast_closed[num / 8] |= (1 << (num % 8)); 

    if (item->mem_after_closed) {
        expanded_nodes.push_back(item);
    } else {
        // Optimization: Recycle this node immediately
        HEAP->delete_SearchNode(item);   
    }
}    

bool SearchTree::was_expanded(int i, int j, int layer_index) {
    /*
     Checks if the state (i, j, layer_index) is in the CLOSED set.
     O(1) complexity via bitmask lookup.
    */
    
    // Check bounds before access (debug only)
    #ifdef DEBUG
    rassert(i >= 0 && i < MAX_MAP_HEIGHT &&
            j >= 0 && j < MAX_MAP_WIDTH &&
            layer_index >= 0 && layer_index < MAX_CONFIGS, "Invalid indices in was_expanded");
    #endif
    
    size_t num = index_in_closed(i, j, layer_index);
    
    // Check if bit is set (AND operation)
    return (fast_closed[num / 8] & (1 << (num % 8))) != 0;
}

ptrSearchNode SearchTree::get_best_node_from_open() {
    /*
     Retrieves the node with the lowest f-value from OPEN.
     Lazy Removal: Nodes that have already been expanded (duplicates) are
     discarded upon extraction.
    */

    while (!open_is_empty()) {
        ptrSearchNode best_node = open.top();
        open.pop();
        
        // Check if this state is already in CLOSED
        // (This handles duplicate states added to OPEN with higher costs)
        // Access union field via config_id (aliases theta)
        int layer_index = best_node->vertex->config_id;
        
        if (was_expanded(best_node->vertex->i, best_node->vertex->j, layer_index)) {
            // Duplicate found -> Discard and recycle
            HEAP->delete_SearchNode(best_node); 
        } else {
            // Valid unexpanded node -> Return it
            return best_node; 
        }
    }
    
    return NULL_Node; // OPEN is exhausted
}

SearchTree::~SearchTree() {
    /*
     Destructor: Cleans up all allocated SearchNodes.
    */

    // 1. Drain OPEN list
    while (!open.empty()) {
        ptrSearchNode node = open.top();
        HEAP->delete_SearchNode(node);
        open.pop();
    }
    
    // 2. Clear EXPANDED nodes (CLOSED list contents)
    for (ptrSearchNode node : expanded_nodes) {
        HEAP->delete_SearchNode(node);
    }
    expanded_nodes.clear();

    // 3. Reset bitset (optional, as vector destructor handles memory)
    // fast_closed is a std::vector, so it cleans itself up.
}
