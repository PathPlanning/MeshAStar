#pragma once

#include <vector>

// -----------------------------------------------------------------------------
// Memory Pool Definitions
// -----------------------------------------------------------------------------

// Null-pointer equivalents for the custom smart pointers.
#define NULL_Node ptrSearchNode(-1)
#define NULL_Vertex ptrVertex(-1)

// Forward declarations
struct SearchNode;
struct Vertex;

using namespace std;

struct ptrVertex {
    /*
     A "smart pointer" (handle) acting as an index into the pre-allocated Memory Pool (MyHEAP).
     Using indices instead of raw pointers (Vertex*) reduces memory overhead on 64-bit systems
     (4-byte int vs 8-byte ptr) and improves cache locality.
     
     It mimics pointer behavior via operator-> for convenience.
    */

    int ind;  // Index of the Vertex instance within MyHEAP::vertices vector.

    ptrVertex();
    ptrVertex(int i);
    Vertex* operator->() const;      // Dereference operator to access the actual Vertex.
    bool operator==(ptrVertex other);
};

struct ptrSearchNode {
    /*
     Analogous smart pointer for SearchNode instances.
    */

    int ind; // Index within MyHEAP::nodes vector.

    ptrSearchNode();
    ptrSearchNode(int i);
    SearchNode* operator->() const;
    bool operator==(ptrSearchNode other);
};

struct MyHEAP {
    /*
     A Memory Pool allocator designed to minimize dynamic memory allocation overhead 
     (new/delete) during the search. It pre-allocates large blocks of Vertex and 
     SearchNode objects and recycles them.
    */

    vector<Vertex> vertices;      // The pool of Vertex objects.
    vector<SearchNode> nodes;     // The pool of SearchNode objects.

    vector<int> index_free_vertices; // Stack of indices for recycled Vertex objects.
    vector<int> index_free_nodes;    // Stack of indices for recycled SearchNode objects.

    int N;  // Current capacity of the pools.

    MyHEAP();
    
    // Expands the memory pool when running out of free objects.
    void resize(); 
    
    // Helper to retrieve a free index from the stack or trigger a resize.
    int get_ind(vector<int> &index_stack);

    // -------------------------------------------------------------------------
    // Allocation Methods
    // -------------------------------------------------------------------------
    
    // Allocates a Vertex for Lattice-based search (State: x, y, theta).
    ptrVertex new_Vertex(int i, int j, int theta);

    // Allocates a Vertex for MeshA* (Extended Cell: x, y, Psi).
    // Note: 'info' legacy parameter removed.
    ptrVertex new_Vertex(int i, int j, int config_id, bool is_mesh); 
    
    // Allocates a SearchNode wrapping a given Vertex.
    ptrSearchNode new_SearchNode(ptrVertex v);
    
    // -------------------------------------------------------------------------
    // Deallocation Methods
    // -------------------------------------------------------------------------
    
    // Returns the object to the pool (marks index as free).
    void delete_Vertex(ptrVertex state);
    void delete_SearchNode(ptrSearchNode node);
};

extern MyHEAP* HEAP;
