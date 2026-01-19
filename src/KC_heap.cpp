#include "KC_heap.hpp"
#include "KC_structs.hpp"
#include "KC_searching.hpp"

// -----------------------------------------------------------------------------
// Smart Pointer Implementation (ptrVertex)
// -----------------------------------------------------------------------------

ptrVertex::ptrVertex() {
    ind = -1; // Represents NULL/Invalid pointer
}

ptrVertex::ptrVertex(int i) {
    ind = i;
}

Vertex* ptrVertex::operator->() const {
    // Direct access to the memory pool's vector.
    // This provides pointer-like semantics: v->i instead of HEAP->vertices[v.ind].i
    return &(HEAP->vertices[ind]);
}

bool ptrVertex::operator==(ptrVertex other) {
    return ind == other.ind;
}

// -----------------------------------------------------------------------------
// Smart Pointer Implementation (ptrSearchNode)
// -----------------------------------------------------------------------------

ptrSearchNode::ptrSearchNode() {
    ind = -1;
}

ptrSearchNode::ptrSearchNode(int i) {
    ind = i;
}

SearchNode* ptrSearchNode::operator->() const {
    return &(HEAP->nodes[ind]);
}

bool ptrSearchNode::operator==(ptrSearchNode other) {
    return ind == other.ind;
}

// -----------------------------------------------------------------------------
// Memory Pool (MyHEAP) Implementation
// -----------------------------------------------------------------------------

MyHEAP::MyHEAP() {
    /*
     Initialize the memory pool with a pre-allocated capacity.
     We use a sufficiently large initial size (N) to avoid frequent reallocations
     during the early stages of search.
    */
    N = 100000;
    
    // Pre-allocate memory for vertices and nodes.
    // Using default constructors for initialization.
    // Note: 'vertices' was renamed from 'vertexs' for grammatical correctness.
    vertices.assign(N, Vertex(0, 0, 0));
    nodes.assign(N, SearchNode(NULL_Vertex));

    // Initialize the stacks of free indices.
    // Initially, all indices [0, N-1] are free.
    for (int i = 0; i < N; i++) {
        index_free_nodes.push_back(i);
        index_free_vertices.push_back(i);
    }
}

void MyHEAP::resize() {
    /*
     Doubles the capacity of the memory pool when it becomes full.
     This ensures amortized constant time overhead for allocations.
    */
    
    // Resize vectors, filling new slots with default objects.
    vertices.resize(2 * N, Vertex(0, 0, 0));
    nodes.resize(2 * N, SearchNode(NULL_Vertex));         

    // Add the newly created indices [N, 2N-1] to the free stacks.
    for (int i = N; i < 2 * N; i++) {
        index_free_nodes.push_back(i);
        index_free_vertices.push_back(i);
    }

    N *= 2;
}

int MyHEAP::get_ind(vector<int> &index_stack) {
    /*
     Helper: Retrieves a free index from the specified stack.
     If the stack is empty, triggers a resize of the entire pool.
    */
    
    if (index_stack.empty()) {
        resize();
        // After resize, index_stack will be populated with N new indices.
    }

    int ind = index_stack.back();
    index_stack.pop_back();
    return ind;
}

// -----------------------------------------------------------------------------
// Allocation Wrappers
// -----------------------------------------------------------------------------

ptrVertex MyHEAP::new_Vertex(int i, int j, int theta) {
    /*
     Allocates a Vertex for Lattice-based planning.
     Reuses an existing object in the pool to avoid 'new' overhead.
    */
    int ind = get_ind(index_free_vertices);
    
    // In-place assignment (move semantics not needed for POD types)
    vertices[ind] = Vertex(i, j, theta);
    
    return ptrVertex(ind);
}

ptrVertex MyHEAP::new_Vertex(int i, int j, int config_id, bool is_mesh) {
    /*
     Allocates a Vertex for MeshA* (Extended Cell).
     Arguments:
       config_id: The Configuration ID (Psi).
       is_mesh: Dummy parameter to distinguish signature from Lattice vertex.
    */
    (void)is_mesh; // Unused, strictly for function overloading
    
    int ind = get_ind(index_free_vertices);
    vertices[ind] = Vertex(i, j, config_id, true); // Calls the Mesh-specific Vertex constructor
    return ptrVertex(ind);
}

ptrSearchNode MyHEAP::new_SearchNode(ptrVertex v) {
    /*
     Allocates a SearchNode wrapping the given Vertex.
    */
    int ind = get_ind(index_free_nodes);
    nodes[ind] = SearchNode(v);
    return ptrSearchNode(ind);
}

// -----------------------------------------------------------------------------
// Deallocation Wrappers
// -----------------------------------------------------------------------------

void MyHEAP::delete_Vertex(ptrVertex v) {
    /*
     Returns the Vertex index to the free stack.
     Does not actually free memory, just marks it for reuse.
    */
    index_free_vertices.push_back(v.ind);
}

void MyHEAP::delete_SearchNode(ptrSearchNode node) {
    /*
     Deletes a SearchNode AND its underlying Vertex.
     This assumes strict ownership: the Vertex belongs solely to this Node.
    */
    delete_Vertex(node->vertex);
    index_free_nodes.push_back(node.ind);
}
