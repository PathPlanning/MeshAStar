#pragma once

// -----------------------------------------------------------------------------
// Global Constants and Dimensions
// -----------------------------------------------------------------------------

// The number of discrete heading values (theta).
// Motion primitives are defined relative to these discretized start headings.
#define NUM_HEADINGS 16 

// Maximum number of distinct primitive configurations (Psi) supported.
// In the context of MeshA*, a configuration is a unique set of motion primitives 
// originating from an extended cell.
// (Increase this value if working with larger control sets requiring more memory).
#define MAX_CONFIGS 1800

// Maximum number of test instances to run per map during benchmarking.
#define MAX_TESTS 10

// Maximum dimensions of the grid map. 
// The environment is represented as a grid where each cell is either traversable or blocked.
#define MAX_MAP_WIDTH 1200
#define MAX_MAP_HEIGHT 1200

// Maximum ID for a motion primitive.
// Primitives are enumerated sequentially during generation/loading.
#define MAX_PRIM_ID 1000
