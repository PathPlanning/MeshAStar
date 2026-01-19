#pragma once

#include <stdexcept>

// Uncomment to enable runtime assertions
#define DEBUG 

#ifdef DEBUG
    // Runtime assertion macro.
    // If 'condition' is false, throws a std::runtime_error with the provided 'message'.
    #define rassert(condition, message) \
        if (!(condition)) { \
            throw std::runtime_error(std::string("Assertion failed: ") + message); \
        }
#else
    // If DEBUG is not defined, rassert compiles to a no-op.
    // (void)0 is used to prevent compiler warnings (-Wempty-body) about empty statements or unused values.
    #define rassert(condition, message) ((void)0)
#endif

