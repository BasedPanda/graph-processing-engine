#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace graph_engine {

// Use 32-bit unsigned integers for vertex and edge IDs
using vertex_id_t = uint32_t;
using edge_id_t = uint32_t;
using weight_t = float;

// Constants for invalid IDs
constexpr vertex_id_t INVALID_VERTEX_ID = std::numeric_limits<vertex_id_t>::max();
constexpr edge_id_t INVALID_EDGE_ID = std::numeric_limits<edge_id_t>::max();

// Edge direction type
enum class EdgeDirection {
    DIRECTED,
    UNDIRECTED
};

// Graph property flags
enum class GraphProperties : uint32_t {
    WEIGHTED = 1 << 0,
    DIRECTED = 1 << 1,
    ALLOW_SELF_LOOPS = 1 << 2
};

// Operator overloading for GraphProperties flags
inline GraphProperties operator|(GraphProperties a, GraphProperties b) {
    return static_cast<GraphProperties>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline GraphProperties operator&(GraphProperties a, GraphProperties b) {
    return static_cast<GraphProperties>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// Common type definitions for graph operations
using adjacency_list_t = std::vector<vertex_id_t>;
using weight_list_t = std::vector<weight_t>;

} // namespace graph_engine