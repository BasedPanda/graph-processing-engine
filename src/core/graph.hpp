#pragma once

#include "types.hpp"
#include "vertex.hpp"
#include "edge.hpp"
#include <memory>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <shared_mutex>

namespace graph_engine {

class Graph {
public:
    // Constructor
    explicit Graph(GraphProperties properties = GraphProperties::DIRECTED);

    // Vertex operations
    vertex_id_t addVertex();
    vertex_id_t addVertex(const std::string& label);
    void removeVertex(vertex_id_t id);
    Vertex* getVertex(vertex_id_t id);
    const Vertex* getVertex(vertex_id_t id) const;
    size_t getVertexCount() const;

    // Edge operations
    edge_id_t addEdge(vertex_id_t source, vertex_id_t target);
    edge_id_t addEdge(vertex_id_t source, vertex_id_t target, weight_t weight);
    void removeEdge(edge_id_t id);
    Edge* getEdge(edge_id_t id);
    const Edge* getEdge(edge_id_t id) const;
    size_t getEdgeCount() const;

    // Graph properties
    bool isDirected() const;
    bool isWeighted() const;
    bool allowsSelfLoops() const;
    
    // Graph queries
    std::vector<vertex_id_t> getNeighbors(vertex_id_t id) const;
    std::vector<edge_id_t> getIncidentEdges(vertex_id_t id) const;
    bool hasEdge(vertex_id_t source, vertex_id_t target) const;
    
    // Graph modification
    void clear();
    void reserve(size_t vertex_count, size_t edge_count);

    // Iterator access
    const std::unordered_map<vertex_id_t, std::unique_ptr<Vertex>>& vertices() const { return vertices_; }
    const std::unordered_map<edge_id_t, std::unique_ptr<Edge>>& edges() const { return edges_; }

private:
    GraphProperties properties_;
    std::unordered_map<vertex_id_t, std::unique_ptr<Vertex>> vertices_;
    std::unordered_map<edge_id_t, std::unique_ptr<Edge>> edges_;
    
    vertex_id_t next_vertex_id_;
    edge_id_t next_edge_id_;
    
    // Thread safety
    mutable std::shared_mutex graph_mutex_;
    
    // Helper methods
    bool validateVertexIds(vertex_id_t source, vertex_id_t target) const;
    void checkSelfLoop(vertex_id_t source, vertex_id_t target) const;
};

} // namespace graph_engine