#pragma once

#include "types.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace graph_engine {

class Vertex {
public:
    // Constructors
    explicit Vertex(vertex_id_t id);
    Vertex(vertex_id_t id, const std::string& label);

    // Getters
    vertex_id_t getId() const { return id_; }
    const std::string& getLabel() const { return label_; }
    
    // Property management
    void setProperty(const std::string& key, const std::string& value);
    const std::string& getProperty(const std::string& key) const;
    bool hasProperty(const std::string& key) const;
    
    // Edge management
    void addOutEdge(vertex_id_t target, edge_id_t edge_id);
    void addInEdge(vertex_id_t source, edge_id_t edge_id);
    void removeOutEdge(vertex_id_t target);
    void removeInEdge(vertex_id_t source);
    
    // Accessors
    const std::vector<std::pair<vertex_id_t, edge_id_t>>& getOutEdges() const { return out_edges_; }
    const std::vector<std::pair<vertex_id_t, edge_id_t>>& getInEdges() const { return in_edges_; }
    size_t getOutDegree() const { return out_edges_.size(); }
    size_t getInDegree() const { return in_edges_.size(); }

private:
    vertex_id_t id_;
    std::string label_;
    std::unordered_map<std::string, std::string> properties_;
    std::vector<std::pair<vertex_id_t, edge_id_t>> out_edges_; // (target_id, edge_id)
    std::vector<std::pair<vertex_id_t, edge_id_t>> in_edges_;  // (source_id, edge_id)
};

} // namespace graph_engine