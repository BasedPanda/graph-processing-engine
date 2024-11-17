#pragma once

#include "types.hpp"
#include <string>
#include <unordered_map>

namespace graph_engine {

class Edge {
public:
    // Constructors
    Edge(edge_id_t id, vertex_id_t source, vertex_id_t target, EdgeDirection direction = EdgeDirection::DIRECTED);
    Edge(edge_id_t id, vertex_id_t source, vertex_id_t target, weight_t weight, EdgeDirection direction = EdgeDirection::DIRECTED);

    // Basic getters
    edge_id_t getId() const { return id_; }
    vertex_id_t getSource() const { return source_; }
    vertex_id_t getTarget() const { return target_; }
    weight_t getWeight() const { return weight_; }
    EdgeDirection getDirection() const { return direction_; }

    // Property management
    void setProperty(const std::string& key, const std::string& value);
    const std::string& getProperty(const std::string& key) const;
    bool hasProperty(const std::string& key) const;

    // Weight management
    void setWeight(weight_t weight) { weight_ = weight; }
    bool isWeighted() const { return weight_ != 1.0f; }

    // Edge information
    bool isDirected() const { return direction_ == EdgeDirection::DIRECTED; }
    bool connects(vertex_id_t v1, vertex_id_t v2) const;

private:
    edge_id_t id_;
    vertex_id_t source_;
    vertex_id_t target_;
    weight_t weight_;
    EdgeDirection direction_;
    std::unordered_map<std::string, std::string> properties_;
};

} // namespace graph_engine