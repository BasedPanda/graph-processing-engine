#include "vertex.hpp"
#include <stdexcept>
#include <algorithm>

namespace graph_engine {

Vertex::Vertex(vertex_id_t id) 
    : id_(id), label_("") {}

Vertex::Vertex(vertex_id_t id, const std::string& label) 
    : id_(id), label_(label) {}

void Vertex::setProperty(const std::string& key, const std::string& value) {
    properties_[key] = value;
}

const std::string& Vertex::getProperty(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) {
        throw std::out_of_range("Property not found: " + key);
    }
    return it->second;
}

bool Vertex::hasProperty(const std::string& key) const {
    return properties_.find(key) != properties_.end();
}

void Vertex::addOutEdge(vertex_id_t target, edge_id_t edge_id) {
    // Check if edge already exists
    auto it = std::find_if(out_edges_.begin(), out_edges_.end(),
                          [target](const auto& p) { return p.first == target; });
    if (it == out_edges_.end()) {
        out_edges_.emplace_back(target, edge_id);
    }
}

void Vertex::addInEdge(vertex_id_t source, edge_id_t edge_id) {
    auto it = std::find_if(in_edges_.begin(), in_edges_.end(),
                          [source](const auto& p) { return p.first == source; });
    if (it == in_edges_.end()) {
        in_edges_.emplace_back(source, edge_id);
    }
}

void Vertex::removeOutEdge(vertex_id_t target) {
    out_edges_.erase(
        std::remove_if(out_edges_.begin(), out_edges_.end(),
                      [target](const auto& p) { return p.first == target; }),
        out_edges_.end());
}

void Vertex::removeInEdge(vertex_id_t source) {
    in_edges_.erase(
        std::remove_if(in_edges_.begin(), in_edges_.end(),
                      [source](const auto& p) { return p.first == source; }),
        in_edges_.end());
}

} // namespace graph_engine