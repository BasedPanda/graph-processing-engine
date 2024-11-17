#include "edge.hpp"
#include <stdexcept>

namespace graph_engine {

Edge::Edge(edge_id_t id, vertex_id_t source, vertex_id_t target, EdgeDirection direction)
    : Edge(id, source, target, 1.0f, direction) {}

Edge::Edge(edge_id_t id, vertex_id_t source, vertex_id_t target, weight_t weight, EdgeDirection direction)
    : id_(id)
    , source_(source)
    , target_(target)
    , weight_(weight)
    , direction_(direction) {}

void Edge::setProperty(const std::string& key, const std::string& value) {
    properties_[key] = value;
}

const std::string& Edge::getProperty(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) {
        throw std::out_of_range("Property not found: " + key);
    }
    return it->second;
}

bool Edge::hasProperty(const std::string& key) const {
    return properties_.find(key) != properties_.end();
}

bool Edge::connects(vertex_id_t v1, vertex_id_t v2) const {
    if (direction_ == EdgeDirection::DIRECTED) {
        return source_ == v1 && target_ == v2;
    } else {
        return (source_ == v1 && target_ == v2) || (source_ == v2 && target_ == v1);
    }
}

} // namespace graph_engine