#include "graph.hpp"
#include <stdexcept>

namespace graph_engine {

Graph::Graph(GraphProperties properties)
    : properties_(properties)
    , next_vertex_id_(0)
    , next_edge_id_(0) {}

vertex_id_t Graph::addVertex() {
    return addVertex("");
}

vertex_id_t Graph::addVertex(const std::string& label) {
    std::unique_lock lock(graph_mutex_);
    vertex_id_t id = next_vertex_id_++;
    vertices_[id] = std::make_unique<Vertex>(id, label);
    return id;
}

void Graph::removeVertex(vertex_id_t id) {
    std::unique_lock lock(graph_mutex_);
    
    auto vertex_it = vertices_.find(id);
    if (vertex_it == vertices_.end()) {
        throw std::out_of_range("Vertex not found");
    }

    // Remove all edges connected to this vertex
    auto* vertex = vertex_it->second.get();
    
    // Remove outgoing edges
    for (const auto& [target, edge_id] : vertex->getOutEdges()) {
        edges_.erase(edge_id);
        if (auto* target_vertex = getVertex(target)) {
            target_vertex->removeInEdge(id);
        }
    }
    
    // Remove incoming edges
    for (const auto& [source, edge_id] : vertex->getInEdges()) {
        edges_.erase(edge_id);
        if (auto* source_vertex = getVertex(source)) {
            source_vertex->removeOutEdge(id);
        }
    }
    
    // Remove the vertex itself
    vertices_.erase(vertex_it);
}

Vertex* Graph::getVertex(vertex_id_t id) {
    std::shared_lock lock(graph_mutex_);
    auto it = vertices_.find(id);
    return it != vertices_.end() ? it->second.get() : nullptr;
}

const Vertex* Graph::getVertex(vertex_id_t id) const {
    std::shared_lock lock(graph_mutex_);
    auto it = vertices_.find(id);
    return it != vertices_.end() ? it->second.get() : nullptr;
}

size_t Graph::getVertexCount() const {
    std::shared_lock lock(graph_mutex_);
    return vertices_.size();
}

edge_id_t Graph::addEdge(vertex_id_t source, vertex_id_t target) {
    return addEdge(source, target, 1.0f);
}

edge_id_t Graph::addEdge(vertex_id_t source, vertex_id_t target, weight_t weight) {
    std::unique_lock lock(graph_mutex_);
    
    if (!validateVertexIds(source, target)) {
        throw std::invalid_argument("Invalid vertex IDs");
    }
    
    checkSelfLoop(source, target);
    
    edge_id_t id = next_edge_id_++;
    EdgeDirection direction = isDirected() ? EdgeDirection::DIRECTED : EdgeDirection::UNDIRECTED;
    
    edges_[id] = std::make_unique<Edge>(id, source, target, weight, direction);
    
    // Update vertex connections
    vertices_[source]->addOutEdge(target, id);
    vertices_[target]->addInEdge(source, id);
    
    // For undirected graphs, add the reverse edge references
    if (!isDirected()) {
        vertices_[target]->addOutEdge(source, id);
        vertices_[source]->addInEdge(target, id);
    }
    
    return id;
}

void Graph::removeEdge(edge_id_t id) {
    std::unique_lock lock(graph_mutex_);
    
    auto edge_it = edges_.find(id);
    if (edge_it == edges_.end()) {
        throw std::out_of_range("Edge not found");
    }
    
    const Edge* edge = edge_it->second.get();
    vertex_id_t source = edge->getSource();
    vertex_id_t target = edge->getTarget();
    
    // Remove edge references from vertices
    if (auto* source_vertex = getVertex(source)) {
        source_vertex->removeOutEdge(target);
    }
    if (auto* target_vertex = getVertex(target)) {
        target_vertex->removeInEdge(source);
    }
    
    // For undirected graphs, remove reverse edge references
    if (!isDirected()) {
        if (auto* target_vertex = getVertex(target)) {
            target_vertex->removeOutEdge(source);
        }
        if (auto* source_vertex = getVertex(source)) {
            source_vertex->removeInEdge(target);
        }
    }
    
    edges_.erase(edge_it);
}

Edge* Graph::getEdge(edge_id_t id) {
    std::shared_lock lock(graph_mutex_);
    auto it = edges_.find(id);
    return it != edges_.end() ? it->second.get() : nullptr;
}

const Edge* Graph::getEdge(edge_id_t id) const {
    std::shared_lock lock(graph_mutex_);
    auto it = edges_.find(id);
    return it != edges_.end() ? it->second.get() : nullptr;
}

size_t Graph::getEdgeCount() const {
    std::shared_lock lock(graph_mutex_);
    return edges_.size();
}

bool Graph::isDirected() const {
    return static_cast<bool>(properties_ & GraphProperties::DIRECTED);
}

bool Graph::isWeighted() const {
    return static_cast<bool>(properties_ & GraphProperties::WEIGHTED);
}

bool Graph::allowsSelfLoops() const {
    return static_cast<bool>(properties_ & GraphProperties::ALLOW_SELF_LOOPS);
}

std::vector<vertex_id_t> Graph::getNeighbors(vertex_id_t id) const {
    std::shared_lock lock(graph_mutex_);
    
    const Vertex* vertex = getVertex(id);
    if (!vertex) {
        throw std::out_of_range("Vertex not found");
    }
    
    std::vector<vertex_id_t> neighbors;
    neighbors.reserve(vertex->getOutDegree());
    
    for (const auto& [target, _] : vertex->getOutEdges()) {
        neighbors.push_back(target);
    }
    
    // For undirected graphs, include incoming edges as well
    if (!isDirected()) {
        for (const auto& [source, _] : vertex->getInEdges()) {
            // Avoid duplicates for self-loops
            if (source != id) {
                neighbors.push_back(source);
            }
        }
    }
    
    return neighbors;
}

std::vector<edge_id_t> Graph::getIncidentEdges(vertex_id_t id) const {
    std::shared_lock lock(graph_mutex_);
    
    const Vertex* vertex = getVertex(id);
    if (!vertex) {
        throw std::out_of_range("Vertex not found");
    }
    
    std::vector<edge_id_t> incident_edges;
    incident_edges.reserve(vertex->getOutDegree() + vertex->getInDegree());
    
    // Add outgoing edge IDs
    for (const auto& [_, edge_id] : vertex->getOutEdges()) {
        incident_edges.push_back(edge_id);
    }
    
    // For undirected graphs, no need to add incoming edges as they're the same
    if (isDirected()) {
        for (const auto& [_, edge_id] : vertex->getInEdges()) {
            incident_edges.push_back(edge_id);
        }
    }
    
    return incident_edges;
}

bool Graph::hasEdge(vertex_id_t source, vertex_id_t target) const {
    std::shared_lock lock(graph_mutex_);
    
    const Vertex* source_vertex = getVertex(source);
    if (!source_vertex) {
        return false;
    }
    
    for (const auto& [target_id, _] : source_vertex->getOutEdges()) {
        if (target_id == target) {
            return true;
        }
    }
    
    return false;
}

void Graph::clear() {
    std::unique_lock lock(graph_mutex_);
    vertices_.clear();
    edges_.clear();
    next_vertex_id_ = 0;
    next_edge_id_ = 0;
}

void Graph::reserve(size_t vertex_count, size_t edge_count) {
    std::unique_lock lock(graph_mutex_);
    vertices_.reserve(vertex_count);
    edges_.reserve(edge_count);
}

bool Graph::validateVertexIds(vertex_id_t source, vertex_id_t target) const {
    return vertices_.find(source) != vertices_.end() && 
           vertices_.find(target) != vertices_.end();
}

void Graph::checkSelfLoop(vertex_id_t source, vertex_id_t target) const {
    if (source == target && !allowsSelfLoops()) {
        throw std::invalid_argument("Self-loops are not allowed in this graph");
    }
}

} // namespace graph_engine