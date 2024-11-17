#include "bfs.hpp"
#include <queue>
#include <stdexcept>

namespace graph_engine {

BFS::BFS(const Graph* graph)
    : AlgorithmBase(graph)
    , start_vertex_(0)
    , is_executed_(false)
{
    initializeArrays();
}

void BFS::execute(ExecutionDevice device) {
    if (!graph_->getVertex(start_vertex_)) {
        throw std::invalid_argument("Invalid start vertex");
    }

    reset();
    
    if (device == ExecutionDevice::GPU && isGPUCompatible()) {
        executeOnGPU();
    } else {
        executeOnCPU();
    }
    
    is_executed_ = true;
    updateResults();
}

void BFS::executeOnCPU() {
    std::queue<vertex_id_t> queue;
    
    // Initialize start vertex
    distances_[start_vertex_] = 0;
    queue.push(start_vertex_);
    
    while (!queue.empty()) {
        vertex_id_t current = queue.front();
        queue.pop();
        
        const Vertex* vertex = graph_->getVertex(current);
        if (!vertex) continue;
        
        // Process all neighbors
        for (const auto& [neighbor_id, _] : vertex->getOutEdges()) {
            // If this vertex hasn't been visited
            if (distances_[neighbor_id] == INFINITY_DISTANCE) {
                distances_[neighbor_id] = distances_[current] + 1;
                parents_[neighbor_id] = current;
                queue.push(neighbor_id);
            }
        }
        
        // For undirected graphs, process incoming edges too
        if (!graph_->isDirected()) {
            for (const auto& [neighbor_id, _] : vertex->getInEdges()) {
                if (distances_[neighbor_id] == INFINITY_DISTANCE) {
                    distances_[neighbor_id] = distances_[current] + 1;
                    parents_[neighbor_id] = current;
                    queue.push(neighbor_id);
                }
            }
        }
    }
}

void BFS::executeOnGPU() {
    // GPU implementation would go here
    // For now, fall back to CPU implementation
    executeOnCPU();
}

void BFS::reset() {
    AlgorithmBase::reset();
    initializeArrays();
    is_executed_ = false;
}

void BFS::setStartVertex(vertex_id_t start) {
    if (!graph_->getVertex(start)) {
        throw std::invalid_argument("Invalid start vertex");
    }
    start_vertex_ = start;
    reset();
}

int BFS::getDistance(vertex_id_t vertex) const {
    validateVertex(vertex);
    return distances_[vertex];
}

vertex_id_t BFS::getParent(vertex_id_t vertex) const {
    validateVertex(vertex);
    return parents_[vertex];
}

const std::vector<int>& BFS::getAllDistances() const {
    if (!is_executed_) {
        throw std::runtime_error("BFS has not been executed yet");
    }
    return distances_;
}

const std::vector<vertex_id_t>& BFS::getAllParents() const {
    if (!is_executed_) {
        throw std::runtime_error("BFS has not been executed yet");
    }
    return parents_;
}

std::vector<vertex_id_t> BFS::getPath(vertex_id_t target) const {
    validateVertex(target);
    
    if (distances_[target] == INFINITY_DISTANCE) {
        return std::vector<vertex_id_t>();  // No path exists
    }
    
    std::vector<vertex_id_t> path;
    for (vertex_id_t current = target; current != INVALID_VERTEX_ID; current = parents_[current]) {
        path.push_back(current);
        if (current == start_vertex_) break;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

void BFS::initializeArrays() {
    const size_t num_vertices = graph_->getVertexCount();
    distances_.assign(num_vertices, INFINITY_DISTANCE);
    parents_.assign(num_vertices, INVALID_VERTEX_ID);
}

void BFS::validateVertex(vertex_id_t vertex) const {
    if (!is_executed_) {
        throw std::runtime_error("BFS has not been executed yet");
    }
    if (vertex >= distances_.size()) {
        throw std::out_of_range("Invalid vertex ID");
    }
}

void BFS::updateResults() {
    results_["distances"] = distances_;
    results_["parents"] = parents_;
    results_["start_vertex"] = start_vertex_;
}

} // namespace graph_engine