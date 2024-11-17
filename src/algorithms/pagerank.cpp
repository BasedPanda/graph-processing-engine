#include "pagerank.hpp"
#include <cmath>
#include <algorithm>
#include <execution>

namespace graph_engine {

PageRank::PageRank(const Graph* graph)
    : AlgorithmBase(graph)
    , damping_factor_(0.85f)
    , max_iterations_(100)
    , tolerance_(1e-6f)
{
    initializeRanks();
}

void PageRank::execute(ExecutionDevice device) {
    reset();
    initializeRanks();
    
    if (device == ExecutionDevice::GPU && isGPUCompatible()) {
        executeOnGPU();
    } else {
        executeOnCPU();
    }
    
    updateResults();
}

void PageRank::executeOnCPU() {
    const size_t num_vertices = graph_->getVertexCount();
    const float random_jump = (1.0f - damping_factor_) / num_vertices;
    
    for (size_t iteration = 0; iteration < max_iterations_; ++iteration) {
        // Store current ranks for convergence check
        prev_ranks_ = ranks_;
        
        // Reset ranks for this iteration
        std::fill(ranks_.begin(), ranks_.end(), random_jump);
        
        // Update ranks
        for (const auto& [vertex_id, vertex_ptr] : graph_->vertices()) {
            const float contribution = damping_factor_ * prev_ranks_[vertex_id] / vertex_ptr->getOutDegree();
            
            for (const auto& [neighbor_id, _] : vertex_ptr->getOutEdges()) {
                ranks_[neighbor_id] += contribution;
            }
        }
        
        // Check for convergence
        if (hasConverged()) {
            break;
        }
    }
}

void PageRank::executeOnGPU() {
    // GPU implementation would go here
    // For now, fall back to CPU implementation
    executeOnCPU();
}

void PageRank::reset() {
    AlgorithmBase::reset();
    initializeRanks();
}

void PageRank::setDampingFactor(float d) {
    if (d <= 0.0f || d >= 1.0f) {
        throw std::invalid_argument("Damping factor must be between 0 and 1");
    }
    damping_factor_ = d;
}

void PageRank::setMaxIterations(size_t iterations) {
    if (iterations == 0) {
        throw std::invalid_argument("Maximum iterations must be greater than 0");
    }
    max_iterations_ = iterations;
}

void PageRank::setTolerance(float tolerance) {
    if (tolerance <= 0.0f) {
        throw std::invalid_argument("Tolerance must be greater than 0");
    }
    tolerance_ = tolerance;
}

float PageRank::getVertexRank(vertex_id_t vertex) const {
    if (vertex >= ranks_.size()) {
        throw std::out_of_range("Invalid vertex ID");
    }
    return ranks_[vertex];
}

const std::vector<float>& PageRank::getAllRanks() const {
    return ranks_;
}

void PageRank::initializeRanks() {
    const size_t num_vertices = graph_->getVertexCount();
    const float initial_rank = 1.0f / num_vertices;
    
    ranks_.resize(num_vertices, initial_rank);
    prev_ranks_.resize(num_vertices, initial_rank);
}

bool PageRank::hasConverged() const {
    return std::transform_reduce(
        std::execution::par_unseq,
        ranks_.begin(), ranks_.end(),
        prev_ranks_.begin(),
        0.0f,
        std::plus<>(),
        [](float a, float b) { return std::abs(a - b); }
    ) < tolerance_;
}

void PageRank::updateResults() {
    results_["ranks"] = ranks_;
}

} // namespace graph_engine