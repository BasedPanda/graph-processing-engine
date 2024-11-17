#pragma once

#include "algorithm_base.hpp"
#include <vector>

namespace graph_engine {

class PageRank : public AlgorithmBase {
public:
    PageRank(const Graph* graph);

    // Override base class methods
    void execute(ExecutionDevice device = ExecutionDevice::CPU) override;
    void reset() override;

    // PageRank specific methods
    void setDampingFactor(float d);
    void setMaxIterations(size_t iterations);
    void setTolerance(float tolerance);
    
    // Results access
    float getVertexRank(vertex_id_t vertex) const;
    const std::vector<float>& getAllRanks() const;

private:
    // Algorithm parameters
    float damping_factor_;
    size_t max_iterations_;
    float tolerance_;
    std::vector<float> ranks_;
    std::vector<float> prev_ranks_;

    // Implementation methods
    void executeOnCPU() override;
    void executeOnGPU() override;
    bool isGPUCompatible() const override { return true; }
    
    // Helper methods
    void initializeRanks();
    bool hasConverged() const;
    void updateResults();
};

} // namespace graph_engine