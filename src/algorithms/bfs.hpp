#pragma once

#include "algorithm_base.hpp"
#include <vector>
#include <limits>

namespace graph_engine {

class BFS : public AlgorithmBase {
public:
    BFS(const Graph* graph);

    // Override base class methods
    void execute(ExecutionDevice device = ExecutionDevice::CPU) override;
    void reset() override;

    // BFS specific methods
    void setStartVertex(vertex_id_t start);
    
    // Results access
    int getDistance(vertex_id_t vertex) const;
    vertex_id_t getParent(vertex_id_t vertex) const;
    const std::vector<int>& getAllDistances() const;
    const std::vector<vertex_id_t>& getAllParents() const;
    std::vector<vertex_id_t> getPath(vertex_id_t target) const;

    static constexpr int INFINITY_DISTANCE = std::numeric_limits<int>::max();

private:
    vertex_id_t start_vertex_;
    std::vector<int> distances_;
    std::vector<vertex_id_t> parents_;
    bool is_executed_;

    // Implementation methods
    void executeOnCPU() override;
    void executeOnGPU() override;
    bool isGPUCompatible() const override { return true; }
    
    // Helper methods
    void initializeArrays();
    void validateVertex(vertex_id_t vertex) const;
    void updateResults();
};

} // namespace graph_engine