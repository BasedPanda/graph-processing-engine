#pragma once

#include "../core/graph.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <any>

namespace graph_engine {

// Enum for execution device
enum class ExecutionDevice {
    CPU,
    GPU
};

class AlgorithmBase {
public:
    explicit AlgorithmBase(const Graph* graph) : graph_(graph) {}
    virtual ~AlgorithmBase() = default;

    // Main execution method
    virtual void execute(ExecutionDevice device = ExecutionDevice::CPU) = 0;
    
    // Algorithm parameters management
    template<typename T>
    void setParameter(const std::string& name, const T& value) {
        parameters_[name] = value;
    }
    
    template<typename T>
    T getParameter(const std::string& name) const {
        auto it = parameters_.find(name);
        if (it == parameters_.end()) {
            throw std::runtime_error("Parameter not found: " + name);
        }
        return std::any_cast<T>(it->second);
    }

    // Results access
    template<typename T>
    T getResult(const std::string& name) const {
        auto it = results_.find(name);
        if (it == results_.end()) {
            throw std::runtime_error("Result not found: " + name);
        }
        return std::any_cast<T>(it->second);
    }

    // Reset the algorithm state
    virtual void reset() {
        results_.clear();
    }

protected:
    const Graph* graph_;
    std::unordered_map<std::string, std::any> parameters_;
    std::unordered_map<std::string, std::any> results_;
    
    // Helper methods for GPU execution
    virtual bool isGPUCompatible() const { return false; }
    virtual void executeOnCPU() = 0;
    virtual void executeOnGPU() = 0;
};

} // namespace graph_engine