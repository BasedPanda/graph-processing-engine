#pragma once

#include "../core/graph.hpp"
#include <vector>
#include <memory>
#include <cuda_runtime.h>

namespace graph_engine {

// Forward declarations of CUDA-specific structures
struct GPUGraphData;

class GPUEngine {
public:
    // Constructor and destructor
    GPUEngine();
    ~GPUEngine();

    // Graph transfer methods
    void transferGraphToDevice(const Graph* graph);
    void transferGraphToHost(Graph* graph);

    // Vertex-parallel execution methods
    template<typename T>
    void parallelVertexOp(std::function<void(vertex_id_t, T&)> kernel,
                         std::vector<T>& vertex_data);

    // Edge-parallel execution methods
    template<typename T>
    void parallelEdgeOp(std::function<void(edge_id_t, T&)> kernel,
                         std::vector<T>& edge_data);

    // Data transfer methods
    template<typename T>
    void copyToDevice(const std::vector<T>& host_data, T* device_data);
    
    template<typename T>
    void copyToHost(const T* device_data, std::vector<T>& host_data);

    // GPU memory management
    template<typename T>
    T* allocateDevice(size_t count);
    
    template<typename T>
    void freeDevice(T* device_ptr);

    // GPU capabilities
    bool isGPUAvailable() const;
    int getComputeCapability() const;
    size_t getAvailableMemory() const;

private:
    std::unique_ptr<GPUGraphData> gpu_data_;
    int device_id_;
    bool initialized_;

    // Initialize GPU context
    void initialize();
    
    // Helper methods for CUDA error checking
    void checkCudaError(cudaError_t error, const char* message);
    bool checkGPUCapabilities();
};

// GPU Graph data structure (device-side)
struct GPUGraphData {
    // CSR format for efficient GPU processing
    vertex_id_t* row_offsets;    // Vertex offsets into edge arrays
    vertex_id_t* column_indices; // Target vertices
    weight_t* weights;          // Edge weights
    size_t num_vertices;
    size_t num_edges;
    bool is_directed;
    bool is_weighted;
};

} // namespace graph_engine