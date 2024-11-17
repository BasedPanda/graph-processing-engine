#include "gpu_engine.hpp"
#include <stdexcept>
#include <cstring>

namespace graph_engine {

// CUDA kernel declarations
namespace kernels {

template<typename T>
__global__ void vertexKernel(vertex_id_t* row_offsets,
                            vertex_id_t* column_indices,
                            weight_t* weights,
                            T* vertex_data,
                            size_t num_vertices);

template<typename T>
__global__ void edgeKernel(vertex_id_t* column_indices,
                          weight_t* weights,
                          T* edge_data,
                          size_t num_edges);

} // namespace kernels

// Constants
constexpr int THREADS_PER_BLOCK = 256;
constexpr int MIN_GPU_COMPUTE_CAPABILITY = 35; // Minimum required: 3.5

GPUEngine::GPUEngine() 
    : device_id_(0)
    , initialized_(false)
{
    initialize();
}

GPUEngine::~GPUEngine() {
    if (gpu_data_) {
        if (gpu_data_->row_offsets) cudaFree(gpu_data_->row_offsets);
        if (gpu_data_->column_indices) cudaFree(gpu_data_->column_indices);
        if (gpu_data_->weights) cudaFree(gpu_data_->weights);
    }
}

void GPUEngine::initialize() {
    if (initialized_) return;

    // Check for CUDA device
    int device_count;
    checkCudaError(cudaGetDeviceCount(&device_count),
                   "Failed to get CUDA device count");

    if (device_count == 0) {
        throw std::runtime_error("No CUDA-capable devices found");
    }

    // Set device
    checkCudaError(cudaSetDevice(device_id_),
                   "Failed to set CUDA device");

    if (!checkGPUCapabilities()) {
        throw std::runtime_error("GPU capabilities insufficient");
    }

    gpu_data_ = std::make_unique<GPUGraphData>();
    initialized_ = true;
}

void GPUEngine::transferGraphToDevice(const Graph* graph) {
    if (!graph) {
        throw std::invalid_argument("Null graph pointer");
    }

    // Convert graph to CSR format
    std::vector<vertex_id_t> row_offsets;
    std::vector<vertex_id_t> column_indices;
    std::vector<weight_t> weights;

    row_offsets.reserve(graph->getVertexCount() + 1);
    column_indices.reserve(graph->getEdgeCount());
    if (graph->isWeighted()) {
        weights.reserve(graph->getEdgeCount());
    }

    size_t current_offset = 0;
    for (vertex_id_t i = 0; i < graph->getVertexCount(); ++i) {
        row_offsets.push_back(current_offset);
        const Vertex* vertex = graph->getVertex(i);
        if (vertex) {
            for (const auto& [target, edge_id] : vertex->getOutEdges()) {
                column_indices.push_back(target);
                if (graph->isWeighted()) {
                    const Edge* edge = graph->getEdge(edge_id);
                    weights.push_back(edge ? edge->getWeight() : 1.0f);
                }
                current_offset++;
            }
        }
    }
    row_offsets.push_back(current_offset);

    // Allocate GPU memory
    size_t vertex_size = (graph->getVertexCount() + 1) * sizeof(vertex_id_t);
    size_t edge_size = graph->getEdgeCount() * sizeof(vertex_id_t);
    size_t weight_size = graph->isWeighted() ? graph->getEdgeCount() * sizeof(weight_t) : 0;

    checkCudaError(cudaMalloc(&gpu_data_->row_offsets, vertex_size),
                   "Failed to allocate GPU memory for row offsets");
    checkCudaError(cudaMalloc(&gpu_data_->column_indices, edge_size),
                   "Failed to allocate GPU memory for column indices");
    
    if (graph->isWeighted()) {
        checkCudaError(cudaMalloc(&gpu_data_->weights, weight_size),
                      "Failed to allocate GPU memory for weights");
    }

    // Copy data to GPU
    checkCudaError(cudaMemcpy(gpu_data_->row_offsets, row_offsets.data(),
                             vertex_size, cudaMemcpyHostToDevice),
                   "Failed to copy row offsets to GPU");
    checkCudaError(cudaMemcpy(gpu_data_->column_indices, column_indices.data(),
                             edge_size, cudaMemcpyHostToDevice),
                   "Failed to copy column indices to GPU");
    
    if (graph->isWeighted()) {
        checkCudaError(cudaMemcpy(gpu_data_->weights, weights.data(),
                                 weight_size, cudaMemcpyHostToDevice),
                      "Failed to copy weights to GPU");
    }

    // Store graph properties
    gpu_data_->num_vertices = graph->getVertexCount();
    gpu_data_->num_edges = graph->getEdgeCount();
    gpu_data_->is_directed = graph->isDirected();
    gpu_data_->is_weighted = graph->isWeighted();
}

template<typename T>
void GPUEngine::parallelVertexOp(std::function<void(vertex_id_t, T&)> kernel,
                                std::vector<T>& vertex_data) {
    if (!initialized_ || !gpu_data_) {
        throw std::runtime_error("GPU engine not properly initialized");
    }

    // Allocate device memory for vertex data
    T* d_vertex_data;
    size_t vertex_data_size = vertex_data.size() * sizeof(T);
    checkCudaError(cudaMalloc(&d_vertex_data, vertex_data_size),
                   "Failed to allocate GPU memory for vertex data");

    // Copy input data to device
    checkCudaError(cudaMemcpy(d_vertex_data, vertex_data.data(),
                             vertex_data_size, cudaMemcpyHostToDevice),
                   "Failed to copy vertex data to GPU");

    // Calculate grid dimensions
    int num_blocks = (gpu_data_->num_vertices + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    // Launch kernel
    kernels::vertexKernel<<<num_blocks, THREADS_PER_BLOCK>>>(
        gpu_data_->row_offsets,
        gpu_data_->column_indices,
        gpu_data_->weights,
        d_vertex_data,
        gpu_data_->num_vertices
    );

    // Check for kernel launch errors
    checkCudaError(cudaGetLastError(), "Kernel launch failed");
    checkCudaError(cudaDeviceSynchronize(), "Kernel execution failed");

    // Copy results back to host
    checkCudaError(cudaMemcpy(vertex_data.data(), d_vertex_data,
                             vertex_data_size, cudaMemcpyDeviceToHost),
                   "Failed to copy results from GPU");

    // Cleanup
    cudaFree(d_vertex_data);
}

template<typename T>
void GPUEngine::parallelEdgeOp(std::function<void(edge_id_t, T&)> kernel,
                              std::vector<T>& edge_data) {
    if (!initialized_ || !gpu_data_) {
        throw std::runtime_error("GPU engine not properly initialized");
    }

    // Allocate device memory for edge data
    T* d_edge_data;
    size_t edge_data_size = edge_data.size() * sizeof(T);
    checkCudaError(cudaMalloc(&d_edge_data, edge_data_size),
                   "Failed to allocate GPU memory for edge data");

    // Copy input data to device
    checkCudaError(cudaMemcpy(d_edge_data, edge_data.data(),
                             edge_data_size, cudaMemcpyHostToDevice),
                   "Failed to copy edge data to GPU");

    // Calculate grid dimensions
    int num_blocks = (gpu_data_->num_edges + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    // Launch kernel
    kernels::edgeKernel<<<num_blocks, THREADS_PER_BLOCK>>>(
        gpu_data_->column_indices,
        gpu_data_->weights,
        d_edge_data,
        gpu_data_->num_edges
    );

    // Check for kernel launch errors
    checkCudaError(cudaGetLastError(), "Kernel launch failed");
    checkCudaError(cudaDeviceSynchronize(), "Kernel execution failed");

    // Copy results back to host
    checkCudaError(cudaMemcpy(edge_data.data(), d_edge_data,
                             edge_data_size, cudaMemcpyDeviceToHost),
                   "Failed to copy results from GPU");

    // Cleanup
    cudaFree(d_edge_data);
}

template<typename T>
T* GPUEngine::allocateDevice(size_t count) {
    T* device_ptr;
    checkCudaError(cudaMalloc(&device_ptr, count * sizeof(T)),
                   "Failed to allocate GPU memory");
    return device_ptr;
}

template<typename T>
void GPUEngine::freeDevice(T* device_ptr) {
    if (device_ptr) {
        cudaFree(device_ptr);
    }
}

template<typename T>
void GPUEngine::copyToDevice(const std::vector<T>& host_data, T* device_data) {
    checkCudaError(cudaMemcpy(device_data, host_data.data(),
                             host_data.size() * sizeof(T), cudaMemcpyHostToDevice),
                   "Failed to copy data to GPU");
}

template<typename T>
void GPUEngine::copyToHost(const T* device_data, std::vector<T>& host_data) {
    checkCudaError(cudaMemcpy(host_data.data(), device_data,
                             host_data.size() * sizeof(T), cudaMemcpyDeviceToHost),
                   "Failed to copy data from GPU");
}

bool GPUEngine::isGPUAvailable() const {
    int device_count;
    cudaGetDeviceCount(&device_count);
    return device_count > 0;
}

int GPUEngine::getComputeCapability() const {
    cudaDeviceProp prop;
    checkCudaError(cudaGetDeviceProperties(&prop, device_id_),
                   "Failed to get device properties");
    return prop.major * 10 + prop.minor;
}

size_t GPUEngine::getAvailableMemory() const {
    size_t free_memory, total_memory;
    checkCudaError(cudaMemGetInfo(&free_memory, &total_memory),
                   "Failed to get memory info");
    return free_memory;
}

void GPUEngine::checkCudaError(cudaError_t error, const char* message) {
    if (error != cudaSuccess) {
        throw std::runtime_error(std::string(message) + ": " + 
                               cudaGetErrorString(error));
    }
}

bool GPUEngine::checkGPUCapabilities() {
    cudaDeviceProp prop;
    checkCudaError(cudaGetDeviceProperties(&prop, device_id_),
                   "Failed to get device properties");
    
    int compute_capability = prop.major * 10 + prop.minor;
    return compute_capability >= MIN_GPU_COMPUTE_CAPABILITY;
}

// CUDA Kernel Implementations
namespace kernels {

template<typename T>
__global__ void vertexKernel(vertex_id_t* row_offsets,
                            vertex_id_t* column_indices,
                            weight_t* weights,
                            T* vertex_data,
                            size_t num_vertices) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_vertices) return;

    // Process vertex with ID 'tid'
    int start = row_offsets[tid];
    int end = row_offsets[tid + 1];
    
    for (int i = start; i < end; i++) {
        vertex_id_t neighbor = column_indices[i];
        weight_t weight = weights ? weights[i] : 1.0f;
        // Custom vertex processing logic here
    }
}

template<typename T>
__global__ void edgeKernel(vertex_id_t* column_indices,
                          weight_t* weights,
                          T* edge_data,
                          size_t num_edges) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_edges) return;

    // Process edge with ID 'tid'
    vertex_id_t target = column_indices[tid];
    weight_t weight = weights ? weights[tid] : 1.0f;
    // Custom edge processing logic here
}

} // namespace kernels

} // namespace graph_engine