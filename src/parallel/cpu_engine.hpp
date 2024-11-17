#pragma once

#include "../core/graph.hpp"
#include <vector>
#include <functional>
#include <omp.h>

namespace graph_engine {

class CPUEngine {
public:
    // Constructor with optional thread count specification
    explicit CPUEngine(int num_threads = 0);

    // Vertex-parallel execution methods
    template<typename T>
    void parallelVertexOp(const Graph* graph, 
                         std::function<void(vertex_id_t, const Vertex*, T&)> op,
                         std::vector<T>& vertex_data);

    // Edge-parallel execution methods
    template<typename T>
    void parallelEdgeOp(const Graph* graph,
                       std::function<void(const Edge*, T&)> op,
                       std::vector<T>& edge_data);

    // Parallel reduction operations
    template<typename T>
    T reduceVertices(const Graph* graph,
                    std::function<T(vertex_id_t, const Vertex*)> map_op,
                    std::function<T(T, T)> reduce_op);

    template<typename T>
    T reduceEdges(const Graph* graph,
                 std::function<T(const Edge*)> map_op,
                 std::function<T(T, T)> reduce_op);

    // Configuration methods
    void setNumThreads(int num_threads);
    int getNumThreads() const { return num_threads_; }

    // Thread-local storage management
    template<typename T>
    std::vector<T> createThreadLocalStorage(size_t size, const T& initial_value);

private:
    int num_threads_;
    static const int MIN_ITEMS_PER_THREAD = 1000; // Minimum items for parallelization

    // Helper methods
    bool shouldParallelize(size_t work_size) const;
};

// Template implementation

template<typename T>
void CPUEngine::parallelVertexOp(
    const Graph* graph,
    std::function<void(vertex_id_t, const Vertex*, T&)> op,
    std::vector<T>& vertex_data)
{
    const size_t num_vertices = graph->getVertexCount();
    if (!shouldParallelize(num_vertices)) {
        for (const auto& [id, vertex] : graph->vertices()) {
            op(id, vertex.get(), vertex_data[id]);
        }
        return;
    }

    #pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1000)
    for (size_t i = 0; i < num_vertices; ++i) {
        if (const Vertex* vertex = graph->getVertex(i)) {
            op(i, vertex, vertex_data[i]);
        }
    }
}

template<typename T>
void CPUEngine::parallelEdgeOp(
    const Graph* graph,
    std::function<void(const Edge*, T&)> op,
    std::vector<T>& edge_data)
{
    const size_t num_edges = graph->getEdgeCount();
    if (!shouldParallelize(num_edges)) {
        for (const auto& [id, edge] : graph->edges()) {
            op(edge.get(), edge_data[id]);
        }
        return;
    }

    #pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1000)
    for (size_t i = 0; i < num_edges; ++i) {
        if (const Edge* edge = graph->getEdge(i)) {
            op(edge, edge_data[i]);
        }
    }
}

template<typename T>
T CPUEngine::reduceVertices(
    const Graph* graph,
    std::function<T(vertex_id_t, const Vertex*)> map_op,
    std::function<T(T, T)> reduce_op)
{
    const size_t num_vertices = graph->getVertexCount();
    if (!shouldParallelize(num_vertices)) {
        T result = T();
        for (const auto& [id, vertex] : graph->vertices()) {
            result = reduce_op(result, map_op(id, vertex.get()));
        }
        return result;
    }

    T final_result = T();
    #pragma omp parallel num_threads(num_threads_)
    {
        T local_result = T();
        #pragma omp for schedule(dynamic, 1000) nowait
        for (size_t i = 0; i < num_vertices; ++i) {
            if (const Vertex* vertex = graph->getVertex(i)) {
                local_result = reduce_op(local_result, map_op(i, vertex));
            }
        }
        #pragma omp critical
        {
            final_result = reduce_op(final_result, local_result);
        }
    }
    return final_result;
}

template<typename T>
T CPUEngine::reduceEdges(
    const Graph* graph,
    std::function<T(const Edge*)> map_op,
    std::function<T(T, T)> reduce_op)
{
    const size_t num_edges = graph->getEdgeCount();
    if (!shouldParallelize(num_edges)) {
        T result = T();
        for (const auto& [_, edge] : graph->edges()) {
            result = reduce_op(result, map_op(edge.get()));
        }
        return result;
    }

    T final_result = T();
    #pragma omp parallel num_threads(num_threads_)
    {
        T local_result = T();
        #pragma omp for schedule(dynamic, 1000) nowait
        for (size_t i = 0; i < num_edges; ++i) {
            if (const Edge* edge = graph->getEdge(i)) {
                local_result = reduce_op(local_result, map_op(edge));
            }
        }
        #pragma omp critical
        {
            final_result = reduce_op(final_result, local_result);
        }
    }
    return final_result;
}

template<typename T>
std::vector<T> CPUEngine::createThreadLocalStorage(size_t size, const T& initial_value) {
    return std::vector<T>(size * num_threads_, initial_value);
}

} // namespace graph_engine