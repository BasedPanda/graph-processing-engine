# Distributed Graph Processing Engine

A high-performance distributed graph processing system implemented in C++, utilizing OpenMP and CUDA for parallel and GPU-accelerated graph computations.

## Features

- **High Performance**: Optimized for large-scale graph processing using parallel CPU and GPU execution
- **Flexible Graph Support**: 
  - Directed and undirected graphs
  - Weighted and unweighted edges
  - Property support for vertices and edges
- **Multiple Algorithm Implementations**:
  - PageRank
  - Breadth-First Search (BFS)
- **Parallel Processing**:
  - OpenMP for CPU parallelization
  - CUDA for GPU acceleration
- **Efficient I/O**:
  - Multiple file format support
  - Fast graph loading and saving
- **Utility Features**:
  - Performance timing
  - Logging system
  - Comprehensive error handling

## Requirements

- C++17 compatible compiler
- CUDA Toolkit (>= 10.0)
- OpenMP support
- CMake (>= 3.14)
- Google Test (automatically downloaded)

## Building

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
cmake --build .

# Run tests
ctest
```