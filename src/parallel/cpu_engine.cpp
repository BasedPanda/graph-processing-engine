#include "cpu_engine.hpp"
#include <algorithm>
#include <thread>

namespace graph_engine {

CPUEngine::CPUEngine(int num_threads) {
    setNumThreads(num_threads);
}

void CPUEngine::setNumThreads(int num_threads) {
    if (num_threads <= 0) {
        // Use number of hardware threads if not specified
        num_threads_ = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    } else {
        num_threads_ = num_threads;
    }
    omp_set_num_threads(num_threads_);
}

bool CPUEngine::shouldParallelize(size_t work_size) const {
    return work_size >= MIN_ITEMS_PER_THREAD * num_threads_;
}

} // namespace graph_engine