#include "timer.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace graph_engine {

Timer& Timer::getInstance() {
    static Timer instance;
    return instance;
}

void Timer::start(const std::string& name) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    auto& timer = timers_[name];
    
    if (timer.running) {
        throw std::runtime_error("Timer '" + name + "' is already running");
    }
    
    timer.start_time = Clock::now();
    timer.running = true;
    timer.paused = false;
}

void Timer::stop(const std::string& name) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    auto& timer = getTimerData(name);
    
    if (!timer.running) {
        throw std::runtime_error("Timer '" + name + "' is not running");
    }
    
    if (!timer.paused) {
        auto end_time = Clock::now();
        timer.lap_time = std::chrono::duration_cast<Duration>(end_time - timer.start_time);
        timer.total_time += timer.lap_time;
        timer.lap_count++;
    }
    
    timer.running = false;
    timer.paused = false;
}

void Timer::pause(const std::string& name) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    auto& timer = getTimerData(name);
    
    if (!timer.running || timer.paused) {
        throw std::runtime_error("Timer '" + name + "' cannot be paused");
    }
    
    auto pause_time = Clock::now();
    timer.lap_time = std::chrono::duration_cast<Duration>(pause_time - timer.start_time);
    timer.total_time += timer.lap_time;
    timer.paused = true;
}

void Timer::resume(const std::string& name) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    auto& timer = getTimerData(name);
    
    if (!timer.running || !timer.paused) {
        throw std::runtime_error("Timer '" + name + "' cannot be resumed");
    }
    
    timer.start_time = Clock::now();
    timer.paused = false;
}

void Timer::reset(const std::string& name) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    auto& timer = getTimerData(name);
    timer = TimerData();
}

void Timer::clear() {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    timers_.clear();
}

double Timer::getElapsedTime(const std::string& name) const {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    const auto& timer = getTimerData(name);
    
    if (timer.running && !timer.paused) {
        auto current_time = Clock::now();
        auto current_lap = std::chrono::duration_cast<Duration>(current_time - timer.start_time);
        return std::chrono::duration<double>(timer.total_time + current_lap).count();
    }
    
    return std::chrono::duration<double>(timer.total_time).count();
}

double Timer::getAverageTime(const std::string& name) const {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    const auto& timer = getTimerData(name);
    
    if (timer.lap_count == 0) {
        return 0.0;
    }
    
    return std::chrono::duration<double>(timer.total_time).count() / timer.lap_count;
}

size_t Timer::getLapCount(const std::string& name) const {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    return getTimerData(name).lap_count;
}

void Timer::printStats(const std::string& name) const {
    const auto& timer = getTimerData(name);
    std::stringstream ss;
    
    ss << "Timer '" << name << "' stats:\n"
       << "  Total time: " << formatDuration(timer.total_time) << "\n"
       << "  Avg time: " << formatDuration(timer.total_time / timer.lap_count) << "\n"
       << "  Lap count: " << timer.lap_count << "\n"
       << "  Status: " << (timer.running ? (timer.paused ? "Paused" : "Running") : "Stopped");

    LOG_INFO(ss.str());
}

void Timer::printAllStats() const {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    
    if (timers_.empty()) {
        LOG_INFO("No timers available.");
        return;
    }

    std::stringstream ss;
    ss << "Timer Statistics:\n";
    ss << std::setw(30) << std::left << "Name"
       << std::setw(20) << "Total Time"
       << std::setw(20) << "Average Time"
       << std::setw(15) << "Lap Count"
       << "Status\n";
    ss << std::string(85, '-') << "\n";

    for (const auto& [name, timer] : timers_) {
        ss << std::setw(30) << std::left << name
           << std::setw(20) << formatDuration(timer.total_time)
           << std::setw(20) << formatDuration(timer.total_time / std::max(size_t(1), timer.lap_count))
           << std::setw(15) << timer.lap_count
           << (timer.running ? (timer.paused ? "Paused" : "Running") : "Stopped")
           << "\n";
    }

    LOG_INFO(ss.str());
}

Timer::TimerData& Timer::getTimerData(const std::string& name) {
    auto it = timers_.find(name);
    if (it == timers_.end()) {
        throw std::runtime_error("Timer '" + name + "' not found");
    }
    return it->second;
}

const Timer::TimerData& Timer::getTimerData(const std::string& name) const {
    auto it = timers_.find(name);
    if (it == timers_.end()) {
        throw std::runtime_error("Timer '" + name + "' not found");
    }
    return it->second;
}

std::string Timer::formatDuration(Duration duration) {
    double value;
    std::string unit;
    
    if (duration < std::chrono::microseconds(1)) {
        value = std::chrono::duration<double, std::nano>(duration).count();
        unit = "ns";
    } else if (duration < std::chrono::milliseconds(1)) {
        value = std::chrono::duration<double, std::micro>(duration).count();
        unit = "µs";
    } else if (duration < std::chrono::seconds(1)) {
        value = std::chrono::duration<double, std::milli>(duration).count();
        unit = "ms";
    } else {
        value = std::chrono::duration<double>(duration).count();
        unit = "s";
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << value << " " << unit;
    return ss.str();
}

// ScopedTimer implementation
Timer::ScopedTimer::ScopedTimer(const std::string& name) : name_(name) {
    Timer::getInstance().start(name_);
}

Timer::ScopedTimer::~ScopedTimer() {
    try {
        Timer::getInstance().stop(name_);
    } catch (...) {
        // Suppress exceptions in destructor
    }
}

} // namespace graph_engine