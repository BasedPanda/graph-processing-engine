#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include "logger.hpp"

namespace graph_engine {

class Timer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration = std::chrono::nanoseconds;

    // Singleton access
    static Timer& getInstance();

    // Delete copy and move constructors
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
    Timer(Timer&&) = delete;
    Timer& operator=(Timer&&) = delete;

    // Timer operations
    void start(const std::string& name);
    void stop(const std::string& name);
    void pause(const std::string& name);
    void resume(const std::string& name);
    void reset(const std::string& name);
    void clear();

    // Timer queries
    double getElapsedTime(const std::string& name) const;
    double getAverageTime(const std::string& name) const;
    size_t getLapCount(const std::string& name) const;
    
    // Output methods
    void printStats(const std::string& name) const;
    void printAllStats() const;

    // Automatic timer using RAII
    class ScopedTimer {
    public:
        explicit ScopedTimer(const std::string& name);
        ~ScopedTimer();
    private:
        std::string name_;
    };

private:
    Timer() = default;

    struct TimerData {
        TimePoint start_time;
        Duration total_time{0};
        Duration lap_time{0};
        size_t lap_count{0};
        bool running{false};
        bool paused{false};
    };

    std::unordered_map<std::string, TimerData> timers_;
    mutable std::mutex timer_mutex_;

    // Helper methods
    TimerData& getTimerData(const std::string& name);
    const TimerData& getTimerData(const std::string& name) const;
    static std::string formatDuration(Duration duration);
};

// Convenience macro for scoped timing
#define SCOPED_TIMER(name) graph_engine::Timer::ScopedTimer timer##__LINE__(name)

// Convenience macros for manual timing
#define START_TIMER(name) graph_engine::Timer::getInstance().start(name)
#define STOP_TIMER(name) graph_engine::Timer::getInstance().stop(name)
#define PAUSE_TIMER(name) graph_engine::Timer::getInstance().pause(name)
#define RESUME_TIMER(name) graph_engine::Timer::getInstance().resume(name)
#define RESET_TIMER(name) graph_engine::Timer::getInstance().reset(name)

} // namespace graph_engine