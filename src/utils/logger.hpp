#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace graph_engine {

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class Logger {
public:
    // Singleton access
    static Logger& getInstance();

    // Delete copy and move constructors
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    // Configuration methods
    void setLogLevel(LogLevel level);
    void setLogFile(const std::string& filename);
    void setConsoleOutput(bool enable);
    void setTimestamp(bool enable);

    // Logging methods
    template<typename... Args>
    void debug(const std::string& format, Args... args);

    template<typename... Args>
    void info(const std::string& format, Args... args);

    template<typename... Args>
    void warning(const std::string& format, Args... args);

    template<typename... Args>
    void error(const std::string& format, Args... args);

    template<typename... Args>
    void fatal(const std::string& format, Args... args);

private:
    Logger();
    ~Logger();

    // Internal logging implementation
    template<typename... Args>
    void log(LogLevel level, const std::string& format, Args... args);
    
    void writeLog(LogLevel level, const std::string& message);
    std::string getCurrentTimestamp() const;
    static std::string levelToString(LogLevel level);

    LogLevel current_level_;
    std::mutex log_mutex_;
    std::ofstream log_file_;
    bool console_output_;
    bool include_timestamp_;
    
    static const std::unordered_map<LogLevel, std::string> level_strings_;
    static const std::unordered_map<LogLevel, std::string> level_colors_;
};

// Template implementations
template<typename... Args>
void Logger::debug(const std::string& format, Args... args) {
    log(LogLevel::DEBUG, format, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::info(const std::string& format, Args... args) {
    log(LogLevel::INFO, format, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::warning(const std::string& format, Args... args) {
    log(LogLevel::WARNING, format, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::error(const std::string& format, Args... args) {
    log(LogLevel::ERROR, format, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::fatal(const std::string& format, Args... args) {
    log(LogLevel::FATAL, format, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::log(LogLevel level, const std::string& format, Args... args) {
    if (level < current_level_) return;

    std::string message = format;
    std::vector<std::string> string_args{std::to_string(std::forward<Args>(args))...};
    
    size_t pos = 0;
    for (const auto& arg : string_args) {
        pos = message.find("{}", pos);
        if (pos == std::string::npos) break;
        message.replace(pos, 2, arg);
        pos += arg.length();
    }

    writeLog(level, message);
}

// Convenience macros for logging
#define LOG_DEBUG(...) graph_engine::Logger::getInstance().debug(__VA_ARGS__)
#define LOG_INFO(...) graph_engine::Logger::getInstance().info(__VA_ARGS__)
#define LOG_WARNING(...) graph_engine::Logger::getInstance().warning(__VA_ARGS__)
#define LOG_ERROR(...) graph_engine::Logger::getInstance().error(__VA_ARGS__)
#define LOG_FATAL(...) graph_engine::Logger::getInstance().fatal(__VA_ARGS__)

} // namespace graph_engine