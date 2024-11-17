#include "logger.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <ctime>

namespace graph_engine {

const std::unordered_map<LogLevel, std::string> Logger::level_strings_ = {
    {LogLevel::DEBUG, "DEBUG"},
    {LogLevel::INFO, "INFO"},
    {LogLevel::WARNING, "WARNING"},
    {LogLevel::ERROR, "ERROR"},
    {LogLevel::FATAL, "FATAL"}
};

const std::unordered_map<LogLevel, std::string> Logger::level_colors_ = {
    {LogLevel::DEBUG, "\033[36m"},    // Cyan
    {LogLevel::INFO, "\033[32m"},     // Green
    {LogLevel::WARNING, "\033[33m"},  // Yellow
    {LogLevel::ERROR, "\033[31m"},    // Red
    {LogLevel::FATAL, "\033[35m"}     // Magenta
};

Logger::Logger()
    : current_level_(LogLevel::INFO)
    , console_output_(true)
    , include_timestamp_(true)
{}

Logger::~Logger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::setLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    current_level_ = level;
}

void Logger::setLogFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_file_.is_open()) {
        log_file_.close();
    }
    log_file_.open(filename, std::ios::app);
    if (!log_file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }
}

void Logger::setConsoleOutput(bool enable) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    console_output_ = enable;
}

void Logger::setTimestamp(bool enable) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    include_timestamp_ = enable;
}

void Logger::writeLog(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);

    std::stringstream log_stream;
    
    // Add timestamp if enabled
    if (include_timestamp_) {
        log_stream << getCurrentTimestamp() << " ";
    }

    // Add log level
    log_stream << "[" << level_strings_.at(level) << "] ";

    // Add message
    log_stream << message;

    // Write to console if enabled
    if (console_output_) {
        std::cout << level_colors_.at(level) << log_stream.str() << "\033[0m" << std::endl;
    }

    // Write to file if opened
    if (log_file_.is_open()) {
        log_file_ << log_stream.str() << std::endl;
        log_file_.flush();
    }
}

std::string Logger::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

std::string Logger::levelToString(LogLevel level) {
    return level_strings_.at(level);
}

} // namespace graph_engine