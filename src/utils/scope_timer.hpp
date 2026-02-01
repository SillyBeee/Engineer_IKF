#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>

// RAII 耗时统计类
class ScopeTimer {
public:
    ScopeTimer(std::string name, rclcpp::Logger logger) 
        : name_(name), logger_(logger), start_(std::chrono::high_resolution_clock::now()) {}
    
    ~ScopeTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_);
        RCLCPP_INFO(logger_, "[PERF] %s took %ld ms", name_.c_str(), duration.count());
    }
private:
    std::string name_;
    rclcpp::Logger logger_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

// 定义宏：在当前作用域创建一个局部变量，利用析构函数自动触发统计
#define MEASURE_TIME() ScopeTimer timer_##__LINE__(__FUNCTION__, this->get_logger())
// ...existing code...