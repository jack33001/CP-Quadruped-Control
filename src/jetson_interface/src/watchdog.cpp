#include "watchdog.hpp"
#include <iostream>

Watchdog::Watchdog(std::chrono::milliseconds timeout_ms, std::function<void()> callback)
    : timeout_(timeout_ms), callback_(callback), running_(false), reset_flag_(false) {
}

Watchdog::~Watchdog() {
    stop();
}

void Watchdog::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    reset_flag_ = false;
    
    watchdog_thread_ = std::thread(&Watchdog::watchdogThreadFunc, this);
}

void Watchdog::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
}

void Watchdog::reset() {
    if (running_) {
        reset_flag_ = true;
    }
}

bool Watchdog::isRunning() const {
    return running_;
}

void Watchdog::setTimeout(std::chrono::milliseconds timeout_ms) {
    timeout_ = timeout_ms;
}

void Watchdog::watchdogThreadFunc() {
    while (running_) {
        reset_flag_ = false;
        
        // Wait for timeout or until reset
        auto start_time = std::chrono::steady_clock::now();
        while (running_ && !reset_flag_ && 
               (std::chrono::steady_clock::now() - start_time) < timeout_) {
            // Sleep for a short period to minimize CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // If we timed out without being reset and we're still running
        if (running_ && !reset_flag_) {
            try {
                // Execute the callback function
                callback_();
            } catch (const std::exception& e) {
                // Log exception, but don't let it crash the watchdog thread
                std::cerr << "Exception in watchdog callback: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Unknown exception in watchdog callback" << std::endl;
            }
        }
    }
}