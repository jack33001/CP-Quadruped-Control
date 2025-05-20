#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

/**
 * @brief A watchdog timer class that executes a callback when the timer expires.
 *
 * This class implements a watchdog timer that runs on a separate thread.
 * If the watchdog is not reset before the timeout period expires, 
 * the provided callback function will be executed.
 */
class Watchdog {
public:
    /**
     * @brief Constructs a watchdog timer with the specified timeout and callback.
     * 
     * @param timeout_ms Timeout period in milliseconds
     * @param callback Function to call when watchdog triggers
     */
    Watchdog(std::chrono::milliseconds timeout_ms, std::function<void()> callback);
    
    /**
     * @brief Destructor that ensures the watchdog is properly stopped.
     */
    ~Watchdog();
    
    /**
     * @brief Starts the watchdog timer.
     */
    void start();
    
    /**
     * @brief Stops the watchdog timer.
     */
    void stop();
    
    /**
     * @brief Resets the watchdog timer to prevent it from triggering.
     */
    void reset();
    
    /**
     * @brief Checks if the watchdog timer is currently running.
     * 
     * @return True if the watchdog is running, false otherwise
     */
    bool isRunning() const;
    
    /**
     * @brief Updates the timeout period.
     * 
     * @param timeout_ms New timeout period in milliseconds
     */
    void setTimeout(std::chrono::milliseconds timeout_ms);
    
private:
    std::chrono::milliseconds timeout_; // Timeout period
    std::function<void()> callback_; // Callback function
    std::atomic<bool> running_; // Flag to indicate if watchdog is running
    std::atomic<bool> reset_flag_; // Flag to indicate if watchdog was reset
    std::thread watchdog_thread_; // Thread for watchdog timer
    
    /**
     * @brief Thread function that implements the watchdog timer logic.
     */
    void watchdogThreadFunc();
};

#endif // WATCHDOG_HPP