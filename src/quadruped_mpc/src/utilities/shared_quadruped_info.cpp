#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"
#include <memory>
#include <mutex>

namespace quadruped_mpc {

// Use a smart pointer for proper cleanup
std::unique_ptr<SharedQuadrupedInfo> SharedQuadrupedInfo::instance_;
std::once_flag SharedQuadrupedInfo::init_flag_;
std::mutex SharedQuadrupedInfo::cleanup_mutex_;

SharedQuadrupedInfo& SharedQuadrupedInfo::getInstance() {
    std::call_once(init_flag_, []() {
        instance_.reset(new SharedQuadrupedInfo());
    });
    return *instance_;
}

void SharedQuadrupedInfo::cleanup() {
    std::lock_guard<std::mutex> lock(cleanup_mutex_);
    instance_.reset();
}

// Global instance accessor function instead of global variable
SharedQuadrupedInfo& get_quadruped_info() {
    return SharedQuadrupedInfo::getInstance();
}

// Replace the global variable with a function
SharedQuadrupedInfo& quadruped_info = SharedQuadrupedInfo::getInstance();

// Add destructor to handle cleanup
extern "C" void __attribute__((destructor)) cleanup_quadruped_info() {
    SharedQuadrupedInfo::cleanup();
}

} // namespace quadruped_mpc
