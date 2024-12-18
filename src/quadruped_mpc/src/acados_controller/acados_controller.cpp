
#include "quadruped_mpc/acados_controller.hpp"
#include "acados_c/ocp_nlp_interface.h"
#include "quadruped_ode_model/quadruped_ode_model.h"
#include "quadruped_ode_model/quadruped_ode_model_impl.h"

namespace quadruped_mpc {

AcadosController::AcadosController(const std::string& build_dir) {
    // Initialize the solver capsule
    capsule_ = quadruped_ode_acados_create_capsule();
    if (capsule_ == nullptr) {
        throw std::runtime_error("Failed to create ACADOS capsule");
    }

    // Initialize the solver
    int status = quadruped_ode_acados_create(capsule_);
    if (status != 0) {
        throw std::runtime_error("Failed to create ACADOS solver");
    }
}

AcadosController::~AcadosController() {
    if (capsule_) {
        quadruped_ode_acados_free(capsule_);
        quadruped_ode_acados_free_capsule(capsule_);
    }
}

void AcadosController::updateFootPositions(
    const std::array<double,3>& p1,
    const std::array<double,3>& p2,
    const std::array<double,3>& p3,
    const std::array<double,3>& p4,
    const std::array<double,3>& com) {
    
    // Pack parameters
    std::copy(p1.begin(), p1.end(), params_.begin());
    std::copy(p2.begin(), p2.end(), params_.begin() + 3);
    std::copy(p3.begin(), p3.end(), params_.begin() + 6);
    std::copy(p4.begin(), p4.end(), params_.begin() + 9);
    std::copy(com.begin(), com.end(), params_.begin() + 12);

    // Update parameters in solver
    for (int stage = 0; stage <= quadruped_ode_N; ++stage) {
        quadruped_ode_acados_update_params(capsule_, stage, params_.data(), params_.size());
    }
}

int AcadosController::solve(
    const std::array<double,12>& x0,
    const std::array<double,12>& x_ref,
    std::array<double,12>& u_optimal) {
    
    // Set initial state
    quadruped_ode_acados_update_params(capsule_, 0, x0.data(), x0.size());

    // Set reference
    for (int stage = 0; stage <= quadruped_ode_N; ++stage) {
        quadruped_ode_acados_update_params(capsule_, stage, x_ref.data(), x_ref.size());
    }

    // Solve
    int status = quadruped_ode_acados_solve(capsule_);
    
    // Get solution
    if (status == 0) {
        ocp_nlp_out_get(capsule_->nlp_config, capsule_->nlp_dims, capsule_->nlp_out, 0, "u", u_optimal.data());
    }

    return status;
}

} // namespace quadruped_mpc