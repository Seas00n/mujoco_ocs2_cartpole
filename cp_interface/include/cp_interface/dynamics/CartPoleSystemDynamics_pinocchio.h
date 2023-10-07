#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "cp_interface/CartPoleModelnfo.h"
namespace ocs2
{
    namespace cp_interface
    {
        class CartPoleSytemDynamics_pinocchio : public SystemDynamicsBaseAD
        {
        public:
            CartPoleSytemDynamics_pinocchio(const CartPoleModelInfo &modelinfo, const std::string &libraryFolder, bool verbose)
                : info_(modelinfo)
            {
                this->initialize(info_.stateDim, info_.inputDim, "cartpole_dynamics", libraryFolder, true, verbose);
            }

            ~CartPoleSytemDynamics_pinocchio() override = default;

            CartPoleSytemDynamics_pinocchio(const CartPoleSytemDynamics_pinocchio &rhs) = default;

            CartPoleSytemDynamics_pinocchio *clone() const override { return new CartPoleSytemDynamics_pinocchio(*this); }

            ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                      const ad_vector_t &parameters) const override
            {
                const ad_scalar_t cosTheta = cos(state(0));
                const ad_scalar_t sinTheta = sin(state(0));
                const ad_scalar_t cos2Theta = cos(2 * state(0));
                // Inertia tensor
                Eigen::Matrix<ad_scalar_t, 2, 2> I;
                I << static_cast<ad_scalar_t>(info_.poleSteinerMoi_), static_cast<ad_scalar_t>(info_.poleMass_ * info_.poleHalfLength_ * cosTheta),
                    static_cast<ad_scalar_t>(info_.poleMass_ * info_.poleHalfLength_ * cosTheta),
                    static_cast<ad_scalar_t>(info_.cartMass_ + info_.poleMass_);

                // RHS
                Eigen::Matrix<ad_scalar_t, 2, 1> rhs(info_.poleMass_ * info_.poleHalfLength_ * info_.gravity_ * sinTheta +
                                                         info_.poleMass_ * info_.poleHalfLength_ * info_.poleHalfLength_ * pow(state(2), 2) * cos2Theta,
                                                     input(0) + info_.poleMass_ * info_.poleHalfLength_ * pow(state(2), 2) * sinTheta);

                // dxdt
                ad_vector_t stateDerivative(info_.stateDim);
                stateDerivative << state.tail<2>(), I.inverse() * rhs;
                return stateDerivative;
            }

        private:
            CartPoleModelInfo info_;
        };

    }
}