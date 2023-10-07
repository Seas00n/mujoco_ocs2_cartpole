#include <iostream>
#include <string>
#include "cp_interface/CartPoleInterface.h"
#include "cp_interface/FactoryFunctions.h"
#include "cp_interface/dynamics/CartPoleSystemDynamics_pinocchio.h"
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
namespace ocs2
{
    namespace cp_interface
    {
        CartPoleInterface::CartPoleInterface(const std::string &taskFile,
                                             const std::string &urdfFile,
                                             const std::string &libraryFolder, bool verbose)
        {
            // check that task file exists
            boost::filesystem::path taskFilePath(taskFile);
            if (boost::filesystem::exists(taskFilePath))
            {
                std::cerr << "[CartPoleInterface] Loading task file: " << taskFilePath << "\n";
            }
            else
            {
                throw std::invalid_argument("[CartPoleInterface] Task file not found: " + taskFilePath.string());
            }
            // check that urdf file exists
            boost::filesystem::path urdfFilePath(urdfFile);
            if (boost::filesystem::exists(urdfFilePath))
            {
                std::cerr << "[CartPoleInterface] Loading urdf file: " << urdfFilePath << "\n";
            }
            else
            {
                throw std::invalid_argument("[CartPoleInterface] urdf file not found: " + urdfFilePath.string());
            }
            // create library folder if it does not exist
            boost::filesystem::path libraryFolderPath(libraryFolder);
            boost::filesystem::create_directories(libraryFolderPath);
            std::cerr << "[CartPoleInterface] Generated library path: " << libraryFolderPath << "\n";

            // read task file
            // Default initial condition
            loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
            loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
            if (verbose)
            {
                std::cerr << "x_init:   " << initialState_.transpose() << "\n";
                std::cerr << "x_final:  " << xFinal_.transpose() << "\n";
            }

            // create pinocchio interface
            pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile)));
            info = cp_interface::createCartPoleModelInfo(*pinocchioInterfacePtr_);

            // DDP-MPC settings
            ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
            mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);

            /*
             * Optimal control problem
             */
            // Cost
            matrix_t Q(STATE_DIM, STATE_DIM);
            matrix_t R(INPUT_DIM, INPUT_DIM);
            matrix_t Qf(STATE_DIM, STATE_DIM);
            loadData::loadEigenMatrix(taskFile, "Q", Q);
            loadData::loadEigenMatrix(taskFile, "R", R);
            loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
            if (verbose)
            {
                std::cerr << "Q:  \n"
                          << Q << "\n";
                std::cerr << "R:  \n"
                          << R << "\n";
                std::cerr << "Q_final:\n"
                          << Qf << "\n";
            }

            problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
            problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

            // Dynamics
            problem_.dynamicsPtr.reset(new CartPoleSytemDynamics_pinocchio(info, libraryFolder, verbose));
            // Rollout
            auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
            rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

            // Constraints
            auto getPenalty = [&]()
            {
                // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
                using penalty_type = augmented::SlacknessSquaredHingePenalty;
                penalty_type::Config boundsConfig;
                loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, verbose);
                return penalty_type::create(boundsConfig);
            };
            auto getConstraint = [&]()
            {
                constexpr size_t numIneqConstraint = 2;
                const vector_t e = (vector_t(numIneqConstraint) << info.maxInput_, info.maxInput_).finished();
                const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0).finished();
                const matrix_t C = matrix_t::Zero(numIneqConstraint, info.stateDim);
                return std::make_unique<LinearStateInputConstraint>(e, C, D);
            };
            problem_.inequalityLagrangianPtr->add("InputLimits", create(getConstraint(), getPenalty()));

            // Initialization
            cartPoleInitializerPtr_.reset(new DefaultInitializer(info.inputDim));
        }

    } // namespace cp_interface

}