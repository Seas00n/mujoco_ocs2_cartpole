#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// CartPole
#include "cp_interface/CartPoleParameters.h"
#include "cp_interface/definitions.h"
#include "cp_interface/CartPoleModelnfo.h"
namespace ocs2
{
    namespace cp_interface
    {

        class CartPoleInterface final : public RobotInterface
        {
        public:
            /**
             * Constructor
             *
             * @note Creates directory for generated library into if it does not exist.
             * @throw Invalid argument error if input task file does not exist.
             *
             * @param [in] taskFile: The absolute path to the configuration file for the MPC.
             * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
             * @param [in] verbose: Flag to determine to print out the loaded settings and status of complied libraries.
             */
            CartPoleInterface(const std::string &taskFile, const std::string &libraryFolder, bool verbose);
            CartPoleInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &libraryFolder, bool verbose);
            /**
             * Destructor
             */
            ~CartPoleInterface() override = default;

            const vector_t &getInitialState() { return initialState_; }

            const vector_t &getInitialTarget() { return xFinal_; }

            ddp::Settings &ddpSettings() { return ddpSettings_; }

            mpc::Settings &mpcSettings() { return mpcSettings_; }

            OptimalControlProblem &optimalControlProblem() { return problem_; }
            const OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }

            const RolloutBase &getRollout() const { return *rolloutPtr_; }

            const Initializer &getInitializer() const override { return *cartPoleInitializerPtr_; }

        private:
            ddp::Settings ddpSettings_;
            mpc::Settings mpcSettings_;

            OptimalControlProblem problem_;

            std::unique_ptr<RolloutBase> rolloutPtr_;
            std::unique_ptr<Initializer> cartPoleInitializerPtr_;
            std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;

            vector_t initialState_{STATE_DIM};
            vector_t xFinal_{STATE_DIM};
            CartPoleModelInfo info;
        };

    } // namespace cartpole
} // namespace ocs2
