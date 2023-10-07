#pragma once
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
namespace ocs2
{
    namespace cp_interface
    {
        struct CartPoleModelInfo
        {
            size_t stateDim = 4;
            size_t inputDim = 1;
            scalar_t cartMass_ = 1.0;      // [kg]
            scalar_t poleMass_ = 1.0;      // [kg]
            scalar_t poleLength_ = 1.0;    // [m]
            scalar_t maxInput_ = 6.0;      // [N]
            scalar_t gravity_ = 9.8;       // [m/s^2]
            scalar_t poleHalfLength_ = -1; // [m]
            scalar_t poleMoi_ = -1;        // [kg*m^2]
            scalar_t poleSteinerMoi_ = -1; // [kg*m^2]
        };
    }
} // namespace ocs2
