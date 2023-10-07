#pragma once
#include <string>
#include <vector>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "cp_interface/CartPoleModelnfo.h"
namespace ocs2
{
    namespace cp_interface
    {
        PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath);
        CartPoleModelInfo createCartPoleModelInfo(const PinocchioInterface &interface);
    }

}