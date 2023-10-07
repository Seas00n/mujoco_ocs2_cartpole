#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include "cp_interface/CartPoleModelnfo.h"
#include "cp_interface/FactoryFunctions.h"
namespace ocs2
{
    namespace cp_interface
    {
        PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath)
        {
            return getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
        }
        CartPoleModelInfo createCartPoleModelInfo(const PinocchioInterface &interface)
        {
            const auto &model = interface.getModel();

            CartPoleModelInfo info;
            info.stateDim = model.nv * 2;
            info.inputDim = 1;
            info.cartMass_ = 2;
            info.poleMass_ = 0.2;
            info.poleLength_ = 1.0;
            info.maxInput_ = 5.0;
            info.gravity_ = 9.81;
            info.poleHalfLength_ = info.poleLength_ / 2.0;
            info.poleMoi_ = 1.0 / 12.0 * info.poleMass_ * (info.poleLength_ * info.poleLength_);
            info.poleSteinerMoi_ = info.poleMoi_ + info.poleMass_ * (info.poleHalfLength_ * info.poleHalfLength_);
            return info;
        }
    }

}