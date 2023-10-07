#include <ros/ros.h>
#include <ros/init.h>
#include <ros/package.h>
#include <mujoco/mujoco.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "cp_controller/CartpoleDummyVisualization.h"
#include "cp_controller/Mujoco_MRT_Dummy_Loop.h"
#include "cp_interface/CartPoleInterface.h"
int main(int argc, char **argv)
{
    const std::string robotName = "cartpole";
    // task file
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1)
    {
        throw std::runtime_error("No task file specified. Aborting.");
    }
    std::string taskFileFolderName = std::string(programArgs[1]);

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mj");
    ros::NodeHandle nodeHandle;

    const std::string taskFile = ros::package::getPath("cp_interface") + "/config/" + taskFileFolderName + "/task.info";
    const std::string libFolder = ros::package::getPath("cp_interface") + "/auto_generated";
    const std::string urdfFile = ros::package::getPath("cp_controller") + "/urdf/cartpole.urdf";
    const std::string xmlFile = ros::package::getPath("cp_controller") + "/urdf/cartpole.xml";

    ocs2::cp_interface::CartPoleInterface cartPoleInterface(taskFile, urdfFile, libFolder, false /*verbose*/);
    ocs2::MRT_ROS_Interface mrt(robotName);
    // mrt.initRollout(&cartPoleInterface.getRollout());
    mrt.launchNodes(nodeHandle);
    // Visualization
    auto cartpoleDummyVisualization = std::make_shared<ocs2::cp_controller::CartpoleDummyVisualization>(nodeHandle);

    ocs2::cp_controller::Mujoco_MRT_Dummy_Loop dummyCartpole(mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                                             cartPoleInterface.mpcSettings().mpcDesiredFrequency_);

    if (!dummyCartpole.initMujocoInterface(xmlFile))
        return 1;
    dummyCartpole.subscribeObservers({cartpoleDummyVisualization});

    // initial state
    ocs2::SystemObservation initObservation;
    initObservation.state = cartPoleInterface.getInitialState();
    initObservation.input.setZero(ocs2::cp_interface::INPUT_DIM);
    initObservation.time = 0.0;

    // initial command
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {cartPoleInterface.getInitialTarget()},
                                                          {ocs2::vector_t::Zero(ocs2::cp_interface::INPUT_DIM)});

    // Run dummy (loops while ros is ok)
    dummyCartpole.run(initObservation, initTargetTrajectories);

    return 0;
}
