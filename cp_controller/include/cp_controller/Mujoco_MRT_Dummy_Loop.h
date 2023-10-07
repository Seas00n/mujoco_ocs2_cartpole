#pragma once
#include "ros/ros.h"
#include "ros/init.h"
#include "mujoco/mujoco.h"
#include <mutex>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
namespace ocs2
{
    namespace cp_controller
    {
        class Mujoco_MRT_Dummy_Loop final : ocs2::MRT_ROS_Dummy_Loop
        {
        public:
            using ocs2::MRT_ROS_Dummy_Loop::MRT_ROS_Dummy_Loop;
            using ocs2::MRT_ROS_Dummy_Loop::run;
            using ocs2::MRT_ROS_Dummy_Loop::subscribeObservers;
            bool initMujocoInterface(const std::string &xmlFile)
            {
                char error[100] = "Could not load binary model";
                model = mj_loadXML(xmlFile.c_str(), 0, error, 100);
                if (!model)
                {
                    mju_error_s("Load model error: %s", error);
                    initMujocoOver = false;
                    return initMujocoOver;
                }
                // make data
                data = mj_makeData(model);
                initMujocoOver = true;
                return initMujocoOver;
            }
            void modifyObservation(SystemObservation &observation)
            {
                if (!initMujocoOver)
                {
                    std::cout << "###Please Init Mujoco Interface first" << std::endl;
                    return;
                }
                // user defined state update
                // theta x dtheta dx
                data->qpos[0] = observation.state(1);
                data->qpos[1] = observation.state(0);
                data->qvel[0] = observation.state(3);
                data->qvel[1] = observation.state(2);
                data->ctrl[0] = observation.input(0);
                std::cout << "&&& New Policy in Mujoco " << data->ctrl[0] << std::endl;
                mj_step1(model, data);
                observation.state(0) = data->qpos[1];
                observation.state(1) = data->qpos[0];
                observation.state(2) = data->qvel[1];
                observation.state(3) = data->qvel[0];
            }
            ~Mujoco_MRT_Dummy_Loop()
            {
                mj_deleteData(data);
                mj_deleteModel(model);
            }

        private:
            mjModel *model;
            mjData *data;
            bool initMujocoOver = false;
            double tempInput = 0;
        };
    }
}