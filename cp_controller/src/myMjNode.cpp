#include <ros/ros.h>
#include <ros/init.h>
#include <ros/package.h>
#include <mujoco/mujoco.h>

void myController(const mjModel *m, mjData *d)
{
    std::cout << "q:" << d->qpos[0] << "," << d->qpos[1] << std::endl;
    d->ctrl[0] = 0;
    std::cout << "F" << d->ctrl[0] << std::endl;
    std::cout << "time:" << d->time << std::endl;
}

int main(int argc, char **argv)
{
    const std::string robotName = "cartpole";
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1)
    {
        throw std::runtime_error("No task file specified. Aborting.");
    }
    std::string taskFileFolderName = std::string(programArgs[1]);
    ros::init(argc, argv, robotName + "_mj");
    ros::NodeHandle nodeHandle;

    const std::string xmlFile = ros::package::getPath("cp_controller") + "/urdf/cartpole.xml";

    // make model
    char error[100] = "Could not load binary model";
    mjModel *m = mj_loadXML(xmlFile.c_str(), 0, error, 100);
    if (!m)
        mju_error_s("Load model error: %s", error);
    // make data
    mjData *d = mj_makeData(m);

    mjcb_control = myController;

    int totaltime = 100;

    while (1)
    {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }
        if (d->time >= totaltime)
        {
            break;
        }
    }
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
