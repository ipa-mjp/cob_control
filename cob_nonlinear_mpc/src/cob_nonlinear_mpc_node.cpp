#include <ros/ros.h>
#include <cob_nonlinear_mpc/cob_nonlinear_mpc.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc");
    CobNonlinearMPC* cob_nonlinear_mpc = new CobNonlinearMPC();

    if (!cob_nonlinear_mpc->initialize())
    {
        ROS_ERROR("Failed to initialize CobNonlinearMPC");
        return -1;
    }

    ros::spin();
    return 0;
}
