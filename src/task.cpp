#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"
#include <tf2_ros/transform_listener.h>

bool askContinue(const std::string &prompt = "")
{
    char ans;
    std::cout << prompt << " - Press y to continue [s to skeep]: ";
    std::cin >> ans;

    if (ans == 'y' || ans == 'Y')
    {
        return true;
    }

    if (ans == 's' || ans == 'S')
    {
        return false;
    }

    throw std::runtime_error("USER STOP!");
}

// Metodo goTo in cartesiano, parametri di ingresso:
// posa desiderata, duration, time, bool wait, bool trapez, double trapez lin, double trapez vel

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task");
    //RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    

}