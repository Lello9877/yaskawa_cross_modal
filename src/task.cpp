#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>

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
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    //robot.waitForServers();
    
    geometry_msgs::Pose posa;
    posa.position.x = 0;
    posa.position.y = -0.18;
    posa.position.z = 0.25;

    if(askContinue("Prova")){
        robot.goTo(posa, ros::Duration(5.0));
        ROS_INFO_STREAM("Posa raggiunta");
    }
    
}