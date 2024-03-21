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
    robot.waitForServers();
  
    geometry_msgs::Pose posa, end_effector;
    posa.position.x = 0;
    posa.position.y = -0.18;
    posa.position.z = 0.25;
    posa.orientation.w = 1;
    posa.orientation.x = 0;
    posa.orientation.y = 0;
    posa.orientation.z = 0;
    std::vector<double> qf = {0.45, 0.20, 0, 0.8, 0.1, 0, 0};

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("finger", "base_link", ros::Time(0), ros::Duration(3.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    end_effector.orientation = transformStamped.transform.rotation;
    end_effector.position.x = transformStamped.transform.translation.x;
    end_effector.position.y = transformStamped.transform.translation.y;
    end_effector.position.z = transformStamped.transform.translation.z;
    robot.set_end_effector(end_effector);

    if(askContinue("Prova")) {
        robot.goTo(posa, ros::Duration(5.0));
        ROS_INFO_STREAM("Posa raggiunta");
    }
  
}